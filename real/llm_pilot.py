#!/usr/bin/env python3
"""A vision-language model drives the ROV from its camera.

Loop, one step at a time ("look, decide, move, stop"):
  1. take the newest frame from /camera/image_raw, shrink it, JPEG-encode it
  2. send that frame plus a few lines of text (goal, time, recent decisions) to Claude
  3. get back ONE small action as JSON: forward speed, turn rate, how long, done?
  4. publish that action on /cmd_vel for at most `hold_s` seconds, then publish zeros
  5. wait --settle seconds so the next frame is not motion-blurred, and repeat

The vehicle stands still while the model thinks, so model latency costs time, not
safety. /cmd_vel goes through cmdvel_to_manual.py like every other controller here, so
its stick limits, geofence and SPACE stop all still apply. This node never arms,
never talks to the autopilot, and stops publishing motion if the API fails.

The model call uses the Anthropic SDK:
  - model claude-opus-5-5 (change with --model)
  - output constrained to a JSON schema (output_config.format), so the reply always parses
  - effort "low": this is a small perception-to-action step and latency matters
  - server-side refusal fallback, so a safety-classifier decline does not stall the run
Credentials come from the environment (ANTHROPIC_API_KEY, or an `ant auth login` profile).

Usage:
  python3 llm_pilot.py --check                      # one test call, no ROS, no motion
  python3 llm_pilot.py --goal "Follow the pipe on the floor to its far end, then stop."
  python3 llm_pilot.py --dry-run --goal "..."       # decide and log, publish zeros only
Every step is saved under --out: the exact JPEG sent, the decision, latency, token use.
"""
import argparse
import base64
import io
import json
import math
import os
import sys
import threading
import time

import anthropic

MODEL = "claude-opus-5-5"
MAX_FORWARD = 0.25        # m/s
MAX_YAW = 0.5             # rad/s
MAX_HOLD = 2.0            # s
MIN_HOLD = 0.3

SYSTEM_PROMPT = """You are piloting a small underwater robot (a BlueROV2, about 46 cm long and 34 cm wide) \
in an indoor test pool about 4 m long, 2 m wide and 1 m deep. You see through its single forward-facing \
camera, which has a wide field of view (about 110 degrees) and looks straight ahead and slightly below the \
horizon. You cannot see behind or directly beneath the robot.

You control it in short steps. Each time you are shown the current camera frame, you choose one small \
action. The robot carries out that action, comes to a stop, and then you are shown a new frame. The robot \
is stationary whenever you are looking, so take the view at face value.

The action has three numbers:
- forward: speed in metres per second, from -0.15 (slowly backward) to 0.25. At 0.2 for 1.5 seconds the \
robot advances roughly 0.3 m. Use 0 to turn on the spot.
- yaw_rate_cw: turn rate in radians per second, positive = turn right (clockwise seen from above), \
negative = turn left, from -0.5 to 0.5. At 0.4 for 1.5 seconds the robot turns roughly 35 degrees.
- hold_s: how long to apply the action, from 0.3 to 2.0 seconds.

How to drive well here:
- The pool is small and the walls are close. Prefer short, modest moves; you will get another look right away.
- The water blurs and tints things, and the surface above acts like a mirror, so reflections of objects \
can appear in the top of the frame. Trust what is on the floor and walls over what appears near the surface.
- If you cannot see what you need, turn on the spot (forward 0) to look around rather than driving blind.
- If a wall fills most of the view or looks closer than about half a metre, do not drive toward it.
- The robot drifts a little and does not hold a perfectly straight line, so correct your heading often.
- When the goal is achieved, or you judge it cannot be achieved safely, set done to true with forward 0 \
and yaw_rate_cw 0.

Reply with the action only, in the required JSON format. In "seeing", say in one short sentence what in \
the frame matters for the goal. In "why", say in one short sentence why this action follows from it."""

ACTION_SCHEMA = {
    "type": "object",
    "properties": {
        "seeing": {"type": "string"},
        "why": {"type": "string"},
        "forward": {"type": "number"},
        "yaw_rate_cw": {"type": "number"},
        "hold_s": {"type": "number"},
        "done": {"type": "boolean"},
    },
    "required": ["seeing", "why", "forward", "yaw_rate_cw", "hold_s", "done"],
    "additionalProperties": False,
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def sanitize(action):
    """Clamp a parsed action to the allowed envelope. Raises on a malformed one."""
    f = float(action["forward"])
    r = float(action["yaw_rate_cw"])
    h = float(action["hold_s"])
    if not all(math.isfinite(v) for v in (f, r, h)):
        raise ValueError("non-finite number in action")
    done = bool(action["done"])
    return {
        "seeing": str(action.get("seeing", ""))[:300],
        "why": str(action.get("why", ""))[:300],
        "forward": 0.0 if done else clamp(f, -0.15, MAX_FORWARD),
        "yaw_rate_cw": 0.0 if done else clamp(r, -MAX_YAW, MAX_YAW),
        "hold_s": clamp(h, MIN_HOLD, MAX_HOLD),
        "done": done,
    }


class Brain:
    """One model call per step. Holds no ROS state."""

    def __init__(self, model=MODEL, effort="low", timeout=20.0, use_fallbacks=True, base_url=None):
        kwargs = {"timeout": timeout, "max_retries": 0}   # we retry by looking again, not by waiting
        if base_url:
            kwargs["base_url"] = base_url
        self.client = anthropic.Anthropic(**kwargs)
        self.model, self.effort, self.use_fallbacks = model, effort, use_fallbacks

    def decide(self, jpeg_bytes, goal, step, elapsed, history):
        """-> (action dict, info dict). Raises anthropic errors and ValueError."""
        lines = [f"Goal: {goal}", f"Step {step}, {elapsed:.0f} s since the run started."]
        if history:
            lines.append("Your most recent actions, oldest first:")
            for h in history:
                lines.append(f"- forward {h['forward']:+.2f} m/s, turn {h['yaw_rate_cw']:+.2f} rad/s, "
                             f"for {h['hold_s']:.1f} s. You saw: {h['seeing']}")
        else:
            lines.append("This is the first frame.")
        lines.append("Here is the current camera frame. Choose the next action.")
        request = dict(
            model=self.model,
            max_tokens=4000,
            system=SYSTEM_PROMPT,
            output_config={"effort": self.effort,
                           "format": {"type": "json_schema", "schema": ACTION_SCHEMA}},
            messages=[{"role": "user", "content": [
                {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                             "data": base64.standard_b64encode(jpeg_bytes).decode("ascii")}},
                {"type": "text", "text": "\n".join(lines)},
            ]}],
        )
        t0 = time.monotonic()
        if self.use_fallbacks:
            resp = self.client.beta.messages.create(
                betas=["server-side-fallback-2026-07-01"], fallbacks="default", **request)
        else:
            resp = self.client.messages.create(**request)
        latency = time.monotonic() - t0
        if resp.stop_reason == "refusal":
            cat = resp.stop_details.category if resp.stop_details else None
            raise ValueError(f"model declined (category {cat})")
        if resp.stop_reason == "max_tokens":
            raise ValueError("reply cut off at max_tokens")
        text = next((b.text for b in resp.content if b.type == "text"), None)
        if text is None:
            raise ValueError("no text block in reply")
        action = sanitize(json.loads(text))
        u = resp.usage
        info = {"latency_s": round(latency, 3), "model": resp.model,
                "input_tokens": u.input_tokens, "output_tokens": u.output_tokens}
        return action, info


def encode_jpeg(rgb, width, quality):
    """HxWx3 uint8 RGB array -> JPEG bytes at the requested width."""
    h, w = rgb.shape[:2]
    new_h = max(1, round(h * width / w))
    try:
        import cv2
        small = cv2.resize(rgb, (width, new_h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", small[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()
    except ImportError:
        from PIL import Image
        im = Image.fromarray(rgb).resize((width, new_h))
        out = io.BytesIO()
        im.save(out, format="JPEG", quality=quality)
        return out.getvalue()


def run_check(a):
    """One real call on a synthetic frame: proves credentials, model access and request shape."""
    import numpy as np
    y, x = np.mgrid[0:360, 0:640]
    img = np.stack([40 + x // 8, 110 + y // 6, 150 + 0 * x], axis=2).astype(np.uint8)
    img[250:270, 100:540] = (60, 40, 30)          # a dark bar on the "floor"
    brain = Brain(a.model, a.effort, a.timeout, not a.no_fallbacks, a.base_url)
    try:
        action, info = brain.decide(encode_jpeg(img, 640, 70), "Test call. Report what you see and stay still.", 1, 0.0, [])
    except anthropic.AuthenticationError:
        sys.exit("CHECK FAILED: credentials rejected. Set ANTHROPIC_API_KEY or run `ant auth login`.")
    except anthropic.PermissionDeniedError as e:
        sys.exit(f"CHECK FAILED: this key may not use {a.model}: {e.message}")
    except anthropic.NotFoundError as e:
        sys.exit(f"CHECK FAILED: model or endpoint not found: {e.message}")
    except anthropic.BadRequestError as e:
        sys.exit(f"CHECK FAILED: request rejected: {e.message}\n(try --no-fallbacks if it names 'fallbacks')")
    except anthropic.RateLimitError:
        sys.exit("CHECK FAILED: rate limited. Wait and retry.")
    except anthropic.APIStatusError as e:
        sys.exit(f"CHECK FAILED: API error {e.status_code}: {e.message}")
    except anthropic.APIConnectionError as e:
        sys.exit(f"CHECK FAILED: cannot reach the API (no internet?): {e}")
    except (anthropic.AnthropicError, TypeError) as e:
        sys.exit(f"CHECK FAILED: no usable credentials or client error: {e}")
    print(f"CHECK OK  model={info['model']}  latency={info['latency_s']:.1f} s  "
          f"tokens in/out={info['input_tokens']}/{info['output_tokens']}")
    print(f"  saw: {action['seeing']}\n  action: forward {action['forward']} yaw {action['yaw_rate_cw']} "
          f"hold {action['hold_s']} done {action['done']}")


def run_ros(a):
    import numpy as np
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("llm_pilot")
    log = node.get_logger()
    try:
        brain = Brain(a.model, a.effort, a.timeout, not a.no_fallbacks, a.base_url)
    except (anthropic.AnthropicError, TypeError) as e:
        sys.exit(f"no usable API credentials: {e}\nSet ANTHROPIC_API_KEY, then run with --check first.")
    os.makedirs(a.out, exist_ok=True)
    steps_f = open(os.path.join(a.out, "steps.jsonl"), "a")
    pub = node.create_publisher(TwistStamped, a.cmd_topic, 10)
    S = {"frame": None, "frame_t": 0.0, "z": None, "phase": "wait", "until": 0.0, "action": None,
         "step": 0, "errors": 0, "t0": None, "history": [], "done": False, "result": None}
    lock = threading.Lock()

    def on_image(msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            return
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        arr = arr.reshape(msg.height, msg.width, 3)
        S["frame"] = arr[:, :, ::-1].copy() if msg.encoding == "bgr8" else arr.copy()
        S["frame_t"] = time.monotonic()

    def on_odom(msg):
        S["z"] = msg.pose.pose.position.z

    node.create_subscription(Image, a.image_topic, on_image, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/deadreckon/odom", on_odom, 50)

    def publish(u, r_cw):
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = float(u)
        m.twist.angular.z = float(r_cw)
        if a.depth_target is not None and S["z"] is not None:
            m.twist.linear.z = clamp(0.6 * (a.depth_target - S["z"]), -0.3, 0.3)
        pub.publish(m)

    def think(jpeg, step, elapsed, history):
        try:
            action, info = brain.decide(jpeg, a.goal, step, elapsed, history)
            result = ("ok", action, info)
        except anthropic.RateLimitError:
            result = ("error", "rate limited", None)
        except anthropic.APITimeoutError:
            result = ("error", f"no reply within {a.timeout:.0f} s", None)
        except anthropic.APIConnectionError:
            result = ("error", "cannot reach the API", None)
        except anthropic.BadRequestError as e:
            result = ("fatal", f"request rejected: {e.message}", None)
        except (anthropic.AuthenticationError, anthropic.PermissionDeniedError, anthropic.NotFoundError) as e:
            result = ("fatal", f"{type(e).__name__}: {e.message}", None)
        except anthropic.APIStatusError as e:
            result = ("error", f"API error {e.status_code}", None)
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            result = ("error", f"unusable reply: {e}", None)
        with lock:
            S["result"] = result

    def tick():
        now = time.monotonic()
        if S["done"]:
            publish(0.0, 0.0)
            return
        if S["t0"] is None:
            if S["frame"] is None:
                return
            S["t0"] = now
            S["phase"] = "look"
            log.info(f"camera is live. Goal: {a.goal}")
        elapsed = now - S["t0"]
        if elapsed > a.max_time or S["step"] >= a.max_steps:
            log.warn(f"limit reached ({S['step']} steps, {elapsed:.0f} s): stopping")
            S["done"] = True
            return
        if S["phase"] == "move":
            if now < S["until"]:
                act = S["action"]
                publish(0.0 if a.dry_run else act["forward"], 0.0 if a.dry_run else act["yaw_rate_cw"])
                return
            S["phase"], S["until"] = "settle", now + a.settle
        if S["phase"] == "settle":
            publish(0.0, 0.0)
            if now >= S["until"]:
                S["phase"] = "look"
            return
        if S["phase"] == "look":
            publish(0.0, 0.0)
            if now - S["frame_t"] > 1.0:
                return                                   # camera stalled: stay still and wait
            S["step"] += 1
            jpeg = encode_jpeg(S["frame"], a.width, a.quality)
            S["jpeg"] = jpeg
            with open(os.path.join(a.out, f"frame_{S['step']:04d}.jpg"), "wb") as f:
                f.write(jpeg)
            S["phase"], S["think_t"] = "think", now
            threading.Thread(target=think, args=(jpeg, S["step"], elapsed, list(S["history"])),
                             daemon=True).start()
            return
        if S["phase"] == "think":
            publish(0.0, 0.0)
            with lock:
                result, S["result"] = S["result"], None
            if result is None:
                return
            kind, payload, info = result
            rec = {"step": S["step"], "t": round(elapsed, 2), "status": kind}
            if kind == "ok":
                S["errors"] = 0
                rec.update(payload)
                rec.update(info)
                log.info(f"step {S['step']} ({info['latency_s']:.1f} s): fwd {payload['forward']:+.2f} "
                         f"yaw {payload['yaw_rate_cw']:+.2f} for {payload['hold_s']:.1f} s | {payload['seeing']}")
                S["history"] = (S["history"] + [payload])[-a.memory:]
                if payload["done"]:
                    log.info(f"model reports done: {payload['why']}")
                    S["done"] = True
                else:
                    S["action"], S["phase"], S["until"] = payload, "move", now + payload["hold_s"]
            else:
                S["errors"] += 1
                rec["error"] = payload
                log.error(f"step {S['step']}: {payload} ({S['errors']} in a row)")
                if kind == "fatal" or S["errors"] >= a.max_errors:
                    log.error("stopping the run")
                    S["done"] = True
                else:
                    S["phase"], S["until"] = "settle", now + 1.0   # stay still, then look again
            steps_f.write(json.dumps(rec) + "\n")
            steps_f.flush()

    node.create_timer(0.1, tick)
    log.info(f"llm_pilot: model={a.model} effort={a.effort} {'DRY RUN (zeros only) ' if a.dry_run else ''}"
             f"frames {a.width}px -> {a.out}")
    stop_count = [0]

    def watchdog():
        if S["done"]:
            stop_count[0] += 1
            if stop_count[0] > 15:
                raise SystemExit(0)

    node.create_timer(0.1, watchdog)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        for _ in range(5):
            publish(0.0, 0.0)
        steps_f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--goal", default="Follow the pipe lying on the pool floor to its far end, staying over it, then stop.")
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--effort", default="low", choices=["low", "medium", "high", "xhigh", "max"])
    ap.add_argument("--timeout", type=float, default=20.0, help="seconds to wait for one model reply")
    ap.add_argument("--no-fallbacks", action="store_true", help="do not request server-side refusal fallback")
    ap.add_argument("--base-url", default=None, help="testing only")
    ap.add_argument("--image-topic", default="/camera/image_raw")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--width", type=int, default=768, help="frame width sent to the model, px")
    ap.add_argument("--quality", type=int, default=70, help="JPEG quality")
    ap.add_argument("--settle", type=float, default=0.8, help="s to stand still before taking the next frame")
    ap.add_argument("--memory", type=int, default=5, help="recent actions described to the model")
    ap.add_argument("--max-steps", type=int, default=40)
    ap.add_argument("--max-time", type=float, default=240.0)
    ap.add_argument("--max-errors", type=int, default=3, help="consecutive failed calls before giving up")
    ap.add_argument("--depth-target", type=float, default=None, help="hold this depth (m, negative down)")
    ap.add_argument("--dry-run", action="store_true", help="decide and log but publish zero motion")
    ap.add_argument("--out", default="llm_runs/run")
    ap.add_argument("--check", action="store_true", help="one test call to the model, then exit")
    a = ap.parse_args()
    if a.check:
        run_check(a)
    else:
        run_ros(a)


if __name__ == "__main__":
    main()
