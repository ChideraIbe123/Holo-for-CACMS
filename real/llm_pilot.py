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

Two model providers are supported, chosen by --provider or by which key is present:
  openai     gpt-6-astra through the Responses API (key: OPENAI_API_KEY or OPENAI_KEY)
  anthropic  claude-opus-5-5 through the Messages API, effort low, with server-side
             refusal fallback (key: ANTHROPIC_API_KEY)
Both constrain the reply to the same JSON schema, so it always parses. Keys are read
from the environment or from a .env file next to this script, in its parent folder, or
in the current folder. Never commit that file.

Usage:
  python3 llm_pilot.py --check                      # one test call, no ROS, no motion
  python3 llm_pilot.py --goal "Follow the pipe on the floor to its far end, then stop."
  python3 llm_pilot.py --dry-run --goal "..."       # decide and log, publish zeros only
  python3 llm_pilot.py --frames DIR --goal "..."    # offline: judge recorded frames, no ROS
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

DEFAULT_MODEL = {"anthropic": "claude-opus-5-5", "openai": "gpt-6-astra"}
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
- forward: speed in metres per second, from -0.15 (slowly backward) to 0.25. The robot takes a moment \
to get moving, so short gentle steps cover very little ground: 0.12 for 1 second moves it under 0.1 m, while \
0.25 for 2 seconds moves it roughly 0.4 m. When the way ahead is clearly open, use the larger steps. Use 0 \
to turn on the spot.
- yaw_rate_cw: turn rate in radians per second, positive = turn right (clockwise seen from above), \
negative = turn left, from -0.5 to 0.5. At 0.4 for 1.5 seconds the robot turns roughly 35 degrees.
- hold_s: how long to apply the action, from 0.3 to 2.0 seconds.

How to drive well here:
- The pool is small and the walls are close. Prefer short, modest moves; you will get another look right away.
- The water blurs and tints things, and the surface above acts like a mirror, so reflections of objects \
can appear in the top of the frame. Trust what is on the floor and walls over what appears near the surface.
- If you cannot see what you need, turn on the spot (forward 0) to look around rather than driving blind.
- Judge how far away a wall is by where its base meets the floor, not by how dark or large it looks. \
Only when the base of the wall has dropped into the bottom fifth of the frame is the wall close \
(about a metre or less): then do not drive toward it. If the base is anywhere above that, there is room.
- Distances are hard to judge by eye here. When a goal can be checked against something visible, such \
as an object reaching the bottom edge of the frame, use that rather than a guessed distance.
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


class PilotError(Exception):
    """A model call that produced no usable action. fatal=True means retrying will not help."""

    def __init__(self, message, fatal=False):
        super().__init__(message)
        self.fatal = fatal


def load_dotenv():
    """Read KEY=VALUE lines from the first .env found. Existing environment wins."""
    here = os.path.dirname(os.path.abspath(__file__))
    for d in (os.getcwd(), here, os.path.dirname(here)):
        p = os.path.join(d, ".env")
        if os.path.isfile(p):
            for line in open(p):
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
            return p
    return None


def user_text(goal, step, elapsed, history, travel=None):
    lines = [f"Goal: {goal}", f"Step {step}, {elapsed:.0f} s since the run started."]
    if travel is not None:
        lines.append(f"Odometry since the start: moved about {travel[0]:.2f} m in total and turned "
                     f"{abs(travel[1]):.0f} degrees {'right' if travel[1] >= 0 else 'left'} of the starting heading. "
                     f"Odometry is approximate but far better than judging distance by eye in this water.")
    if history:
        lines.append("Your most recent actions, oldest first:")
        for h in history:
            lines.append(f"- forward {h['forward']:+.2f} m/s, turn {h['yaw_rate_cw']:+.2f} rad/s, "
                         f"for {h['hold_s']:.1f} s. You saw: {h['seeing']}")
    else:
        lines.append("This is the first frame.")
    lines.append("Here is the current camera frame. Choose the next action.")
    return "\n".join(lines)


def parse_action(text):
    try:
        return sanitize(json.loads(text))
    except (ValueError, KeyError, TypeError) as e:
        raise PilotError(f"unusable reply: {e}")


class AnthropicBrain:
    """claude-opus-5-5 by default. One Messages API call per step."""
    provider = "anthropic"

    def __init__(self, model, effort, timeout, use_fallbacks=True, base_url=None):
        import anthropic
        self.sdk = anthropic
        kwargs = {"timeout": timeout, "max_retries": 0}   # we retry by looking again, not by waiting
        if base_url:
            kwargs["base_url"] = base_url
        try:
            self.client = anthropic.Anthropic(**kwargs)
        except (anthropic.AnthropicError, TypeError) as e:
            raise PilotError(f"no usable Anthropic credentials: {e}", fatal=True)
        self.model, self.effort, self.use_fallbacks = model, effort or "low", use_fallbacks

    def decide(self, jpeg_bytes, goal, step, elapsed, history, travel=None):
        sdk = self.sdk
        request = dict(
            model=self.model,
            max_tokens=4000,
            system=SYSTEM_PROMPT,
            output_config={"effort": self.effort,
                           "format": {"type": "json_schema", "schema": ACTION_SCHEMA}},
            messages=[{"role": "user", "content": [
                {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                             "data": base64.standard_b64encode(jpeg_bytes).decode("ascii")}},
                {"type": "text", "text": user_text(goal, step, elapsed, history, travel)},
            ]}],
        )
        t0 = time.monotonic()
        try:
            if self.use_fallbacks:
                resp = self.client.beta.messages.create(
                    betas=["server-side-fallback-2026-07-01"], fallbacks="default", **request)
            else:
                resp = self.client.messages.create(**request)
        except sdk.BadRequestError as e:
            raise PilotError(f"request rejected: {e.message}", fatal=True)
        except (sdk.AuthenticationError, sdk.PermissionDeniedError, sdk.NotFoundError) as e:
            raise PilotError(f"{type(e).__name__}: {e.message}", fatal=True)
        except sdk.RateLimitError:
            raise PilotError("rate limited")
        except sdk.APITimeoutError:
            raise PilotError("no reply in time")
        except sdk.APIConnectionError:
            raise PilotError("cannot reach the API")
        except sdk.APIStatusError as e:
            raise PilotError(f"API error {e.status_code}")
        except (sdk.AnthropicError, TypeError) as e:
            raise PilotError(f"no usable Anthropic credentials: {e}", fatal=True)
        latency = time.monotonic() - t0
        if resp.stop_reason == "refusal":
            cat = resp.stop_details.category if resp.stop_details else None
            raise PilotError(f"model declined (category {cat})")
        if resp.stop_reason == "max_tokens":
            raise PilotError("reply cut off at max_tokens")
        text = next((b.text for b in resp.content if b.type == "text"), None)
        if text is None:
            raise PilotError("no text block in reply")
        return parse_action(text), {"latency_s": round(latency, 3), "model": resp.model,
                                    "input_tokens": resp.usage.input_tokens,
                                    "output_tokens": resp.usage.output_tokens}


class OpenAIBrain:
    """gpt-6-astra by default. One Responses API call per step."""
    provider = "openai"

    def __init__(self, model, effort, timeout, base_url=None):
        import openai
        self.sdk = openai
        key = os.environ.get("OPENAI_API_KEY") or os.environ.get("OPENAI_KEY")
        if not key:
            raise PilotError("no OpenAI key: set OPENAI_API_KEY (or OPENAI_KEY) in the environment or .env", fatal=True)
        kwargs = {"api_key": key, "timeout": timeout, "max_retries": 0}
        if base_url:
            kwargs["base_url"] = base_url
        self.client = openai.OpenAI(**kwargs)
        self.model, self.effort = model, effort

    def decide(self, jpeg_bytes, goal, step, elapsed, history, travel=None):
        sdk = self.sdk
        b64 = base64.standard_b64encode(jpeg_bytes).decode("ascii")
        request = dict(
            model=self.model,
            instructions=SYSTEM_PROMPT,
            input=[{"role": "user", "content": [
                {"type": "input_image", "image_url": f"data:image/jpeg;base64,{b64}"},
                {"type": "input_text", "text": user_text(goal, step, elapsed, history, travel)},
            ]}],
            text={"format": {"type": "json_schema", "name": "rov_action", "strict": True,
                             "schema": ACTION_SCHEMA}},
            max_output_tokens=2000,
        )
        if self.effort:
            request["reasoning"] = {"effort": self.effort}
        t0 = time.monotonic()
        try:
            resp = self.client.responses.create(**request)
        except sdk.BadRequestError as e:
            raise PilotError(f"request rejected: {e.message}", fatal=True)
        except (sdk.AuthenticationError, sdk.PermissionDeniedError, sdk.NotFoundError) as e:
            raise PilotError(f"{type(e).__name__}: {e.message}", fatal=True)
        except sdk.RateLimitError:
            raise PilotError("rate limited")
        except sdk.APITimeoutError:
            raise PilotError("no reply in time")
        except sdk.APIConnectionError:
            raise PilotError("cannot reach the API")
        except sdk.APIStatusError as e:
            raise PilotError(f"API error {e.status_code}")
        latency = time.monotonic() - t0
        if resp.status != "completed":
            why = getattr(resp.incomplete_details, "reason", None) if resp.incomplete_details else None
            raise PilotError(f"reply {resp.status} ({why})")
        for item in resp.output:
            for part in getattr(item, "content", None) or []:
                if part.type == "refusal":
                    raise PilotError("model declined")
        if not resp.output_text:
            raise PilotError("empty reply")
        return parse_action(resp.output_text), {"latency_s": round(latency, 3), "model": resp.model,
                                                "input_tokens": resp.usage.input_tokens,
                                                "output_tokens": resp.usage.output_tokens}


def make_brain(a):
    load_dotenv()
    provider = a.provider
    if provider == "auto":
        has_openai = bool(os.environ.get("OPENAI_API_KEY") or os.environ.get("OPENAI_KEY"))
        has_anthropic = bool(os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("ANTHROPIC_AUTH_TOKEN"))
        provider = "openai" if has_openai and not has_anthropic else "anthropic"
    model = a.model or DEFAULT_MODEL[provider]
    if provider == "openai":
        return OpenAIBrain(model, a.effort, a.timeout, a.base_url)
    return AnthropicBrain(model, a.effort, a.timeout, not a.no_fallbacks, a.base_url)


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
    try:
        brain = make_brain(a)
        action, info = brain.decide(encode_jpeg(img, 640, 70), "Test call. Report what you see and stay still.", 1, 0.0, [])
    except PilotError as e:
        sys.exit(f"CHECK FAILED: {e}")
    print(f"CHECK OK  provider={brain.provider}  model={info['model']}  latency={info['latency_s']:.1f} s  "
          f"tokens in/out={info['input_tokens']}/{info['output_tokens']}")
    print(f"  saw: {action['seeing']}\n  action: forward {action['forward']} yaw {action['yaw_rate_cw']} "
          f"hold {action['hold_s']} done {action['done']}")


def run_frames(a):
    """Show the model recorded frames, one independent decision each. No ROS, no vehicle.
    Answers 'what would it do if it saw this?' for real footage. Each frame is judged on its
    own (no action history), because recorded frames do not respond to the model's actions."""
    import glob
    import numpy as np
    from PIL import Image
    paths = sorted(p for p in glob.glob(os.path.join(a.frames, "*")) if p.lower().endswith((".png", ".jpg", ".jpeg")))
    paths = paths[::max(1, a.every)][:a.max_steps]
    if not paths:
        sys.exit(f"no .png/.jpg frames in {a.frames}")
    try:
        brain = make_brain(a)
    except PilotError as e:
        sys.exit(str(e))
    os.makedirs(a.out, exist_ok=True)
    print(f"offline: {len(paths)} frames from {a.frames}  provider={brain.provider} model={brain.model}\nGoal: {a.goal}\n")
    with open(os.path.join(a.out, "steps.jsonl"), "w") as f:
        for i, p in enumerate(paths, 1):
            rgb = np.asarray(Image.open(p).convert("RGB"))
            jpeg = encode_jpeg(rgb, a.width, a.quality)
            with open(os.path.join(a.out, f"frame_{i:04d}.jpg"), "wb") as jf:
                jf.write(jpeg)
            rec = {"step": i, "source": os.path.basename(p)}
            try:
                action, info = brain.decide(jpeg, a.goal, 1, 0.0, [])
                rec.update(status="ok", **action, **info)
                print(f"{os.path.basename(p):<22} {info['latency_s']:4.1f} s  fwd {action['forward']:+.2f}  "
                      f"turn {action['yaw_rate_cw']:+.2f}  hold {action['hold_s']:.1f}  done={str(action['done']):<5}  "
                      f"SEES: {action['seeing']}\n{'':<22} WHY: {action['why']}")
            except PilotError as e:
                rec.update(status="error", error=str(e))
                print(f"{os.path.basename(p):<22} ERROR: {e}")
            f.write(json.dumps(rec) + "\n")
    print(f"\nsaved to {a.out}")


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
        brain = make_brain(a)
    except PilotError as e:
        sys.exit(f"{e}\nRun with --check first.")
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
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        S["z"] = p.z
        if S["t0"] is None:
            S["odo"] = None                      # travel is counted from the first step
            return
        if S.get("odo") is None:
            S["odo"] = {"x": p.x, "y": p.y, "yaw0": yaw, "dist": 0.0, "yaw": yaw}
        o = S["odo"]
        o["dist"] += math.hypot(p.x - o["x"], p.y - o["y"])
        o["x"], o["y"], o["yaw"] = p.x, p.y, yaw

    def travel_now():
        o = S.get("odo")
        if not o:
            return None
        d = math.atan2(math.sin(o["yaw"] - o["yaw0"]), math.cos(o["yaw"] - o["yaw0"]))
        return (o["dist"], -math.degrees(d))     # positive = turned right (clockwise)

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

    def think(jpeg, step, elapsed, history, travel):
        try:
            action, info = brain.decide(jpeg, a.goal, step, elapsed, history, travel)
            result = ("ok", action, info)
        except PilotError as e:
            result = ("fatal" if e.fatal else "error", str(e), None)
        except Exception as e:                      # never let a worker-thread bug leave the vehicle moving
            result = ("error", f"unexpected {type(e).__name__}: {e}", None)
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
            threading.Thread(target=think, args=(jpeg, S["step"], elapsed, list(S["history"]), travel_now()),
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
    log.info(f"llm_pilot: provider={brain.provider} model={brain.model} {'DRY RUN (zeros only) ' if a.dry_run else ''}"
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
    ap.add_argument("--provider", default="auto", choices=["auto", "openai", "anthropic"],
                    help="auto: openai if only an OpenAI key is present, else anthropic")
    ap.add_argument("--model", default=None, help="default: gpt-6-astra (openai) or claude-opus-5-5 (anthropic)")
    ap.add_argument("--effort", default=None, help="reasoning effort. anthropic default low; openai default: not sent")
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
    ap.add_argument("--frames", default=None, metavar="DIR",
                    help="offline: judge each recorded .png/.jpg in DIR independently. No ROS, no vehicle.")
    ap.add_argument("--every", type=int, default=1, help="with --frames: use every Nth image")
    a = ap.parse_args()
    if a.check:
        run_check(a)
    elif a.frames:
        run_frames(a)
    else:
        run_ros(a)


if __name__ == "__main__":
    main()
