#!/usr/bin/env python3
"""Hardware-free, network-free test of llm_pilot.py.
A local HTTP server stands in for the Anthropic API and records every request; a fake
camera publishes 1080p frames; the real llm_pilot node runs against both. Checks the
request the SDK actually sends (model, image, schema, effort, fallback opt-in) and the
node's behaviour on good replies, an out-of-range action, a server error, an unparseable
reply, a refusal, and "done".
Usage: ROS_DOMAIN_ID=74 python3 test_llm_fake.py"""
import base64, io, json, os, shutil, signal, subprocess, sys, tempfile, threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 18765
requests, cmds = [], []


def msg(text=None, stop="end_turn", details=None):
    return {"id": "msg_test", "type": "message", "role": "assistant", "model": "claude-opus-5-5",
            "content": [] if text is None else [{"type": "text", "text": text}],
            "stop_reason": stop, "stop_sequence": None, "stop_details": details,
            "usage": {"input_tokens": 1200, "output_tokens": 60}}


def act(f, r, h, done=False):
    return json.dumps({"seeing": "a dark pipe ahead on the floor", "why": "keep it centred",
                       "forward": f, "yaw_rate_cw": r, "hold_s": h, "done": done})


SCRIPT = [
    (200, msg(act(0.9, 0.1, 0.6))),            # 1 out-of-range forward -> must be clamped to 0.25
    (500, {"type": "error", "error": {"type": "api_error", "message": "boom"}}),   # 2 server error
    (200, msg(act(0.0, -2.0, 0.5))),           # 3 out-of-range turn -> clamped to -0.5
    (200, msg("this is not json")),            # 4 unparseable
    (200, msg(None, "refusal", {"type": "refusal", "category": "cyber", "explanation": "x"})),  # 5 refusal
    (200, msg(act(0.2, 0.0, 1.0, done=True))), # 6 done -> must not move despite forward 0.2
]


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["content-length"])))
        requests.append({"path": self.path, "beta": self.headers.get("anthropic-beta", ""), "body": body})
        code, payload = SCRIPT[min(len(requests) - 1, len(SCRIPT) - 1)]
        data = json.dumps(payload).encode()
        self.send_response(code); self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(data))); self.end_headers(); self.wfile.write(data)


def main():
    srv = HTTPServer(("127.0.0.1", PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    out = tempfile.mkdtemp(prefix="llmtest_")
    env = dict(os.environ, ANTHROPIC_API_KEY="test-key-not-real")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "llm_pilot.py"), "--provider", "anthropic", "--base-url", f"http://127.0.0.1:{PORT}",
                             "--settle", "0.3", "--out", out, "--goal", "Follow the pipe.", "--max-errors", "3"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    rclpy.init()
    n = rclpy.create_node("fake_camera")
    ipub = n.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
    n.create_subscription(TwistStamped, "/cmd_vel", lambda m: cmds.append((time.time(), m.twist.linear.x, m.twist.angular.z)), 50)
    frame = np.zeros((1080, 1920, 3), np.uint8); frame[:, :, 0] = 150; frame[:, :, 1] = 110; frame[700:760, 300:1600] = 30
    im = Image(); im.height, im.width, im.encoding, im.step = 1080, 1920, "bgr8", 1920 * 3; im.data = frame.tobytes()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()   # listen continuously
    t_end = time.time() + 40
    while time.time() < t_end and proc.poll() is None:
        ipub.publish(im); time.sleep(0.2)
    time.sleep(0.5)
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    log = proc.communicate(timeout=10)[0]
    res = []
    def check(name, ok, detail=""):
        res.append(bool(ok)); print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))
    r0 = requests[0] if requests else {"body": {}, "beta": "", "path": ""}
    b = r0["body"]
    print(f"model calls: {len(requests)}   cmd_vel messages: {len(cmds)}   exit code: {proc.returncode}")
    check("six calls, node exits cleanly by itself", len(requests) == 6 and proc.returncode == 0)
    check("model id", b.get("model") == "claude-opus-5-5", b.get("model"))
    check("refusal fallback requested", b.get("fallbacks") == "default" and "server-side-fallback-2026-07-01" in r0["beta"], f"{b.get('fallbacks')!r} / {r0['beta']}")
    oc = b.get("output_config", {})
    check("effort low + JSON schema output", oc.get("effort") == "low" and oc.get("format", {}).get("type") == "json_schema"
          and oc["format"]["schema"].get("additionalProperties") is False)
    check("no thinking / sampling params sent (they 400 on this model)", not any(k in b for k in ("thinking", "temperature", "top_p", "top_k", "tool_choice")))
    content = b.get("messages", [{}])[0].get("content", [])
    img = next((c for c in content if c.get("type") == "image"), None)
    ok_img = False; dims = None
    if img:
        raw = base64.b64decode(img["source"]["data"])
        from PIL import Image as PI
        dims = PI.open(io.BytesIO(raw)).size
        ok_img = raw[:2] == b"\xff\xd8" and img["source"]["media_type"] == "image/jpeg" and dims == (768, 432)
        kb = len(raw) / 1024
    check("frame sent as 768x432 JPEG, image before text", ok_img and content[0]["type"] == "image" and content[-1]["type"] == "text", f"{dims}, {kb:.0f} kB" if img else "no image")
    check("system prompt is static text; goal travels in the user turn", isinstance(b.get("system"), str) and "Follow the pipe." in content[-1]["text"] and "Follow the pipe." not in b["system"])
    check("history reaches the model on later steps", "most recent actions" in requests[2]["body"]["messages"][0]["content"][-1]["text"])
    moving = [(x, r) for _, x, r in cmds if abs(x) > 1e-6 or abs(r) > 1e-6]
    kinds = sorted(set((round(x, 2), round(r, 2)) for x, r in moving))
    check("only the two valid actions ever moved the vehicle, both clamped", kinds == [(0.0, -0.5), (0.25, 0.1)], kinds)
    n1 = sum(1 for x, r in moving if round(x, 2) == 0.25); n2 = sum(1 for x, r in moving if round(r, 2) == -0.5)
    check("each held about as long as asked (0.6 s and 0.5 s at 10 Hz)", 4 <= n1 <= 8 and 3 <= n2 <= 7, f"{n1} and {n2} messages")
    check("ends on zeros", len(cmds) > 5 and all(abs(x) < 1e-6 and abs(r) < 1e-6 for _, x, r in cmds[-5:]))
    steps = [json.loads(l) for l in open(os.path.join(out, "steps.jsonl"))]
    check("step log: ok, error, ok, error, error, ok", [s["status"] for s in steps] == ["ok", "error", "ok", "error", "error", "ok"], [s.get("error", s["status"]) for s in steps])
    check("a frame saved per step", len([f for f in os.listdir(out) if f.endswith(".jpg")]) == 6)
    print("\nALL PASS" if all(res) else "\nFAILURES"); print("--- pilot log ---"); print("\n".join(l.split("] ", 1)[-1] for l in log.splitlines()[-12:]))
    shutil.rmtree(out, ignore_errors=True); n.destroy_node(); rclpy.shutdown(); srv.shutdown(); sys.exit(0 if all(res) else 1)


if __name__ == "__main__":
    main()
