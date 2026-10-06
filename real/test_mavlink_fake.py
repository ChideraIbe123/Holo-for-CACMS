#!/usr/bin/env python3
"""Hardware-free test of cmdvel_to_manual.py's MAVLink path.
A fake ArduSub (heartbeats, obeys arm/disarm, records every MANUAL_CONTROL frame) listens on
UDP; the real converter node connects to it; a scripted controller publishes /cmd_vel and a
scripted estimator publishes /deadreckon/odom. Checks: gains, signs, limits, timeout to neutral,
geofence latch, disarm on exit.
Usage: ROS_DOMAIN_ID=74 python3 test_mavlink_fake.py"""
import math, os, signal, subprocess, sys, threading, time
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pymavlink import mavutil

PORT = 14599
HERE = os.path.dirname(os.path.abspath(__file__))
frames, cmds, stop = [], [], threading.Event()
state = {'armed': False}


def fake_vehicle():
    v = mavutil.mavlink_connection(f'udpout:127.0.0.1:{PORT}', source_system=1, source_component=1)
    m = mavutil.mavlink
    last = 0
    while not stop.is_set():
        if time.time() - last > 0.5:
            base = m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (m.MAV_MODE_FLAG_SAFETY_ARMED if state['armed'] else 0)
            v.mav.heartbeat_send(m.MAV_TYPE_SUBMARINE, m.MAV_AUTOPILOT_ARDUPILOTMEGA, base, 19, 0)  # 19 = MANUAL
            last = time.time()
        msg = v.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue
        if msg.get_type() == 'MANUAL_CONTROL':
            frames.append((time.time(), msg.x, msg.y, msg.z, msg.r))
        elif msg.get_type() == 'COMMAND_LONG' and msg.command == m.MAV_CMD_COMPONENT_ARM_DISARM:
            state['armed'] = bool(msg.param1)
            cmds.append((time.time(), 'ARM' if msg.param1 else 'DISARM'))


def window(t0, t1):
    return [f for f in frames if t0 <= f[0] <= t1]


def main():
    threading.Thread(target=fake_vehicle, daemon=True).start()
    time.sleep(1.0)
    node_proc = subprocess.Popen([sys.executable, os.path.join(HERE, 'cmdvel_to_manual.py'),
                                  '--connect', f'udpin:127.0.0.1:{PORT}', '--depth-mode', 'passthrough', '--no-keys'],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    n = rclpy.create_node('fake_inputs')
    cpub = n.create_publisher(TwistStamped, '/cmd_vel', 10)
    opub = n.create_publisher(Odometry, '/deadreckon/odom', 10)
    yaw0 = math.radians(-69)

    def odom(x, y):          # course-frame point -> estimator frame rotated by yaw0
        o = Odometry()
        o.pose.pose.position.x = 3.0 + math.cos(yaw0) * x - math.sin(yaw0) * y
        o.pose.pose.position.y = -1.0 + math.sin(yaw0) * x + math.cos(yaw0) * y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
        opub.publish(o)

    def drive(u, w, r_cw, secs, xy):
        t_end = time.time() + secs
        while time.time() < t_end:
            c = TwistStamped(); c.twist.linear.x, c.twist.linear.z, c.twist.angular.z = u, w, r_cw
            cpub.publish(c); odom(*xy); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.05)

    def idle(secs, xy):
        t_end = time.time() + secs
        while time.time() < t_end:
            odom(*xy); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.05)

    idle(3.0, (0, 0))                                   # link comes up, neutral frames
    t = {}
    t['a0'] = time.time(); drive(0.3, 0.0, 0.0, 2.0, (0.2, 0)); t['a1'] = time.time()      # forward 0.3 m/s
    t['b0'] = time.time(); drive(0.3, 0.05, 0.5, 2.0, (0.5, 0)); t['b1'] = time.time()     # + clockwise 0.5 rad/s, up 0.05
    t['c0'] = time.time(); drive(2.0, 0.0, -5.0, 2.0, (0.8, 0)); t['c1'] = time.time()     # absurd command -> limits
    t['d0'] = time.time(); idle(2.5, (0.8, 0)); t['d1'] = time.time()                      # controller silent -> neutral
    t['e0'] = time.time(); drive(0.3, 0.0, 0.0, 1.0, (1.0, 0)); t['e1'] = time.time()      # new run, inside fence
    t['f0'] = time.time(); drive(0.3, 0.0, 0.0, 2.0, (1.0, 1.4)); t['f1'] = time.time()    # estimator leaves the fence
    node_proc.send_signal(signal.SIGINT); time.sleep(2.0); stop.set()
    out = node_proc.communicate(timeout=5)[0]

    def last(w):   # settled frame near the end of a window
        fr = window(w[0] + 1.2, w[1])
        return fr[-1][1:] if fr else None
    res = []
    def check(name, ok, detail):
        res.append(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
    print(f"frames received: {len(frames)}  ({len(frames)/(frames[-1][0]-frames[0][0]):.1f} Hz)   commands: {[c[1] for c in cmds]}")
    a = last((t['a0'], t['a1'])); check('forward 0.3 m/s -> x=450, z=500, r=0', a == (450, 0, 500, 0), a)
    b = last((t['b0'], t['b1'])); check('cw 0.5 rad/s -> r=+200; up 0.05 -> z=582 or 583', b[0] == 450 and b[3] == 200 and b[2] in (582, 583), b)
    c = last((t['c0'], t['c1'])); check('limits: x<=600, r>=-400', c[0] == 600 and c[3] == -400, c)
    d = window(t['d0'] + 1.0, t['d1']); check('controller silent -> neutral, still sending', len(d) > 10 and all(f[1:] == (0, 0, 500, 0) for f in d), f'{len(d)} frames, last {d[-1][1:] if d else None}')
    e = window(t['e0'] + 0.5, t['e1']); check('next run resumes', len(e) > 5 and e[-1][1] > 300, e[-1][1:] if e else None)
    f = window(t['f0'] + 0.5, t['f1']); check('geofence exit -> neutral while controller still commands', len(f) > 10 and all(g[1:] == (0, 0, 500, 0) for g in f), f'{len(f)} frames, last {f[-1][1:] if f else None}')
    check('fence message logged', 'STOP LATCHED: geofence' in out, [l for l in out.splitlines() if 'LATCHED' in l][:1])
    check('DISARM sent on exit', cmds and cmds[-1][1] == 'DISARM', cmds[-1] if cmds else None)
    check('vehicle state read back (mode MANUAL)', 'mode MANUAL' in out, [l.split('] ')[-1] for l in out.splitlines() if 'vehicle:' in l][:1])
    print('\nALL PASS' if all(res) else '\nFAILURES'); print('--- converter log tail ---'); print('\n'.join(out.splitlines()[-8:]))
    n.destroy_node(); rclpy.shutdown(); sys.exit(0 if all(res) else 1)


if __name__ == '__main__':
    main()
