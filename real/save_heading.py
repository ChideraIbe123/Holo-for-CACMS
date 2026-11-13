#!/usr/bin/env python3
"""Save the heading (theta = 0) that course_runner --init turns to before every run.
One heading is shared by both courses (rect and line).

Point the ROV at the back wall of the pool (the direction the course's first leg goes),
hold it still, and run:
    python3 save_heading.py
It averages the estimator's heading (/deadreckon/odom, the same IMU heading the runner
steers on) for --seconds, refuses if the heading moved more than --max-spread degrees
while sampling, and writes it into heading_ref.json next to this script. Re-run it
whenever the start position or pool layout changes.
"""
import argparse
import json
import math
import os
import sys
import time


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--course', choices=['rect', 'line'], default=None,
                    help='ignored: both courses share one heading (kept so old commands still work)')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--max-spread', type=float, default=3.0, help='degrees')
    ap.add_argument('--file', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'heading_ref.json'))
    a = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('save_heading')
    yaws = []
    node.create_subscription(Odometry, '/deadreckon/odom',
                             lambda m: yaws.append(yaw_from_quat(m.pose.pose.orientation)), 50)
    # the window starts at the first message: connecting to the topic takes a second or two,
    # which used to eat most of a window started at launch (only 4 headings in 3 s)
    t_give_up = time.monotonic() + 10.0
    t_end = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if t_end is None:
            if yaws:
                t_end = now + a.seconds
            elif now > t_give_up:
                break
        elif now >= t_end:
            break
    node.destroy_node()
    rclpy.shutdown()
    if len(yaws) < 5:
        sys.exit(f'only {len(yaws)} headings received on /deadreckon/odom: is dead reckoning running?')

    mean = math.atan2(sum(math.sin(v) for v in yaws), sum(math.cos(v) for v in yaws))
    spread = max(abs(math.atan2(math.sin(v - mean), math.cos(v - mean))) for v in yaws)
    if math.degrees(spread) > a.max_spread:
        sys.exit(f'heading moved {math.degrees(spread):.1f} deg while sampling (limit {a.max_spread:.1f}): '
                 'hold the ROV still and try again. Nothing saved.')

    ref = {}
    if os.path.exists(a.file):
        with open(a.file) as f:
            ref = json.load(f)
    old = ref.get('heading')
    ref = {'heading': dict(yaw_rad=mean, yaw_deg=math.degrees(mean), spread_deg=math.degrees(spread),
                         samples=len(yaws), saved=time.strftime('%Y-%m-%d %H:%M:%S'))}
    with open(a.file, 'w') as f:
        json.dump(ref, f, indent=1)
    print(f'saved heading {math.degrees(mean):+.1f} deg (used by rect and line) '
          f'(spread {math.degrees(spread):.1f} deg, {len(yaws)} samples) -> {a.file}')
    if old:
        d = math.degrees(math.atan2(math.sin(mean - old['yaw_rad']), math.cos(mean - old['yaw_rad'])))
        print(f'previous heading was {old["yaw_deg"]:+.1f} deg (saved {old["saved"]}): changed by {d:+.1f} deg')


if __name__ == '__main__':
    main()
