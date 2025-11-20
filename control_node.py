#!/usr/bin/env python3
"""ROS node that listens to perception detections and runs a simple policy loop.
This node is intentionally conservative and meant as a scaffold only.
"""
import argparse
import json
import sys
try:
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
except Exception:
    rospy = None

class ControlNode:
    def __init__(self, detections_topic="/perception/detections", cmd_topic="/cmd_vel"):
        if rospy is None:
            raise RuntimeError("rospy not available. Install ROS and rospy to run this node.")
        self.sub = rospy.Subscriber(detections_topic, String, self._cb)
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.last_dets = []
        rospy.loginfo("Control node started..")

    def _cb(self, msg):
        try:
            dets = json.loads(msg.data)
        except Exception:
            dets = []
        self.last_dets = dets

    def run_loop(self, rate_hz=10):
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            cmd = Twist()
            # Simple reactive policy: if obstacle detected straight ahead, yaw away; else move forward
            forward = 0.5
            yaw = 0.0
            for d in self.last_dets:
                x1,y1,x2,y2 = d.get("xyxy", [0,0,0,0])
                cx = (x1 + x2) / 2
                # frame width unknown; approximate center-based yaw
                # If detection near center, slow forward and yaw
                # This is simplistic — replace with policy/model output in real system
                if abs(cx - 320) < 80:  # assuming 640x480
                    forward = 0.0
                    yaw = 0.6 if cx < 320 else -0.6
            cmd.linear.x = forward
            cmd.angular.z = yaw
            self.pub.publish(cmd)
            rate.sleep()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dets-topic", default="/perception/detections")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    args = parser.parse_args()
    if rospy is None:
        print("ROS not found on this system.")
        sys.exit(1)
    rospy.init_node("autonav_control")
    cn = ControlNode(detections_topic=args.dets_topic, cmd_topic=args.cmd_topic)
    cn.run_loop()

if __name__ == "__main__":
    main()
