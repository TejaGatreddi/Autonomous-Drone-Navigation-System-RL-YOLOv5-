#!/usr/bin/env python3
"""ROS node that subscribes to a camera topic, runs YOLO wrapper, and publishes detections.
This is written for rospy (ROS1). Adjust for ROS2 as needed.
"""
import argparse
import json
import sys
try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
except Exception:
    rospy = None  # When ROS is not installed

from detection.yolo_wrapper import YOLOWrapper
import cv2
import numpy as np

class PerceptionNode:
    def __init__(self, weights="yolov8n.pt", camera_topic="/camera/rgb/image_raw", out_topic="/perception/detections"):
        if rospy is None:
            raise RuntimeError("rospy not available. Install ROS and rospy to run this node.")
        self.bridge = CvBridge()
        self.model = YOLOWrapper(weights=weights)
        self.pub = rospy.Publisher(out_topic, String, queue_size=1)
        rospy.Subscriber(camera_topic, Image, self._cb)
        rospy.loginfo("Perception node started..")

    def _cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        dets = self.model.detect(cv_img)
        self.pub.publish(json.dumps(dets))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default="yolov8n.pt")
    parser.add_argument("--camera-topic", default="/camera/rgb/image_raw")
    parser.add_argument("--out-topic", default="/perception/detections")
    args = parser.parse_args()
    if rospy is None:
        print("ROS not available on this system.")
        sys.exit(1)
    rospy.init_node("autonav_perception")
    pn = PerceptionNode(weights=args.weights, camera_topic=args.camera_topic, out_topic=args.out_topic)
    rospy.spin()

if __name__ == "__main__":
    main()
