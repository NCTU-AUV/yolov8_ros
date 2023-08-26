#!/usr/bin/env python3
import cv2
import torch
import random
import rospy

from cv_bridge import CvBridge

from ultralytics import YOLO
#from ultralytics.tracker import BOTSORT, BYTETracker
#from ultralytics.tracker.trackers.basetrack import BaseTrack
from ultralytics.yolo.utils import IterableSimpleNamespace, yaml_load
from ultralytics.yolo.utils.checks import check_requirements, check_yaml

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool


class Yolov8Node:

    def __init__(self):
        # self.model = YOLO("/home/orca-auv/catkin_ws/src/yolov8_ros/best.pt")
        self.model = YOLO("yolov8n.pt")
        self.model = YOLO("/home/orca-auv/model/best.pt")

        self.model.to("cpu")
        self.threshold = 0.5
        self.cv_bridge = CvBridge()
        # topcis
        self._pub = rospy.Publisher("/detections", Detection2DArray, queue_size=10)
        self._sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_cb)
        print('hello yolo')

        rospy.spin()

    def image_cb(self, msg: Image):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.model.predict(source=img, show=False)

        # create detections msg
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        results = results[0].cpu()

        for b in results.boxes:

            label = self.model.names[int(b.cls)]
            score = float(b.conf)

            if score < self.threshold:
                continue

            detection = Detection2D()

            box = b.xywh[0]

            # get boxes values
            detection.bbox.center.x = float(box[0])
            detection.bbox.center.y = float(box[1])
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])

            # get track id
            # track_id = ""
            # if not b.id is None:
            #     track_id = str(int(b.id))
            # detection.id = track_id

            # get hypothesis
            hypothesis = ObjectHypothesisWithPose()
            # hypothesis.hypothesis.class_id = label
            hypothesis.score = score
            detection.results.append(hypothesis)

            # draw boxes for debug
            # if label not in self._class_to_color:
            #     r = random.randint(0, 255)
            #     g = random.randint(0, 255)
            #     b = random.randint(0, 255)
            #     self._class_to_color[label] = (r, g, b)
            # color = self._class_to_color[label]

            # min_pt = (round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0),
            #             round(detection.bbox.center.position.y - detection.bbox.size_y / 2.0))
            # max_pt = (round(detection.bbox.center.position.x + detection.bbox.size_x / 2.0),
            #             round(detection.bbox.center.position.y + detection.bbox.size_y / 2.0))
            # cv2.rectangle(img, min_pt, max_pt, color, 2)

            # label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
            # pos = (min_pt[0] + 5, min_pt[1] + 25)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # cv2.putText(img, label, pos, font,
            #             1, color, 1, cv2.LINE_AA)

            # append msg
            detections_msg.detections.append(detection)
        #cv2.imshow("res", img)
        #cv2.waitKey(33)

        # publish detections and dbg image
        self._pub.publish(detections_msg)

if __name__ == "__main__":
    try:
        rospy.init_node("yolov8_node")
        node = Yolov8Node()
    except rospy.ROSInternalException:
        exit()

# rospy.init_node('listener', anonymous=True)
# rospy.Subscriber("chatter", String, callback)
# rospy.spin()
