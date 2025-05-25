#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse

class DocumentRectifier:
    def __init__(self):
        rospy.init_node('document_rectifier', anonymous=True)
        
        self.bridge = CvBridge()
        self.cv_image = None
        self.corners = []
        self.is_completed = False
        
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        self.capture_service = rospy.Service('capture_and_rectify', Trigger, self.handle_capture)
        
        rospy.loginfo("Document Rectifier is ready. Call the '/capture_and_rectify' service to capture and process an image.")
    
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))
    
    def handle_capture(self, req):
        if self.cv_image is None:
            return TriggerResponse(False, "No image available")
        
        image = self.cv_image.copy()
        
        corners = self.select_corners_interactive(image)
        if corners is None:
            return TriggerResponse(False, "Failed to select 4 corners")
        
        width, height = 2480, 3508  # A4 at 300 DPI (210x297mm)
        dst_corners = np.array([
            [0, 0],
            [width, 0],
            [width, height],
            [0, height]
        ], dtype=np.float32)
        
        H = cv2.getPerspectiveTransform(corners, dst_corners)
        
        corrected_img = cv2.warpPerspective(image, H, (width, height))
        
        h1, w1 = image.shape[:2]
        h2, w2 = corrected_img.shape[:2]
        
        scale = h1 / h2
        corrected_resized = cv2.resize(corrected_img, (int(w2 * scale), h1))
        
        separator = np.ones((h1, 32, 3), dtype=np.uint8) * 255
        
        combined = np.hstack([image, separator, corrected_resized])
        
        cv2.imwrite('/home/alessandra/catkin_ws/src/Homography/result/document_rectified.png', combined)
        rospy.loginfo("Result saved as 'document_rectified.png'")
        
        return TriggerResponse(True, "Document rectification completed successfully")
    
    def select_corners_interactive(self, image):
        display_image = image.copy()
        corners = []
        
        def mouse_callback(event, x, y, flags, param):
            nonlocal display_image, corners
            
            if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
                corners.append((x, y))
                rospy.loginfo(f"Selected point {len(corners)}: ({x}, {y})")
                
                display_image = image.copy()
                for i, (cx, cy) in enumerate(corners, 1):
                    cv2.circle(display_image, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.putText(display_image, str(i), (cx + 15, cy - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow("Select 4 Corners", display_image)
        
        cv2.namedWindow("Select 4 Corners")
        cv2.setMouseCallback("Select 4 Corners", mouse_callback)
        
        rospy.loginfo("Instructions:")
        rospy.loginfo("1. Click LMB on 4 document corners (clockwise or counter-clockwise)")
        rospy.loginfo("2. Press ENTER when done or ESC to cancel")
        
        cv2.imshow("Select 4 Corners", display_image)
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == 13 and len(corners) == 4:
                break
            elif key == 27:
                corners = []
                break
        
        cv2.destroyAllWindows()
        
        if len(corners) != 4:
            return None
        
        return np.array(corners, dtype=np.float32)

if __name__ == '__main__':
    try:
        rectifier = DocumentRectifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass