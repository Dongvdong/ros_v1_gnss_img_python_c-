#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
import os

class ImageGPSReader:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_gps_publisher', anonymous=True)
        
        # Initialize publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load GPS and image data
        self.gps_data = self.load_gps_data("/home/dongdong/2project/0data/NWPU/config/gps.txt")
        self.image_files = self.load_image_filenames("/home/dongdong/2project/0data/NWPU/img")

    def load_gps_data(self, gps_file):
        gps_data = {}
        with open(gps_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                timestamp = parts[0]
                lat, lon, alt = map(float, parts[1:])
                gps_data[timestamp] = (lat, lon, alt)
        return gps_data

    def load_image_filenames(self, img_folder):
        image_files = [f for f in os.listdir(img_folder) if f.endswith('.jpg')]
        image_files.sort()  # Ensure consistent ordering
        return image_files

    def publish_data(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        for img_file in self.image_files:
            timestamp = img_file.split('.')[0]+ "."+ img_file.split('.')[1]  # Assuming timestamp is in the filename
            print("timestamp ",timestamp)

            if timestamp in self.gps_data:
                img_path = os.path.join("/home/dongdong/2project/0data/NWPU/img", img_file)
                cv_image = cv2.imread(img_path)

                if cv_image is not None:
                    # Publish image
                    img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                    img_msg.header.stamp = rospy.Time.from_sec(float(timestamp))
                    self.image_pub.publish(img_msg)

                    # Publish GPS data
                    lat, lon, alt = self.gps_data[timestamp]
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = rospy.Time.from_sec(float(timestamp))
                    gps_msg.latitude = lat
                    gps_msg.longitude = lon
                    gps_msg.altitude = alt
                    self.gps_pub.publish(gps_msg)

                    rospy.loginfo(f"Published image: {img_file} and GPS data")

            rate.sleep()  # Sleep to maintain the loop rate


            

if __name__ == '__main__':
    try:
        reader = ImageGPSReader()
        reader.publish_data()
    except rospy.ROSInterruptException:
        pass
