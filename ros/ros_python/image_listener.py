
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class PoseImagePublisher {
public:
    PoseImagePublisher() {
        // Initialize the ROS publisher and subscriber
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);
        image_sub_ = nh_.subscribe("image_topic", 10, &PoseImagePublisher::imageCallback, this);

        // Initialize OpenCV window
        cv::namedWindow("Image Window");
    }

    ~PoseImagePublisher() {
        cv::destroyWindow("Image Window");
    }

    void publishPose() {
        ros::Rate rate(1); // 1 Hz

        while (ros::ok()) {
            Eigen::Matrix4d matrix;
            matrix.setIdentity();

            // Populate matrix with some values (for example purposes)
            matrix(0, 3) = 1.0; // x translation
            matrix(1, 3) = 2.0; // y translation
            matrix(2, 3) = 3.0; // z translation

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "world";

            // Convert Eigen::Matrix4d to Pose
            Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);
            Eigen::Vector3d translation_vector = matrix.block<3, 1>(0, 3);

            Eigen::Quaterniond quaternion(rotation_matrix);
            pose_msg.pose.position.x = translation_vector.x();
            pose_msg.pose.position.y = translation_vector.y();
            pose_msg.pose.position.z = translation_vector.z();
            pose_msg.pose.orientation.x = quaternion.x();
            pose_msg.pose.orientation.y = quaternion.y();
            pose_msg.pose.orientation.z = quaternion.z();
            pose_msg.pose.orientation.w = quaternion.w();

            pose_pub_.publish(pose_msg);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::imshow("Image Window", cv_ptr->image);
            cv::waitKey(30);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber image_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_combined");

    PoseImagePublisher pose_image_publisher;
    pose_image_publisher.publishPose();

    return 0;
}
