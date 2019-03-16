#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

class LandmarkSimulator
{
public:
    LandmarkSimulator(void);
    void waypoint_callback(const geometry_msgs::PoseArrayConstPtr&);
    void process(void);

private:
    void set_marker_pose_xyz(visualization_msgs::Marker&, double, double, double);
    void set_marker_pose_rpy(visualization_msgs::Marker&, double, double, double);
    void set_marker_scale_xyz(visualization_msgs::Marker&, double, double, double);
    void set_marker_color_rgba(visualization_msgs::Marker&, double, double, double, double);

    ros::NodeHandle nh;
    ros::Publisher landmark_pub;
    ros::Publisher landmark_label_pub;
    visualization_msgs::MarkerArray landmarks;
    visualization_msgs::MarkerArray landmarks_labels;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "landmark_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    LandmarkSimulator landmark_simulator;
    landmark_simulator.process();
}

LandmarkSimulator::LandmarkSimulator(void)
{
    landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmark/sim", 1);
    landmark_label_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmark/label/sim", 1);
}

void LandmarkSimulator::process(void)
{
    ros::Rate loop_rate(10);

    std::cout << "landmark simulator" << std::endl;

    visualization_msgs::Marker landmark;
    visualization_msgs::Marker landmark_label;
    landmark.header.frame_id = "world";
    landmark.header.stamp = ros::Time::now();
    landmark.ns = "landmark";
    landmark.type = visualization_msgs::Marker::SPHERE;
    landmark.action = visualization_msgs::Marker::ADD;
    set_marker_pose_xyz(landmark, 2, 3, 1);
    set_marker_pose_rpy(landmark, 0, 0, 0);
    set_marker_scale_xyz(landmark, 1, 1, 1);
    set_marker_color_rgba(landmark, 1, 0, 0, 1);
    landmark.lifetime = ros::Duration();
    landmarks.markers.push_back(landmark);

    while(ros::ok()){
        landmark_pub.publish(landmarks);
        ros::spinOnce();
        loop_rate.sleep();

    }
}

void LandmarkSimulator::set_marker_pose_xyz(visualization_msgs::Marker& marker, double x, double y, double z)
{
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
}

void LandmarkSimulator::set_marker_pose_rpy(visualization_msgs::Marker& marker, double r, double p, double y)
{
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
}

void LandmarkSimulator::set_marker_scale_xyz(visualization_msgs::Marker& marker, double x, double y, double z)
{
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
}

void LandmarkSimulator::set_marker_color_rgba(visualization_msgs::Marker& marker, double r, double g, double b, double a)
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}
