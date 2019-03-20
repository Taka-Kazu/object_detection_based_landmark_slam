#include <vector>
#include <random>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include "landmark_slam_msgs/LandmarkArray.h"

std::mt19937 mt{std::random_device{}()};
std::uniform_real_distribution<> dist(-1.0, 1.0);

class ObservationSimulator
{
public:
    ObservationSimulator(void);
    void process(void);
    void landmark_callback(const visualization_msgs::MarkerArrayConstPtr&);
    void landmark_label_callback(const visualization_msgs::MarkerArrayConstPtr&);

private:
    void set_marker_pose_xyz(visualization_msgs::Marker&, double, double, double);
    void set_marker_pose_rpy(visualization_msgs::Marker&, double, double, double);
    void set_marker_scale_xyz(visualization_msgs::Marker&, double, double, double);
    void set_marker_color_rgba(visualization_msgs::Marker&, double, double, double, double);
    void add_noise(geometry_msgs::Pose&);

    ros::NodeHandle nh;
    ros::Publisher landmark_pub;
    ros::Subscriber landmark_sub;
    ros::Subscriber landmark_label_sub;
    visualization_msgs::MarkerArray landmarks;
    visualization_msgs::MarkerArray landmark_labels;
    landmark_slam_msgs::LandmarkArray lm;
    bool landmark_updated;
    bool landmark_label_updated;
    tf::TransformListener listener;
    std::vector<double> R = {0.2*0.2, (M_PI/18.0)*(M_PI/18.0)};// input noise
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "observation_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    ObservationSimulator observation_simulator;
    observation_simulator.process();
}

ObservationSimulator::ObservationSimulator(void)
{
    landmark_pub = nh.advertise<landmark_slam_msgs::LandmarkArray>("/observation/sim", 1);
    landmark_sub = nh.subscribe("/landmark/sim", 1, &ObservationSimulator::landmark_callback, this);
    landmark_label_sub = nh.subscribe("/landmark/label/sim", 1, &ObservationSimulator::landmark_label_callback, this);
    landmark_updated = false;
    landmark_label_updated = false;
}

void ObservationSimulator::process(void)
{
    ros::Rate loop_rate(10);

    std::cout << "observation simulator" << std::endl;

    while(ros::ok()){
        lm.landmarks.clear();
        if(landmark_updated && landmark_label_updated){
            try{
                tf::StampedTransform transform;
                listener.lookupTransform("world", "base_link_truth", ros::Time(0), transform);
                geometry_msgs::PoseStamped pose;
                tf::poseTFToMsg(transform, pose.pose);
                pose.header.frame_id = transform.frame_id_;
                pose.header.stamp = transform.stamp_;
                for(auto it=landmarks.markers.begin();it!=landmarks.markers.end();++it){
                    geometry_msgs::PoseStamped pt_in;
                    geometry_msgs::PoseStamped pt_out;
                    pt_in.pose = it->pose;
                    pt_out.header = pt_in.header = it->header;
                    pt_out.header.frame_id = "base_link_truth";
                    listener.transformPose(pt_out.header.frame_id, pt_in, pt_out);
                    landmark_slam_msgs::Landmark landmark;
                    landmark.pose = pt_out;
                    add_noise(landmark.pose.pose);
                    landmark.header = pt_out.header;
                    landmark.label = landmark_labels.markers[it - landmarks.markers.begin()].text;
                    lm.landmarks.push_back(landmark);
                }
            }catch(tf::TransformException ex){
                ROS_ERROR("%s\n", ex.what());
            }
            landmark_updated = false;
            landmark_label_updated = false;
        }
        lm.header.stamp = ros::Time::now();
        landmark_pub.publish(lm);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ObservationSimulator::set_marker_pose_xyz(visualization_msgs::Marker& marker, double x, double y, double z)
{
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
}

void ObservationSimulator::set_marker_pose_rpy(visualization_msgs::Marker& marker, double r, double p, double y)
{
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
}

void ObservationSimulator::set_marker_scale_xyz(visualization_msgs::Marker& marker, double x, double y, double z)
{
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
}

void ObservationSimulator::set_marker_color_rgba(visualization_msgs::Marker& marker, double r, double g, double b, double a)
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

void ObservationSimulator::landmark_callback(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    landmarks = *msg;
    landmark_updated = true;
}

void ObservationSimulator::landmark_label_callback(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    landmark_labels = *msg;
    landmark_label_updated = true;
}

void ObservationSimulator::add_noise(geometry_msgs::Pose& pose)
{
    double distance = pose.position.x * pose.position.x + pose.position.y * pose.position.y;
    distance = sqrt(distance) + dist(mt) * R[0];
    double angle = atan2(pose.position.y, pose.position.x) + dist(mt) * R[1];
    pose.position.x = distance * cos(angle);
    pose.position.y = distance * sin(angle);
    pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}
