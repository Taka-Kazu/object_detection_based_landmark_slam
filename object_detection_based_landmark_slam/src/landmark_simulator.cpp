#include <vector>

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
    visualization_msgs::MarkerArray landmark_labels;
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

    std::vector<std::vector<double> > landmark_list = {{10.0, -2.0},
                                                      {15.0, 10.0},
                                                      {3.0, 15.0},
                                                      {-5.0, 20.0}};
    /*
    for(auto i=landmark_list.begin();i!=landmark_list.end();++i){
        for(auto j=i->begin();j!=i->end();++j){
            std::cout << *j << ", ";
        }
        std::cout << std::endl;
    }
    */

    std::vector<std::string> landmark_label_list = {"chair", "traffic_light", "chair", "desk"};

    for(auto i=landmark_list.begin();i!=landmark_list.end();++i){
        visualization_msgs::Marker landmark;
        visualization_msgs::Marker landmark_label;
        landmark.header.frame_id = "world";
        landmark.header.stamp = ros::Time::now();
        landmark.ns = "landmark";
        landmark.id = i - landmark_list.begin();
        landmark.type = visualization_msgs::Marker::SPHERE;
        landmark.action = visualization_msgs::Marker::ADD;
        set_marker_pose_xyz(landmark, (*i)[0], (*i)[1], 0);
        set_marker_pose_rpy(landmark, 0, 0, 0);
        set_marker_scale_xyz(landmark, 1, 1, 1);
        set_marker_color_rgba(landmark, 1, 0, 0, 1);
        landmark.lifetime = ros::Duration();
        landmarks.markers.push_back(landmark);

        landmark_label = landmark;
        landmark_label.ns = "landmark_label";
        landmark_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        landmark_label.text = landmark_label_list[landmark_label.id];
        landmark_labels.markers.push_back(landmark_label);
    }

    while(ros::ok()){
        for(auto i=landmarks.markers.begin();i!=landmarks.markers.end();++i){
            int index = i - landmarks.markers.begin();
            landmarks.markers[index].header.stamp = landmark_labels.markers[index].header.stamp= ros::Time::now();
        }
        landmark_pub.publish(landmarks);
        landmark_label_pub.publish(landmark_labels);
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
