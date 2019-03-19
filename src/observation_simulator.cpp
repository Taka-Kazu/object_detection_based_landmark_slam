#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

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

    ros::NodeHandle nh;
    ros::Publisher landmark_pub;
    ros::Publisher landmark_label_pub;
    ros::Subscriber landmark_sub;
    ros::Subscriber landmark_label_sub;
    visualization_msgs::MarkerArray landmarks;
    visualization_msgs::MarkerArray landmark_labels;
    visualization_msgs::MarkerArray lms;
    visualization_msgs::MarkerArray lm_labels;
    bool landmark_updated;
    bool landmark_label_updated;
    tf::TransformListener listener;
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
    landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("/observation/sim", 1);
    landmark_label_pub = nh.advertise<visualization_msgs::MarkerArray>("/observation/label/sim", 1);
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
        lms.markers.clear();
        lm_labels.markers.clear();
        if(landmark_updated && landmark_label_updated){
            try{
                tf::StampedTransform transform;
                listener.lookupTransform("world", "base_link", ros::Time(0), transform);
                geometry_msgs::PoseStamped pose;
                tf::poseTFToMsg(transform, pose.pose);
                pose.header.frame_id = transform.frame_id_;
                pose.header.stamp = transform.stamp_;
                for(auto it=landmarks.markers.begin();it!=landmarks.markers.end();++it){
                    geometry_msgs::PoseStamped pt_in;
                    geometry_msgs::PoseStamped pt_out;
                    pt_in.pose = it->pose;
                    pt_out.header = pt_in.header = it->header;
                    pt_out.header.frame_id = "base_link";
                    visualization_msgs::Marker marker;
                    marker = *it;
                    marker.pose = pt_out.pose;
                    marker.header = pt_out.header;
                    lms.markers.push_back(marker);
                }
            }catch(tf::TransformException ex){
                ROS_ERROR("%s\n", ex.what());
            }
            landmark_updated = false;
            landmark_label_updated = false;
        }
        landmark_pub.publish(lms);
        landmark_label_pub.publish(lm_labels);
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
