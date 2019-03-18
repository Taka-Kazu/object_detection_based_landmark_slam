#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ObservationSimulator
{
public:
    ObservationSimulator(void);
    void process(void);
    void callback(const visualization_msgs::MarkerArrayConstPtr&, const visualization_msgs::MarkerArrayConstPtr&);

private:
    void set_marker_pose_xyz(visualization_msgs::Marker&, double, double, double);
    void set_marker_pose_rpy(visualization_msgs::Marker&, double, double, double);
    void set_marker_scale_xyz(visualization_msgs::Marker&, double, double, double);
    void set_marker_color_rgba(visualization_msgs::Marker&, double, double, double, double);

    ros::NodeHandle nh;
    ros::Publisher landmark_pub;
    ros::Publisher landmark_label_pub;
    message_filters::Subscriber<visualization_msgs::MarkerArray> landmark_sub;
    message_filters::Subscriber<visualization_msgs::MarkerArray> landmark_label_sub;
    typedef message_filters::sync_policies::ApproximateTime<visualization_msgs::MarkerArray, visualization_msgs::MarkerArray> sync_subs;
    message_filters::Synchronizer<sync_subs> sync;
    visualization_msgs::MarkerArray landmarks;
    visualization_msgs::MarkerArray landmark_labels;
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
    :landmark_sub(nh, "/landmark/sim", 1), landmark_label_sub(nh, "/landmark/label/sim", 1),
     sync(sync_subs(30), landmark_sub, landmark_label_sub)
{
    sync.registerCallback(boost::bind(&ObservationSimulator::callback, this, _1, _2));
    landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("/observation/sim", 1);
    landmark_label_pub = nh.advertise<visualization_msgs::MarkerArray>("/observation/label/sim", 1);
}

void ObservationSimulator::process(void)
{
    ros::Rate loop_rate(10);

    std::cout << "observation simulator" << std::endl;

    std::vector<std::vector<double> > landmark_list = {{10.0, -2.0},
                                                      {15.0, 10.0},
                                                      {3.0, 15.0},
                                                      {-5.0, 20.0}};

    while(ros::ok()){
        landmark_pub.publish(landmarks);
        landmark_label_pub.publish(landmark_labels);
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

void ObservationSimulator::callback(const visualization_msgs::MarkerArrayConstPtr& landmark, const visualization_msgs::MarkerArrayConstPtr& label)
{

}
