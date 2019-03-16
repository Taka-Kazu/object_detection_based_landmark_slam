#include <vector>
#include <random>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

std::mt19937 mt{std::random_device{}()};
std::uniform_real_distribution<> dist(-1.0, 1.0);

class RobotMoveSimulator
{
public:
    RobotMoveSimulator(void);
    void process(void);

private:
    ros::NodeHandle nh;
    ros::Publisher odom_truth_pub;
    ros::Publisher odom_with_noise_pub;
    nav_msgs::Odometry odom_truth;
    nav_msgs::Odometry odom_with_noise;

    const double DT = 0.05;// [s]
    std::vector<double> Q = {0.5*0.5, 0.5*0.5, (M_PI/6.0)*(M_PI/6.0)};// process noise
    std::vector<double> R = {1.0*1.0, (M_PI/18.0)*(M_PI/18.0)};// input noise
    double VX = 1.0;// [m/s]
    double YAWRATE = 0.1;// [rad/s]
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_move_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    RobotMoveSimulator robot_move_simulator;
    robot_move_simulator.process();
}

RobotMoveSimulator::RobotMoveSimulator(void)
{
    odom_truth_pub = nh.advertise<nav_msgs::Odometry>("/odom/sim/truth", 1);
    odom_with_noise_pub = nh.advertise<nav_msgs::Odometry>("/odom/sim/noise", 1);
    odom_truth.header.frame_id = "world";
    odom_truth.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    odom_with_noise = odom_truth;
}

void RobotMoveSimulator::process(void)
{
    ros::Rate loop_rate(1.0 / DT);

    std::cout << "robot move simulator" << std::endl;

    while(ros::ok()){
        double yaw = tf::getYaw(odom_truth.pose.pose.orientation);
        odom_truth.pose.pose.position.x += VX * cos(yaw) * DT;
        odom_truth.pose.pose.position.y += VX * sin(yaw) * DT;
        odom_truth.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + YAWRATE * DT);
        odom_truth_pub.publish(odom_truth);
        double _vx = VX + dist(mt) * R[0];
        double _yawrate = YAWRATE + dist(mt) * R[1];
        double _yaw = tf::getYaw(odom_with_noise.pose.pose.orientation);
        odom_with_noise.pose.pose.position.x += _vx * cos(_yaw) * DT;
        odom_with_noise.pose.pose.position.y += _vx * sin(_yaw) * DT;
        odom_with_noise.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_yaw + _yawrate * DT);
        odom_with_noise_pub.publish(odom_with_noise);
        ros::spinOnce();
        loop_rate.sleep();

    }
}
