#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>  // 引入 Bool 訊息類型
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <fstream>
#include <vector>

#define PI 3.14159265359

// 全局變數
bool stop_signal_received = false;  // 用於標記是否接收到停止訊號
double v = 0.0000001, delta = 0.0, a = 0.0; // 初始速度和方向角
double L = 1.66, dt = 0.1;
double mirror_theta = 0.0; // 用於記錄轉彎前的角度
double omega_back = 0;

double x, y, theta, mirror_x, mirror_y;
bool got_initial_pose = false;
bool mirror_started = false;
bool back = false;

int turn_index_ = 0; // 用於存儲轉彎點的索引
int start_id = 0; // 用於存儲起始點的索引
std::vector<double> path_coords;
double turn_x = 0.0, turn_y = 0.0;   // 第 turn_index_ 點的座標

// 回調函數更新速度和方向角
void mpcResultCallback(const std_msgs::Float64MultiArray::ConstPtr& mpc_result_data) {
    if (!mpc_result_data->data.empty()) {
        //v = mpc_result_data->data[0];
        a = mpc_result_data->data[0];
        delta = mpc_result_data->data[1];
        ROS_INFO("Received MPC result - a: %f,- delta: %f",a,delta);
    } else {
        ROS_WARN("Empty MPC result received!");
    }
}

// 停止訊號的回調函數
void stopSignalCallback(const std_msgs::Bool::ConstPtr& stop_msg) {
    stop_signal_received = stop_msg->data;
    if (stop_signal_received) {
        ROS_INFO("Stop signal received. Stopping the simulation.");
    }
}

void initialPoseCallback(const std_msgs::Float64MultiArray::ConstPtr& arr) {
  if (!got_initial_pose && arr->data.size() >= 4) {
    path_coords = arr->data;
    // 拿第一個點
    x = arr->data[0];
    y = arr->data[1];
    // x = arr->data[2];
    // y = arr->data[3];
    // 用第二個點計算朝向
    double x1 = arr->data[2];
    double y1 = arr->data[3];
    theta = std::atan2(y1 - y, x1 - x);
    got_initial_pose = true;
    ROS_INFO("Initial pose from array: x=%.3f, y=%.3f, theta=%.3f", x, y, theta);
  }
}

void turnIndexCallback(const std_msgs::Int32::ConstPtr &msg)
{
        turn_index_ = msg->data;
        ROS_INFO("turn point:%d", turn_index_);
        // 把第 turn_index_ 點的座標抓出來
}

void startidCallback(const std_msgs::Int32::ConstPtr &msg)
{
        start_id = msg->data;
        ROS_INFO("start_id :%d", start_id);
}

geometry_msgs::PoseStamped mirror_pose;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mpc_simulate");
    ros::NodeHandle nh;

    ros::Publisher curent_pose_to_mpc_pub = nh.advertise<nav_msgs::Odometry>("/mpc_new_pose", 10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/vehicle_path", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/vehicle_marker", 10);
    ros::Publisher mirror_path_pub = nh.advertise<nav_msgs::Path>("/mirror_vehicle_path", 10);
    ros::Publisher mirror_marker_pub = nh.advertise<visualization_msgs::Marker>("/mirror_vehicle_marker", 10);

    ros::Subscriber mpc_result_sub = nh.subscribe("/mpc_result_test", 10, mpcResultCallback);
    ros::Subscriber stop_signal_sub = nh.subscribe("/stop_signal", 10, stopSignalCallback);  // 訂閱停止訊號
    ros::Subscriber init_sub = nh.subscribe("array_topic",    10,  initialPoseCallback);
    ros::Subscriber turn_sub = nh.subscribe("turn_index", 10, turnIndexCallback);
    ros::Subscriber start_id_sub = nh.subscribe("/start_id", 10, startidCallback);

    std::ofstream csv_file("/home/gihsiu0530/mpc/mirror_positions.csv");
    if (!csv_file.is_open()) {
        ROS_ERROR("無法打開 mirror_positions.csv 進行寫入！");
        return 1;
    }

    csv_file << "x,y\n";  // 只輸出 x,y

    nav_msgs::Path path;
    nav_msgs::Path mirror_path;
    path.header.frame_id = "map";

    ros::Rate loop_rate(10);  // 10 Hz

    ROS_INFO("Waiting for first message on /array_topic ");
    auto init_msg = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/array_topic", nh);
    if (!init_msg || init_msg->data.size() < 4) {
        ROS_FATAL("Failed to get initial pose (need at least 4 values)");
        return 1;
    }
    // 解析初始 x, y, theta
    x = init_msg->data[2];
    y = init_msg->data[3];
    {
        double x1 = init_msg->data[4];
        double y1 = init_msg->data[5];
        theta = std::atan2(y1 - y, x1 - x);
        //theta = std::atan2(y1 - y, x1 - x)+ M_PI;
    }
    ROS_INFO("Initial pose set: x=%.3f, y=%.3f, theta=%.3f", x, y, theta);


    while (ros::ok() && !stop_signal_received) 
    {
        ros::spinOnce();

        ROS_INFO("v: %.3f  a: %.3f  delta: %.3f", v, a, delta);

        // 計算車輛運動
        v += a * dt;
        double omega = v * tan(delta) / L;
        theta += omega * dt;

        // if(start_id == turn_index_)
        // {
        //     mirror_theta = theta;
        // }


        if (theta > PI) theta -= 2 * PI;
        if (theta < -PI) theta += 2 * PI;

        double v_x = v * cos(theta);
        double v_y = v * sin(theta);
        x += v_x * dt;
        y += v_y * dt;



        // 發佈車輛位置
        nav_msgs::Odometry msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.orientation.w = cos(theta / 2.0);
        msg.pose.pose.orientation.z = sin(theta / 2.0);
        msg.twist.twist.linear.x = v_x;
        msg.twist.twist.linear.y = v_y;
        msg.twist.twist.angular.z = omega;

        curent_pose_to_mpc_pub.publish(msg);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp    = ros::Time::now();
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";              // 或是你的 base_link/frame_id
        marker.header.stamp = ros::Time::now();
        marker.ns = "vehicle_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker.mesh_resource = "file:///home/gihsiu0530/golf.stl";
        marker.mesh_use_embedded_materials = true;

        //int turn_index_ = 100000;//43 //112 //65 // 61 //47 //33
        turn_x = path_coords[2*turn_index_];
        turn_y = path_coords[2*turn_index_ + 1];
        double dist_to_turn = std::hypot(x - turn_x, y - turn_y);

        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        marker.pose.position.x = x;
        marker.pose.position.y = y;

        double mx = marker.pose.position.x;
        double my = marker.pose.position.y;
        csv_file << mx << "," << my << "\n";

        // orientation 都一樣
        pose_stamped.pose.orientation.w = cos(theta/2.0);
        pose_stamped.pose.orientation.z = sin(theta/2.0);
        //marker.pose.orientation.w       = cos(theta/2.0);
        //marker.pose.orientation.z       = sin(theta/2.0);
        tf2::Quaternion q;
        q.setRPY(0, 0, theta + M_PI/2.0 );
        //q.setRPY(0, 0, theta + M_PI/2.0+ M_PI );
        marker.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose_stamped);
        path_pub.publish(path);
        marker_pub.publish(marker);

        // 顯示資訊
        ROS_INFO("x: %f, y: %f, theta: %f", x, y, theta);
        
        loop_rate.sleep();
    }

    csv_file.close();
    ROS_INFO("Simulation ended.");
    return 0;
}

