// C++ Standard Library
#include <iostream>
#include <sstream>  //字串切割
#include <fstream>  //檔案的輸入輸出
#include <math.h>
#include <cmath>
#include <vector>

// ROS Core
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h> 


double k;
std::vector<std::string> pose_vector;
std::vector<double> pose_array ;
std::vector<double> pose_to_localpath;
std_msgs::Float32 path_length_msg;


int main (int argc, char** argv)
{
    ros::init(argc, argv,"amcl_global_path");
    ROS_INFO("~~~~~global path start~~~~");
    ros::NodeHandle nh;
    //Publisher
    // 1. 發布路徑座標陣列 (給控制節點用，如 MPC)
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("array_topic",100);    //to local_path  //to change
    
    // 2. 發布路徑總長度 (給其他監控節點用)
    ros::Publisher path_length_pub = nh.advertise<std_msgs::Float32>("global_path_length", 1);

    // 3. 發布 RViz 視覺化標記 (給人類看)
    ros::Publisher cached_waypoint_show_pub_ = nh.advertise<visualization_msgs::MarkerArray>("global_path",1000);

    std::string line;

    //開啟csv 檔案路徑
    //std::ifstream inFile("/home/king/mpc/turn0106.csv",std::ios::in);
    //std::ifstream inFile("/home/cyc/golf_ws/mpc/0103_straight.csv",std::ios::in); 
    //std::ifstream inFile("/home/golfking/mpc/test3.csv",std::ios::in); 
    //std::ifstream inFile("/home/king/mpc/1212.csv",std::ios::in);
    //std::ifstream inFile("/home/golfking/mpc/456.csv",std::ios::in);
    //std::ifstream inFile("/home/golfking/mpc/turn0106_3.csv",std::ios::in);  
    //std::ifstream inFile("/home/golfking/mpc/test0116.csv",std::ios::in); 
    //std::ifstream inFile("/home/cyc/golf_ws/mpc/happynewyear.csv",std::ios::in); 
    //std::ifstream inFile("/home/cyc/golf_ws/mpc/test0120.csv",std::ios::in); 
    //std::ifstream inFile("/home/king/golf_ws/mpc/long_test0120_3.csv",std::ios::in); 
    //std::ifstream inFile("/home/cyc/golf_ws/lego/path_data.csv",std::ios::in);
    //std::ifstream inFile("/home/cyc/golf_ws/mpc/sadyear_3.csv",std::ios::in); 
    //std::ifstream inFile("/home/king/golf_ws/mpc/human.csv",std::ios::in); 
    //std::ifstream inFile("/home/cyc/golf_ws/mpc/straight.csv",std::ios::in); 
    //std::ifstream inFile("/home/king/golf_ws/mpc/turn0410.csv",std::ios::in); 
    //std::ifstream inFile("/home/king/golf_ws/mpc/path_points.csv",std::ios::in); 
    //std::ifstream inFile("/home/cyc/path_points.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_XY_02m_high_curvature.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_XY_05m_high_curvature1.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_XY_05m_high_curvature2.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_XY_05m.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_XY_05m_v2.csv",std::ios::in);
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_Selective_Smooth.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/Final_Path_Selective_Smooth_high_curvature2.csv",std::ios::in); 
    // std::ifstream inFile("/home/gihsiu0530/path_mpc_simulate/smoothed/20260123.csv",std::ios::in); 
    std::ifstream inFile("/home/cyc/campus_ws/src/mpc_4state/src/transformed_points.csv",std::ios::in); 
    // std::ifstream inFile("/home/cyc/campus_ws/path/gym_clockwise.csv",std::ios::in); 

    if(inFile.fail()){
        ROS_ERROR("Failed to read waypoint.csv ");
        return 1;
    }
    
    //讀檔
    while(getline(inFile,line))                        //讀取每一行，存進line，格式string。第一行x,y，第二行-0.080,0.139，...。
    {      
        std::istringstream cut(line);                  //切割字串
        std::string num;
        while(getline(cut,num,','))                    //切割後的字串存到num，格式string
        {
            pose_vector.push_back(num);                //數據存進vector<string> pose_vector
        }    
    }


    for(int m=0; m<pose_vector.size()-2; m++){        //減去csv第一行x,y後，將數值存至pose_array[]
        k = std::stod(pose_vector[m+2]);              //格式string轉成double
        pose_array.push_back(k);                      //數據存至<vector>pose_array
    }


    //for(size_t i = 0 ;i<pose_array.size() ;i+=3)
    
    for(size_t i = 0 ;i<pose_array.size() ;i+=2)
    {
        //std::cout << " transform "  << i << std::endl;

        pose_to_localpath.push_back(pose_array[i]);
        pose_to_localpath.push_back(pose_array[i+1]);
        //pose_to_localpath.push_back(pose_array[i+2]);

    }
    
    /*
    for(int i = pose_array.size()-1 ;i>-1 ;i-=2)
    {
        //std::cout << " transform "  << i << std::endl;

        pose_to_localpath.push_back(pose_array[i-1]);
        pose_to_localpath.push_back(pose_array[i]);
        //pose_to_localpath.push_back(pose_array[i+2]);

    }
     */

     
    //發布全域路徑,格式Float64MultiArray
    std_msgs::Float64MultiArray pose_msg;
    pose_msg.data = pose_to_localpath;
    int size = pose_to_localpath.size();

    
    ROS_INFO("x,y coordinate total array_size is %d ",size);
    ROS_INFO("x coordinate array_size is %d ",size/2);
    ROS_INFO("Global path publish start... ");

    //可視化
    visualization_msgs::MarkerArray global_path_plot_msg;
    visualization_msgs::Marker global_path_plot_marker;
    global_path_plot_marker.header.frame_id = "map";
    global_path_plot_marker.ns = "cached_waypoint_marker";
    global_path_plot_marker.type = visualization_msgs::Marker::LINE_STRIP;
    global_path_plot_marker.action = visualization_msgs::Marker::ADD;
    global_path_plot_marker.scale.x = 0.1;
    global_path_plot_marker.scale.y = 0.1;
    global_path_plot_marker.scale.z = 0.1;
    global_path_plot_marker.color.a = 0.5;
    global_path_plot_marker.color.r = 1;
    global_path_plot_marker.color.g = 0;
    global_path_plot_marker.color.b = 0;

    global_path_plot_msg.markers.clear();
    geometry_msgs::Point caxhed_point;

    for(size_t i =0; i<pose_to_localpath.size();i+=2){
       caxhed_point.x = pose_to_localpath[i];
       caxhed_point.y = pose_to_localpath[i+1];  
       global_path_plot_marker.points.push_back(caxhed_point); 
    }

    global_path_plot_msg.markers.push_back(global_path_plot_marker);

    double path_length = 0.0;
    for(size_t i = 2; i < pose_to_localpath.size(); i += 2) 
    {
        double x0 = pose_to_localpath[i-2];
        double y0 = pose_to_localpath[i-1];
        double x1 = pose_to_localpath[i];
        double y1 = pose_to_localpath[i+1];
        double dx = x1 - x0;
        double dy = y1 - y0;
        path_length += std::hypot(dx, dy);// std::hypot(dx, dy) 等同於 sqrt(dx*dx + dy*dy)
    }
    ROS_INFO("Global path length: %.3f m", path_length);
    path_length_msg.data = path_length;


    while(ros::ok())                //持續發布 
    {    
        pub.publish(pose_msg);                                     //發布pose_msg,格式std_msgs::Float64MultiArray
        path_length_pub.publish(path_length_msg);
        cached_waypoint_show_pub_.publish(global_path_plot_msg);   //發布可視化
        sleep(1);                    //等待1秒
    }

    ROS_INFO("publish_end!");
    return 0;
}


//---------------程式說明---------------
//讀檔： 全域路徑csv檔
//      包含x,y
//
//計算：儲存成vector -> Float64MultiArray
//     儲存成Lane
//
//發布：全域路徑Float64MultiArray，給local_path, change
//     全域路徑Lane，給lane_master
//
//可視化：全域路徑

