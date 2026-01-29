#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <fstream>
#include <iomanip> 

// ====== 全域變數 ======
double g_v_real = 0.0;      // 實際速度 (m/s)
bool   g_has_last = false;  // 是否已有前一筆資料
double g_last_x = 0.0;
double g_last_y = 0.0;
ros::Time g_last_t;

const double VREAL_MAX = 1.0;  // 實際速度上限 (m/s)
std::ofstream g_csv;           // 寫入 CSV 的全域檔案物件

// ====== odom callback：根據 odom 計算實際速度 ======
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    ros::Time now = msg->header.stamp;

    if (!g_has_last) {
        g_last_x = x;
        g_last_y = y;
        g_last_t = now;
        g_has_last = true;
        return;
    }

    double dt = (now - g_last_t).toSec();
    if (dt <= 0.0) return;

    double dx = x - g_last_x;
    double dy = y - g_last_y;
    double v = std::sqrt(dx*dx + dy*dy) / dt;

    // 限幅處理
    if (v > VREAL_MAX) v = VREAL_MAX;
    if (v < 0.0) v = 0.0;

    g_v_real = v;

    g_last_x = x;
    g_last_y = y;
    g_last_t = now;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_and_vreal_publisher");
    ros::NodeHandle nh;

    // === 開啟 CSV 檔 ===
    g_csv.open("velocity_log.csv");
    if (!g_csv.is_open()) {
        ROS_WARN("Cannot open velocity_log.csv for writing!");
    } else {
        g_csv << "time_sec,planned_vel,v_real\n";
    }

    ros::Publisher mpc_pub   = nh.advertise<std_msgs::Float64>("/mpc_result_test", 10);
    ros::Publisher vreal_pub = nh.advertise<std_msgs::Float64>("/v_real", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 20, odomCallback);

    ros::Rate loop_rate(10);  // 10 Hz
    double planned_vel = 0.0;
    int i = 0;

    while (ros::ok())
    {
        planned_vel = (i < 20) ? 0.6 : 0.3;

        // 發布規劃與實際速度
        std_msgs::Float64 mpc_msg;
        mpc_msg.data = planned_vel;
        mpc_pub.publish(mpc_msg);

        std_msgs::Float64 vreal_msg;
        vreal_msg.data = g_v_real;
        vreal_pub.publish(vreal_msg);

        // 取得 ROS 時間並寫入 CSV
        double t_now = ros::Time::now().toSec();
        if (g_csv.is_open()) {
            g_csv << std::fixed << std::setprecision(6)
                  << t_now << "," << planned_vel << "," << g_v_real << "\n";
        }

        ROS_INFO("t=%.2f | plan=%.3f m/s | real=%.3f m/s", t_now, planned_vel, g_v_real);

        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }

    if (g_csv.is_open()) g_csv.close();
    return 0;
}
