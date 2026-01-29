#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <Eigen/Dense>

using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;

class EkfFusionNode {
public:
  EkfFusionNode(ros::NodeHandle& nh) : L_(1.66) { // 車軸距，請改成你的實車值
    x_.setZero(4);
    P_ = Mat::Identity(4,4) * 1e-2;

    Q_ = (Mat(4,4) << 1e-4,0,0,0,
                      0,1e-4,0,0,
                      0,0,2e-4,0,
                      0,0,0,5e-4).finished();
    R_lidar_ = (Mat(3,3) << 2e-3,0,0,
                            0,2e-3,0,
                            0,0,5e-4).finished();

    sub_wheel_odom_ = nh.subscribe("/v_real", 50, &EkfFusionNode::wheelOdomCb, this);
    sub_lidar_odom_ = nh.subscribe("/odom", 10, &EkfFusionNode::lidarOdomCb, this);
    sub_imu_ = nh.subscribe("/imu/data", 200, &EkfFusionNode::imuCb, this);

    pub_state_ = nh.advertise<nav_msgs::Odometry>("/state_estimate", 50);
    last_time_ = ros::Time::now();
  }

private:
  void wheelOdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    double v_meas = msg->twist.twist.linear.x;
    double dt = getDtAndUpdate();
    double a = (v_meas - x_(3)) / dt;
    double delta = last_delta_; // 若可讀取轉角，請改成實測值

    predict(a, delta, dt);
    x_(3) = v_meas; // 把編碼器速度強制更新
    publishState(msg->header.stamp);
  }

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_wz_ = msg->angular_velocity.z;
  }

  void lidarOdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 延遲補償，先前推 50ms
    predict(last_a_, last_delta_, 0.05);

    Vec z(3); 
    z << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
    updateLidar(z);
    publishState(msg->header.stamp);
  }

  void predict(double a, double delta, double dt) {
    double x = x_(0), y = x_(1), th = x_(2), v = x_(3);
    double th_next = th + (v/L_)*std::tan(delta)*dt;
    double x_next  = x  + v*std::cos(th)*dt;
    double y_next  = y  + v*std::sin(th)*dt;
    double v_next  = v  + a*dt;

    Mat F = Mat::Identity(4,4);
    F(0,2) = -v*std::sin(th)*dt;
    F(0,3) =  std::cos(th)*dt;
    F(1,2) =  v*std::cos(th)*dt;
    F(1,3) =  std::sin(th)*dt;
    F(2,3) = (1.0/L_)*std::tan(delta)*dt;

    x_ << x_next, y_next, normYaw(th_next), v_next;
    P_  = F*P_*F.transpose() + Q_;
    last_a_ = a; last_delta_ = delta;
  }

  void updateLidar(const Vec& z) {
    Mat H(3,4); H.setZero();
    H(0,0)=1; H(1,1)=1; H(2,2)=1;

    Vec y = z - H*x_;
    y(2) = normYaw(y(2));
    Mat S = H*P_*H.transpose() + R_lidar_;
    Mat K = P_*H.transpose()*S.inverse();
    x_    = x_ + K*y;
    P_    = (Mat::Identity(4,4) - K*H)*P_;
  }

  double getDtAndUpdate() {
    ros::Time t = ros::Time::now();
    double dt = (t - last_time_).toSec();
    dt = std::max(1e-4, std::min(dt, 0.2));
    last_time_ = t;
    return dt;
  }

  void publishState(const ros::Time& stamp) {
    nav_msgs::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";
    out.child_frame_id  = "base_link";
    out.pose.pose.position.x = x_(0);
    out.pose.pose.position.y = x_(1);

    tf::Quaternion q;
    q.setRPY(0,0,x_(2));
    tf::quaternionTFToMsg(q, out.pose.pose.orientation);

    out.twist.twist.linear.x  = x_(3);
    pub_state_.publish(out);
  }

  static double normYaw(double a){
    while(a >  M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
  }

  ros::Subscriber sub_wheel_odom_, sub_lidar_odom_, sub_imu_;
  ros::Publisher pub_state_;
  Vec x_{4}; Mat P_{4,4}, Q_{4,4}, R_lidar_{3,3};
  double L_;
  double last_a_ = 0.0, last_delta_ = 0.0, imu_wz_ = 0.0;
  ros::Time last_time_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ekf_fusion_node");
  ros::NodeHandle nh;
  EkfFusionNode node(nh);
  ros::spin();
  return 0;
}
