// #ifdef MPC_PLANNER_H
// #define MPC_PLANNER_H


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

//#include "local_planner.h"
#include <ros/ros.h>
//#include "math_helper.h"
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

    class MPCPlanner_path 
    //class MPCPlanner_path : public nav_core::BaseLocalPlanner,local_planner::LocalPlanner
    {   //ros::NodeHandle nh_;
        //std::string name;
        public:
            //typedef Eigen::Matrix<double,3,3> Matrix3x3;
            void MPCPlanner(ros::NodeHandle *nh);
            void initialize();
            void setPlan(const  std_msgs::Float64MultiArrayConstPtr &msg);
            //void setPlan(const nav_msgs::PathConstPtr &path_msg);
            /////////////////////////////////////////
            void outputProcessing(geometry_msgs::Pose current_pose, int node_num);
	    /////////////////////////////////////////
            void computelocalpath(const nav_msgs::OdometryConstPtr &msg);

            void calculatempc(const std_msgs::Float64MultiArrayConstPtr &msg);

            double getLookAheadDistance(double vt);
            Eigen::Vector3d getEulerAngles(geometry_msgs::PoseStamped & ps); 

            void getLookAheadPoint(double lookahead_dist,geometry_msgs::PoseStamped robot_pose_global,
                                   const std::vector<geometry_msgs::PoseStamped>& prune_plan ,
                                   geometry_msgs::PoseStamped global_pose,
                                   geometry_msgs::PointStamped& pt,
                                   double& theta,double &kappa);
            double dist(const geometry_msgs::PoseStamped& node1,const geometry_msgs::PoseStamped& node2);
            std::vector<std::pair<double,double>> circleSegmentIntersection(const std::pair<double,double>& p1,
                                                                            const std::pair<double,double>& p2,
                                                                            double r); 
            double regularizeAngle(double angle);

            Eigen::Vector2d _mpcControl(Eigen::Vector3d s,Eigen::VectorXd s_d,Eigen::Vector2d u_r,Eigen::Vector2d du_p);
            
            Eigen::SparseMatrix<double> _convertTOSparseMatrix(Eigen::MatrixXd A);

            double linearRegularization(nav_msgs::Odometry& base_odometry , double v_d, int start_id);
            
            double anglarRegularization(nav_msgs::Odometry& base_odometry , double w_d);
            
            double steeringAngleRegularization(nav_msgs::Odometry& base_odometry , double w_d);
            
            Eigen::VectorXd polyfit(const Eigen::VectorXd& x,const Eigen::VectorXd& y,int degree);

            double calculatekappa(const Eigen::VectorXd& coeffs,double x);
         
            double polyeval(Eigen::VectorXd coeffs,double x);
            
            double getDesiredSpeed(int start_id) ;
         
         //MPCPlanner(std::string *name);

         //mpcPlanner(ros::NodeHandle* nh );
         //MPCPlanner(std::string name);

        //void ~MPCPlanner ();
        //void Spin();
        //double get_goal_dist_tol_(){return goal_dist_tol_;}   

        
        //Eigen::MatrixXd Q_;          //狀態誤差權重矩陣

        private:
            
            //lookhead
            double lookahead_time_ = 1.5;     //前視時間
            double min_lookahead_dist_ = 0.5; //最小前視距離
            double max_lookahead_dist_ = 1.0; //最大前視距離
            
            ros::Subscriber global_path_sub;
            ros::Subscriber car_pose_sub;
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            std::string filename_;

            std::vector<double> global_path_x;
            std::vector<double> global_path_y;
            std::vector<double> global_path_v;


            Eigen::Vector3d goal_rpy;

            int start_id = 0 ;
            int last_id = 0;

            std::vector<double> org_wp_rearange_waypoint_x;
            std::vector<double> org_wp_rearange_waypoint_y;

            std_msgs::Float64MultiArray local_path;
            std::vector<double> local_path_x_and_y;

            std::vector<geometry_msgs::PoseStamped> prune_plan;

            std::vector<geometry_msgs::PoseStamped> orig_global_plan;
            geometry_msgs::PoseStamped global_path;

            //Eigen::Matrix<double,3,3> 
            Eigen::Matrix3d Q_;
            bool initialize_ = false;      //判斷是否初始化
            double d_t_;           //控制採樣時間
            Eigen::Matrix2d R_;    //控制誤差權重矩陣
            int p_;                //預測時域
            int m_;                //控制時域
            Eigen::Vector2d du_p_; //先前控制誤差
            ros::Publisher target_pt_pub,current_pose_pub_;
            ros::Publisher local_path_to_matlab_pub;
            ros::Publisher mpc_result_pub;
            ros::Publisher v_pub ;
            ros::Publisher delta_pub ;
            ros::Publisher cte ;
            
            //curve speed limit
            double curve_max_speed_ = 1.5;      // 彎道最大速度
            double curve_threshold_ = 0.03;      // 曲率閾值，超過這個值時認為是彎道

            //linear velocity
            double max_v_ = 2.5;              //最大線速度
            double min_v_ = 0.15;                //最小線速度
            double max_v_inc_ = 0.7;          //線速度差值


            //angular velocity
            double max_w_ = 0.5;             //最大角速度
            double min_w_ = -0.5;            //最小角速度
            double max_w_inc_ = 1;          //角速度差值
            
            //angle 
            double max_delta_ = 0.4;             //最大角度
            double min_delta_ = -0.4;            //最小角度
            //double max_delta_inc_ = 1.046;          //角度差值
            double max_delta_inc_ = 0.05;          //角度差值

            double caculate_mpc_start = 0;
            double caculate_mpc_finish = 0; 

            double px = 0;
            double py = 0;
            double next_px = 0;
            double next_py = 0;
            double theta1 = 0;
            double theta = 0;
            double vx = 0;
            double vy = 0;
            double omega = 0;
            double delta = 0;

            double pwm = 0;
            double b = 1;
            double r = 0.24/2;
            double wl = 0;
            double wr = 0;
            double pwml = 0;
            double pwmr = 0; 


    };
//#endif //MPC_PLANNER_H
