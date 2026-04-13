// #ifdef MPC_PLANNER_H
// #define MPC_PLANNER_H


#include <Eigen/Sparse>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

// #include "local_planner.h"
#include <ros/ros.h>
// #include "math_helper.h"
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>



    class MPCPlanner_path 
    //class MPCPlanner_path : public nav_core::BaseLocalPlanner,local_planner::LocalPlanner
    {   //ros::NodeHandle nh_;
        //std::string name;
        public:
            //typedef Eigen::Matrix<double,3,3> Matrix3x3;
            void MPCPlanner(ros::NodeHandle *nh);
            void initialize();
            /** 新增：訂閱 odom 用的 callback */
            void odomCallback(const nav_msgs::OdometryConstPtr &msg);

            /** 新增：0.1s timer 觸發的控制迴圈 */
            void controlLoop(const ros::TimerEvent &event);
            void setPlan(const  std_msgs::Float64MultiArrayConstPtr &msg);
            void computelocalpath(const nav_msgs::OdometryConstPtr &msg);
            void maxVCallback(const std_msgs::Float64::ConstPtr &msg);
            void maxVIncCallback(const std_msgs::Float64::ConstPtr &msg);
            void hybridPathCallback(const nav_msgs::Path::ConstPtr& msg);
            void turnIndexCallback(const std_msgs::Int32::ConstPtr& msg);
            void turnIndex2Callback(const std_msgs::Int32::ConstPtr &msg);
            void vRealCallback(const std_msgs::Float64::ConstPtr &msg);
            void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
            void steerCallback(const std_msgs::Float32::ConstPtr &msg);

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

            Eigen::Vector2d _mpcControl(Eigen::Vector4d s,Eigen::Vector4d s_d,Eigen::Vector2d u_r,Eigen::Vector2d du_p);
            
            Eigen::SparseMatrix<double> _convertTOSparseMatrix(Eigen::MatrixXd A);

            //Eigen::Vector2d _mpcControl_Frenet(const Eigen::Vector3d&, const Eigen::Vector2d&, const Eigen::Vector2d&);
            Eigen::Vector2d _mpcControl_Frenet(const Eigen::Vector3d &x,const Eigen::Vector2d &u_r,const Eigen::Vector2d &du_p,double v_ref);

            double linearRegularization(nav_msgs::Odometry& base_odometry , double v_d);
            
            double anglarRegularization(nav_msgs::Odometry& base_odometry , double w_d);
            
            double steeringAngleRegularization(nav_msgs::Odometry& base_odometry , double w_d);
            
            Eigen::VectorXd polyfit(const Eigen::VectorXd& x,const Eigen::VectorXd& y,int degree);

            double calculatekappa(const Eigen::VectorXd& coeffs,double x);
            //double calculatekappa(const Eigen::VectorXd& coeffs,double x,double y);

         
            double polyeval(Eigen::VectorXd coeffs,double x);
         
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
            
            ros::Subscriber hybrid_path_sub;
            ros::Subscriber global_path_sub;
            ros::Subscriber car_pose_sub;
            ros::Subscriber max_v_sub;
            ros::Subscriber max_v_inc_sub;
            ros::Subscriber turn_sub;
            ros::Subscriber turn2_sub;
            ros::Subscriber point_sub;
            ros::Subscriber vreal_sub;
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber steer_sub;
            std::string filename_;

            std::vector<double> global_path_x;
            std::vector<double> global_path_y;


            Eigen::Vector3d goal_rpy;

            int start_id = 0 ;
            int turn_index_ = 0;
            int turn_index_2 = 0;
            int count_back = 0;
            int last_start_id_ = -1;
            double v_real = 0.0000001;
            double steer_real = 0.0;
            geometry_msgs::Point point;

            std::vector<double> org_wp_rearange_waypoint_x;
            std::vector<double> org_wp_rearange_waypoint_y;

            std_msgs::Float64MultiArray local_path;
            std::vector<double> local_path_x_and_y;

            std::vector<geometry_msgs::PoseStamped> prune_plan;

            std::vector<geometry_msgs::PoseStamped> orig_global_plan;
            geometry_msgs::PoseStamped global_path;

            //Eigen::Matrix<double,3,3> 
            Eigen::MatrixXd Q_;
            Eigen::Matrix4d Qf_;
            bool initialize_ = false;      //判斷是否初始化
            double d_t_;           //控制採樣時間
            Eigen::MatrixXd R_;    //控制誤差權重矩陣
            int p_;                //預測時域
            int m_;                //控制時域
            Eigen::Vector2d du_p_; //先前控制誤差
            
            ros::Publisher target_pt_pub,current_pose_pub_;
            ros::Publisher local_path_to_matlab_pub;
            ros::Publisher mpc_result_pub;
            ros::Publisher v_pub ;
            ros::Publisher delta_pub ;
            ros::Publisher cte ;
            ros::Publisher start_id_pub ;

            Eigen::Vector2d u_prev;
            bool first = false; 

                // 新增：定時器與緩存最新里程計
            ros::Timer               control_timer_;
            nav_msgs::Odometry       latest_odom_;
            bool                     odom_received_ = false;

            double min_v_forward_ = 2;   // 前進最小速度 0.6
            double max_v_forward_ = 4;   // 前進最大速度 4
            double min_v_reverse_ = 0.22;   // 倒退最小速度（較慢）
            double max_v_reverse_ = 0.40;   // 倒退最大速度（較小）
             

            //linear velocity
            double max_v_ = 0.6;              //最大線速度
            double min_v_ = 0.4;                //最小線速度
            double max_v_inc_ = 0.03;          //線速度差值


            //angular velocity
            //double max_w_ = 0.2;             //最大角速度
            //double min_w_ = -0.2;            //最小角速度
            //double max_w_inc_ = 0.05;          //角速度差值
            
            //angle 
            double max_delta_ = 0.45;             //最大角度
            double min_delta_ = -0.45;            //最小角度
            double max_delta_inc_= 0.0084;        //角度差值 //0.0067

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
            double beta_;


    };
//#endif //MPC_PLANNER_Hmax_delta_inc_
