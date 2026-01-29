#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "mpcbitch/mpc_new.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <tf2/utils.h>
#include <OsqpEigen/OsqpEigen.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <deque>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>



std::string filename_;
ros::Publisher stop_signal_pub;
ros::Publisher carla_control_pub;

static double prev_theta = 0.0;
int final_point;

double MPCPlanner_path::dist(const geometry_msgs::PoseStamped& node1,const geometry_msgs::PoseStamped& node2)
{
    return std::hypot(node1.pose.position.x - node2.pose.position.x ,node1.pose.position.y - node2.pose.position.y);
}


//實現多項式擬合 (polynomial fitting)，即使用一組已知數據點 來計算多項式的係數，使該多項式最佳地逼近這些數據點
Eigen::VectorXd MPCPlanner_path::polyfit(const Eigen::VectorXd& xvals,const Eigen::VectorXd& yvals,int order)
{
    assert(xvals.size() == yvals.size());                   // 向量xvals的尺寸要等於向量yvals的尺寸
    assert(order >= 1 && order <= xvals.size() - 1); // 曲線擬和的次方要大於等於1 且 要小於向量xvals的尺寸-1
    Eigen::MatrixXd A(xvals.size(), order + 1);       // 創建Eigen::MatrixXd 類 為 點的數量 * 曲線擬合的階數+1 的矩陣
    
      for (int i = 0; i < xvals.size(); i++) 
      {
            A(i, 0) = 1.0;
      }
    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    //auto Q = A.householderQr();
    auto Q = A.colPivHouseholderQr();
    auto result = Q.solve(yvals);
    return result;
}

//評估多項式的值，即給定多項式的係數向量和 𝑥值，計算該多項式在x處的值。
double MPCPlanner_path::polyeval(Eigen::VectorXd coeffs,double x)
{
    double result = 0;
    for(int i = 0;i<coeffs.size();i++)
    {
        result+= coeffs[i] * pow(x,i);
        //std::cout<<"result = "<< result<<std::endl;
    }
    return result;
}

double MPCPlanner_path::calculatekappa(const Eigen::VectorXd& coeffs, double x)
{
    /*
    // 檢查係數數量是否至少有 4 項
    if (coeffs.size() != 4) {
        throw std::invalid_argument("Polynomial coefficients must have exactly 4 terms for cubic curves");
    }

    // 計算一階導數 dx = 3*a3*x^2 + 2*a2*x + a1
    double dx = 3 * coeffs[3] * pow(x, 2) + 2 * coeffs[2] * x + coeffs[1];
    std::cout << "dx = " << dx << std::endl;

    // 計算二階導數 ddx = 6*a3*x + 2*a2
    double ddx = 6 * coeffs[3] * x + 2 * coeffs[2];
    std::cout << "ddx = " << ddx << std::endl;
    */

    if (coeffs.size() != 5) { // 檢查是否為四階多項式
        throw std::invalid_argument("Polynomial coefficients must have exactly 5 terms for quartic curves");
    }

    double dx = 4 * coeffs[4] * pow(x, 3) + 3 * coeffs[3] * pow(x, 2) + 2 * coeffs[2] * x + coeffs[1]; // 一階導數
    double ddx = 12 * coeffs[4] * pow(x, 2) + 6 * coeffs[3] * x + 2 * coeffs[2];                      // 二階導數

    // 計算曲率 kappa = |ddx| / (2 + dx^2)^(3/2)
    double denominator = pow(2 + pow(dx, 2), 1.5);
    if (denominator < 1e-6) { // 避免分母過小
        denominator = 1e-6;
    }
    double kappa = fabs(ddx) / denominator;
    //std::cout << "ddx = " << ddx << std::endl;

    return kappa;
}

//std::floor 為向下取整數
// angle + M_PI 將原始角度angle+3.14 使原本的角度變成0～6.28
//然後在除以2*3.14將原始角度映射成相應的整數值 這個整數值表示原始角度在2*3.14的區段中的位置
//將上一步的整數值*2*3.14 得到原始角度在相應區段中的起始值 將原始角度映射到[0~2*3.14]
//將angle減去其對應區段的起始值 將角度調整成[-3.14~3.14]
/*
double MPCPlanner_path::regularizeAngle(double angle)
{
    return angle - 2.0 *M_PI * std::floor((angle + M_PI) / (2.0*M_PI));
}
*/
double MPCPlanner_path::regularizeAngle(double angle)
{
    double a = fmod(angle, 2.0 * M_PI);
    if (a < -M_PI)
        a += 2.0 * M_PI;
    else if (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
}


double unwrapAngle(double current_angle, double previous_angle) 
{
    double diff = current_angle - previous_angle;
    if(diff > M_PI) {
        current_angle -= 2.0 * M_PI;
    } else if(diff < -M_PI) {
        current_angle += 2.0 * M_PI;
    }
    return current_angle;
}

double MPCPlanner_path::linearRegularization(nav_msgs::Odometry& base_odometry,double v_d)
{
    //std::hypot(x,y)為先將x,y平方再開根號
    //base_odometry.twist.twist.linear.x為車輛在x方向的速度分量
    //base_odometry.twist.twist.linear.y為車輛在y方向的速度分量
    //double v = std::hypot(base_odometry.twist.twist.linear.x,base_odometry.twist.twist.linear.y);
    theta1 = tf2::getYaw(base_odometry.pose.pose.orientation);
    double v =  std::cos(theta1) * base_odometry.twist.twist.linear.x + std::sin(theta1) * base_odometry.twist.twist.linear.y;

    //v_d 為模型預測控制器計算出來期望車輛的速度
    //v 為車輛目前的速度
    //v_inc 為期望速度和車輛目前速度的差異
    double v_inc = v_d - v;


    //std::fabs(x)為計算絕對值
    //std::copysign(x,y)將第一個數的絕對值與第二個參數的符號相乘
    //如果速度差異的絕對值超過最大速度增量
    //會將最大速度增量的絕對值*速度差異正負號
    if(std::fabs(v_inc)>max_v_inc_)
    {
        v_inc = std::copysign(max_v_inc_,v_inc);
        
        //std::cout<<"v_inc_new = "<< v_inc <<std::endl;

    }

    // v_cmd 為調整過後的速度
    // v_inc 為調整過後的速度差異
    double v_cmd = v + v_inc;
    
    std::cout<<"v_cmd_new = "<< v_cmd <<std::endl;


    //如果調整過後的速度的絕對值大於設定最大速度上限
    //將max_v_的絕對值*v_cmd的正負號
    //將速度限制在最大速度上限
    if(std::fabs(v_cmd) > max_v_)
    {
        v_cmd = std::copysign(max_v_,v_cmd);
        
        std::cout<<"v_cmd_max = "<< v_cmd <<std::endl;

    }

    //如果調整過後的速度的絕對值小於設定最小速度上限
    // 將max_v_的絕對值*v_cmd的正負號
    //將速度限制在最小上限
    else if(std::fabs(v_cmd) < min_v_)
    {
        //v_cmd = std::copysign(min_v_,v_cmd);

        v_cmd =  min_v_;
        std::cout<<"v_cmd_min = "<< v_cmd <<std::endl;

    }

    return v_cmd;
}


double MPCPlanner_path::anglarRegularization(nav_msgs::Odometry& base_odometry,double delta_d)
{
    //delta_d 為模型預測控制計算出來的角度
    //max_delta_為設定得最大角速度上限
    //如果計算出來的角速度上線曲絕對值後大於設定的度上限
    //就把設定上限的值取絕對值 在乘以計算出來的正負號
    double L = 1.66; 
    
    if(std::fabs(delta_d) > max_delta_)
    {
        delta_d = std::copysign(max_delta_,delta_d);
    }

    //delta為當前前輪夾角
    theta1 = tf2::getYaw(base_odometry.pose.pose.orientation);
    double omega = base_odometry.twist.twist.angular.z;
    double vx = base_odometry.twist.twist.linear.x;
    double vy = base_odometry.twist.twist.linear.y;
    //double vt = std::hypot(vx,vy);
    double v_body =  std::cos(theta1) * vx + std::sin(theta1) * vy;
    double vt = std::fabs(v_body);
    double delta = atan(omega*L/vt);
    
    
    //delta_inc為計算出來的角度和當前方向盤角度的增量
    double delta_inc = delta_d - delta;
    
    //如果增量的絕對值大於最大角速增量
    //那最大角速增量的絕對值乘以增量的正負號
    if(std::fabs(delta_inc) > max_delta_inc_)
    {
        delta_inc = std::copysign(max_delta_inc_,delta_inc);
    }

    //delta_cmd為調整增量過後的角速度
    double delta_cmd = delta + delta_inc;
    
    //如果調整過後的角速度的絕對值大於設定最大角速度上限
    //將max_w_的絕對值*w_cmd的正負號
    //將角速度限制在最大角速度上限
    if(std::fabs(delta_cmd) >  max_delta_)
    {
        delta_cmd = std::copysign(max_delta_,delta_cmd);
    }
    //如果調整過後的角速度的絕對值小於設定最小角速度上限
    //將min_w_的絕對值*w_cmd的正負號
    //將角速度限制在最小角速度上限
    else if(std::fabs(delta_cmd) < min_delta_)
    {
        delta_cmd = std::copysign(min_delta_,delta_cmd);
    }
    return delta_cmd;
}




    void MPCPlanner_path::MPCPlanner(ros::NodeHandle *nh)
    {
        ROS_INFO("MPCPlanner START");
        initialize();
        
        
    }


    void MPCPlanner_path::initialize()
    {
        caculate_mpc_start = ros::Time::now().toSec();
 
        Eigen::Vector2d u_prev;
        u_prev = Eigen::Vector2d(min_v_, 0);

        private_nh_.param<std::string>("save_filename", filename_, "/home/cyc/new_golf/src/mpc/mpcdata/simulation.csv");
        ROS_INFO("Data will be saved to: %s", filename_.c_str());

        ROS_INFO("MPC Planner initialized START");
        if(!initialize_ )
        {
            
               //目標距離誤差
            double goal_dist_tol_  =0.2;   //目標距離誤差
            double rotate_tol_ = 0.5;      //旋轉誤差
            double convert_offset_ = 0;    //轉換偏移量


            //解黎卡提方程式迭代次數
            //預測時間域:是指在每個控制週期內 模型預測控制器用來預測系統行為的時間範圍
            //  而這個時間範圍由控制器在每個控制週期內向前預測的時間步數確定
            //  例:如果在每個控制週期內預測未來5個時間步 那預測時間域就是從當前時刻開始 
            //  往後推5個時間步的時間範圍 

            //控制時間域:在每個控制週期內 控制器用來計算最優控制輸入的時間範圍
            // 其時間範圍 由控制週期和系統動態決定
            // 控制時間域可能比預測時間域短 
            // 因為控制器只能在當下控制週期內計算出最優控制輸入 無法預測未來更遠的時間
            //
            //b1 simulation success 
            //p_ = 20;                    //預測時間域
            //m_ = 2;                    //控制時域*/

            p_ = 22;//22                //預測時間域
            m_ = 1;                    //控制時域*/


            //權重矩陣：用於懲罰在進行路徑追蹤控制時的狀態誤差[x,y,theta] 
            //宣告為 3*3 矩陣 [Q_ 0  0 ]
            //              [0  Q_ 0 ]
            //              [0  0  Q_]

            //Q_.resize(3,3);
            Q_.setZero();
            Q_(0,0) = 100; //100
            Q_(1,1) = 100; //100
            Q_(2,2) = 100;//50
            //std::cout<<"Q_ \n = "<< Q_ <<std::endl;

            //權重矩陣：用於懲罰在進行路徑追蹤控制時的輸入誤差[v,w]
            //宣告 2*2 矩陣 [R_ 0 ]
            //             [0  R_]
            //R_.resize(2,2);
            R_.setZero();
            R_(0,0) = 30;
            R_(1,1) = 60;

            //R_(0,0) = 5;
            //R_(1,1) = 5;

            //std::cout<<"R_ \n = "<< R_ <<std::endl;


            //採樣時間
            double controller_frequency = 5;
            d_t_ = 1/controller_frequency;
            
            
            //global_path_sub = nh_.subscribe("/global_waypoint_from_matlab",1000,&MPCPlanner_path::setPlan,this);
            global_path_sub = nh_.subscribe("array_topic",1000,&MPCPlanner_path::setPlan,this);  


            car_pose_sub = nh_.subscribe("/mpc_new_pose",1000,&MPCPlanner_path::computelocalpath,this);

            local_path_to_matlab_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/local_path",1000);


            mpc_result_pub = nh_.advertise<std_msgs::Float64MultiArray>("/mpc_result_test",1000);
            
            stop_signal_pub = nh_.advertise<std_msgs::Bool>("/stop_signal", 1000);

            start_id_pub = nh_.advertise<std_msgs::Int32>("/start_id", 1000);

            max_v_sub = nh_.subscribe("/max_v", 10, &MPCPlanner_path::maxVCallback, this);
            max_v_inc_sub = nh_.subscribe("/max_v_inc", 10, &MPCPlanner_path::maxVIncCallback, this);
            turn_sub = nh_.subscribe("turn_index", 10, &MPCPlanner_path::turnIndexCallback, this);
            //point_sub = nh_.subscribe("split_point", 10,&MPCPlanner_path::pointCallback, this);

            carla_control_pub = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);

            //hybrid_path_sub = nh_.subscribe<nav_msgs::Path>("/sPath", 1000,&MPCPlanner_path::hybridPathCallback, this);


            //ROS_INFO("MPC Planner initialized !");
     }
        else
        {
            ROS_WARN("MPC planner has already been initialized ");
        }
    }

    void MPCPlanner_path::setPlan(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        global_path_x.clear();
        global_path_y.clear();

        for(int i = 0;i<msg->data.size();i+=2)
        {
            global_path_x.emplace_back(msg->data[i]);
            global_path_y.emplace_back(msg->data[i+1]);
        }

        std::cout<<"global_path_x.size() = "<< global_path_x.size() <<std::endl;
        std::cout<<"global_path_y.size() = "<< global_path_y.size() <<std::endl;
                   
    }

    void MPCPlanner_path::turnIndexCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        turn_index_ = msg->data;
        ROS_INFO("turn point:%d", turn_index_);
    }


    void MPCPlanner_path::maxVCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        max_v_ = msg->data;
        ROS_INFO("Updated max_v_ to: %f", max_v_);
    }

    void MPCPlanner_path::maxVIncCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        max_v_inc_ = msg->data;
        ROS_INFO("Updated max_v_inc_ to: %f", max_v_inc_);
    }
    
    void publishStopSignal(bool stop) 
    {
	    std_msgs::Bool stop_msg;
	    stop_msg.data = stop;
	    stop_signal_pub.publish(stop_msg);
	    ROS_INFO("Stop signal published: %s", stop ? "true" : "false");
    }

    // 計算 throttle 指令
    double compute_throttle(double current_speed, double target_speed)
    {
        double Kp = 0.5;  // 比例增益，自己可以調（建議 0.3～0.7 之間）
        //target_speed = u_v

        double speed_error = target_speed - current_speed;  // 目標 - 當前速度
        double throttle_command = Kp * speed_error;         // 簡單P控制

        // 把 throttle 限制在 0.0～1.0 之間
        throttle_command = std::min(std::max(throttle_command, 0.0), 1.0);

        return throttle_command;
    }

    void MPCPlanner_path::computelocalpath(const nav_msgs::OdometryConstPtr &msg)
    {
    
        nav_msgs::Odometry base_odom;
        double L = 1.66;
        px = msg->pose.pose.position.x;
        py = msg->pose.pose.position.y;
        //theta1 = msg->pose.pose.orientation.w;
        theta1 = tf2::getYaw(msg->pose.pose.orientation);
        vx = msg->twist.twist.linear.x;
        vy = msg->twist.twist.linear.y;
        omega = msg->twist.twist.angular.z;

        double v_body =  std::cos(theta1) * vx + std::sin(theta1) * vy;

        //double vt = std::hypot(vx,vy);
        double vt = std::fabs(v_body);
        
        theta = regularizeAngle(theta1);
        
        next_px = px + vt*cos(theta)*d_t_;
        next_py = py + vt*sin(theta)*d_t_;
 
        // base_odom.twist.twist.linear.x = vx;
        // base_odom.twist.twist.linear.y = vy;
        base_odom.twist.twist.linear.x = v_body;
        base_odom.twist.twist.linear.y = 0;
        base_odom.twist.twist.angular.z = omega;
        
        double delta = atan(base_odom.twist.twist.angular.z * L / vt);

        std::cout<<"car_x = "<< px <<std::endl;
        std::cout<<"car_y = "<< py <<std::endl;
        std::cout<<"car_theta = "<< theta<<std::endl;
        std::cout<<"car_vel_x = "<<vx<<std::endl;
        std::cout<<"car_vel_y = "<< vy<<std::endl;
        std::cout<<" delta = "<< delta  <<std::endl;



        double nearest_distance = 999;                                       // declare the double variable "nearest_distance" is 100

        if (global_path_x.size()!=0)
        {

         for(size_t i = 0;i<global_path_x.size();i++)
        {
                double distance_calc = sqrt(pow(global_path_x[i]-px,2)+
                                 pow(global_path_y[i]-py,2));
            

            if(distance_calc < nearest_distance){
                    nearest_distance = distance_calc;

                    //std::cout<<"nearest_distance = "<<nearest_distance<<std::endl;
                    start_id  = i;
               
                    
                    //std::cout<<"start id position  = "<< global_path_x.at(i)<<std::endl;
                    //std::cout<<"~~~~~~~~~~first_start_id = "<<start_id<<std::endl;
                } 
        }  
        // === 新作法：只在「往前 200 點」視窗內搜尋最近點 ===
        // static int last_start_id_ = 0;             // ▲放在類別成員更乾淨

        // double nearest_distance = 999;
        // int    candidate_id     = last_start_id_;  // 起點＝上一圈結果

        // size_t search_end = std::min(global_path_x.size(),
        //                               static_cast<size_t>(last_start_id_ + 200));

        // for (size_t i = last_start_id_; i < search_end; ++i)
        // {
        //      double distance_calc = hypot(global_path_x[i]-px,
        //                                   global_path_y[i]-py);

        //      if (distance_calc < nearest_distance) {
        //          nearest_distance = distance_calc;
        //          candidate_id     = static_cast<int>(i);   // 只記錄，不馬上回寫
        //      }
        // }

        //  // 偏離 >5 m → 重抓起點
        // if (nearest_distance > 5.0) 
        // {
        //      candidate_id   = 0;
        //      last_start_id_ = 0;
        // }

        // start_id       = candidate_id;   // ►真正寫回 start_id
        // last_start_id_ = candidate_id;   // 同步更新記錄

        if(nearest_distance >5)   //如果最短距離大於5 重設start_id
        {
            start_id = -1; 
        }

        org_wp_rearange_waypoint_x.clear();
        org_wp_rearange_waypoint_y.clear();


        ROS_INFO("starting_waypoint_for mpc is: %d", start_id);
        final_point ++;

        
        if(start_id < 0)
        {
            start_id = 0;
        }

        if (start_id >= global_path_x.size() - 2) 
        {          
            ROS_INFO("finish");
            publishStopSignal(true);  // 發布停止訊號
            ros::shutdown();          // 結束 ROS 節點
            return;                   // 確保不再執行後續代碼
        }

        std_msgs::Int32 start_id_msg;
        start_id_msg.data = start_id;

        start_id_pub.publish(start_id_msg);
       

       //---------NEW caculate kappa finish 須改成用cte 和 espi算---------


        //---------NEW calculate cte & epsi START---------
        
       // 1. 車輛全局位置
        Eigen::Vector2d vehiclePos(px, py);

        // 2. 在全局路徑中找出車輛位置的投影點（用相鄰點線段近似參考曲線）
        double minDistance = 1e6;
        int nearestIndex = -1;
        Eigen::Vector2d projection;  // 投影點
        double refHeading = 0.0;     // 局部路徑切線方向
        Eigen::Vector2d p2(0, 0);

        for (size_t i = 0; i < global_path_x.size()-1; i++) {
            Eigen::Vector2d p1(global_path_x[i], global_path_y[i]);
            Eigen::Vector2d p2(global_path_x[i+1], global_path_y[i+1]);
            
            Eigen::Vector2d v = p2 - p1;
            double t = (vehiclePos - p1).dot(v) / v.squaredNorm();
            // 限制 t 在 [0,1]
            if(t < 0) t = 0;
            if(t > 1) t = 1;
            Eigen::Vector2d proj = p1 + t * v;
            double dist = (vehiclePos - proj).norm();
            if (dist < minDistance) {
                minDistance = dist;
                projection = proj;
                refHeading = atan2(v.y(), v.x());
                nearestIndex = i;
            }
        }

        // 3. 計算橫向誤差 d（正負依據車輛在參考曲線哪側）
        Eigen::Vector2d refDir(cos(refHeading), sin(refHeading));
        Eigen::Vector2d errorVec = vehiclePos - projection;
        double cte_real = 0.0;
        double cte = errorVec.norm();
        double sign = (refDir.x() * errorVec.y() - refDir.y() * errorVec.x()) >= 0 ? 1.0 : -1.0;
        cte *= sign;

        cte_real = cte;

        double epsi = regularizeAngle(theta - refHeading);

        if (fabs(cte) > 0.3) 
        {
            cte = 0.3 * (cte > 0 ? 1.0 : -1.0);
        }

        // 輸出調試信息
        ROS_INFO("Frenet: Projection=(%.3f, %.3f), d=%.3f, refHeading=%.3f, epsi=%.3f", 
                projection.x(), projection.y(), cte, refHeading, epsi);

        // ----- 使用 Frenet 誤差組成 MPC 的狀態誤差 -----
        // 此處的設計會依你的 MPC 模型而定，下面僅作為一個範例
        // 假設原來 s 為 (px, py, theta)，而 s_d 代表目標狀態，
        // 這裡我們直接用投影點與 refHeading 作為目標（你也可以根據 d 與 epsi 設計誤差向量）
        Eigen::Vector3d s(px, py, theta);
        Eigen::Vector3d s_d(projection.x(), projection.y(), refHeading);
        Eigen::Vector2d p0(global_path_x[nearestIndex-1], global_path_y[nearestIndex-1]);
        Eigen::Vector2d p1(global_path_x[nearestIndex],   global_path_y[nearestIndex]);


        int count = 0;
        for (int i = nearestIndex + 1; i < global_path_x.size() && i <= nearestIndex + 10; ++i) 
        {
            p2 += Eigen::Vector2d(global_path_x[i], global_path_y[i]);
            count++;
        }
        if (count > 0)
        {
            p2 /= static_cast<double>(count);
        }
        

        // 計算向量
        Eigen::Vector2d v1 = p1 - p0;
        Eigen::Vector2d v2 = p2 - p0;

        // 計算三角形面積
        double area = 0.5 * fabs(v1.x() * v2.y() - v1.y() * v2.x());

        // 計算邊長
        double d01 = v1.norm();
        double d12 = (p2 - p1).norm();
        double d02 = (p2 - p0).norm();

        // 防止除以零
        double kappa = 0.0;
        double last_kappa = 0.0;

        if(d01 > 1e-3 && d12 > 1e-3 && d02 > 1e-3) 
        {
            last_kappa = kappa;
            kappa = 2 * area / (d01 * d12 * d02);
        }
        

        int window_size = 20;
        std::vector<double> local_x;
        std::vector<double> local_y;

        // 以 projection 為原點、refHeading 為 x 軸方向，將局部全局路徑點轉換到局部座標系
        for (int i = nearestIndex; i < nearestIndex + window_size && i < global_path_x.size(); i++) {
            double gx = global_path_x[i];
            double gy = global_path_y[i];
            // 平移，使 projection 成為原點
            double dx = gx - projection.x();
            double dy = gy - projection.y();
            // 旋轉：將點從全局座標轉換到局部座標
            double local_xi = dx * cos(-refHeading) - dy * sin(-refHeading);
            double local_yi = dx * sin(-refHeading) + dy * cos(-refHeading);
            local_x.push_back(local_xi);
            local_y.push_back(local_yi);
        }

        // 將 std::vector 轉換為 Eigen 向量
        Eigen::VectorXd x_vals = Eigen::Map<Eigen::VectorXd>(local_x.data(), local_x.size());
        Eigen::VectorXd y_vals = Eigen::Map<Eigen::VectorXd>(local_y.data(), local_y.size());

        // 使用二次多項式擬合
        int poly_order = 2;
        //Eigen::VectorXd coeffs = polyfit(x_vals, y_vals, poly_order);

        // 定義 lambda 表達式計算一階與二階導數
        auto polyDerivative = [&](const Eigen::VectorXd &coeffs, double x) -> double {
            double derivative = 0.0;
            for (int i = 1; i < coeffs.size(); i++) {
                derivative += i * coeffs[i] * pow(x, i - 1);
            }
            return derivative;
        };

        auto polySecondDerivative = [&](const Eigen::VectorXd &coeffs, double x) -> double {
            double secondDerivative = 0.0;
            for (int i = 2; i < coeffs.size(); i++) {
                secondDerivative += i * (i - 1) * coeffs[i] * pow(x, i - 2);
            }
            return secondDerivative;
        };
           
        // 根據曲率計算 delta_d（車輛前輪角度前饋量）
        double delta_d = atan(kappa * L);
    
        // 參數設定
        static double prev_v_ref = vt;  // 使用當前速度 vt 作為初始值
        double v_min_ref = min_v_+0.2;  
        double v_max_ref = max_v_;
        double desired_v_ref = v_min_ref;  // 預設目標速度為最低速度
        double curvature_threshold = 0.0003; // 曲率門檻
        double cte_threshold = 0.05;         // cte 門檻
        double cte_scale = 0.2;              // cte 調整尺度
        int lookahead_points = 20;           // 預知點數
        int stable_window = 3;               // cte 平滑窗口大小

        // 2. 根據不同區段計算 desired_v_ref
        if (start_id < 5) 
        {
            // 路徑前段：從當前速度 vt 平滑過渡到 v_min_ref
            double ratio = static_cast<double>(start_id) / 5.0;
            //desired_v_ref = vt * (1.0 - ratio) + v_min_ref * ratio;
            desired_v_ref = vt * (1.0 - ratio) + 0.3 * ratio;
        }
        else if (start_id >= (global_path_x.size() - 15)) 
        {
            // 路徑末段：平滑過渡到 v_max_ref
            double r = static_cast<double>(global_path_x.size() - start_id) / 15.0;
            double smooth_ratio = r * r;  // 二次衰減，平滑過渡
            desired_v_ref = v_min_ref + smooth_ratio * (vt - v_min_ref);
        }
        else if (turn_index_-start_id < 10) 
        {
            // 轉彎區段：從當前速度 vt 平滑過渡到 v_min_ref
            double ratio = static_cast<double>(turn_index_ - start_id) / 10.0;
            double smooth_ratio = r * r;  // 二次衰減，平滑過渡
            desired_v_ref = 0 + smooth_ratio * vt;
        }
        else 
        {
            
            // 中間區段：根據 cte 與預測曲率計算 desired_v_ref
            
            static std::deque<double> cte_window;  
            cte_window.push_back(fabs(cte));
            if (cte_window.size() > stable_window)
                cte_window.pop_front();
            
            // 計算 cte 的平均值
            double avg_cte = 0.0;
            for (double val : cte_window)
                avg_cte += val;
            avg_cte /= cte_window.size();
            
            double predicted_curvature = 0.0;
            int count = 0;
            if (start_id + lookahead_points < global_path_x.size()) 
            {
                for (int i = start_id; i < start_id + lookahead_points - 2; ++i) {
                    Eigen::Vector2d p0(global_path_x[i], global_path_y[i]);
                    Eigen::Vector2d p1(global_path_x[i+1], global_path_y[i+1]);
                    Eigen::Vector2d p2(global_path_x[i+2], global_path_y[i+2]);
                    Eigen::Vector2d v1 = p1 - p0;
                    Eigen::Vector2d v2 = p2 - p0;
                    double area = 0.5 * fabs(v1.x() * v2.y() - v1.y() * v2.x());
                    double d01 = v1.norm();
                    double d12 = (p2 - p1).norm();
                    double d02 = (p2 - p0).norm();
                    double local_kappa = 0.0;
                    if (d01 > 1e-3 && d12 > 1e-3 && d02 > 1e-3) {
                        local_kappa = 2 * area / (d01 * d12 * d02);
                    }
                    predicted_curvature += local_kappa;
                    count++;
                }
                if (count > 0)
                    predicted_curvature /= count;
            } 
            else 
            {
                predicted_curvature = fabs(refHeading);
            }
            
            if (avg_cte < cte_threshold) 
            {
                double factor_curvature = 1.0;
                if (predicted_curvature > curvature_threshold) {
                    factor_curvature = std::max(0.0, 1.0 - (predicted_curvature - curvature_threshold) * 5);
                }
                
                double factor_cte = 1.0;
                if (avg_cte > cte_threshold) {
                    factor_cte = std::max(0.0, 1.0 - ((avg_cte - cte_threshold) / cte_scale));
                }
                
                double combined_factor = 0.5 * factor_curvature + 0.5 * factor_cte;
                combined_factor = std::min(1.0, std::max(0.0, combined_factor));
                desired_v_ref = v_min_ref + combined_factor * (v_max_ref - v_min_ref);
            } 
            else 
            {
                desired_v_ref = v_min_ref;
            }
            
            // 分段提高速度：當滿足直線條件且 avg_cte 較小時，每次僅增加一個固定步階
            double straight_threshold = 0.0003; // 判斷直線的曲率門檻
            if (fabs(predicted_curvature) < straight_threshold && avg_cte < cte_threshold) 
            {
                double speed_step = 0.1;  // 每次遞增的速度步階（可根據實際需求調整）
                if (desired_v_ref + speed_step <= v_max_ref)
                    desired_v_ref += speed_step;
                else
                    desired_v_ref = v_max_ref;
            }
            
          
        }

        // 3. 使用 EMA 平滑更新 v_ref，並限制一次變化不超過 0.05
        double alpha = 0.15; // 平滑係數，alpha 越小更新越平滑
        double new_v_ref = alpha * desired_v_ref + (1.0 - alpha) * prev_v_ref;
        double max_delta_v = 0.02; // 每次最大變化限制
        double v_ref = std::clamp(new_v_ref, prev_v_ref - max_delta_v, prev_v_ref + max_delta_v);
        prev_v_ref = v_ref;


        Eigen::Vector2d u_r(v_ref,delta_d);

        
        Eigen::Vector2d u = _mpcControl(s,s_d,u_r,du_p_);


        double u_v = linearRegularization(base_odom,u[0]);

        double u_delta = anglarRegularization(base_odom,u[1]);

        
        if(kappa <= 0.0002 && fabs(last_kappa - kappa) <= 0.0003)
        {
            if (fabs(cte) < 0.01 && fabs(epsi) < 0.01) 
            {
                // 認為誤差可忽略，直接把方向量清零
                u_delta = 0.0;
            }
        }
      
        du_p_ = Eigen::Vector2d(u_v - u_r[0],regularizeAngle(u_delta - u_r[1]));
        //double L = 1.66;
        // 先定義車輛參數，假設 L 為車輛軸距，令 l_r = L/2 (你可以根據實際數值調整)
        double l_r = 0.4;

        // 根據前輪轉角 u_r[1] 計算側滑角 β
        double beta_1 = atan((l_r / L) * tan(u_r[1]));

     
        std_msgs::Float64MultiArray cmd_to_matlab;
        std_msgs::Float64MultiArray cte_test;

        carla_msgs::CarlaEgoVehicleControl control_cmd;
        //control_cmd.throttle = std::min(std::max(u_v / 4.1625, 0.0), 1.0);
        double current_speed = sqrt(vx * vx + vy * vy);  // or 讀 /vehicle_status 拿速度（單位 m/s）
        //double target_speed = u_v;
        //double throttle_command = compute_throttle(current_speed, u_v);
        control_cmd.throttle = compute_throttle(current_speed, u_v);
        control_cmd.steer = std::min(std::max(u_delta / 0.4, -1.0), 1.0);
        control_cmd.brake = 0.0;
        control_cmd.hand_brake = false;
        control_cmd.reverse = false;

        carla_control_pub.publish(control_cmd);
      
        //ROS_INFO("u_v=%.2f m/s, current_speed=%.2f m/s, error=%.2f, throttle=%.2f", 
            //u_v, current_speed, target_speed - current_speed, throttle_command);
   
        
        cmd_to_matlab.data.clear();
        cte_test.data.clear();
        
        cmd_to_matlab.data.emplace_back(u_v);
        cmd_to_matlab.data.emplace_back(u_delta);
        cte_test.data.emplace_back(cte);

    
        ROS_INFO("-----------------------------------------------");

        std::cout<<"linear_velocity = "<< u_v <<std::endl;
        std::cout<<"steeringangle = "<< u_delta <<std::endl;
        std::cout<<"theta = "<< theta <<std::endl;
        std::cout<<" lateral error = \n"<< cte <<std::endl;

        ros::Duration gan(0.1);

        
        //local_path_to_matlab_pub.publish(local_path);
        //gan.sleep();
            

        mpc_result_pub.publish(cmd_to_matlab);
       // cte_pub.publish(cte_test);
        
        gan.sleep();

        caculate_mpc_finish = ros::Time::now().toSec();
        //std::cout<<" 控制週期 = "<< caculate_mpc_start - caculate_mpc_finish<<std::endl;

	
	    std::ofstream ofs(filename_, std::ios::app);
	    if (!ofs.is_open()) {
		ROS_ERROR("Failed to open file: %s", filename_.c_str());
		return;
	    }
        //std::cout<<"kappa2 = \n"<< kappa <<std::endl;
	    static bool header_written = false;
	    if (!header_written) {
		ofs << "u_v,v_ref,u_delta,delta_d,px,py,theta1,cte,cte_real,epsi,vx,vy,kappa,beta" << std::endl;
		header_written = true;
	    }

	    ofs << std::fixed << std::setprecision(4)
		<< u_v << ","<< v_ref << "," << u_delta << ","<< delta_d << "," << px << "," << py << ","
		<< theta1 << "," << cte << "," << cte_real << ","<< epsi << "," << vx << ","
		<< vy <<","<<kappa<< "," << beta_1<<std::endl;

	    ofs.close(); // 確保每次操作後關閉文件
        }
	    

	    //車輛換道實驗
	    //private_nh_.param<std::string>("save_filename",filename_,std::string("/home/king/mpc/mpcdata"));
        
    }


Eigen::Vector2d MPCPlanner_path::_mpcControl(Eigen::Vector3d s,Eigen::VectorXd s_d,Eigen::Vector2d u_r,Eigen::Vector2d du_p)
{
    int dim_u = 2;
    int dim_x = 3;
    double L = 1.66;
    // 先定義車輛參數，假設 L 為車輛軸距，令 l_r = L/2 (你可以根據實際數值調整)
    double l_r = 0.4;

    // 根據前輪轉角 u_r[1] 計算側滑角 β
    //double beta = atan((l_r / L) * tan(u_r[1]));
    double beta = 0;
    OsqpEigen::Solver solver;
    


    // state vector (5*1)
    // 宣告一個5*1的列向量 系統的狀態訊息和控制輸入訊息
    // topLeftCorner(3,1) 為左上角的3*1向量 s為車輛目前位置及方向角 s_d為車輛目標位置及方向角
    // s-s_d 為狀態誤差的計算
    // x[2]為列向量中的第三個方向角的狀態誤差 對方向角做正規化
    // bottomLeftCorner(2,1) 為左下角的2*1向量 為 du_p 2*1向量
    // du_p 第一項為經過mpc計算的速度減目前車輛的速度 第二項為經過mpc計算過後的角度減目前車輛的速度*道路曲率   
    Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
    x.topLeftCorner(dim_x,1) = s - s_d;
    //x.topLeftCorner(dim_x,1) =  s_d;
    x[2] = regularizeAngle(x[2]);
    x.bottomLeftCorner(dim_u,1) = du_p;
    //std::cout<<"x = \n"<< x <<std::endl;



    //original state matrix
    //宣告一個3*3的單位矩陣 系統的原始狀態轉移矩陣
    //u_r為一2*1向量 第一項為車輛速度(vx及vy) 第二項為前輪轉角
    //A_o(0,2)為矩陣第(0,2)項 為車輛速度*cos(車輛目標方向角)*取樣時間
    //A_o(1,2)為矩陣第(1,2)項 為車輛速度*sin(車輛目標方向角)*取樣時間
    Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
    A_o(0,2) = -u_r[0] * sin(s_d[2] + beta) * d_t_;
    A_o(1,2) =  u_r[0] * cos(s_d[2] + beta) * d_t_;
    //std::cout<<"A_o = \n"<< A_o <<std::endl;


    //original control matrix
    //宣告一個3*2的零矩陣 系統的原始控制輸入矩陣
    //B_o(0,0)為矩陣第(0,0)項 為cos(車輛目標方向角)*取樣時間
    //B_o(1,0)為矩陣第(1,0)項 為sin(車輛目標方向角)*取樣時間
    //B_o(2,1)為矩陣第(2,1)項 為取樣時間
    Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x,dim_u);
    B_o(0,0) = cos(s_d[2] + beta) * d_t_;
    B_o(1,0) = sin(s_d[2] + beta) * d_t_;
    // 此處 yaw rate 改為考慮側滑角，使用公式: (v*cos(β)*tan(δ)) / L
    B_o(2,0) = d_t_ * (u_r[0] * cos(beta) * tan(u_r[1]) / L);
    // B_o(2,1) 保持原來設定，如有需要也可以進一步調整
    B_o(2,1) = (d_t_ * u_r[0]) / (L * cos(u_r[1]) * cos(u_r[1]));
    //std::cout<<"B_o = \n"<< B_o <<std::endl;


    //state matrix(5*5)
    //宣告一個5*5的零矩陣 系統的狀態轉移矩陣
    //topLeftCorner(dim_x,dim_x)    為左上角3*3的矩陣為A_o
    //topRightCorner(dim_x,dim_u)   為右上角3*2的矩陣為B_o
    //bottomLeftCorner(dim_u,dim_x) 為左下角的2*3的矩陣為零矩陣
    //bottomRightCorner(dim_u,dim_u)為右下角的2*2的矩陣為單位矩陣
    //建立系統模型
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x+dim_u,dim_x+dim_u);
    A.topLeftCorner(dim_x,dim_x) = A_o;
    A.topRightCorner(dim_x,dim_u) = B_o;
    A.bottomLeftCorner(dim_u,dim_x) = Eigen::MatrixXd::Zero(dim_u,dim_x);
    A.bottomRightCorner(dim_u,dim_u) = Eigen::Matrix2d::Identity();
    //std::cout<<"A = \n"<< A <<std::endl;


    //control matrix(5*2)
    //宣告一個5*2矩陣 系統的控制輸入矩陣
    //topLeftCorner(dim_x,dim_u)   為左上角3*2的矩陣為B_o
    //bottomLeftCorner(dim_u,dim_u)為左下角2*2矩陣為單位矩陣
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x + dim_u , dim_u);
    B.topLeftCorner(dim_x,dim_u) = B_o;
    B.bottomLeftCorner(dim_u,dim_u) = Eigen::Matrix2d::Identity();
    //std::cout<<"B = \n"<< B <<std::endl;


    //output matrix (3*5)
    //宣告一個3*5的零矩陣 系統的控制輸出矩陣
    //topLeftCorner(dim_x,dim_x) 為左上角3*3的矩陣為單位矩陣
    //topRightCorner(dim_x,dim_u)為右上角3*2的矩陣為零矩陣
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x,dim_x + dim_u);
    C.topLeftCorner(dim_x,dim_x) = Eigen::Matrix3d::Identity();
    C.topRightCorner(dim_x,dim_u) = Eigen::MatrixXd::Zero(dim_x,dim_u);
    //std::cout<<"C = \n"<< C <<std::endl;



    //mpc state matrix (3p *5 )
    //Eigen::MatrixPower<Eigen::MatrixXd>為矩陣的次方項表示
    //A_pow(A)為對A矩陣進行次方項乘法
    Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
    
    //宣告一個(3*預測時域,5)的零矩陣 模型預測控制的狀態矩陣
    //透過for迴圈及middleRows(0,3)開始寫入,
    //middleRows(0,3)為矩陣中從第0行開始連續3行寫入C*A矩陣的1次方
    //middleRows(3,3)為矩陣中從第3行開始連續3行寫入C*A矩陣的2次方
    //middleRows(6,3)為矩陣中從第6行開始連續3行寫入C*A矩陣的3次方
    Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_,dim_x + dim_u);
    //std::cout<<"S_x_initial_size() = \n" << S_x <<std::endl;
    
    for(int i = 0; i<p_; i++)
    {
        //S_x.middleRows(dim_x * i,dim_x) = C  * A_pow(i + 1);
        S_x.middleRows(dim_x * i,dim_x) = C  * A_pow(i + 1);
    }
    //std::cout<<"S_x = \n"<< S_x <<std::endl;

    //mpc control matrix (3p * 2m) 模型預測控制的控制矩陣
    //宣告ㄧ(3*預測時域,2*控制時域)的零矩陣 
    //如果預測時域大於等於控制時域時 透過block功能從(0,0,3,2) 從第0行0列開始的3*2矩陣寫入C*A矩陣的次方*B
    //這裡用到兩個for迴圈 所以這裡執行的順序是 
    //i = 0,j = 0 ; i = 0,j =1; i = 1,j = 0; 
    //如果預測時域小於控制時域時    透過block功能寫入零矩陣
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_ ,dim_u * m_);
    //std::cout<<"S_u_initial_size() = \n" << S_u <<std::endl;

    for(int i = 0; i<p_; i++)
    {
        for(int j = 0; j<m_; j++)
        {
            if(j <= i)
            {
                S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
                //std::cout<<"預測時域大於等於控制時域"<<std::endl;
                //std::cout<<"S_u1\n"<<S_u<<std::endl;
            }
            else
            {
                S_u.block(dim_x * i, dim_u * j,dim_x ,dim_u) = Eigen::MatrixXd::Zero(dim_x,dim_u);
                //std::cout<<"預測時域小於控制時域時"<<std::endl;
                //std::cout<<"S_u2\n"<<S_u<<std::endl;
            }
        }
    }
    //std::cout<<"S_u = \n"<< S_u <<std::endl;

    //optimization start
    // (3p * 1 )
    //宣告一(3*預測時域)的零向量
    Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x*p_);

    // (3p * 3p)
    //宣告一動態矩陣為 Q 3p*3p 的矩陣 計算模型預測控制的狀態誤差權重矩陣
    //kroneckerProduct為計算兩個矩陣的克羅內克積
    //在克羅內克積中有一個(預測時域,預測時域)的單位矩陣 乘 狀態誤差權重矩陣
    Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_,p_),Q_);
    //std::cout<<"Eigen::MatrixXd::Identity(p_,p_)  = \n"<< Eigen::MatrixXd::Identity(p_,p_) <<std::endl;
    //std::cout<<"Q  = \n"<< Q <<std::endl;


    // (2m *2m)
    //宣告一動態矩陣為 R 為2m*2m的矩陣  計算模型預測控制的控制誤差權重矩陣
    //kroneckerProduct為計算兩個矩陣的克羅內克積
    //在克羅內克積中有一個(控制時域,控制時域)的單位矩陣 乘 控制誤差權重矩陣
    Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_,m_),R_);
    //std::cout<<"R = \n"<< R  <<std::endl;


    //宣告一動態矩陣為 H 計算模型預測控制的 Hessian矩陣
    //(2m*2m)
    int n_var  = dim_u * m_;
    int n_cons = 2 * dim_u * m_;
    Eigen::MatrixXd H = S_u.transpose() * Q * S_u + R;
    H = (H + H.transpose()) * 0.5;      // 保證對稱
    H.diagonal().array() += 1e-6;      // 加 tiny damping，避免負或零特徵值


    
    //std::cout<<" S_u.transpose() = \n"<< S_u.transpose() <<std::endl;
    //std::cout<<" S_u.transpose()* Q = \n"<< S_u.transpose() * Q <<std::endl;
    //std::cout<<" S_u.transpose()* Q *S_u = \n"<< S_u.transpose() * Q * S_u <<std::endl;

    //std::cout<<" H = \n"<< H <<std::endl;

    //宣告一動態向量為 g 計算模型預測控制的梯度向量
    //(2m*1)
    Eigen::VectorXd g = S_u.transpose() * Q * (S_x * x - Yr);
    //Eigen::VectorXd g = 2*(S_x * x).transpose()*Q*S_u;
    //std::cout<<"g = \n"<< g <<std::endl;

    //optimization finish



    //---------boundary start---------

    //宣告一個2*1向量為 u_min 控制輸入最小值
    //min_v_為最小速度 min_w_為最小角速度
    Eigen::Vector2d u_min(min_v_,min_delta_);
    //std::cout<<"u_min = \n "<< u_min<<std::endl;
    //宣告一個2*1向量為 u_max 控制輸入最大值
    //max_v_為最大速度 max_w_為最大角速度
    Eigen::Vector2d u_max(max_v_,max_delta_);
    Eigen::Vector2d u_k_1;

    //宣告一個2*1向量為 u_k_1 上一刻的控制輸入值
    //du_p[0]為上一刻速度變化量 du_p[1]為上一刻角速度變化量
    if (!first)
    {
        u_k_1 = u_prev;
        first = true;
    }
    else
    {
        // 如果是第一次計算，則使用上一刻的控制輸入值
        //Eigen::Vector2d u_k_1(du_p[0], du_p[1]);
        u_k_1 = Eigen::Vector2d (du_p[0], du_p[1]);
    }

    

    //宣告一個2*1向量為 du_min 控制輸入變化量最小值
    //第一項為車速變化量 第二項為角速度變化量
    Eigen::Vector2d du_min(-0.2,-0.1);

    //宣告一個2*1向量為 du_max 控制輸入變化量最大值
    //第一項為速度變化量 第二項為角速度變化量
    Eigen::Vector2d du_max(0.2,0.1);

    // 宣告一個(2*控制時域,1)向量 
    // 將控制輸入最小值的向量擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    // U_min表示在每個控制時域中控制輸入的最小值
    // 在克羅內克積 中有一個 都為1長度為控制時域大小的向量 和 控制輸入最小值的向量
    Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),u_min);
    
    //宣告一個(2*控制時域,1)向量
    // 將控制輸入最大值的向量擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    // U_max表示在每個控制時域中控制輸入的最大值
    // 在克羅內克積 中有一個 都為1長度為控制時域大小的向量 和 控制輸入最大值的向量
    Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),u_max);
    
    //宣告一個(2*控制時域,1)向量
    // 將(車輛速度,車輛角速度)的向量擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    // U_r表示在每個控制時域中目標車速和目標角速度
    // 在克羅內克積 中有一個 元素都為1 長度為控制時域大小的向量 和 (車輛速度,車輛角速度)的向量
    Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),u_r);
    
    //宣告一個(2*控制時域,1)向量
    // 將上一刻控制輸入向量 擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    // U_k_1表示在每個控制時域中上一時刻的控制輸入
    // 在克羅內克積 中有一個 元素都為1 長度為控制時域大小的向量 和 上一刻控制輸入的向量
    Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),u_k_1);

    //宣告一個(2*控制時域,1)向量
    //將控制輸入變化量最小值的向量擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    //dU_min表示在每個控制時域中控制輸入變化量的最小值
    //在克羅內克積 中有一個 元素都為1 長度為控制時域大小的向量 和 控制輸入變化量最小值的向量
    Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),du_min);
    
    //宣告一個(2*控制時域,1)向量
    //將控制輸入變化量最大值的向量擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    //dU_max表示在每個控制時域中控制輸入變化量的最大值
    //在克羅內克積 中有一個 元素都為1 長度為控制時域大小的向量 和 控制輸入變化量最大值的向量
    Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),du_max);
    //std::cout<<"dU_max.size() = \n"<< dU_max.size() <<std::endl;

    //---------boundary finish---------




    //constraints start

    //宣告一個元素為1 (控制時域,控制時域)的矩陣
    //triangularView<Eigen::Lower>()保留下三角 剩下為0
    Eigen::MatrixXd temp = Eigen::MatrixXd::Ones(m_,m_).triangularView<Eigen::Lower>();
    
    //宣告一個動態矩陣 A_I
    //在克羅內克積 中有上一步計算得到的矩陣 和 2*2的單位矩陣
    Eigen::MatrixXd A_I = Eigen::kroneckerProduct(temp,Eigen::MatrixXd::Identity(dim_u,dim_u));

    //宣告一個動態矩陣P (2*2*控制時域,2*控制時域)
    //topRows為將上(2*控制時域)的行 賦值給A_I 上一步算出的克羅內克積結果
    //bottomRows 為將下(2*控制時域)的行 宣告為(2*控制時域2*控制時域)的單位矩陣
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2 * dim_u *m_,dim_u * m_);
    P.topRows(dim_u * m_) = A_I;
    P.bottomRows(dim_u * m_) = Eigen::MatrixXd::Identity(dim_u * m_,dim_u * m_);

    //宣告一動態向量lower (2*2*控制時域)
    //topRows為將上(2*控制時域)的行 
    //控制時域中控制輸入的最小值 - 制時域中上一時刻的控制輸入 - 控制時域中目標車速和目標角速度
    //bottomRows 為將下(2*控制時域)的行 
    //控制時域中控制輸入變化量的最小值
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);
    lower.topRows(dim_u * m_) = U_min - U_k_1 - U_r;
    lower.bottomRows(dim_u * m_) = dU_min;

    //宣告一動態向量upper (2*2*控制時域)
    //opRows為將上(2*控制時域)的行
    //控制時域中控制輸入的最大值 - 控制時域中上一時刻的控制輸入 - 控制時域中目標車速和目標角速度
    //bottomRows 為將下(2*控制時域)的行 
    //控制時域中控制輸入變化量的最大值
    Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);
    upper.topRows(dim_u * m_) = U_max - U_k_1 -U_r;
    upper.bottomRows(dim_u * m_) = dU_max;

    
    // solver.settings()->setVerbosity(false);
    // solver.settings()->setWarmStart(true);
    // solver.data()->setNumberOfVariables(dim_u * m_);
    // solver.data()->setNumberOfConstraints(2 * dim_u * m_);
    // solver.data()->setHessianMatrix(_convertTOSparseMatrix(H));
    // solver.data()->setGradient(g);
    // solver.data()->setLinearConstraintsMatrix(_convertTOSparseMatrix(P));
    // solver.data()->setLowerBound(lower);
    // solver.data()->setUpperBound(upper);

    // solver.initSolver();

    // // if (!solver.initSolver()) {
    // // ROS_ERROR("OSQP 初始化失敗");
    // // return Eigen::Vector2d::Zero();
    // // }


    // // 再下求解
    // solver.solveProblem();

    // Eigen::VectorXd solution = solver.getSolution();

    // （已經算好 H, g, P, lower, upper …）
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // **一次設定所有**  
    solver.data()->setNumberOfVariables(   n_var);
    solver.data()->setNumberOfConstraints( n_cons);
    solver.data()->setHessianMatrix(       _convertTOSparseMatrix(H));
    solver.data()->setGradient(            g);
    solver.data()->setLinearConstraintsMatrix(_convertTOSparseMatrix(P));
    solver.data()->setLowerBound(          lower);
    solver.data()->setUpperBound(          upper);

    // **只呼叫一次** initSolver
    if (!solver.initSolver()) {
      ROS_ERROR("OSQP 初始化失敗");
      return Eigen::Vector2d::Zero();
    }

    // 再呼叫一次 solveProblem
    if (!solver.solve()) {
    ROS_ERROR("OSQP 求解失敗");
    return Eigen::Vector2d::Zero();
    }


    auto solution = solver.getSolution();
    // …後面照常 return u  


    

    //real control
    Eigen::Vector2d u(solution[0] + du_p[0]+u_r[0],regularizeAngle(solution[1] + du_p[1] + u_r[1]));
    
    u_prev = u;

    return u;

}

Eigen::SparseMatrix<double> MPCPlanner_path::_convertTOSparseMatrix(Eigen::MatrixXd A)
{
    int row = A.rows();
    int col = A.cols();
    Eigen::SparseMatrix<double> A_s(row,col);
    
    for(int i = 0; i < row; i++)
    {
        for(int j = 0;j< col; j++ )
        {
            A_s.insert(i,j) = A(i,j);
        }
    }
    return A_s;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv ,"mpc_new");
    ros::NodeHandle nh;
    MPCPlanner_path mpc_path;
    mpc_path.MPCPlanner(&nh);
    ros::spin();
    return 0 ;


}