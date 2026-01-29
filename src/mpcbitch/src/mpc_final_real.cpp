#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "mpcbitch/mpc_final.h"
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
ros::Publisher trans_signal_pub;
ros::Publisher carla_control_pub;

static double prev_theta = 0.0;
int final_point;
bool trans = false; 
double off_rx_ = 50.985;
double off_ry_ = 39.385;
double u_delta  = 0;

double MPCPlanner_path::dist(const geometry_msgs::PoseStamped& node1,const geometry_msgs::PoseStamped& node2)
{
    return std::hypot(node1.pose.position.x - node2.pose.position.x ,node1.pose.position.y - node2.pose.position.y);
}


//std::floor 為向下取整數
// angle + M_PI 將原始角度angle+3.14 使原本的角度變成0～6.28
//然後在除以2*3.14將原始角度映射成相應的整數值 這個整數值表示原始角度在2*3.14的區段中的位置
//將上一步的整數值*2*3.14 得到原始角度在相應區段中的起始值 將原始角度映射到[0~2*3.14]
//將angle減去其對應區段的起始值 將角度調整成[-3.14~3.14]

double MPCPlanner_path::regularizeAngle(double angle)
{
    double a = fmod(angle, 2.0 * M_PI);
    if (a < -M_PI)
        a += 2.0 * M_PI;
    else if (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
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
    //double v_body =  std::cos(theta1) * vx + std::sin(theta1) * vy;
    double v_body = v_real;
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

    private_nh_.param<std::string>("save_filename", filename_, "/home/cyc/new_golf/src/mpc/mpcdata/real.csv");
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

        p_ = 40;//22   //30             //預測時間域
        m_ = 15;       //8             //控制時域*/


        //權重矩陣：用於懲罰在進行路徑追蹤控制時的狀態誤差[x,y,theta] 
        //宣告為 3*3 矩陣 [Q_ 0  0 ]
        //              [0  Q_ 0 ]
        //              [0  0  Q_]

        //Q_.resize(3,3);
        int dim_x = 4;
        int dim_u = 2;
        Q_.resize(dim_x, dim_x);         
        Q_.setZero();
        Q_(0,0) = 500; //200
        Q_(1,1) = 500; //200
        Q_(2,2) = 300; //180
        Q_(3,3) = 100;

        //權重矩陣：用於懲罰在進行路徑追蹤控制時的輸入誤差[v,w]
        //宣告 2*2 矩陣 [R_ 0 ]
        //             [0  R_]
        R_.resize(dim_u, dim_u);
        R_.setZero();
        R_(0,0) = 50;
        R_(1,1) = 20;  //50

        //採樣時間
        double controller_frequency = 10;
        d_t_ = 1/controller_frequency;
        
        
        //global_path_sub = nh_.subscribe("/global_waypoint_from_matlab",1000,&MPCPlanner_path::setPlan,this);
        global_path_sub = nh_.subscribe("array_topic",1000,&MPCPlanner_path::setPlan,this);  


        //car_pose_sub = nh_.subscribe("/mpc_new_pose",1000,&MPCPlanner_path::computelocalpath,this);
        car_pose_sub = nh_.subscribe("/odom",1000,&MPCPlanner_path::computelocalpath,this);
        // car_pose_sub = nh_.subscribe("/state_estimate",1000,&MPCPlanner_path::computelocalpath,this);
        // 用你原本的 nh_（或對應 NodeHandle）來訂閱
        vreal_sub = nh_.subscribe<std_msgs::Float64>("v_real", 1000, &MPCPlanner_path::vRealCallback, this);

        // std::string odom_topic;
        // nh_.param<std::string>("odom_topic", odom_topic, "/carla/ego_vehicle/odometry");
        // car_pose_sub = nh_.subscribe(odom_topic, 10, &MPCPlanner_path::computelocalpath, this);


        local_path_to_matlab_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/local_path",1000);


        mpc_result_pub = nh_.advertise<std_msgs::Float64MultiArray>("/mpc_result_test",1000);
        
        stop_signal_pub = nh_.advertise<std_msgs::Bool>("/stop_signal", 1000);
        trans_signal_pub = nh_.advertise<std_msgs::Bool>("/trans_signal", 1000);

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

// void MPCPlanner_path::setPlan(const std_msgs::Float64MultiArrayConstPtr &msg)
// {
//     global_path_x.clear();
//     global_path_y.clear();

//     const auto &v = msg->data;
//     const size_t n = v.size() / 2;          // (x,y) 配對數
//     global_path_x.reserve(n);
//     global_path_y.reserve(n);

//     for (size_t i = 0; i + 1 < v.size(); i += 2)
//     {
//         // 來自上游的是 map/RViz 座標
//         const double x_r = v[i];
//         const double y_r = v[i + 1];

//         // 轉成 CARLA world
//         const double x_c = y_r - off_ry_;   // x_c = y_r - OFF_RY
//         const double y_c = x_r - off_rx_;   // y_c = x_r - OFF_RX

//         global_path_x.emplace_back(x_c);
//         global_path_y.emplace_back(y_c);
//     }

//     ROS_INFO_STREAM("global_path size = " << global_path_x.size());
// }


void MPCPlanner_path::turnIndexCallback(const std_msgs::Int32::ConstPtr &msg)
{
    turn_index_ = msg->data;
    ROS_INFO("turn point:%d", turn_index_);
}

void MPCPlanner_path::vRealCallback(const std_msgs::Float64::ConstPtr &msg)
{
    v_real = msg->data;
    if(v_real == 0)
    {
        v_real = 0.000001;
    }
    ROS_INFO("v_real: %f", v_real);
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

void publishTransSignal(bool trans) 
{
    std_msgs::Bool trans_msg;
    trans_msg.data = trans;
    trans_signal_pub.publish(trans_msg);
    ROS_INFO("trans signal published: %s", trans ? "true" : "false");
}

void MPCPlanner_path::computelocalpath(const nav_msgs::OdometryConstPtr &msg)
{
    nav_msgs::Odometry base_odom;
    double L = 1.66;
    double v_body;
    double pxx;
    double pyy;
    double xx;
    double yy;

   

    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;

    theta1 = tf2::getYaw(msg->pose.pose.orientation);
    theta1 = (theta1- 1.571) ;

    // 車頭 → 車中心，往車尾方向退 0.4 m
    double offset = -0.4;  
    px = px + offset * cos(theta1);
    py = py + offset * sin(theta1);

    pxx = px;
    pyy = py;

    // const double turn_x = global_path_x[turn_index_];
    // const double turn_y = global_path_y[turn_index_];
    const double turn_x = -1.39362; //17.3483 //15.7773  //-0.437151  //-1.12856
    const double turn_y = 27.4522;  //24.861 //30.1295  //30.1307   //30.3111
    const double turn_theta = 0.0;

    double dist_to_turn = std::hypot(turn_x - px, turn_y - py);
    if (px > turn_x && !trans)
    {
        trans = true;
        publishTransSignal(true);
        xx = px;
        yy = py;
    }
    // if (dist_to_turn < 0.5)
    // {
    //     trans = true;
    //     publishTransSignal(true);
    // }

    if (trans)
    {
        px = 2.0*turn_x - px;
        py = 2.0*turn_y - py;  //直線倒車不用鏡射y
        //theta1 = 2.0*turn_theta-theta1;

    }
    
    //theta1 = msg->pose.pose.orientation.w;
    
    

    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;

    
    //omega = msg->twist.twist.angular.z;

    //double v_body =  std::cos(theta1) * vx + std::sin(theta1) * vy;
    if(!trans)
    {
        v_body = v_real;
    }
    else
    {
        v_body = -v_real;
    }

    //double v_body = v_real;
    omega = v_body * tan(u_delta) / L; 

    //double vt = std::hypot(vx,vy);
    double vt = std::fabs(v_body);
    
    //theta = regularizeAngle(theta1);
    theta = theta1;

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

        // 先決定本輪搜尋範圍
        size_t begin = (last_start_id_ < 0) ? 0 : last_start_id_;
        size_t window = 50; // 只往前看最多 50 個點，可依路徑密度調整
        size_t end   = std::min(begin + window, global_path_x.size());

        double nearest_distance = 999;
        int    candidate_id     = begin;

        for (size_t i = begin; i < end; ++i) 
        {
            double dist = hypot(global_path_x[i] - px,
                                global_path_y[i] - py);
            if (dist < nearest_distance) {
                nearest_distance = dist;
                candidate_id     = static_cast<int>(i);
            }
        }

        // 若距離太遠仍視為脫離路徑
        if (nearest_distance > 5.0) {
            candidate_id = -1;
        }

        // 保證不回跳
        if (last_start_id_ >= 0 && candidate_id < last_start_id_) {
            candidate_id = last_start_id_;
        }

        start_id        = candidate_id;
        last_start_id_  = start_id;   // 更新供下一回合使用

        org_wp_rearange_waypoint_x.clear();
        org_wp_rearange_waypoint_y.clear();


        ROS_INFO("starting_waypoint_for mpc is: %d", start_id);
        final_point ++;

        
        if(start_id < 0)
        {
            start_id = 0;
        }

        if (start_id >= global_path_x.size() - 6) //9
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
        //double epsi = regularizeAngle(theta - refHeading+ (gear == -1 ? M_PI : 0.0));

        if (fabs(cte) > 1) 
        {
            cte = 1 * (cte > 0 ? 1.0 : -1.0);
        }

        // 輸出調試信息
        ROS_INFO("Frenet: Projection=(%.3f, %.3f), d=%.3f, refHeading=%.3f, epsi=%.3f", 
                projection.x(), projection.y(), cte, refHeading, epsi);

        // ----- 使用 Frenet 誤差組成 MPC 的狀態誤差 -----
        // 此處的設計會依你的 MPC 模型而定，下面僅作為一個範例
        // 假設原來 s 為 (px, py, theta)，而 s_d 代表目標狀態，
        // 這裡我們直接用投影點與 refHeading 作為目標（你也可以根據 d 與 epsi 設計誤差向量）
        // 參數設定
        // ====== Velocity Planning with Mirror-Line Pause Logic ======
        static double alpha               = 0.15;   // EMA 平滑係數
        static double prev_v_ref          = vt;     // 上一時刻輸出速度
        static double last_v_ref          = vt;     // 計算 a_ref 用

        double v_min_ref           = min_v_; // 最低速度
        double v_max_ref           = max_v_; // 最高速度
        double curvature_threshold = 0.05;   // 曲率門檻
        double cte_threshold       = 0.1;    // CTE 門檻
        double cte_scale           = 0.2;    // CTE 影響比例
        double max_delta_v         = 0.02;   // max Δv/sampling

        // 鏡像線(turn point)狀態機
        enum VelState { CRUISE, APPROACH, DWELL, RESUME };
        static VelState vel_state      = CRUISE;
        static double  dwell_start_time = 0.0;
        double T_dwell = 2.0;  // 停留秒數
        int    K_slow  = 8;    // 接近前降速點數
        turn_index_ = 47;//92  //83


        // 當前時間與索引
        double now            = ros::Time::now().toSec();
        int    idx            = nearestIndex;
        int    turnIdx        = turn_index_;
        int    remain_to_turn = turnIdx - idx;

        // 1. 計算平均曲率 kappa_avg
        auto computeKappaAt = [&](int i)->double {
            int N = (int)global_path_x.size(); double ds = 0.3;
            if (i>0 && i<N-1) {
                double dx = (global_path_x[i+1] - global_path_x[i-1])/(2.0*ds);
                double dy = (global_path_y[i+1] - global_path_y[i-1])/(2.0*ds);
                double ddx = (global_path_x[i+1] - 2.0*global_path_x[i] + global_path_x[i-1])/(ds*ds);
                double ddy = (global_path_y[i+1] - 2.0*global_path_y[i] + global_path_y[i-1])/(ds*ds);
                double denom = std::pow(dx*dx + dy*dy, 1.5);
                if (denom > 1e-6) return (dx*ddy - dy*ddx)/denom;
            }
            return 0.0;
        };

        int M = 4; //5  mirror_back4  //6 mirror_back3  //3 all the other
        double kappa_sum = 0.0;
        for (int j = 0; j < M; ++j) 
        {
            int ii = std::min(nearestIndex + j, (int)global_path_x.size() - 1);
            kappa_sum += computeKappaAt(ii);
        }
        double kappa_avg = kappa_sum / double(M);

        // 2. 狀態機轉換
        switch (vel_state) {
        case CRUISE:
            if (remain_to_turn <= K_slow && remain_to_turn > 0) vel_state = APPROACH;
            break;
        case APPROACH:
            if (idx >= turnIdx) { vel_state = DWELL; dwell_start_time = now; }
            break;
        // case DWELL:
        //     if (now - dwell_start_time >= T_dwell) vel_state = RESUME;
        //     break;
        case DWELL:
            // 檢查速度是否已經夠小
            if (fabs(v_real) <= 0.02) 
            {
                if (now - dwell_start_time >= T_dwell) 
                {
                    vel_state = RESUME;
                }
            } 
            else 
            {
                // 如果速度還沒停下來，重置 dwell_start_time
                dwell_start_time = now;
            }
            break;
        case RESUME:
            if (prev_v_ref >= v_min_ref + 0.1 * (v_max_ref - v_min_ref)) vel_state = CRUISE;
            break;
        }

        // 3. 計算 CTE
        double cte_current = cte;

        // 4. 曲率/CTE 基本降速
        double v_allowed = v_max_ref;
        if (std::fabs(kappa_avg) > curvature_threshold) {
            double f = std::min(std::fabs(kappa_avg) / curvature_threshold, 1.0);
            v_allowed = v_max_ref - f * (v_max_ref - v_min_ref);
        }
        if (cte_current > cte_threshold) {
            double fcte = std::min((cte_current - cte_threshold) / cte_threshold, 1.0);
            v_allowed = std::max(v_min_ref, v_allowed * (1.0 - cte_scale * fcte));
        }

        // 5. 狀態機附加降速
        if (vel_state == APPROACH && remain_to_turn > 0) {
            double f = double(remain_to_turn) / double(K_slow);
            v_allowed *= std::clamp(f, 0.0, 1.0);
        } else if (vel_state == DWELL) {
            v_allowed = 0.0;
        }

        // 6. 終點前 N 點緩降（加速版）
        int N_end_slow = 8;
        int remain_pts  = (int)global_path_x.size() - nearestIndex;
        bool endpoint_phase = (remain_pts <= N_end_slow);
        if (endpoint_phase) {
            double ratio  = double(remain_pts) / double(N_end_slow);
            double factor = std::pow(ratio, 3.0);
            v_allowed *= std::clamp(factor, 0.0, 1.0);
        }

        // 7. EMA 平滑 + Δv 限幅 (方案A)
        double v_ref;
        double raw_v = alpha * v_allowed + (1.0 - alpha) * prev_v_ref;
        // 動態下限: CRUISE/RESUME 階段且非終點才用 v_min_ref，下限可降至 0
        bool lower_phase = ((vel_state == CRUISE || vel_state == RESUME) && !endpoint_phase);
        double lower = lower_phase ? v_min_ref : 0.0;
        raw_v = std::clamp(raw_v, lower, v_max_ref);
        double dv = std::clamp(raw_v - prev_v_ref, -max_delta_v, max_delta_v);
        v_ref = prev_v_ref + dv;
        //v_ref = 0.4; //debug
        prev_v_ref = v_ref;

        if (nearestIndex >= turnIdx) 
        {
            v_ref      = std::min(v_ref, v_min_ref);
            prev_v_ref = v_ref;
        }



        ROS_INFO("nearest=%d  turn=%d  remain=%d  state=%d", 
            nearestIndex, turn_index_, turn_index_ - nearestIndex, vel_state);


        // 3. 用平均曲率做前饋轉向
        double delta_d = std::atan(kappa_avg * L);




        Eigen::Vector4d s(px, py, theta, v_body);
        Eigen::Vector4d s_d(projection.x(), projection.y(), refHeading, v_ref);
        Eigen::Vector2d p0(global_path_x[nearestIndex-1], global_path_y[nearestIndex-1]);
        Eigen::Vector2d p1(global_path_x[nearestIndex],   global_path_y[nearestIndex]);


        

        // 1) 計算參考加速度 a_ref = (v_ref - prev_v_ref_before)/dt
        
        double a_ref = (v_ref - last_v_ref) / d_t_;
        last_v_ref   = v_ref;


        // 2) 用加速度＋角度當作參考輸入
        Eigen::Vector2d u_r(a_ref, delta_d);


        Eigen::Vector2d u = _mpcControl(s,s_d,u_r,du_p_);

        double u_a = u[0];

        u_delta = anglarRegularization(base_odom,u[1]);

  

        
        if(kappa_avg <= 0.0002 )
        {
            if (fabs(cte) < 0.08 && fabs(epsi) < 0.01) 
            {
                // 認為誤差可忽略，直接把方向量清零
                u_delta = 0.0;
            }
        }
        
        du_p_ = Eigen::Vector2d(u_a - u_r[0],regularizeAngle(u_delta - u_r[1]));
        //double L = 1.66;
        // 先定義車輛參數，假設 L 為車輛軸距，令 l_r = L/2 (你可以根據實際數值調整)
        double l_r = 0.4;

        // 根據前輪轉角 u_r[1] 計算側滑角 β
        double beta_1 = atan((l_r / L) * tan(u_r[1]));

        
        std_msgs::Float64MultiArray cmd_to_matlab;
        std_msgs::Float64MultiArray cte_test;

        carla_msgs::CarlaEgoVehicleControl control_cmd;
        control_cmd.throttle = u_a;
        control_cmd.steer = std::min(std::max(u_delta / 0.4, -1.0), 1.0);
        control_cmd.brake = 0.0;
        control_cmd.hand_brake = false;
        control_cmd.reverse = false;

        carla_control_pub.publish(control_cmd);
        
        cmd_to_matlab.data.clear();
        cte_test.data.clear();
        
        cmd_to_matlab.data.emplace_back(u_a);
        if(!trans)
        {
            cmd_to_matlab.data.emplace_back(u_delta);
        }
        else
        {
            cmd_to_matlab.data.emplace_back(-u_delta);
        }

        cte_test.data.emplace_back(cte);


        ROS_INFO("-----------------------------------------------");

        std::cout<<"acceration = "<< u_a <<std::endl;
        std::cout<<"steeringangle = "<< u_delta <<std::endl;
        std::cout<<"theta = "<< theta <<std::endl;
        std::cout<<" lateral error = \n"<< cte <<std::endl;

        ros::Duration gan(0.001);

        
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
        ofs << "u_a,v_real,v_ref,u_delta,delta_d,px,py,pxx,pyy,theta,cte,cte_real,epsi,vx,vy,kappa,beta" << std::endl;
        header_written = true;
        }

        ofs << std::fixed << std::setprecision(4)
        << u_a << ","<< v_real << "," << v_ref << "," << u_delta << ","<< delta_d << "," << px << "," << py << ","
        << pxx << ","<< pyy << ","<< theta << "," << cte << "," << cte_real << ","<< epsi << "," << vx << ","
        << vy <<","<<kappa_avg<< "," << beta_1<<std::endl;

        ofs.close(); // 確保每次操作後關閉文件
    }
    

    //車輛換道實驗
    //private_nh_.param<std::string>("save_filename",filename_,std::string("/home/king/mpc/mpcdata"));
    
}

Eigen::Vector2d MPCPlanner_path::_mpcControl(Eigen::Vector4d s, Eigen::Vector4d s_d,Eigen::Vector2d u_r, Eigen::Vector2d du_p)
{
    int dim_u = 2;
    int dim_x = 4;
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
    x(0) = s(0) - s_d(0);
    x(1) = s(1) - s_d(1);
    x(2) = regularizeAngle(s(2) - s_d(2));
    x(3) = s(3) - s_d(3);           // 新增：速度誤差 v–v_ref
    x.tail(dim_u) = du_p; 



    //original state matrix
    //宣告一個3*3的單位矩陣 系統的原始狀態轉移矩陣
    //u_r為一2*1向量 第一項為車輛速度(vx及vy) 第二項為前輪轉角
    //A_o(0,2)為矩陣第(0,2)項 為車輛速度*cos(車輛目標方向角)*取樣時間
    //A_o(1,2)為矩陣第(1,2)項 為車輛速度*sin(車輛目標方向角)*取樣時間
    Eigen::Matrix4d A_o = Eigen::Matrix4d::Identity();
    // 位置與朝向更新（同樣線性化）  
    A_o(0,2) = -s_d(3)*sin(s_d[2])*d_t_;  
    A_o(1,2) =  s_d(3)*cos(s_d[2])*d_t_;  
    // 速度 v_{k+1}=v_k + a_k*d_t_，所以 A_o(3,3)=1，B_o 會負責加速度影響

    Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x,dim_u);
    // 加速度 a 對位置、朝向沒直接影響  
    B_o(0,0) = cos(s_d[2] + beta) * d_t_;  
    B_o(1,0) = sin(s_d[2] + beta) * d_t_;
    B_o(3,0) = d_t_;                    // v_{k+1}=v_k + a_k·d_t_  
    // 方向盤角度對 θ 的影響（保留原轉角模型）  
    B_o(2,1) = d_t_ * s_d(3) / (L * pow(cos(u_r[1]),2));
    //std::cout<<"A_o = \n"<< A_o <<std::endl;



    //state matrix(5*5)
    //宣告一個5*5的零矩陣 系統的狀態轉移矩陣
    //topLeftCorner(dim_x,dim_x)    為左上角3*3的矩陣為A_o
    //topRightCorner(dim_x,dim_u)   為右上角3*2的矩陣為B_o
    //bottomLeftCorner(dim_u,dim_x) 為左下角的2*3的矩陣為零矩陣
    //bottomRightCorner(dim_u,dim_u)為右下角的2*2的矩陣為單位矩陣
    //建立系統模型

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x+dim_u, dim_x+dim_u);
    // block 清成 0 已經在 Zero() 做過了
    A.topLeftCorner(   dim_x, dim_x).noalias()      = A_o;
    A.topRightCorner(  dim_x, dim_u).noalias()      = B_o;
    A.bottomLeftCorner(dim_u, dim_x).setZero();     // ← 無參數版
    A.bottomRightCorner(dim_u,dim_u).noalias()      = Eigen::MatrixXd::Identity(dim_u,dim_u);


    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x+dim_u, dim_u);
    B.topLeftCorner(   dim_x, dim_u).noalias()      = B_o;
    B.bottomLeftCorner(dim_u, dim_u).noalias()      = Eigen::MatrixXd::Identity(dim_u,dim_u);

    //output matrix (3*5)
    //宣告一個3*5的零矩陣 系統的控制輸出矩陣
    //topLeftCorner(dim_x,dim_x) 為左上角3*3的矩陣為單位矩陣
    //topRightCorner(dim_x,dim_u)為右上角3*2的矩陣為零矩陣
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x+dim_u);
    C.topLeftCorner(  dim_x, dim_x).noalias()       = Eigen::MatrixXd::Identity(dim_x, dim_x);
    C.topRightCorner( dim_x, dim_u).setZero();      // ← 清成 0

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
        S_x.middleRows(dim_x * i, dim_x).noalias() = C * A_pow(i + 1);
    }

    //mpc control matrix (3p * 2m) 模型預測控制的控制矩陣
    //宣告ㄧ(3*預測時域,2*控制時域)的零矩陣 
    //如果預測時域大於等於控制時域時 透過block功能從(0,0,3,2) 從第0行0列開始的3*2矩陣寫入C*A矩陣的次方*B
    //這裡用到兩個for迴圈 所以這裡執行的順序是 
    //i = 0,j = 0 ; i = 0,j =1; i = 1,j = 0; 
    //如果預測時域小於控制時域時    透過block功能寫入零矩陣
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);

    // 只填 j<=i 的部分，其它留 0
    for(int i = 0; i < p_; i++){
      for(int j = 0; j <= std::min(i, m_-1); j++){
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u)
           .noalias() = C * A_pow(i - j) * B;
      }
    }
    //optimization start
    // (3p * 1 )
    //宣告一(3*預測時域)的零向量
    Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x*p_);

    // (3p * 3p)
    //宣告一動態矩陣為 Q 3p*3p 的矩陣 計算模型預測控制的狀態誤差權重矩陣
    //kroneckerProduct為計算兩個矩陣的克羅內克積
    //在克羅內克積中有一個(預測時域,預測時域)的單位矩陣 乘 狀態誤差權重矩陣
    Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_,p_),Q_);
    // (2m *2m)
    //宣告一動態矩陣為 R 為2m*2m的矩陣  計算模型預測控制的控制誤差權重矩陣
    //kroneckerProduct為計算兩個矩陣的克羅內克積
    //在克羅內克積中有一個(控制時域,控制時域)的單位矩陣 乘 控制誤差權重矩陣
    Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_,m_),R_);

    //宣告一動態矩陣為 H 計算模型預測控制的 Hessian矩陣
    //(2m*2m)
    int n_var  = dim_u * m_;
    int n_cons = 2 * dim_u * m_;
    Eigen::MatrixXd H = S_u.transpose() * Q * S_u + R;
    H = (H + H.transpose()) * 0.5;      // 保證對稱
    H.diagonal().array() += 1e-6;      // 加 tiny damping，避免負或零特徵值


    //宣告一動態向量為 g 計算模型預測控制的梯度向量
    //(2m*1)
    Eigen::VectorXd g = S_u.transpose() * Q * (S_x * x - Yr);

    //---------boundary start---------

    //宣告一個2*1向量為 u_min 控制輸入最小值
    //min_v_為最小速度 min_w_為最小角速度
    //Eigen::Vector2d u_min(min_v_,min_delta_);
    //std::cout<<"u_min = \n "<< u_min<<std::endl;
    //宣告一個2*1向量為 u_max 控制輸入最大值
    //max_v_為最大速度 max_w_為最大角速度
    //Eigen::Vector2d u_max(max_v_,max_delta_);
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

    // 1) 真正的加速度上下限
    double min_a = -0.2;      // or 從 param 讀 max_acc
    double max_a = +0.2;
    Eigen::Vector2d u_min(min_a,      min_delta_);
    Eigen::Vector2d u_max(max_a,      max_delta_);

    // 2) 真正的加速度增量上下限
    double max_da = 0.2;      // or 從 param 讀 max_acc_inc
    Eigen::Vector2d du_min(-max_da,  -max_delta_inc_);
    Eigen::Vector2d du_max(+max_da,  +max_delta_inc_);

    // 3) Expand 到 m_ 個控制時域
    Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);
    Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);

    Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);
    Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);

    // 4) lower/upper constraints
    
    // //宣告一個(2*控制時域,1)向量
    // // 將(車輛速度,車輛角速度)的向量擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    // // U_r表示在每個控制時域中目標車速和目標角速度
    // // 在克羅內克積 中有一個 元素都為1 長度為控制時域大小的向量 和 (車輛速度,車輛角速度)的向量
    Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),u_r);
    
    // //宣告一個(2*控制時域,1)向量
    // // 將上一刻控制輸入向量 擴增為一個長度為控制時域的向量 為了與模型預測控制算法中的矩陣相乘
    // // U_k_1表示在每個控制時域中上一時刻的控制輸入
    // // 在克羅內克積 中有一個 元素都為1 長度為控制時域大小的向量 和 上一刻控制輸入的向量
    Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_),u_k_1);

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
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2*dim_u*m_, dim_u*m_);
    // 不要 block = … 而用 noalias
    P.topRows( dim_u*m_).noalias() = A_I;
    P.bottomRows(dim_u*m_).noalias() = Eigen::MatrixXd::Identity(dim_u*m_, dim_u*m_);

    //宣告一動態向量lower (2*2*控制時域)
    //topRows為將上(2*控制時域)的行 
    //控制時域中控制輸入的最小值 - 制時域中上一時刻的控制輸入 - 控制時域中目標車速和目標角速度
    //bottomRows 為將下(2*控制時域)的行 
    //控制時域中控制輸入變化量的最小值
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);
    lower.topRows(dim_u*m_)    = U_min - U_k_1 - U_r;
    lower.bottomRows(dim_u*m_) = dU_min;

    //宣告一動態向量upper (2*2*控制時域)
    //opRows為將上(2*控制時域)的行
    //控制時域中控制輸入的最大值 - 控制時域中上一時刻的控制輸入 - 控制時域中目標車速和目標角速度
    //bottomRows 為將下(2*控制時域)的行 
    //控制時域中控制輸入變化量的最大值
    Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);
    upper.topRows(dim_u*m_)    = U_max - U_k_1 - U_r;
    upper.bottomRows(dim_u*m_) = dU_max;

    // // 再下求解
    // （已經算好 H, g, P, lower, upper …）
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // **一次設定所有**  
    solver.data()->setNumberOfVariables(n_var);
    solver.data()->setNumberOfConstraints(n_cons);
    solver.data()->setHessianMatrix(_convertTOSparseMatrix(H));
    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(_convertTOSparseMatrix(P));
    solver.data()->setLowerBound(lower);
    solver.data()->setUpperBound(upper);

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