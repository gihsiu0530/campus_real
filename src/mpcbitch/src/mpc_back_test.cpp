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
#include <algorithm>
#include <cmath>


std::string filename_;
ros::Publisher stop_signal_pub;
ros::Publisher carla_control_pub;

static double prev_theta = 0.0;
static bool reversed_mode = false;
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
    double v_body =  std::cos(theta1) * vx + std::sin(theta1) * vy;
    double vt = v_body;
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

    private_nh_.param<std::string>("save_filename", filename_, "/home/cyc/campus_ws/mpcdata/real.csv");
    private_nh_.param("use_state_projection", use_state_projection_, true);   //false 關閉
    private_nh_.param("state_projection_delay", state_projection_delay_, 0.2);
    ROS_INFO("Data will be saved to: %s", filename_.c_str());
    ROS_INFO("State projection: %s, delay=%.3f sec", use_state_projection_ ? "on" : "off", state_projection_delay_);

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

        p_ = 45;//40   //30             //預測時間域
        m_ = 15;//15       //8             //控制時域*/


        //權重矩陣：用於懲罰在進行路徑追蹤控制時的狀態誤差[x,y,theta,v] 

        int dim_x = 4;
        int dim_u = 2;
        Q_.resize(dim_x, dim_x);         
        Q_.setZero();
        Q_(0, 0) = 500; // 500  //600  // 縱向位置誤差 
        Q_(1, 1) = 500; // 400  // CTE
        Q_(2, 2) = 400; // 400  //航向角誤差
        Q_(3, 3) = 100;                //速度誤差

        // 權重矩陣：用於懲罰在進行路徑追蹤控制時的輸入誤差[v,w]
        // 宣告 2*2 矩陣 [R_ 0 ]
        //              [0  R_]
        R_.resize(dim_u, dim_u);
        R_.setZero();
        R_(0, 0) = 50;       //加速度變化率
        R_(1, 1) = 30;  //50 //方向盤變化率

        //採樣時間
        double controller_frequency = 10;
        d_t_ = 1/controller_frequency;
        
        global_path_sub = nh_.subscribe("array_topic",1000,&MPCPlanner_path::setPlan,this);  
        car_pose_sub = nh_.subscribe("/odom",1000,&MPCPlanner_path::computelocalpath,this);

        local_path_to_matlab_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/local_path",1000);


        mpc_result_pub = nh_.advertise<std_msgs::Float64MultiArray>("/mpc_result_test",1000);
        
        stop_signal_pub = nh_.advertise<std_msgs::Bool>("/stop_signal", 1000);

        start_id_pub = nh_.advertise<std_msgs::Int32>("/start_id", 1000);
        vreal_sub = nh_.subscribe<std_msgs::Float64>("v_real", 1000, &MPCPlanner_path::vRealCallback, this);
        max_v_sub = nh_.subscribe("/max_v", 10, &MPCPlanner_path::maxVCallback, this);
        max_v_inc_sub = nh_.subscribe("/max_v_inc", 10, &MPCPlanner_path::maxVIncCallback, this);
        turn_sub = nh_.subscribe("turn_index", 10, &MPCPlanner_path::turnIndexCallback, this);
        steer_sub = nh_.subscribe("/steering_sensor", 10, &MPCPlanner_path::steerCallback, this);

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
    turn_index_ = 10000; //66 30               調超大讓他不倒車
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

void MPCPlanner_path::vRealCallback(const std_msgs::Float64::ConstPtr &msg)
{
    v_real = msg->data;
    ROS_INFO("v_real: %f", v_real);
    if(v_real == 0) 
    {
        v_real = 0.0001;  // 避免除以零
    }
}

void MPCPlanner_path::steerCallback(const std_msgs::Float32::ConstPtr &msg)
{
    steer_real = -(msg->data / 19.8) * (M_PI / 180.0);
    ROS_INFO("steer_real: %f", steer_real);
}
    
void publishStopSignal(bool stop) 
{
    std_msgs::Bool stop_msg;
    stop_msg.data = stop;
    stop_signal_pub.publish(stop_msg);
    ROS_INFO("Stop signal published: %s", stop ? "true" : "false");
}

void MPCPlanner_path::computelocalpath(const nav_msgs::OdometryConstPtr &msg)
{

    nav_msgs::Odometry base_odom;
    double L = 1.66;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    
    
    theta1 = tf2::getYaw(msg->pose.pose.orientation);
    theta1 = theta1 - M_PI/3;
    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
    

    double v_body = v_real;

    double offset = -0.5;
    px = px + offset * cos(theta1);
    py = py + offset * sin(theta1);

    omega = v_body * tan(u_delta) / L;
    //double vt = std::hypot(vx,vy);
    double vt = v_body;
    
    theta = regularizeAngle(theta1);

    // Compensate actuator/sensing delay by projecting state to the future.
    double px_proj = px;
    double py_proj = py;
    double theta_proj = theta;
    const double delay_s = std::max(0.0, state_projection_delay_);

    if (use_state_projection_ && delay_s > 1e-4)
    {
        double delta_proj = std::isfinite(steer_real) ? steer_real : u_delta;
        delta_proj = std::clamp(delta_proj, -max_delta_, max_delta_);
        double omega_proj = v_body * std::tan(delta_proj) / L;

        if (std::fabs(omega_proj) < 1e-6)
        {
            px_proj += v_body * std::cos(theta_proj) * delay_s;
            py_proj += v_body * std::sin(theta_proj) * delay_s;
        }
        else
        {
            double theta_next = theta_proj + omega_proj * delay_s;
            px_proj += (v_body / omega_proj) * (std::sin(theta_next) - std::sin(theta_proj));
            py_proj += -(v_body / omega_proj) * (std::cos(theta_next) - std::cos(theta_proj));
            theta_proj = theta_next;
        }

        theta_proj = regularizeAngle(theta_proj);
    }
    
    next_px = px + vt*cos(theta)*d_t_;
    next_py = py + vt*sin(theta)*d_t_;

    base_odom.twist.twist.linear.x = v_body;
    base_odom.twist.twist.linear.y = 0;
    base_odom.twist.twist.angular.z = omega;
    
    double delta = atan(omega * L / vt);

    std::cout<<"car_x = "<< px <<std::endl;
    std::cout<<"car_y = "<< py <<std::endl;
    std::cout<<"car_theta = "<< theta<<std::endl;
    std::cout<<"car_vel_x = "<<vx<<std::endl;
    std::cout<<"car_vel_y = "<< vy<<std::endl;
    std::cout<<" delta = "<< delta  <<std::endl;
    std::cout<<" projected_x = "<< px_proj <<std::endl;
    std::cout<<" projected_y = "<< py_proj <<std::endl;
    std::cout<<" projected_theta = "<< theta_proj <<std::endl;


    double nearest_distance = 999;                                       // declare the double variable "nearest_distance" is 100

    if (global_path_x.size()!=0)
    {

        // === 參數 ===
        const int    GUARD  = 3;        // 轉折點前保留幾個索引，避免沾到倒退段
        const size_t WINDOW = 200;       // 單次最近點搜尋窗口（依路徑密度調整）

        // === 投影/模式狀態（全域或成員變數，請視你的架構放置） ===
        
        static int  proj_idx      = 0;   // 上一次投影索引

        // === 先決定本輪搜尋範圍 ===
        size_t begin = (last_start_id_ < 0) ? 0 : (size_t)last_start_id_;
        size_t end   = std::min(begin + WINDOW, global_path_x.size());

        // 未進入倒退：把 end 限制在 turn_index_-GUARD 以前
        if (!reversed_mode) {
            size_t cap = (turn_index_ > GUARD) ? (size_t)(turn_index_ - GUARD) : 0;
            if (cap < end) end = cap;                 // 上限：最多掃到 turn_index_-GUARD
            if (begin > end) begin = end;             // 防呆：避免 begin > end
        } else {
            // 進入倒退：只從 turn_index_ 之後開始找
            size_t cap = std::min((size_t)turn_index_, global_path_x.size());
            if (begin < cap) begin = cap;
            if (end   < begin) end = std::min(begin + WINDOW, global_path_x.size());
        }

        // === 最近點搜尋（限於上述 begin..end 範圍） ===
        double nearest_distance = 1e9;
        int    candidate_id     = (int)begin;

        for (size_t i = begin; i < end; ++i) {
            double dx = global_path_x[i] - px_proj;
            double dy = global_path_y[i] - py_proj;
            double dist = std::hypot(dx, dy);
            if (dist < nearest_distance) {
                nearest_distance = dist;
                candidate_id     = (int)i;
            }
        }

        // 若距離太遠仍視為脫離路徑
        if (nearest_distance > 5.0) candidate_id = -1;

        // 保證不回跳（僅在同一段內生效）
        if (last_start_id_ >= 0 && candidate_id >= 0 && candidate_id < last_start_id_) {
            candidate_id = last_start_id_;
        }

        start_id       = candidate_id;
        last_start_id_ = start_id;  // 供下一回合使用

        // === 切換時鎖定到轉折點（避免剛切換又被拉回前進段） ===
        if (!reversed_mode && proj_idx >= turn_index_) {
            // 在你啟動倒退的那一刻才把 reversed_mode 置為 true（外部條件判斷）
            proj_idx = turn_index_;  // 鎖到轉折點
        }
        if (reversed_mode && proj_idx < turn_index_) {
            proj_idx = turn_index_;
        }

        // 清空/重排路徑容器（原程式保持）
        org_wp_rearange_waypoint_x.clear();
        org_wp_rearange_waypoint_y.clear();


        ROS_INFO("starting_waypoint_for mpc is: %d", start_id);
   
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
        Eigen::Vector2d vehiclePos(px_proj, py_proj);

        // 2. 在全局路徑中找出車輛位置的投影點（用相鄰點線段近似參考曲線）
        double minDistance = 1e6;
        int nearestIndex = -1;
        static int last_nearest_index = 0;
        Eigen::Vector2d projection;  // 投影點
        double refHeading = 0.0;     // 局部路徑切線方向
        Eigen::Vector2d p2(0, 0);

        size_t Y = global_path_x.size();
        if (Y < 2) return;  // 防呆：沒有線段可掃

        // 依速度決定搜尋範圍（只改這段與 for 迴圈的邊界）
        size_t i_begin = 0;
        size_t i_end   = Y - 1;          // 會掃描 [i_begin, i_end) 的線段 i..i+1

        const double V_EPS = 0.05;       // 死區，避免 0 附近抖動
        if (vt > V_EPS) 
        {
            // 前進：只找 turn_index_ 之前的線段
            i_begin = 0;
            i_end   = (turn_index_ > 0) ? (size_t)turn_index_ : 0;  // 不含 turn_index_ 本身 (i_end 是不含上限)
        } else if (vt < -V_EPS) {
            // 倒退：只找 turn_index_ 之後的線段
            i_begin = std::min((size_t)turn_index_, Y - 1);
            i_end   = Y - 1;             // 掃到最後一條線段 (N-2)
        }
        // 防呆：確保有範圍可掃，否則直接返回或保留原 nearestIndex
        if (i_end <= i_begin) 
        {
            i_begin = 0;
            i_end   = Y - 1;
        }

        for (size_t i = i_begin; i < i_end; ++i) 
        {      // 只有這行的邊界改了
            Eigen::Vector2d p1(global_path_x[i],   global_path_y[i]);
            Eigen::Vector2d p2(global_path_x[i+1], global_path_y[i+1]);

            Eigen::Vector2d v = p2 - p1;
            double denom = v.squaredNorm();
            double t = 0.0;
            if (denom > 1e-9) {
                t = (vehiclePos - p1).dot(v) / denom;
                if (t < 0) t = 0;
                if (t > 1) t = 1;
            }
            Eigen::Vector2d proj = p1 + t * v;
            double dist = (vehiclePos - proj).norm();
            if (dist < minDistance) {
                minDistance = dist;
                projection  = proj;
                refHeading  = std::atan2(v.y(), v.x());
                nearestIndex = (int)i;
            }
        }

        if (nearestIndex > last_nearest_index)
        {
            last_nearest_index = nearestIndex;
        }
        // 3. 計算橫向誤差 d（正負依據車輛在參考曲線哪側
        double pi_offset = 0.0;
        if (reversed_mode && v_body < 0)
        {
            pi_offset = M_PI;
        }
        double refHeading_adj = refHeading + pi_offset;
        //double refHeading_adj = refHeading + (reversed_mode ? M_PI : 0.0);
        Eigen::Vector2d refDir(cos(refHeading_adj), sin(refHeading_adj));
        Eigen::Vector2d errorVec = vehiclePos - projection;
        double cte_real = 0.0;
        double cte = errorVec.norm();
        double sign = (refDir.x() * errorVec.y() - refDir.y() * errorVec.x()) >= 0 ? 1.0 : -1.0;
        cte *= sign;

        cte_real = cte;

        // ===== 反車鎖存與負速參考 =====
        double epsi = regularizeAngle(theta_proj - refHeading_adj);
        if(abs(epsi) > 3)
        {
            epsi = 0;
        }

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
        static double alpha               = 0.1;   // EMA 平滑係數
        static double prev_v_ref          = vt;     // 上一時刻輸出速度
        static double last_v_ref          = vt;     // 計算 a_ref 用

        double v_min_ref = reversed_mode ? min_v_reverse_  : min_v_forward_;
        double v_max_ref = reversed_mode ? max_v_reverse_  : max_v_forward_;
        // double curvature_threshold = 0.02;   // 0.05 曲率門檻
        // double cte_threshold       = 0.1;    // CTE 門檻
        // double cte_scale           = 0.2;    // CTE 影響比例
        double max_delta_v         = 0.04;   // max Δv/sampling

        // 鏡像線(turn point)狀態機
        enum VelState { CRUISE, APPROACH, DWELL, RESUME };
        static VelState vel_state      = CRUISE;
        static double  dwell_start_time = 0.0;
        double T_dwell = 2.0;  // 停留秒數
        int    K_slow  = 8;    // 接近前降速點數
        //turn_index_ = 52;//47
        double v_ref;
        double kappa_avg_steering;
        bool back = false;

        // 當前時間與索引
        double now            = ros::Time::now().toSec();
        int    idx            = nearestIndex;
        int    turnIdx        = turn_index_;
        int    remain_to_turn = turnIdx - idx;
        
        auto computeKappaAt = [&](int i)->double
        {
            const int N = (int)global_path_x.size();
            if (N < 3) return 0.0;
            if (i <= 0) i = 1;
            if (i >= N-1) i = N-2;

            // 當地前/後段弧長
            double dx_f = global_path_x[i+1] - global_path_x[i];
            double dy_f = global_path_y[i+1] - global_path_y[i];
            double dx_b = global_path_x[i]   - global_path_x[i-1];
            double dy_b = global_path_y[i]   - global_path_y[i-1];

            double ds_f = std::hypot(dx_f, dy_f);
            double ds_b = std::hypot(dx_b, dy_b);
            double ds   = 0.5 * (ds_f + ds_b);
            if (ds < 1e-6) return 0.0;  // 避免除以 0

            // 仍採「等距中心差分」的簡化，但 ds 用當地弧長（很小改動、已顯著穩定）
            double dx  = (global_path_x[i+1] - global_path_x[i-1]) / (2.0 * ds);
            double dy  = (global_path_y[i+1] - global_path_y[i-1]) / (2.0 * ds);
            double ddx = (global_path_x[i+1] - 2.0*global_path_x[i] + global_path_x[i-1]) / (ds*ds);
            double ddy = (global_path_y[i+1] - 2.0*global_path_y[i] + global_path_y[i-1]) / (ds*ds);

            double denom = std::pow(dx*dx + dy*dy, 1.5);
            if (denom < 1e-9) return 0.0;

            double kappa = (dx*ddy - dy*ddx) / denom;

            // 異常值保護（依你的車輛極限曲率調整，例如 ±1 m^-1）
            if (!std::isfinite(kappa)) return 0.0;
            if (kappa >  1.0) kappa =  1.0;
            if (kappa < -1.0) kappa = -1.0;
            return kappa;
        };

        //----------------------------------------------------
        // 在曲率計算完或建立路徑切線角之後加入這段
        std::vector<double> theta_ref(global_path_x.size(), 0.0);
        for (size_t i = 1; i + 1 < global_path_x.size(); ++i) 
        {
            double dx = global_path_x[i+1] - global_path_x[i-1];
            double dy = global_path_y[i+1] - global_path_y[i-1];
            theta_ref[i] = std::atan2(dy, dx);
        }

        // Unwrap：確保相鄰角度連續，避免 ±π 跳變
        for (size_t i = 1; i < theta_ref.size(); ++i) 
        {
            double diff = theta_ref[i] - theta_ref[i - 1];
            if (diff >  M_PI) theta_ref[i] -= 2.0 * M_PI;
            if (diff < -M_PI) theta_ref[i] += 2.0 * M_PI;
        }

//=================================================================
    int M = 8 ; // 4  mirror_back4  //12 for 0.2m間隔  //5 for 0.5m間隔
               // //轉彎的時候看後N個路徑點的方向盤角度做角度平均
    double kappa_sum = 0.0;

    int steer_lookahead = 1; // p4 2

    for (int j = 0; j < M; ++j) {
      int ii = std::min(nearestIndex + steer_lookahead + j,
                        (int)global_path_x.size() - 1);
      kappa_sum += computeKappaAt(ii);
    }
    kappa_avg_steering = kappa_sum / double(M);
    //=================================================================


    // ==========================================
    // --- 依照「曲率大小」動態調整看前方的點數 M ---
    // ==========================================
    
    // int steer_lookahead = 1; // 往前預視的起始偏移量
    
    // // 1. 先探測車頭前方的「局部最大曲率」
    // // 看前方最近 3 個點，抓出最大的彎度(取絕對值)，避免單一雜訊干擾
    // double local_max_kappa = 0.0;
    // for (int k = 0; k < 3; ++k) {
    //     int check_idx = std::min(nearestIndex + steer_lookahead + k, (int)global_path_x.size() - 1);
    //     local_max_kappa = std::max(local_max_kappa, std::fabs(computeKappaAt(check_idx)));
    // }

    // // 2. 設定 M 的「天花板」與「地板」 (這兩個數字你可以自由微調)
    // int M_max = 13;  // 直道時，最多看 18 個點 (看很遠，保持直行穩定)
    // int M_min = 6;   // 急彎時，最少看 5 個點 (看很近，緊貼彎角不提早回正)
    
    // // 設定多彎算「急彎」(例如曲率達到 0.15 rad/m 就視為最急的彎)
    // double kappa_threshold = 0.18; 
    
    // // 計算彎度比例 (0.0 代表全直，1.0 代表達到急彎門檻)
    // double kappa_ratio = std::min(local_max_kappa / kappa_threshold, 1.0);
    
    // // 3. 線性內插計算當下的「動態點數 M」
    // // 邏輯：曲率越大(kappa_ratio 越接近 1)，M 會被扣得越多(變小)；曲率越小，M 越大
    // int dynamic_M = M_max - static_cast<int>(std::round(kappa_ratio * (M_max - M_min)));

    // // 防呆機制，確保 dynamic_M 絕對落在 M_min 到 M_max 之間
    // dynamic_M = std::max(M_min, std::min(M_max, dynamic_M));

    // // 4. 依照算出來的 dynamic_M，去加總未來的方向盤曲率
    // double kappa_sum = 0.0;
    // int valid_points_count = 0;

    // for (int j = 0; j < dynamic_M; ++j) {
    //   int ii = std::min(nearestIndex + steer_lookahead + j, (int)global_path_x.size() - 1);
      
    //   kappa_sum += computeKappaAt(ii);
    //   valid_points_count++;
      
    //   // 如果已經讀到陣列最後一個點，提早結束
    //   if (ii == (int)global_path_x.size() - 1) {
    //     break;
    //   }
    // }

    // // 防呆除以零
    // kappa_avg_steering = kappa_sum / std::max(1.0, double(valid_points_count));
    // ==========================================



    // ==========================================
    // --- 修改後的設計：看前方固定「物理距離」 ---2026/03/31
    // ==========================================
    
    // int steer_lookahead = 1; // 往前預視的起始偏移量

    // // 1. 先探測車頭前方的「局部最大曲率」
    // // 為了避免單一點的雜訊，我們看前方最近 3 個點，抓出最大的彎度(取絕對值)
    // double local_max_kappa = 0.0;
    // for (int k = 0; k < 3; ++k) {
    //     int check_idx = std::min(nearestIndex + steer_lookahead + k, (int)global_path_x.size() - 1);
    //     local_max_kappa = std::max(local_max_kappa, std::fabs(computeKappaAt(check_idx)));
    // }

    // // 2. 設定預視距離的「天花板」與「地板」 (你可以微調這兩個數字)
    // double max_lookahead_dist = 3.5;  // 直道時，最遠看 4.0 公尺 (保持直行穩定，不畫龍)
    // double min_lookahead_dist = 2.5;  // 急彎時，最近看 2.0 公尺 (精準貼死彎角，不提早回正)
    
    // // 設定多彎算「急彎」(例如曲率達到 0.3 rad/m 就視為最急的彎)
    // double kappa_threshold = 0.2; 
    
    // // 計算彎度比例 (0.0 代表全直，1.0 代表達到急彎門檻)
    // double kappa_ratio = std::min(local_max_kappa / kappa_threshold, 1.0);
    
    // // 3. 線性內插計算當下的「目標預視距離」
    // // 邏輯：曲率越大(kappa_ratio 越接近 1)，看越近；曲率越小，看越遠
    // double target_lookahead_dist_m = max_lookahead_dist - kappa_ratio * (max_lookahead_dist - min_lookahead_dist);

    // // 4. 依照算出來的距離，去加總未來的方向盤曲率
    // double kappa_sum = 0.0;
    // int valid_points_count = 0;
    // double accumulated_dist = 0.0;

    // for (int j = 0; j < 50; ++j) {
    //   int curr_idx = std::min(nearestIndex + steer_lookahead + j, (int)global_path_x.size() - 1);
      
    //   // 計算累計走過的物理距離
    //   if (j > 0) {
    //     int prev_idx = std::min(nearestIndex + steer_lookahead + j - 1, (int)global_path_x.size() - 1);
    //     double dx = global_path_x[curr_idx] - global_path_x[prev_idx];
    //     double dy = global_path_y[curr_idx] - global_path_y[prev_idx];
    //     accumulated_dist += std::hypot(dx, dy); 
    //   }
      
    //   // 當累計距離超過動態算出的 target_lookahead_dist_m，就停止加總
    //   if (accumulated_dist > target_lookahead_dist_m) {
    //     break;
    //   }
      
    //   // 注意：這裡不取絕對值，因為方向盤有分左轉(+)右轉(-)
    //   kappa_sum += computeKappaAt(curr_idx);
    //   valid_points_count++;
      
    //   if (curr_idx == (int)global_path_x.size() - 1) {
    //     break;
    //   }
    // }

    // // 防呆除以零
    // kappa_avg_steering = kappa_sum / std::max(1.0, double(valid_points_count));
    //=================================================================================


        // ==========================================
        // 速度規劃用(曲率)
        // ==========================================

        double max_future_kappa = 0.0;

        // -------------根據速度動態調整 look ahead 距離(20250108)-------------
        double min_lookahead = 10.0;  // 最少看 15 點 (7.5m) -> 低速時反應快 //15  //p4 5
        double max_lookahead = 30.0;  // 最多看 50 點 (25m) -> 高速時安全 //50
        double lookahead_gain =10.0;  // 速度每增加 1m/s，多看 10 點 10 //p4 10

        // // 公式： 點數 = 最小 + (速度 * 增益)
        int look_ahead_dist = (int)(min_lookahead + std::abs(vt) * lookahead_gain);

        // // // 限制範圍 (Clamp)
        look_ahead_dist = std::max((int)min_lookahead, std::min(look_ahead_dist, (int)max_lookahead));
        // // ----------------------------------------------------------------
        
        for (int k = 0; k < look_ahead_dist; ++k)
        {
            int ii = std::min(nearestIndex + k, (int)global_path_x.size() - 1);
            double k_temp = std::fabs(computeKappaAt(ii));
            
            // 找出未來這段路「最彎」的那個值
            if (k_temp > max_future_kappa) {
                max_future_kappa = k_temp;
            }
        }
        double kappa_for_speed = std::max(std::fabs(kappa_avg_steering), max_future_kappa);

     
     
        // 2. 狀態機轉換
        switch (vel_state) {
        case CRUISE:
            if (remain_to_turn <= K_slow && remain_to_turn > 0) vel_state = APPROACH;
            break;
        case APPROACH:
            if (idx >= turnIdx) { vel_state = DWELL; dwell_start_time = now; }
            break;
        case DWELL:
            if (now - dwell_start_time >= T_dwell) 
            {
                vel_state = RESUME;
                back = true;
            }
            break;
        case RESUME:
            if (std::abs(prev_v_ref) >= v_min_ref + 0.1 * (v_max_ref - v_min_ref)) vel_state = CRUISE;
            break;
        
        }

        // 3. 計算 CTE
        double cte_current = cte;

        // 4. 曲率/CTE 基本降速
        // double curvature_threshold = 0.035;  //0.03 for 0.2m間隔 //0.02 for 0.5m間隔 //曲率門檻
        double v_allowed = v_max_ref;
        double cte_threshold = 0.3;    // CTE 門檻0.2
        double cte_scale     = 0.3;    // CTE 影響比例0.2


        // if (kappa_for_speed > curvature_threshold) {
        //     double f = std::min(kappa_for_speed / curvature_threshold, 1.0);
        //     v_allowed = v_max_ref - f * (v_max_ref - v_min_ref);
        // } 舊的

        // 1. 取得當前車速 (絕對值)
        double current_v = std::abs(vt);

        // 2. 定義速度區間 (根據你的車輛極限設定)
        double v_slow_bound = 2.5;   // 低速區間 (m/s) -> 在此速度以下視為低速
        double v_fast_bound = 2.8;   // 高速區間 (m/s) -> 在此速度以上視為高速

        // 3. 定義門檻區間 (核心設定)
        // 低速時 (Loose)：容忍度高 (0.02)，忽略路徑抖動，利於出彎加速
        // 高速時 (Strict)：容忍度低 (0.005)，對彎道超敏感，確保入彎前提早煞車
        double k_thresh_loose  = 0.01;   //0.02 
        double k_thresh_strict = 0.005;  //0.005 ：））

        // 4. 計算速度比例 (0.0 ~ 1.0)
        double speed_ratio = (current_v - v_slow_bound) / (v_fast_bound - v_slow_bound);
        speed_ratio = std::clamp(speed_ratio, 0.0, 1.0);

        // 5. 線性插值計算「動態 k_min」
        // 速度越快 -> ratio越接近1 -> 減去越多 -> 門檻越小 (越嚴格)
        double k_min_dynamic = k_thresh_loose - speed_ratio * (k_thresh_loose - k_thresh_strict);

        // 6. 設定 k_max (減速區間寬度)
        // 建議 k_max 保持比 k_min 大一個固定值 (例如 0.05)，維持一致的煞車手感
        double k_max_dynamic = k_min_dynamic + 0.05; 

        // 7. 應用減速邏輯
        if (kappa_for_speed > k_min_dynamic) {
            // 計算減速比例 f (0.0 ~ 1.0)
            double f = (kappa_for_speed - k_min_dynamic) / (k_max_dynamic - k_min_dynamic);
            
            // 限制 f 最大為 1.0
            f = std::clamp(f, 0.0, 1.0);
            
            // 使用平方公式讓減速更平滑但有效
            v_allowed = v_max_ref - (f * f) * (v_max_ref - v_min_ref);
        }
        
        // (選用) 用於除錯，觀察門檻變化
        // ROS_INFO("V:%.2f, K_Thresh:%.3f, K_Speed:%.3f", current_v, k_min_dynamic, kappa_for_speed);

        if (cte_current > cte_threshold) {
            double fcte = std::min((cte_current - cte_threshold) / cte_threshold, 1.0);
            v_allowed = std::max(v_min_ref, v_allowed * (1.0 - cte_scale * fcte));
        }

        // 5. 狀態機附加降速
        if (vel_state == APPROACH && remain_to_turn > 0) 
        {
            double f = double(remain_to_turn) / double(K_slow);
            v_allowed *= std::clamp(f, 0.0, 1.0);
        }
        else if (vel_state == DWELL) 
        {
            v_ref      = 0.0;
            prev_v_ref = 0.0; 
        }

        //6. 終點前 N 點緩降（加速版）
        int N_end_slow = 20; //終點前20m降速 //40
        int remain_pts  = (int)global_path_x.size() - nearestIndex;
        bool endpoint_phase = (remain_pts <= N_end_slow);
        if (endpoint_phase) 
        {
            double ratio  = double(remain_pts) / double(N_end_slow);
            double factor = std::pow(ratio, 3.0);
            v_allowed *= std::clamp(factor, 0.0, 1.0);
        }

        // 7. EMA 平滑 + Δv 限幅 (方案A)
        double raw_v = alpha * v_allowed + (1.0 - alpha) * prev_v_ref;
        // 動態下限: CRUISE/RESUME 階段且非終點才用 v_min_ref，下限可降至 0
        bool lower_phase = ((vel_state == CRUISE || vel_state == RESUME) && !endpoint_phase);
        double lower = lower_phase ? v_min_ref : 0.0;
        raw_v = std::clamp(raw_v, lower, v_max_ref);
        // double dv = std::clamp(raw_v - prev_v_ref, -max_delta_v, max_delta_v);

        //20260302: 進一步區分加速與減速的限制，讓加速更平順但減速更果斷
        double max_accel = 0.02; // 限制加速（回正時）
        double max_decel = 0.04; // 允許較快的減速（入彎或遇到障礙時）
        double dv = std::clamp(raw_v - prev_v_ref, -max_decel, max_accel);

        v_ref = prev_v_ref + dv;
        //v_ref = 0.4; //debug
        prev_v_ref = v_ref;
        
        static bool enddd = false;
        // 進入反車模式後，參考速度一律為負：保持規劃出的速度幅度，僅改符號
        // if (reversed_mode ) 
        // {
        //     double mag;
            

        //     if(!enddd)
        //     {
        //         mag = std::max(std::abs(v_ref), v_min_ref); // 至少保有最小速度幅度
        //     }
            
        //     mag = std::min(mag, v_max_ref);                    // 不超過上限
        //     // 1. 直接取用 v_ref 的 "幅度" (它已經包含了所有動態降速邏輯)

        //     // --- 這是您要求的「終點前 5 點減速」邏輯 ---
        //     const int DECEL_POINTS_END = 4; // 在最後 5 個點減速
        //     const double V_AT_END = 0.0;    // 減速到 0

        //     // 3. 計算剩餘點數 (使用第 860 行的變數)
        //     int remain_pts = (int)global_path_x.size() - nearestIndex;

        //     // 4. 如果進入了您指定的 5 點減速區
        //     if (remain_pts <= DECEL_POINTS_END && remain_pts >= 0)
        //     {
        //         enddd = true;
        //         // 5. 計算減速比例 (從 1.0 線性下降到 0.0)
        //         //double scale = std::cbrt(static_cast<double>(remain_pts) / DECEL_POINTS_END);
        //         double scale = std::pow(static_cast<double>(remain_pts) / DECEL_POINTS_END, 0.25);                
        //         // 6. 線性插值計算新速度
        //         //    (mag 是減速前的速度, V_AT_END 是 0)
        //         double target_mag = V_AT_END + scale * (mag - V_AT_END);
                
        //         mag = target_mag; // 覆蓋掉 'mag'
        //     }

        //     v_ref = -mag;
        //     prev_v_ref = v_ref; // 與後續 a_ref 計算保持一致
        // }
        
        ROS_INFO("nearest=%d  turn=%d  remain=%d  state=%d", 
            nearestIndex, turn_index_, turn_index_ - nearestIndex, vel_state);

        // 3. 用平均曲率做前饋轉向
        //double delta_d = std::atan(kappa_avg_steering * L);
        double sign_v  = (v_ref >= 0.0) ? 1.0 : -1.0;
        double delta_d = sign_v * std::atan(kappa_avg_steering * L);
        
        double ref_theta = refHeading + (reversed_mode ? M_PI : 0.0);

        Eigen::Vector4d s(px_proj, py_proj, theta_proj, v_body);
        //Eigen::Vector4d s_d(projection.x(), projection.y(), ref_theta, v_ref);
        Eigen::Vector4d s_d(projection.x(), projection.y(), refHeading_adj, v_ref);
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

        
        if(kappa_avg_steering <= 0.0002 )
        {
            if (fabs(cte) < 0.01 && fabs(epsi) < 0.01) 
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
        
        cmd_to_matlab.data.clear();
        cte_test.data.clear();
        
        cmd_to_matlab.data.emplace_back(u_a);
        cmd_to_matlab.data.emplace_back(u_delta);
        cmd_to_matlab.data.emplace_back(v_ref);
        cte_test.data.emplace_back(cte);


        ROS_INFO("-----------------------------------------------");

        std::cout<<"acceration = "<< u_a <<std::endl;
        std::cout<<"steeringangle = "<< u_delta <<std::endl;
        std::cout<<"theta = "<< theta <<std::endl;
        std::cout<<" lateral error = \n"<< cte <<std::endl;

        ros::Duration gan(0.001);

        mpc_result_pub.publish(cmd_to_matlab);
        
        gan.sleep();

        caculate_mpc_finish = ros::Time::now().toSec();
        //std::cout<<" 控制週期 = "<< caculate_mpc_start - caculate_mpc_finish<<std::endl;

        //===========================csv不會覆蓋上一筆的數據========================
        // std::ofstream ofs(filename_, std::ios::app);
        // if (!ofs.is_open()) {
        // ROS_ERROR("Failed to open file: %s", filename_.c_str());
        // return;
        // }
        // //std::cout<<"kappa2 = \n"<< kappa <<std::endl;
        // static bool header_written = false;
        // if (!header_written) {
        // ofs << "u_a,v_real,v_ref,u_delta,delta_d,px,py,theta1,cte,cte_real,epsi,vx,vy,kappa,beta" << std::endl;
        // header_written = true;
        // }

        // ofs << std::fixed << std::setprecision(4)
        // << u_a << ","<< v_body << "," << v_ref << "," << u_delta << ","<< delta_d << "," << px << "," << py << ","
        // << theta1 << "," << cte << "," << cte_real << ","<< epsi << "," << vx << ","
        // << vy <<","<<kappa_avg_steering<< "," << beta_1<<std::endl;

        // ofs.close(); // 確保每次操作後關閉文件

        //=====================================================================


        //======================csv會覆蓋上一筆的數據================================
        static bool first_write = true;
    std::ofstream ofs;

    if (first_write) {
      // 程式剛啟動的第一次寫入，使用 std::ios::trunc 模式（清空並覆蓋舊檔案）
      ofs.open(filename_, std::ios::out | std::ios::trunc);
      if (!ofs.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename_.c_str());
        return;
      }
      // 寫入標題行
      ofs << "u_a,v_real,v_ref,u_delta,delta_d,px,py,theta1,cte,cte_real,epsi,"
             "vx,vy,kappa,beta,steer\n";
      first_write = false;
    } else {
      // 之後的寫入，使用 std::ios::app 模式（接續寫在同一份檔案的尾端）
      ofs.open(filename_, std::ios::app);
      if (!ofs.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename_.c_str());
        return;
      }
    }

    // 寫入當前週期的數據
    ofs << std::fixed << std::setprecision(4) << u_a << "," << v_body << ","
        << v_ref << "," << u_delta << "," << delta_d << "," << px << "," << py
        << "," << theta1 << "," << cte << "," << cte_real << "," << epsi << ","
        << vx << "," << vy << "," << kappa_avg_steering << "," << beta_1<< ","<< steer_real
        << std::endl;

    ofs.close(); // 確保每次操作後關閉文件
    //=====================================================================
        
    }    
}

Eigen::Vector2d MPCPlanner_path::_mpcControl(Eigen::Vector4d s, Eigen::Vector4d s_d,Eigen::Vector2d u_r, Eigen::Vector2d du_p)
{
    int dim_u = 2;
    int dim_x = 4;
    double L = 1.66;
    double l_r = 0.4; // 質心到後軸距離

    // 根據前輪轉角 u_r[1] 計算側滑角 β
    // 這裡我們暫時禁用，若要提高模型精度可以重新啟用並推導 A_o 和 B_o
    double beta = 0;

    OsqpEigen::Solver solver;

    // 狀態誤差向量 x_e = [x-x_d, y-y_d, theta-theta_d, v-v_d, du_a, du_delta]
    Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
    x(0) = s(0) - s_d(0);
    x(1) = s(1) - s_d(1);
    x(2) = regularizeAngle(s(2) - s_d(2));
    x(3) = s(3) - s_d(3);
    x.tail(dim_u) = du_p;

    // 狀態轉移矩陣 A_o (描述狀態間的關係)
    Eigen::Matrix4d A_o = Eigen::Matrix4d::Identity();
    A_o(0,2) = -s_d(3) * sin(s_d(2)) * d_t_;
    A_o(1,2) =  s_d(3) * cos(s_d(2)) * d_t_;
    A_o(0,3) = cos(s_d(2)) * d_t_;
    A_o(1,3) = sin(s_d(2)) * d_t_;

    // 控制輸入矩陣 B_o (描述控制輸入對狀態的影響)
    Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x,dim_u);
    B_o(3,0) = d_t_; // 加速度 a 只影響速度 v
    B_o(2,1) = d_t_ * s_d(3) / (L * pow(cos(u_r[1]),2)); // 轉向角 delta 影響航向角 theta

    // 增廣系統狀態矩陣 A
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x+dim_u, dim_x+dim_u);
    A.topLeftCorner(dim_x, dim_x) = A_o;
    A.topRightCorner(dim_x, dim_u) = B_o;
    A.bottomRightCorner(dim_u,dim_u) = Eigen::MatrixXd::Identity(dim_u,dim_u);

    // 增廣系統控制矩陣 B
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x+dim_u, dim_u);
    B.topLeftCorner(dim_x, dim_u) = B_o;
    B.bottomLeftCorner(dim_u, dim_u) = Eigen::MatrixXd::Identity(dim_u,dim_u);

    // 輸出矩陣 C
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x+dim_u);
    C.topLeftCorner(dim_x, dim_x) = Eigen::MatrixXd::Identity(dim_x, dim_x);

    // MPC 狀態預測矩陣 S_x
    Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
    Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_, dim_x + dim_u);
    for(int i = 0; i<p_; i++)
    {
        S_x.middleRows(dim_x * i, dim_x) = C * A_pow(i + 1);
    }

    // MPC 控制預測矩陣 S_u
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);
    for(int i = 0; i < p_; i++){
      for(int j = 0; j <= std::min(i, m_-1); j++){
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
      }
    }

    // 優化問題設定
    Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x*p_); // 目標參考軌跡設為0 (因為我們追蹤的是誤差)

    // =================== 倒車時動態調整權重 (主要修改處) ===================

    // 1. 宣告這次計算專用的 Q 和 R 矩陣，預設為全域設定值
    Eigen::MatrixXd Q_dynamic = Q_;
    Eigen::MatrixXd R_dynamic = R_;

    // 2. 檢查是否處於倒車模式 (假設 reversed_mode 是 MPCPlanner_path 類別的成員變數)
    if (reversed_mode)
    {
        // 若是倒車模式，採用更激進的權重以抑制不穩定性
        // 大幅提高對橫向誤差(cte)的懲罰
        Q_dynamic(1,1) = 800; // 原為 500
        // 大幅提高對航向誤差(epsi)的懲罰
        Q_dynamic(2,2) = 750; // 原為 300
    }

    // 3. 使用動態權重來產生最終的 Q 和 R 矩陣
    Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_,p_), Q_dynamic);
    Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_,m_), R_dynamic);

    // ============================== 修改結束 ==============================


    // Hessian 矩陣 H
    int n_var  = dim_u * m_;
    int n_cons = 2 * dim_u * m_;
    Eigen::MatrixXd H = S_u.transpose() * Q * S_u + R;
    H = (H + H.transpose()) * 0.5;
    H.diagonal().array() += 1e-6; // 增加數值穩定性

    // 梯度向量 g
    Eigen::VectorXd g = S_u.transpose() * Q * (S_x * x - Yr);

    // 約束條件設定
    Eigen::Vector2d u_k_1;
    if (!first)
    {
        u_k_1 = u_prev;
        first = true;
    }
    else
    {
        u_k_1 = Eigen::Vector2d (du_p[0], du_p[1]);
    }

    // 控制輸入量 u = [a, delta] 的上下限
    double min_a = -0.2;
    double max_a = +0.2;
    Eigen::Vector2d u_min(min_a, min_delta_);
    Eigen::Vector2d u_max(max_a, max_delta_);

    // 控制輸入增量 du = [da, d_delta] 的上下限
    double max_da = 0.2;
    Eigen::Vector2d du_min(-max_da, -max_delta_inc_);
    Eigen::Vector2d du_max(+max_da, +max_delta_inc_);

    // 擴展到整個控制時域
    Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);
    Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);
    Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);
    Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);
    Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_r);
    Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_k_1);

    // 建立約束矩陣 P
    Eigen::MatrixXd temp = Eigen::MatrixXd::Ones(m_,m_).triangularView<Eigen::Lower>();
    Eigen::MatrixXd A_I = Eigen::kroneckerProduct(temp,Eigen::MatrixXd::Identity(dim_u,dim_u));
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2*dim_u*m_, dim_u*m_);
    P.topRows(dim_u*m_) = A_I;
    P.bottomRows(dim_u*m_) = Eigen::MatrixXd::Identity(dim_u*m_, dim_u*m_);

    // 建立約束向量 lower 和 upper
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);
    lower.topRows(dim_u*m_)    = U_min - U_k_1 - U_r;
    lower.bottomRows(dim_u*m_) = dU_min;

    Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);
    upper.topRows(dim_u*m_)    = U_max - U_k_1 - U_r;
    upper.bottomRows(dim_u*m_) = dU_max;

    // OSQP 求解器設定
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(n_var);
    solver.data()->setNumberOfConstraints(n_cons);
    solver.data()->setHessianMatrix(_convertTOSparseMatrix(H));
    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(_convertTOSparseMatrix(P));
    solver.data()->setLowerBound(lower);
    solver.data()->setUpperBound(upper);

    if (!solver.initSolver()) {
      ROS_ERROR("OSQP 初始化失敗");
      return Eigen::Vector2d::Zero();
    }

    if (!solver.solve()) {
      ROS_ERROR("OSQP 求解失敗");
      return Eigen::Vector2d::Zero();
    }

    auto solution = solver.getSolution();

    // 計算並回傳最終的控制指令 [加速度, 前輪轉角]
    Eigen::Vector2d u(solution[0] + du_p[0] + u_r[0], regularizeAngle(solution[1] + du_p[1] + u_r[1]));
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