#include <ros/ros.h>
#include <math.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64MultiArray.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "vector"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"

constexpr double pi() { return M_PI; }              // pi = 3.14159265358979323846...
double deg2rad(double x) { return x * pi() / 180; } // 角度轉徑度
double rad2deg(double x) { return x * 180 / pi(); } // 徑度轉角度

class MPCWaypointGenerator
{
    public:
    MPCWaypointGenerator() // class initialization
    {     
        //Subscriber//
        amcl_pose_sub = nh_.subscribe("/odom",1000,&MPCWaypointGenerator::current_pose_callback,this);  // amcl定位資訊
        //astar_path_sub = nh_.subscribe("/nav_path",100,&MPCWaypointGenerator::astar_callback,this);          // astar全域路徑
        astar_path_sub = nh_.subscribe("/sPath",100,&MPCWaypointGenerator::astar_callback,this);            //hybrid A*
        csv_waypoint_sub = nh_.subscribe("/array_topic",1000,&MPCWaypointGenerator::callbackorgwp,this);     // from global_path.cpp 全域路徑來自csv檔

        //Pubisher//
        to_mpc_following_wp = nh_.advertise<std_msgs::Float64MultiArray>("/mpc_waypoints",1000);  // 發布給控制器 pid/mpc...
        pub_stop_end = nh_.advertise<std_msgs::Bool>("/stop_end",1000);                           // 發布，讓載具停下

        //rviz//
        astar_local_show = nh_.advertise<visualization_msgs::MarkerArray>("/astar_local",1);

        //csv//
        private_nh_.param<std::string>("save_filename",filename,std::string("/home/king/mpc/mpcdata/local.csv")); //存成csv檔，默認filename_為"waypoint.csv"，發布話題名稱為/waypoint_saver/save_filename


    };

    void run(){
        // 在ROS系統正常運行 (ros::ok() 返回 true) 的情況下，程式會不斷重複執行循環內的內容。
        /*while(ros::ok()){
            //std::cout<<"mpc_waypoint_generator_running_start"<<std::endl;
        }*/
    }
 
    private:
    // ROS中管理通信的對象。它負責創建ROS節點、訂閱和發布話題、服務和客戶端的呼叫等。
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //publisher//
    ros::Publisher to_mpc_following_wp;
    ros::Publisher pub_stop_end;
    ros::Publisher astar_local_show;

    //subscriber// 
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber astar_path_sub;
    ros::Subscriber csv_waypoint_sub;
    ////////////


    //data type//
    std::vector<double> csv_waypoint_vector_x;          // csv路徑
    std::vector<double> csv_waypoint_vector_y;

    std::vector<double> astar_waypoint_vector_x;        // astar路徑
    std::vector<double> astar_waypoint_vector_y;

    std::vector<double> current_waypoint_vector_x;      // 當前使用的全域路徑
    std::vector<double> current_waypoint_vector_y;

    std::vector<double> local_waypoint_x;               // local waypoints，預進行曲線擬合的一段路徑
    std::vector<double> local_waypoint_y;

    std::vector<double> final_pub_wp_x;                 // [x1, x2, ..., x99 , x100]
    std::vector<double> final_pub_wp_y;                 // [y1, y2, ..., y99 , y100]
    std::vector<double> fullpath_pub_to_localpath;      // 最終發布出去的路徑   [x1, y1, x2, y2, ..., x99, y99, x100, y100]

    std_msgs::Float64MultiArray full_path_msg;          // 發布給控制器的訊息格式
    std_msgs::Bool stop_msg;                            // 發布給載具，"停下"的訊息格式
    geometry_msgs::Pose2D base_link_point;              // 儲存當下座標x,y
    float current_yaw = 0;                              // 儲存當下車頭角度
    int start_id = -1 ;                                 // 儲存最近路徑點的id，預設初始值為"-1"。
    int id_range = 0;                                   // 儲存start_id的範圍，[0]~[id_range]，預設初始值為"0"。
    int wp_start = 0, wp_end = 0;
    bool stop = false;                                  // 預設"停下"指令＝0，表示載具不用停止不動 ; 若為1(true)，則表示載具須停止不移動
    bool astar_path_bool = true;                       // 預設"是否使用astar規劃的路徑"＝false，表示使用的是csv路徑 ; 若為true，表示使用astar的路徑
    std::string filename;
 
//---------------callback function---------------//
 
// csv global path 使用csv檔
    void callbackorgwp(const std_msgs::Float64MultiArrayConstPtr &msg)   
    {
        ROS_INFO("get_from_waypoint_loader_data");
        csv_waypoint_vector_x.clear();
        csv_waypoint_vector_y.clear();
        for(size_t i = 0 ; i < msg->data.size() ; i += 2)
        {
            csv_waypoint_vector_x.push_back(msg->data[i]);
            csv_waypoint_vector_y.push_back(msg->data[i+1]);        
        }
        astar_path_bool = false;

        //std::cout<<"csv_waypoint_vector_x.size() = "<<csv_waypoint_vector_x.size()<<std::endl;
        //std::cout<<"csv_waypoint_vector_y.size() = "<<csv_waypoint_vector_y.size()<<std::endl;
    }

// astar global path 使用astar規劃的路徑
    void astar_callback(const nav_msgs::PathConstPtr &msg)  
    {
        std::cout << "Get astar waypoints " << std::endl;
        astar_waypoint_vector_x.clear();
        astar_waypoint_vector_y.clear();
        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            geometry_msgs::PoseStamped pose = msg->poses[i];    // 獲取當前的PoseStamped
            // nav_msgs::Path path;
            // path.poses.push_back(pose);                      // 累加到path.poses中

            // 座標轉換形式xyz
            // double pose_x = pose.pose.position.x; 
            // double pose_y = pose.pose.position.y;
            // double pose_z = pose.pose.position.z;

            astar_waypoint_vector_x.push_back(pose.pose.position.x);
            astar_waypoint_vector_y.push_back(pose.pose.position.y);
        }
        //std::cout<<"astar_waypoint_vector_x.size() = "<<astar_waypoint_vector_x.size()<<std::endl;
        //std::cout<<"astar_waypoint_vector_y.size() = "<<astar_waypoint_vector_y.size()<<std::endl;

        // 表示使用astar規劃的路徑
        astar_path_bool = true;

    }

// current pose 當前位置，來自amcl
    void current_pose_callback(const nav_msgs::OdometryConstPtr &msg)
    {      
        // 當下座標x,y
        base_link_point.x = msg->pose.pose.position.x;                      // vehicle coordinate x 
        base_link_point.y = msg->pose.pose.position.y;                      // vehicle coordinate y
        
        tf::Quaternion q(msg->pose.pose.orientation.x,                      //用 tf::Quaternion(四元數)宣告一個q,將msg(current pose)中提取的四元數傳給q
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);                                                 // 將四元數轉換成 3*3 旋轉矩陣形式
        double roll;                                                        // Declare roll angle(Rotate around the x-axis)
        double pitch;                                                       // Declare pitch angle(Rotate around the y-axis)
        double yaw;                                                         // Declare yaw angle(Rotate around the z-axis)
        m.getRPY(roll,pitch,yaw);                                           // Extract roll, pitch, yaw from m
        current_yaw = yaw;                                                  // Temporarily store the calculated yaw into current yaw
        if(current_yaw < 0)
        {                                                                   // If the yaw angle is less than zero
            current_yaw = M_PI + M_PI + current_yaw;                        // current yaw = current + 2*pi
        }
        base_link_point.theta = yaw;                                        // Temporarily store the yaw angle into theta in the structure base_link_point

// Use csv or astar of global path
        current_waypoint_vector_x.clear();
        current_waypoint_vector_y.clear();
        if(astar_path_bool == false)
        {
            std::cout << "Use csv_waypoint_vector !" << std::endl;
            current_waypoint_vector_x = csv_waypoint_vector_x;
            current_waypoint_vector_y = csv_waypoint_vector_y;
        }
        else if(astar_path_bool == true)
        {
            std::cout << "Use astar_waypoint_vector !" << std::endl;
            if(astar_waypoint_vector_x.size() < 1)
            {
                current_waypoint_vector_x.push_back(0);
                current_waypoint_vector_y.push_back(0);
                std::cout << "~~ Wait astar path ~~~" << std::endl;
            }
            else{
                current_waypoint_vector_x = astar_waypoint_vector_x;
                current_waypoint_vector_y = astar_waypoint_vector_y;
            }

        }
        // std::cout << "current_waypoint_vector_x.size() = " << current_waypoint_vector_x.size() << std::endl;
        // std::cout << "current_waypoint_vector_y.size() = " << current_waypoint_vector_y.size() << std::endl;
        id_range = current_waypoint_vector_x.size() - 1;                     // vector有值的範圍，current_waypoint_vector_x[0] ~ [id_range]
        // std::cout << "id_range = " << id_range << std::endl;

// 計算與當下座標最近的路徑點
        double nearest_distance = 999;                                       // declare the double variable "nearest_distance" is 999
        for(size_t i = 0 ; i < current_waypoint_vector_x.size() ; i++)
        {
            // 計算當下座標與各個路徑點的距離
            double distance_calc = sqrt(pow(current_waypoint_vector_x[i]-base_link_point.x,2)+
                                        pow(current_waypoint_vector_y[i]-base_link_point.y,2));
            //std::cout<<"distance_calc = "<<distance_calc<<std::endl;

            if(distance_calc < nearest_distance)
            {
                nearest_distance = distance_calc;
                //std::cout<<"nearest_distance = "<<nearest_distance<<std::endl;

                // 最近路徑點的id
                start_id  = i;
                //std::cout<<"~~~~~~~~~~first_start_id = "<<start_id<<std::endl;
            } 
        }  

        // 如果最短距離大於5 重設start_id
        if(nearest_distance > 5)   
        {
            start_id = -1; 
        }

        if(start_id < 0 || start_id > id_range)                                          //ex: current_waypoint_vector_x.size()=101，start_id:[0]~[100]，id_range=100
        {
            start_id = id_range;                                                         //ex: start_id=[100]
            std::cout << "NOW start_id = " << start_id << std::endl;
        }
        ROS_INFO("NOW id is: %d ", start_id);
        std::cout << "current_waypoint_vector_x.size() = " << current_waypoint_vector_x.size() << std::endl;
        std::cout << "current_waypoint_vector_y.size() = " << current_waypoint_vector_y.size() << std::endl;
        std::cout << "id_range = " << id_range << std::endl;
        local_waypoint_x.clear();
        local_waypoint_y.clear();


// 取後?個點  
        int back_wp_num = 4;   // 後4個
        int back_id = start_id - back_wp_num;                                            // 當下start_id減4以後，表示局部路徑點local waypoint，從第back_id個點開始，current_waypoint_vector_x[back_id]
        std::cout << "Set: back_wp_num  = " << back_wp_num << std::endl;
        
        if (start_id > 0)                                                                // 只有當start_id>0，才會有向後的點
        {
            if(back_id < 0 )                                                             // start_id<0的路徑點是不存在的，current_waypoint_vector_x[-1]不存在
            {
                back_id = 0;
            }
            std::cout << "back_id  = " << back_id << std::endl;

            for(int i = back_id ; i < start_id ; i+=1)                                   // 從[back_id]開始，到[start_id - 1]，累加進vector
            {
                local_waypoint_x.push_back(current_waypoint_vector_x.at(i)); 
                local_waypoint_y.push_back(current_waypoint_vector_y.at(i));
            }

            //～～～～～
            // 若接近終點，前n個點會開始縮減，後方的點也要縮減？
            // int warn_back = (id_range - start_id) + 1;                                //計算前方剩幾個點 //(100-97)+1=4 [97]~[100]  //(100-98)+1=3 [98]~[100]
            // if(back_wp_num > warn_back)                                               //表示後方點個數已經大於前方點個數
            // {
            //     local_waypoint_x.clear();
            //     local_waypoint_y.clear();
            //     int new_back_id = back_id + (back_wp_num - warn_back);                //(back_wp_num - warn_back)表示要縮減的數量
            //     for(int i = new_back_id ; i < start_id ; i+=1)   
            //     {
            //         local_waypoint_x.push_back(current_waypoint_vector_x.at(i)); 
            //         local_waypoint_y.push_back(current_waypoint_vector_y.at(i));
            //     }
            // }
            //～～～～～

        }
        else{
            back_id = 0;
            std::cout << "back_id  = []" << std::endl;
            local_waypoint_x.clear();
            local_waypoint_y.clear();
        }
        wp_start = back_id;                                                             // 紀錄最終vector起始點是全域路徑id多少


// 取前?個點  
        int front_wp_num = 10;  // 前10個
        int front_id = start_id + front_wp_num;                                         // 當下start_id加10以後，表示局部路徑點local waypoint，結束在front_id前一點，current_waypoint_vector_x[front_id - 1]，start_id也包含在前10個點裡面
        std::cout << "Set: front_wp_num  = " << front_wp_num << std::endl;
        
        if(start_id < 0)                                                                // start_id<0的路徑點是不存在的，current_waypoint_vector_x[-1]不存在
        {
            int id = 0;                                                                 // 所以需要從[0]開始
            for(int i = id ; i < front_id ; i+=1)                                       // 從[id]開始，到[front_id - 1]，累加進vector
            {
                local_waypoint_x.push_back(current_waypoint_vector_x.at(i));   
                local_waypoint_y.push_back(current_waypoint_vector_y.at(i));
            }
        }
        else{
            if(front_id > id_range)                                                     // 因為結束在front_id前一點，所以front_id最多只能是id_range + 1
            {
                front_id = id_range + 1;
            }
            std::cout << "front_id  = " << front_id << std::endl;

            for(int i = start_id ; i < front_id ; i+=1)                                 // 從[start_id]開始，到[front_id - 1]，累加進vector
            {
                local_waypoint_x.push_back(current_waypoint_vector_x.at(i)); 
                local_waypoint_y.push_back(current_waypoint_vector_y.at(i)); 
            }

            // 太接近終點，載具須停下！
            int warn_front = front_id - start_id;                                       // 計算前方剩幾個點
            if(warn_front <= 3)
            {
                stop = true;
                stop_msg.data = stop;
                pub_stop_end.publish(stop_msg);
                ROS_WARN("STOP!!!!!!!!!!!!!!!!");
            }

        }
        wp_end = front_id - 1;                                                          // 紀錄最終vector結束點是全域路徑id多少

        // 最終vector資訊
        std::cout << "local_waypoint_x.size =  " << local_waypoint_x.size() << std::endl;
        std::cout << "local_waypoint: from id = " << wp_start << " to id = " << wp_end << std::endl;    


//rviz & publish
        final_pub_wp_x = local_waypoint_x;                                             // 將合併後?個點以及前?個點的路徑，儲存至另一個vector
        final_pub_wp_y = local_waypoint_y;
        //std::cout<<"final_pub_wp_x.size =  "<<final_pub_wp_x.size()<<std::endl;
        
        visualization_msgs::MarkerArray show_local_wp_msg;
        visualization_msgs::Marker show_waypoint_marker;
        show_waypoint_marker.header.frame_id = "map";                                 // 可視化時每個點的座標系
        show_waypoint_marker.ns = "show_local_wp_marker";
        show_waypoint_marker.type = visualization_msgs::Marker::POINTS;               // 可視化時每個點的形狀
        show_waypoint_marker.action = visualization_msgs::Marker::ADD;                // 可視化時每個點的添加方式
        show_waypoint_marker.scale.x = 0.25;                                          // 可視化時每個點的大小
        show_waypoint_marker.scale.y = 0.25; 

        show_local_wp_msg.markers.clear();
        fullpath_pub_to_localpath.clear(); 
        for(size_t i = 0 ; i < final_pub_wp_x.size() ; i++)                          // 從0開始，所有的局部路徑local waypoint
        {
            geometry_msgs::Point point;                                              // 取局部路徑的每個點
            point.x = final_pub_wp_x[i];
            point.y = final_pub_wp_y[i];

            show_waypoint_marker.points.push_back(point);                            // 可視化時每個點的座標值
            show_waypoint_marker.color.a = 0.9;                                      // 可視化時每個點的透明度
            show_waypoint_marker.color.r = 0;                                      // 可視化時每個點的顏色
            show_waypoint_marker.color.g = 0;
            show_waypoint_marker.color.b = 1;
            fullpath_pub_to_localpath.push_back(point.x);                            // 每個x值累加至新vector
            fullpath_pub_to_localpath.push_back(point.y);                            // 每個y值累加至新vector

            //～～～～～
            // 如果需要每次更新local waypoint都存成csv
            // std::ofstream ofs (filename.c_str(),std::ios::app); 
            // static bool recevice_once = false; 
            // if(!recevice_once)
            // {
            //     ofs << "id,x,y"<< std::endl;  
            //     ofs << i << "," << point.x << "," << point.y << std::endl;   
            //     recevice_once = true;
            // }
            // else{
            //     ofs << i << "," << point.x << "," << point.y << std::endl;  
            // }
            //～～～～～

        }

// 最終發布
        show_local_wp_msg.markers.push_back(show_waypoint_marker);  // 轉換訊息格式     
        astar_local_show.publish(show_local_wp_msg);                // 發布可視化
        full_path_msg.data = fullpath_pub_to_localpath;             // 轉換訊息格式
        to_mpc_following_wp.publish(full_path_msg);                 // 發布local waypoints
    }

};


/*--------------------main function--------------------*/
int main(int argc, char **argv)
{
    ros::init(argc, argv,"local_path");          // 初始化節點node = "amr_local_waypoint2"
    ROS_INFO("~~~~~golf_local_waypoint code start~~~~");  // 終端顯示
    MPCWaypointGenerator obj;                             // 創建一個MPCWaypointGenerator類型的對象，名稱為obj
    obj.run();                                            // 調用obj對象的run方法來執行此程式
    //sleep(5);                                           // 暫停5秒
    ros::spin();                                          // ROS的函數，進入事件處理循環，等待並處理來自ROS的消息
    return 0;

}
