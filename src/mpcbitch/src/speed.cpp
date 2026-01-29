#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <vector>

int openSerial(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        ROS_ERROR("Cannot open serial port: %s", port.c_str());
        return -1;
    }

    // 設定為 blocking
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    // baudrate
    speed_t br;
    switch (baudrate) {
        case 9600: br = B9600; break;
        case 19200: br = B19200; break;
        case 38400: br = B38400; break;
        case 115200: br = B115200; break;
        default: br = B9600; break;
    }
    cfsetispeed(&options, br);
    cfsetospeed(&options, br);

    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 無 flow control
    options.c_cflag &= ~CRTSCTS;

    // 開啟接收，忽略 modem 控制線
    options.c_cflag |= (CLOCAL | CREAD);

    // raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 設定 read timeout
    options.c_cc[VMIN]  = 0;   // 至少讀到幾個字元才回傳
    options.c_cc[VTIME] = 10;  // 1 = 0.1s, 10 = 1s

    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "curtis_serial_node");
    ros::NodeHandle nh("~");  // private namespace

    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 9600);

    // 要把原始資料丟出來
    ros::Publisher raw_pub = nh.advertise<std_msgs::ByteMultiArray>("curtis/raw_bytes", 10);
    // 如果控制器其實是傳 ASCII，也可以順便丟一份 string
    ros::Publisher str_pub = nh.advertise<std_msgs::String>("curtis/raw_string", 10);

    int fd = openSerial(port, baudrate);
    if (fd < 0) {
        ROS_FATAL("Failed to open serial port. Exit.");
        return 1;
    }
    ROS_INFO("Curtis serial node started. Port: %s, baud: %d", port.c_str(), baudrate);

    ros::Rate loop(100);  // 100Hz 嘗試讀資料
    uint8_t buf[256];

    while (ros::ok()) {
        // 讀串列
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            // 1) 發 ByteMultiArray
            std_msgs::ByteMultiArray msg;
            msg.data.insert(msg.data.end(), buf, buf + n);
            raw_pub.publish(msg);

            // 2) 發 String（如果資料是可視 ASCII 可以看一下）
            std_msgs::String s;
            s.data.assign((char*)buf, (char*)buf + n);
            str_pub.publish(s);

            // 你要在這裡解析也可以，例如：
            // parseCurtisFrame(buf, n);
        }

        ros::spinOnce();
        loop.sleep();
    }

    close(fd);
    return 0;
}
