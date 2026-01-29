// main.cpp

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QTextBrowser>
#include <QPushButton>
#include <QTimer>
#include <QDateTime>
#include <QImage>
#include <QPixmap>
#include <QIcon>
#include <QTransform>
#include <QDebug>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32.h>

//==============================================================================
// 1. ImageWidget：用來顯示任意 ROS 傳來的 sensor_msgs::Image
//==============================================================================
class ImageWidget : public QLabel {
    Q_OBJECT
public:
    explicit ImageWidget(QWidget* parent = nullptr) : QLabel(parent) {
        setMinimumSize(640, 480);
        setStyleSheet("border: 1px solid black;");
        setScaledContents(true);
    }

public Q_SLOTS:
    void updateImage(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            cv::resize(frame, frame, cv::Size(640, 480));

            cv::Mat rgb;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
            QImage qimg(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
            QImage qimg_copy = qimg.copy();
            setPixmap(QPixmap::fromImage(qimg_copy));
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

//==============================================================================
// 2. CarCamWidget：專門訂閱 Carla 相機 ("/carla_camera/image_raw") 的影像
//==============================================================================
class CarCamWidget : public QLabel {
    Q_OBJECT
public:
    explicit CarCamWidget(QWidget* parent = nullptr) : QLabel(parent) {
        setMinimumSize(800, 600);
        setStyleSheet("border: 1px solid black; background-color: black;");
        setScaledContents(true);
    }

public Q_SLOTS:
    void updateCarlaImage(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            cv::resize(frame, frame, cv::Size(800, 600));

            cv::Mat rgb;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
            QImage qimg(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
            QImage qimg_copy = qimg.copy();
            setPixmap(QPixmap::fromImage(qimg_copy));
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (Carla): %s", e.what());
        }
    }
};

//==============================================================================
// 3. ControlWidget：右下控制區，包含訊息框、電池顯示、剩餘里程、現在時間與按鈕
//==============================================================================
class ControlWidget : public QWidget {
    Q_OBJECT
public:
    explicit ControlWidget(QWidget* parent = nullptr) : QWidget(parent) {
        // 最外層水平佈局：左半部為訊息框，右半部為電池/里程/時間/按鈕
        QHBoxLayout* mainLayout = new QHBoxLayout(this);

        // 左側：對話框 (TextBrowser)
        textBrowser = new QTextBrowser(this);
        textBrowser->setStyleSheet("border: 1px solid black;");

        // 右側：電池、里程、時間、按鈕
        QWidget* rightWidget = new QWidget(this);
        QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);

        // 【電池顯示區】
        QWidget* batteryWidget = new QWidget(this);
        QHBoxLayout* batteryLayout = new QHBoxLayout(batteryWidget);
        batteryButton = new QPushButton(this);
        // 預設載入 100% 電池圖示，並順時針旋轉 90 度
        QPixmap pix100("/home/king/Downloads/100.jpg");
        QPixmap rotated100 = pix100.transformed(QTransform().rotate(90));
        batteryButton->setIcon(QIcon(rotated100));
        batteryButton->setIconSize(QSize(100, 100));
        batteryButton->setFlat(true);
        batteryPercentageLabel = new QLabel("80%", this);
        batteryPercentageLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        batteryPercentageLabel->setStyleSheet("font-size: 50px;");
        batteryLayout->addWidget(batteryButton);
        batteryLayout->addWidget(batteryPercentageLabel);
        batteryLayout->setContentsMargins(0,0,0,0);
        rightLayout->addWidget(batteryWidget, 0);

        // 【剩餘里程】
        mileageLabel = new QLabel(this);
        mileageLabel->setAlignment(Qt::AlignCenter);
        mileageLabel->setText("剩餘里程：100 km");
        mileageLabel->setStyleSheet("font-size: 32px;");
        rightLayout->addWidget(mileageLabel, 1);

        // 【現在時間】
        timeLabel = new QLabel(this);
        timeLabel->setAlignment(Qt::AlignCenter);
        rightLayout->addWidget(timeLabel, 1);

        // 【按鈕區：緊急停止、完成】
        QWidget* buttonWidget = new QWidget(this);
        QVBoxLayout* buttonLayout = new QVBoxLayout(buttonWidget);
        stopButton = new QPushButton("緊急停止", buttonWidget);
        finishButton = new QPushButton("完成", buttonWidget);
        finishButton->setEnabled(false);
        stopButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        finishButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        buttonLayout->addWidget(stopButton, 1);
        buttonLayout->addWidget(finishButton, 1);
        rightLayout->addWidget(buttonWidget, 1);

        // 把左側 textBrowser 與右側 rightWidget 各自 stretch 設為 1：1
        mainLayout->addWidget(textBrowser);
        mainLayout->addWidget(rightWidget);
        mainLayout->setStretch(0, 1);  // index 0 = textBrowser
        mainLayout->setStretch(1, 1);  // index 1 = rightWidget

        // QTimer 每秒更新一次時間
        QTimer* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &ControlWidget::updateTime);
        timer->start(1000);
        updateTime();

        // 按鈕訊號連接
        connect(stopButton, &QPushButton::clicked, this, &ControlWidget::handleStop);
        connect(finishButton, &QPushButton::clicked, this, &ControlWidget::handleFinish);
        connect(batteryButton, &QPushButton::clicked, this, &ControlWidget::openBatteryUI);

        // 預設把電量設為 50%，里程設為 21.6 km
        updateBatteryStatus(50);
        updateMileage(21.6);
    }

public Q_SLOTS:
    void updateTextBrowser(const QString &msg) {
        textBrowser->append(msg);
        if (msg.contains("方向盤已回正，結束停車")) {
            finishButton->setEnabled(true);
        }
    }
    void updateTime() {
        QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
        timeLabel->setText("現在時間：" + currentTime);
        timeLabel->setStyleSheet("font-size: 32px;");
    }
    void openBatteryUI() {
        QWidget* batteryScreen = new QWidget;
        batteryScreen->setWindowTitle("電池詳細資訊");
        batteryScreen->resize(400, 300);
        QVBoxLayout* layout = new QVBoxLayout(batteryScreen);
        QLabel* infoLabel = new QLabel("這裡顯示更詳細的電池資訊...", batteryScreen);
        layout->addWidget(infoLabel);
        batteryScreen->show();
    }
    void updateBatteryStatus(int percentage) {
        batteryPercentageLabel->setText(QString::number(percentage) + "%");
        QString path;
        if (percentage >= 90)        path = "/home/cyc/golf_ws/100.jpg";
        else if (percentage >= 70)   path = "/home/cyc/golf_ws/80.jpg";
        else if (percentage >= 50)   path = "/home/cyc/golf_ws/60.jpg";
        else if (percentage >= 30)   path = "/home/cyc/golf_ws/40.jpg";
        else if (percentage >= 10)   path = "/home/cyc/golf_ws/20.jpg";
        else                         path = "/home/cyc/golf_ws/0.jpg";

        QPixmap pix(path);
        QPixmap rotated = pix.transformed(QTransform().rotate(90));
        batteryButton->setIcon(QIcon(rotated));
        batteryButton->setIconSize(QSize(100, 100));
        batteryButton->setFlat(true);
    }
    void updateMileage(double mileage) {
        mileageLabel->setText("剩餘里程：" + QString::number(mileage) + " km");
    }
    void handleStop() {
        ROS_INFO("緊急停止按鈕被按下！");
    }
    void handleFinish() {
        ROS_INFO("完成按鈕被按下，結束程式");
        ros::shutdown();
        QApplication::quit();
    }

private:
    QLabel* mileageLabel;
    QTextBrowser* textBrowser;
    QLabel* timeLabel;
    QPushButton* stopButton;
    QPushButton* finishButton;
    QPushButton* batteryButton;
    QLabel* batteryPercentageLabel;
};

//==============================================================================
// 4. ROSHandler：訂閱原本影像、Carla 影像與文字，並透過信號發送給各 Widget
//==============================================================================
class ROSHandler : public QObject {
    Q_OBJECT
public:
    explicit ROSHandler(QObject* parent = nullptr) : QObject(parent) {
        ros::NodeHandle nh;
        // 訂閱「原本要顯示的那條影像」(範例假設為 /usb_cam/image_raw)
        original_image_sub_ = nh.subscribe("/usb_cam/image_raw", 1,
                                           &ROSHandler::originalImageCallback, this);
        // 訂閱 Carla 相機影像 ("/carla_camera/image_raw")
        carla_image_sub_ = nh.subscribe("/carla/image_raw", 1,
                                        &ROSHandler::carlaImageCallback, this);
        // 訂閱文字訊息 (範例 topic 名： "chinese_topic")
        msg_sub_ = nh.subscribe("chinese_topic", 1,
                                &ROSHandler::msgCallback, this);

        // 訂閱電池剩餘百分比 (Float32)
        battery_sub_ = nh.subscribe("/soc", 1, &ROSHandler::batteryCallback, this);
    }

signals:
    void newOriginalImage(const sensor_msgs::ImageConstPtr& msg);
    void newCarlaImage(const sensor_msgs::ImageConstPtr& msg);
    void newMessage(const QString& message);
    void newBatteryPercentage(int percentage);

private:
    void originalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        emit newOriginalImage(msg);
    }
    void carlaImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        emit newCarlaImage(msg);
    }
    void msgCallback(const std_msgs::String::ConstPtr& msg) {
        emit newMessage(QString::fromStdString(msg->data));
    }
    void batteryCallback(const std_msgs::Float32::ConstPtr& msg) {
        int percentage = static_cast<int>(msg->data);
        emit newBatteryPercentage(percentage);
    }

    ros::Subscriber original_image_sub_;
    ros::Subscriber carla_image_sub_;
    ros::Subscriber msg_sub_;
    ros::Subscriber battery_sub_;
};

//==============================================================================
// 5. MainWindow：將三個區塊佈局為「左：Carla影像；右上：原本影像；右下：控制面板」
//==============================================================================
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr) : QMainWindow(parent) {
        QWidget* centralWidget = new QWidget(this);
        QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

        // --------------- 左側：Carla 相機影像 (CarCamWidget) ---------------
        CarCamWidget* carlaWidget = new CarCamWidget(centralWidget);
        carlaWidget->setToolTip("Carla Camera");
        // 左側比重設為 3（佔總寬度的 3 份）
        mainLayout->addWidget(carlaWidget, /*比重=*/ 1);

        // --------------- 右側：垂直布局 ---------------
        QWidget* rightContainer = new QWidget(centralWidget);
        QVBoxLayout* rightLayout = new QVBoxLayout(rightContainer);

        // 右上：原本影像 (ImageWidget)
        ImageWidget* originalWidget = new ImageWidget(rightContainer);
        originalWidget->setToolTip("Original Camera");
        // 右上原始影像比重改為 1（佔右側高度 1 份）
        rightLayout->addWidget(originalWidget, /*比重=*/ 1);

        // 右下：控制面板 (ControlWidget)
        ControlWidget* controlWidget = new ControlWidget(rightContainer);
        // 右下控制面板比重改為 1（佔右側高度 1 份）
        rightLayout->addWidget(controlWidget, /*比重=*/ 1);

        // 顯式將上方 index 0、下方 index 1 的 stretch 都設為 1：1
        rightLayout->setStretch(0, 1); // index 0 = originalWidget
        rightLayout->setStretch(1, 1); // index 1 = controlWidget

        // 整個右側容器比重設為 1（佔總寬度的 1 份）
        mainLayout->addWidget(rightContainer, /*比重=*/ 1);

        setCentralWidget(centralWidget);
        setWindowTitle("巡位介面 (左Carla影像 + 右上原始影像 + 右下控制)");

        // ------------- 建立 ROSHandler 並連接信號/槽 -------------
        rosHandler_ = new ROSHandler(this);
        // Carla 影像 → 左側
        connect(rosHandler_, &ROSHandler::newCarlaImage,
                carlaWidget, &CarCamWidget::updateCarlaImage);
        // 原本影像 → 右上
        connect(rosHandler_, &ROSHandler::newOriginalImage,
                originalWidget, &ImageWidget::updateImage);
        // 文字 → 控制視窗
        connect(rosHandler_, &ROSHandler::newMessage,
                controlWidget, &ControlWidget::updateTextBrowser);
        // 電池百分比 → 控制視窗
        connect(rosHandler_, &ROSHandler::newBatteryPercentage,
                controlWidget, &ControlWidget::updateBatteryStatus);

        // ------------- 用 QTimer 不斷呼叫 ros::spinOnce() -------------
        QTimer* rosTimer = new QTimer(this);
        connect(rosTimer, &QTimer::timeout, this, []() {
            if (ros::ok()) ros::spinOnce();
        });
        rosTimer->start(30);
    }

private:
    ROSHandler* rosHandler_;
};

#include "ui.moc"

int main(int argc, char** argv) {
    // 1. 啟動 ROS node
    ros::init(argc, argv, "integrated_dual_image_node");

    // 2. 啟動 Qt Application
    QApplication app(argc, argv);

    MainWindow w;
    w.show();

    return app.exec();
}
