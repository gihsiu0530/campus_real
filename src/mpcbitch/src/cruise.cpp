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
#include <ros/ros.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/display.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QImageReader>
#include <QDebug>

//==============================================================================
// 1. RvizWidget：利用 RViz 的 RenderPanel 與 VisualizationManager 建立 RViz 地圖顯示
//==============================================================================
class RvizWidget : public QWidget {
    Q_OBJECT
public:
    RvizWidget(QWidget* parent = nullptr) : QWidget(parent) {
        QVBoxLayout* layout = new QVBoxLayout(this);

        // 建立 RViz 渲染面板
        render_panel_ = new rviz::RenderPanel();
        layout->addWidget(render_panel_);

        // 建立 VisualizationManager 並連結至渲染面板
        manager_ = new rviz::VisualizationManager(render_panel_);
        render_panel_->initialize(manager_->getSceneManager(), manager_);
        manager_->setFixedFrame("map");
        manager_->initialize();

        // // 切換至 Orbit 視角，並設定參數
        manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
        rviz::ViewController* view_controller = manager_->getViewManager()->getCurrent();
        if (view_controller) {
            view_controller->subProp("Focal Point")->setValue("20;20;0");
            view_controller->subProp("Yaw")->setValue(-1.5708);
            view_controller->subProp("Distance")->setValue("50");
        }

        // // (示範) 建立 MarkerArray 顯示
        // rviz::Display* markerarray_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray", true);
        // if (markerarray_display) {
        //     markerarray_display->subProp("Marker Topic")->setValue("/global_path");
        //     markerarray_display->subProp("Color")->setValue(QColor(255, 0, 0));
        // }

        // // 建立 PointCloud2 顯示 (Lidar)
        // rviz::Display* lidar_display = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", true);
        // if (lidar_display) {
        //     lidar_display->subProp("Topic")->setValue("/aligned_points");
        //     lidar_display->subProp("Style")->setValue("Flat Squares");
        //     lidar_display->subProp("Size (m)")->setValue(0.2);
        //     lidar_display->subProp("Alpha")->setValue(1.0);
        //     lidar_display->subProp("Decay Time")->setValue(0.0);
        //     lidar_display->subProp("Position Transform")->setValue("XYZ");
        //     lidar_display->subProp("Color Transformer")->setValue("Intensity");
        //     lidar_display->subProp("Channel Name")->setValue("Intensity");
        //     lidar_display->subProp("Max Color")->setValue(QColor(255, 85, 0));
        //     lidar_display->subProp("Min Color")->setValue(QColor(255, 85, 0));
        //     lidar_display->subProp("Autocompute Intensity")->setValue(true);
        // }

        //         // 建立 PointCloud2 顯示 (Lidar)
        // rviz::Display* lidar_display = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", true);
        // if (lidar_display) {
        //     lidar_display->subProp("Topic")->setValue("/aligned_points");
        //     lidar_display->subProp("Style")->setValue("Flat Squares");
        //     lidar_display->subProp("Size (m)")->setValue(0.2);
        //     lidar_display->subProp("Alpha")->setValue(1.0);
        //     lidar_display->subProp("Decay Time")->setValue(0.0);
        //     lidar_display->subProp("Position Transform")->setValue("XYZ");
        //     lidar_display->subProp("Color Transformer")->setValue("Intensity");
        //     lidar_display->subProp("Channel Name")->setValue("Intensity");
        //     lidar_display->subProp("Max Color")->setValue(QColor(255, 85, 0));
        //     lidar_display->subProp("Min Color")->setValue(QColor(255, 85, 0));
        //     lidar_display->subProp("Autocompute Intensity")->setValue(true);
        // }
        
        // // 建立 PointCloud2 顯示 (地圖)
        // rviz::Display* map_display = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", true);
        // if (map_display) {
        //     map_display->subProp("Topic")->setValue("/globalmap");
        //     map_display->subProp("Style")->setValue("Flat Squares");
        //     map_display->subProp("Size (m)")->setValue(0.05);
        //     map_display->subProp("Alpha")->setValue(1.0);
        //     map_display->subProp("Decay Time")->setValue(0.0);
        //     map_display->subProp("Position Transform")->setValue("XYZ");
        //     map_display->subProp("Color Transformer")->setValue("Intensity");
        //     map_display->subProp("Channel Name")->setValue("z");
        //     map_display->subProp("Use rainbow")->setValue(true);
        //     map_display->subProp("Invert Rainbow")->setValue(false);
        //     map_display->subProp("Min Color")->setValue(QColor(0, 0, 0));
        //     map_display->subProp("Max Color")->setValue(QColor(255, 255, 255));
        //     map_display->subProp("Autocompute Intensity")->setValue(true);
        // }
        
        // // 建立 TF 顯示
        // rviz::Display* TF_display = manager_->createDisplay("rviz/TF", "TF", true);
        // if (TF_display) {
        //     TF_display->subProp("Show Axes")->setValue(true);
        //     TF_display->subProp("Marker Scale")->setValue(10);
        // }
        
        // // 建立 Marker 顯示
        // rviz::Display* marker_display = manager_->createDisplay("rviz/Marker", "Marker", true);
        // if (marker_display) {
        //     marker_display->subProp("Marker Topic")->setValue("/vehicle_marker");
        // }

        // // 建立 Path 顯示
        // rviz::Display* path_display = manager_->createDisplay("rviz/Path", "Path", true);
        // if (path_display) {
        //     path_display->subProp("Topic")->setValue("/vehicle_path");
        //     path_display->subProp("Color")->setValue(QColor(255, 255, 255));
        // }
          // 全域參數 Global Options
        // manager_->setFixedFrame("map");                               // Fixed Frame = map
        // manager_->setUpdateRate(30.0f);                                // Frame Rate = 30 Hz
        // manager_->setDefaultLightVisible(true);                        // Default Light = true
        // render_panel_->setBackgroundColor(QColor(48, 48, 48));        // 背景顏色 = (48,48,48)
        // manager_->initialize();

        // 設定視角（Views）：TopDownOrtho
        // manager_->getViewManager()->setCurrent("rviz/TopDownOrtho");
        // rviz::ViewController* vc = manager_->getViewManager()->getCurrent();
        // if (vc) {
        //     vc->subProp("Angle")->setValue(-0.005004099570214748);
        //     vc->subProp("Near Clip Distance")->setValue(0.009999999776482582);
        //     vc->subProp("Invert Z Axis")->setValue(false);
        //     vc->subProp("Target Frame")->setValue("<Fixed Frame>");
        //     vc->subProp("Scale")->setValue(64.3658447265625);
        //     vc->subProp("X")->setValue(14.080611228942871);
        //     vc->subProp("Y")->setValue(22.65968132019043);
        // }

        // --- Displays: 按照您提供的 YAML 依序建立並設定屬性 ---

        // 1. Grid (rviz/Grid)
        rviz::Display* grid_display = manager_->createDisplay("rviz/Grid", "Grid", true);
        if (grid_display) {
            grid_display->subProp("Alpha")->setValue(0.5);
            grid_display->subProp("Cell Size")->setValue(1.0);
            grid_display->subProp("Plane Cell Count")->setValue(10);
            grid_display->subProp("Plane")->setValue("XY");
            grid_display->subProp("Line Style")->setValue("Lines");
            grid_display->subProp("Line Width")->setValue(0.029999999329447746);
            grid_display->subProp("Color")->setValue(QColor(160, 160, 164));
            grid_display->subProp("Normal Cell Count")->setValue(0);
            grid_display->subProp("Offset")->setValue("0;0;0");
            grid_display->subProp("Reference Frame")->setValue("<Fixed Frame>");
        }

        // 2. Map (rviz/Map)
        rviz::Display* map_display = manager_->createDisplay("rviz/Map", "Map", true);
        if (map_display) {
            map_display->subProp("Topic")->setValue("/map");
            map_display->subProp("Alpha")->setValue(0.7);
            map_display->subProp("Color Scheme")->setValue("map");
            map_display->subProp("Draw Behind")->setValue(false);
        }

        // 3. Pose（Goal）(rviz/Pose)
        rviz::Display* pose_goal_display = manager_->createDisplay("rviz/Pose", "Pose Goal", true);
        if (pose_goal_display) {
            pose_goal_display->subProp("Topic")->setValue("/move_base_simple/goal");
            pose_goal_display->subProp("Axes Length")->setValue(1.0);
            pose_goal_display->subProp("Axes Radius")->setValue(0.10000000149011612);
            pose_goal_display->subProp("Head Length")->setValue(0.30000001192092896);
            pose_goal_display->subProp("Head Radius")->setValue(0.10000000149011612);
            pose_goal_display->subProp("Shaft Length")->setValue(1.0);
            pose_goal_display->subProp("Shaft Radius")->setValue(0.05000000074505806);
            pose_goal_display->subProp("Color")->setValue(QColor(255, 25, 0));
        }

        // 4. Pose（Start）(rviz/Pose)
        rviz::Display* pose_start_display = manager_->createDisplay("rviz/Pose", "Pose Start", true);
        if (pose_start_display) {
            pose_start_display->subProp("Topic")->setValue("/move_base_simple/start");
            pose_start_display->subProp("Axes Length")->setValue(1.0);
            pose_start_display->subProp("Axes Radius")->setValue(0.10000000149011612);
            pose_start_display->subProp("Head Length")->setValue(0.30000001192092896);
            pose_start_display->subProp("Head Radius")->setValue(0.10000000149011612);
            pose_start_display->subProp("Shaft Length")->setValue(1.0);
            pose_start_display->subProp("Shaft Radius")->setValue(0.05000000074505806);
            pose_start_display->subProp("Color")->setValue(QColor(255, 25, 0));
        }

        // 5. Path (/mirror/sPath) (rviz/Path)
        // rviz::Display* mirror_sPath_display = manager_->createDisplay("rviz/Path", "Path Mirror S", true);
        // if (mirror_sPath_display) {
        //     mirror_sPath_display->subProp("Topic")->setValue("/mirror/sPath");
        //     mirror_sPath_display->subProp("Color")->setValue(QColor(196, 160, 0));
        //     mirror_sPath_display->subProp("Line Style")->setValue("Lines");
        //     mirror_sPath_display->subProp("Line Width")->setValue(0.029999999329447746);
        //     mirror_sPath_display->subProp("Head Diameter")->setValue(0.30000001192092896);
        //     mirror_sPath_display->subProp("Head Length")->setValue(0.20000000298023224);
        //     mirror_sPath_display->subProp("Length")->setValue(0.30000001192092896);
        //     mirror_sPath_display->subProp("Pose Style")->setValue("None");
        //     mirror_sPath_display->subProp("Pose Color")->setValue(QColor(255, 85, 255));
        //     mirror_sPath_display->subProp("Radius")->setValue(0.029999999329447746);
        //     mirror_sPath_display->subProp("Shaft Diameter")->setValue(0.10000000149011612);
        //     mirror_sPath_display->subProp("Shaft Length")->setValue(0.10000000149011612);
        // }

        // 6. MarkerArray (/pathNodes) – Disabled
        // rviz::Display* pathNodes_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray PathNodes", false);
        // if (pathNodes_display) {
        //     pathNodes_display->subProp("Marker Topic")->setValue("/pathNodes");
        // }

        // 7. MarkerArray (/pathVehicle) – Disabled
        // rviz::Display* pathVehicle_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray PathVehicle", false);
        // if (pathVehicle_display) {
        //     pathVehicle_display->subProp("Marker Topic")->setValue("/pathVehicle");
        // }

        // 8. Path (/sPath) (rviz/Path)
        rviz::Display* sPath_display = manager_->createDisplay("rviz/Path", "Path S", true);
        if (sPath_display) {
            sPath_display->subProp("Topic")->setValue("/sPath");
            sPath_display->subProp("Color")->setValue(QColor(25, 255, 0));
            sPath_display->subProp("Line Style")->setValue("Lines");
            sPath_display->subProp("Line Width")->setValue(0.029999999329447746);
            sPath_display->subProp("Head Diameter")->setValue(0.30000001192092896);
            sPath_display->subProp("Head Length")->setValue(0.20000000298023224);
            sPath_display->subProp("Length")->setValue(0.30000001192092896);
            sPath_display->subProp("Pose Style")->setValue("None");
            sPath_display->subProp("Pose Color")->setValue(QColor(255, 85, 255));
            sPath_display->subProp("Radius")->setValue(0.029999999329447746);
            sPath_display->subProp("Shaft Diameter")->setValue(0.10000000149011612);
            sPath_display->subProp("Shaft Length")->setValue(0.10000000149011612);
        }

        // 9. MarkerArray (/sPathNodes) – Enabled
        rviz::Display* sPathNodes_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray SPathNodes", true);
        if (sPathNodes_display) {
            sPathNodes_display->subProp("Marker Topic")->setValue("/sPathNodes");
        }

        // 10. MarkerArray (/sPathVehicle) – Enabled
        // rviz::Display* sPathVehicle_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray SPathVehicle", true);
        // if (sPathVehicle_display) {
        //     sPathVehicle_display->subProp("Marker Topic")->setValue("/sPathVehicle");
        // }

        // 11. PointStamped (/goal) (rviz/PointStamped)
        // rviz::Display* pointStamped_display = manager_->createDisplay("rviz/PointStamped", "PointStamped", true);
        // if (pointStamped_display) {
        //     pointStamped_display->subProp("Topic")->setValue("/goal");
        //     pointStamped_display->subProp("Color")->setValue(QColor(204, 41, 204));
        //     pointStamped_display->subProp("History Length")->setValue(1);
        //     pointStamped_display->subProp("Radius")->setValue(0.20000000298023224);
        // }

        // // 12. Marker (/mirror_line) (rviz/Marker)
        // rviz::Display* mirrorLine_display = manager_->createDisplay("rviz/Marker", "Marker MirrorLine", true);
        // if (mirrorLine_display) {
        //     mirrorLine_display->subProp("Marker Topic")->setValue("/mirror_line");
        // }

        // // 13. MarkerArray (/mirror/sPathNodes) – Enabled
        // rviz::Display* mirror_sPathNodes_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray MirrorSPathNodes", true);
        // if (mirror_sPathNodes_display) {
        //     mirror_sPathNodes_display->subProp("Marker Topic")->setValue("/mirror/sPathNodes");
        // }

        // // 14. MarkerArray (/mirror/sPathVehicle) – Enabled
        // rviz::Display* mirror_sPathVehicle_display = manager_->createDisplay("rviz/MarkerArray", "MarkerArray MirrorSPathVehicle", true);
        // if (mirror_sPathVehicle_display) {
        //     mirror_sPathVehicle_display->subProp("Marker Topic")->setValue("/mirror/sPathVehicle");
        // }

        // 15. Marker (/vehicle_marker) (rviz/Marker)
        // rviz::Display* vehicleMarker_display = manager_->createDisplay("rviz/Marker", "Marker Vehicle", true);
        // if (vehicleMarker_display) {
        //     vehicleMarker_display->subProp("Marker Topic")->setValue("/vehicle_marker");
        // }

        // 16. Path (/vehicle_path) (rviz/Path)
        // rviz::Display* vehiclePath_display = manager_->createDisplay("rviz/Path", "Path Vehicle", true);
        // if (vehiclePath_display) {
        //     vehiclePath_display->subProp("Topic")->setValue("/vehicle_path");
        //     vehiclePath_display->subProp("Color")->setValue(QColor(117, 80, 123));
        //     vehiclePath_display->subProp("Line Style")->setValue("Lines");
        //     vehiclePath_display->subProp("Line Width")->setValue(0.029999999329447746);
        //     vehiclePath_display->subProp("Head Diameter")->setValue(0.30000001192092896);
        //     vehiclePath_display->subProp("Head Length")->setValue(0.20000000298023224);
        //     vehiclePath_display->subProp("Length")->setValue(0.30000001192092896);
        //     vehiclePath_display->subProp("Pose Style")->setValue("None");
        //     vehiclePath_display->subProp("Pose Color")->setValue(QColor(255, 85, 255));
        //     vehiclePath_display->subProp("Radius")->setValue(0.029999999329447746);
        //     vehiclePath_display->subProp("Shaft Diameter")->setValue(0.10000000149011612);
        //     vehiclePath_display->subProp("Shaft Length")->setValue(0.10000000149011612);
        // }

        
        manager_->startUpdate();
    }
private:
    rviz::RenderPanel* render_panel_;
    rviz::VisualizationManager* manager_;
};

//==============================================================================
// 2. ImageWidget：用於顯示 ROS 影像（倒車影像區）
//==============================================================================
class ImageWidget : public QLabel {
    Q_OBJECT
public:
    ImageWidget(QWidget* parent = nullptr) : QLabel(parent) {
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
            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

//==============================================================================
// 3. ControlWidget：右下區域，包含對話框、電池顯示、剩餘里程、現在時間與按鈕
//==============================================================================
class ControlWidget : public QWidget {
    Q_OBJECT
public:
    ControlWidget(QWidget* parent = nullptr) : QWidget(parent) {
        // 建立主要水平布局，分左右兩區
        QHBoxLayout* mainLayout = new QHBoxLayout(this);

        // 左側：對話框
        textBrowser = new QTextBrowser(this);
        textBrowser->setStyleSheet("border: 1px solid black;");
        mainLayout->addWidget(textBrowser, 1);

        // 右側：建立一個垂直布局，用來放電池區、剩餘里程、現在時間與按鈕
        QWidget* rightWidget = new QWidget(this);
        QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);

        // 新增【電池區】：利用水平布局放置電池 icon 與百分比文字
        QWidget* batteryWidget = new QWidget(this);
        QHBoxLayout* batteryLayout = new QHBoxLayout(batteryWidget);
        batteryButton = new QPushButton(this);
        // 設定預設 icon (依電量狀態可動態更新)
        QPixmap pix("/home/king/Downloads/100.jpg");
        QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
        batteryButton->setIcon(QIcon(rotatedPixmap));
        batteryButton->setIconSize(QSize(100, 100));
        batteryButton->setFlat(true);
        batteryPercentageLabel = new QLabel("80%", this);
        batteryPercentageLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        batteryPercentageLabel->setStyleSheet("font-size: 50px;");
        batteryLayout->addWidget(batteryButton);
        batteryLayout->addWidget(batteryPercentageLabel);
        batteryLayout->setContentsMargins(0, 0, 0, 0);
        rightLayout->addWidget(batteryWidget, 0);

        // 新增【剩餘里程】顯示，放在現在時間上方
        mileageLabel = new QLabel(this);
        mileageLabel->setAlignment(Qt::AlignCenter);
        mileageLabel->setText("剩餘里程：100 km"); // 可根據需要動態更新數值
        mileageLabel->setStyleSheet("font-size: 32px;");
        rightLayout->addWidget(mileageLabel, 1);

        // 右側上半部：現在時間顯示
        timeLabel = new QLabel(this);
        timeLabel->setAlignment(Qt::AlignCenter);
        rightLayout->addWidget(timeLabel, 1);

        // 右側下半部：按鈕區，垂直排列並填滿
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

        mainLayout->addWidget(rightWidget, 1);

        // QTimer 每秒更新一次時間
        QTimer* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &ControlWidget::updateTime);
        timer->start(1000);
        updateTime();  // 初始更新

        // 按鈕訊號連接
        connect(stopButton, &QPushButton::clicked, this, &ControlWidget::handleStop);
        connect(finishButton, &QPushButton::clicked, this, &ControlWidget::handleFinish);

        // 連接電池 icon 的點擊訊號，點擊後切換至電池詳細資訊畫面
        connect(batteryButton, &QPushButton::clicked, this, &ControlWidget::openBatteryUI);
        updateBatteryStatus(50);
        updateMileage(21.6);
    }
public Q_SLOTS:
    // 更新對話框訊息
    void updateTextBrowser(const QString &msg) {
        textBrowser->append(msg);
        if (msg.contains("方向盤已回正，結束停車")) {
            finishButton->setEnabled(true);
        }
    }
    // 更新時間顯示
    void updateTime() {
        QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
        timeLabel->setText("現在時間：" + currentTime);
        timeLabel->setStyleSheet("font-size: 32px;");
    }
    // 點擊電池 icon 時，開啟電池詳細資訊畫面
    void openBatteryUI() {
        QWidget* batteryScreen = new QWidget;
        batteryScreen->setWindowTitle("電池詳細資訊");
        batteryScreen->resize(400, 300);
        QVBoxLayout* layout = new QVBoxLayout(batteryScreen);
        QLabel* infoLabel = new QLabel("這裡顯示更詳細的電池資訊...", batteryScreen);
        layout->addWidget(infoLabel);
        batteryScreen->show();
    }
    // 根據電池狀態更新電池圖示與百分比
    void updateBatteryStatus(int percentage) {
        batteryPercentageLabel->setText(QString::number(percentage) + "%");
        if (percentage >= 90) {
            QPixmap pix("/home/king/Downloads/100.jpg");
            QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
            batteryButton->setIcon(QIcon(rotatedPixmap));
            batteryButton->setIconSize(QSize(100, 100));
            batteryButton->setFlat(true);
        } else if (percentage >= 70 && percentage < 90) {
            QPixmap pix("/home/king/Downloads/80.jpg");
            QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
            batteryButton->setIcon(QIcon(rotatedPixmap));
            batteryButton->setIconSize(QSize(100, 100));
            batteryButton->setFlat(true);
        } else if (percentage >= 50 && percentage < 70) {
            QPixmap pix("/home/king/Downloads/60.jpg");
            QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
            batteryButton->setIcon(QIcon(rotatedPixmap));
            batteryButton->setIconSize(QSize(100, 100));
            batteryButton->setFlat(true);
        } else if (percentage >= 30 && percentage < 50) {
            QPixmap pix("/home/king/Downloads/40.jpg");
            QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
            batteryButton->setIcon(QIcon(rotatedPixmap));
            batteryButton->setIconSize(QSize(100, 100));
            batteryButton->setFlat(true);
        } else if (percentage >= 10 && percentage < 30) {
            QPixmap pix("/home/king/Downloads/20.jpg");
            QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
            batteryButton->setIcon(QIcon(rotatedPixmap));
            batteryButton->setIconSize(QSize(100, 100));
            batteryButton->setFlat(true);
        } else {
            QPixmap pix("/home/king/Downloads/0.jpg");
            QPixmap rotatedPixmap = pix.transformed(QTransform().rotate(90));
            batteryButton->setIcon(QIcon(rotatedPixmap));
            batteryButton->setIconSize(QSize(100, 100));
            batteryButton->setFlat(true);
        }
    }
    void updateMileage(double mileage) {
        mileageLabel->setText("剩餘里程：" + QString::number(mileage) + " km");
    }
    void handleStop() {
        ROS_INFO("緊急停止按鈕被按下！");
        // 可加入緊急停止相關邏輯
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
// 4. ROSHandler：訂閱 ROS 的影像與文字訊息，並發送信號
//==============================================================================
class ROSHandler : public QObject {
    Q_OBJECT
public:
    ROSHandler(QObject* parent = nullptr) : QObject(parent) {
        ros::NodeHandle nh;
        // 訂閱影像話題
        image_sub_ = nh.subscribe("/camera/color/image_raw", 1, &ROSHandler::imageCallback, this);
        // 訂閱文字訊息
        msg_sub_ = nh.subscribe("/reverse_status", 1, &ROSHandler::msgCallback, this);
    }
signals:
    void newImage(const sensor_msgs::ImageConstPtr& msg);
    void newMessage(const QString& message);
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        emit newImage(msg);
    }
    void msgCallback(const std_msgs::String::ConstPtr& msg) {
        emit newMessage(QString::fromStdString(msg->data));
    }
    ros::Subscriber image_sub_;
    ros::Subscriber msg_sub_;
};

//==============================================================================
// 5. MainWindow：整體介面配置
//    左側：RViz 地圖顯示
//    右側上半部：倒車影像
//    右側下半部：ControlWidget（包含對話框、電池顯示、剩餘里程、現在時間與按鈕）
//==============================================================================
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr) : QMainWindow(parent) {
        QWidget* centralWidget = new QWidget(this);
        QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

        // 左側：RViz 顯示
        RvizWidget* rvizWidget = new RvizWidget(centralWidget);
        mainLayout->addWidget(rvizWidget, 1);

        // 右側：垂直排列倒車影像與 ControlWidget
        QWidget* rightWidget = new QWidget(centralWidget);
        QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);

        // 右側上半部：倒車影像
        ImageWidget* imageWidget = new ImageWidget(rightWidget);
        rightLayout->addWidget(imageWidget, 1);

        // 右側下半部：ControlWidget
        ControlWidget* controlWidget = new ControlWidget(rightWidget);
        rightLayout->addWidget(controlWidget, 1);

        mainLayout->addWidget(rightWidget, 1);
        setCentralWidget(centralWidget);
        setWindowTitle("巡位介面");

        // 建立 ROSHandler 並連接影像與文字訊號
        rosHandler = new ROSHandler(this);
        connect(rosHandler, &ROSHandler::newImage, imageWidget, &ImageWidget::updateImage);
        connect(rosHandler, &ROSHandler::newMessage, controlWidget, &ControlWidget::updateTextBrowser);

        // 融合 ROS 與 Qt 事件循環
        QTimer* rosTimer = new QTimer(this);
        connect(rosTimer, &QTimer::timeout, [](){
            ros::spinOnce();
        });
        rosTimer->start(30);
    }
private:
    ROSHandler* rosHandler;
};

#include "cruise.moc"

int main(int argc, char** argv) {
    ros::init(argc, argv, "integrated_display_node");
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}
