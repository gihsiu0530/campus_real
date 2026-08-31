#ifndef VIO_RVIZ_PANEL_IMAGE_DASHBOARD_PANEL_H
#define VIO_RVIZ_PANEL_IMAGE_DASHBOARD_PANEL_H

#include <array>
#include <memory>

#include <QImage>
#include <QString>
#include <QWidget>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <rviz/panel.h>
#include <sensor_msgs/Image.h>

class QLabel;
class QLineEdit;
class QTimer;

namespace rviz {
class Config;
}

namespace vio_rviz_panel {

class ImageCanvas : public QWidget {
 public:
  explicit ImageCanvas(QWidget* parent = nullptr);

  void setImage(const QImage& image);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QImage image_;
};

class ImageDashboardPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit ImageDashboardPanel(QWidget* parent = nullptr);
  ~ImageDashboardPanel() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 Q_SIGNALS:
  void imageReady(int index, const QImage& image);
  void imageError(int index, const QString& message);

 private Q_SLOTS:
  void showImage(int index, const QImage& image);
  void showError(int index, const QString& message);
  void topicEdited();
  void updateConnectionStatus();

 private:
  struct Tile {
    QLineEdit* topic_edit = nullptr;
    ImageCanvas* canvas = nullptr;
    QLabel* status = nullptr;
    qint64 fps_window_started_ms = 0;
    qint64 last_frame_ms = 0;
    int frames_in_window = 0;
    double fps = 0.0;
    int image_width = 0;
    int image_height = 0;
  };

  QWidget* createImageTile(int index, const QString& title);
  void imageCallback(const sensor_msgs::ImageConstPtr& message, int index);
  void subscribe(int index);
  void subscribeAll();
  void setWaitingStatus(int index, const QString& text);

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  std::array<image_transport::Subscriber, 3> subscribers_;
  std::array<QString, 3> topics_;
  std::array<Tile, 3> tiles_;
  QTimer* status_timer_ = nullptr;
};

}  // namespace vio_rviz_panel

#endif  // VIO_RVIZ_PANEL_IMAGE_DASHBOARD_PANEL_H
