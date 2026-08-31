#ifndef VIO_RVIZ_PANEL_SINGLE_IMAGE_PANEL_H
#define VIO_RVIZ_PANEL_SINGLE_IMAGE_PANEL_H

#include <QImage>
#include <QString>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <rviz/panel.h>
#include <sensor_msgs/Image.h>

#include "vio_rviz_panel/image_dashboard_panel.h"

class QLabel;

namespace rviz {
class Config;
}

namespace vio_rviz_panel {

class SingleImagePanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit SingleImagePanel(QWidget* parent = nullptr);
  ~SingleImagePanel() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 Q_SIGNALS:
  void imageReady(const QImage& image);

 private Q_SLOTS:
  void showImage(const QImage& image);

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& message);
  void subscribe();
  void applySizePreference();

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber subscriber_;

  QString display_title_;
  QString topic_;
  bool prefer_square_ = false;

  QLabel* heading_ = nullptr;
  ImageCanvas* canvas_ = nullptr;
};

}  // namespace vio_rviz_panel

#endif  // VIO_RVIZ_PANEL_SINGLE_IMAGE_PANEL_H
