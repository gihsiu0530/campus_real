#include "vio_rviz_panel/single_image_panel.h"

#include <QLabel>
#include <QVBoxLayout>

#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <rviz/config.h>
#include <sensor_msgs/image_encodings.h>

namespace vio_rviz_panel {

SingleImagePanel::SingleImagePanel(QWidget* parent)
    : rviz::Panel(parent), image_transport_(node_handle_) {
  display_title_ = tr("影像 Topic");
  topic_ = "/camera/image";

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(5, 5, 5, 5);
  layout->setSpacing(4);

  heading_ = new QLabel(display_title_, this);
  QFont heading_font = heading_->font();
  heading_font.setBold(true);
  heading_font.setPointSize(heading_font.pointSize() + 1);
  heading_->setFont(heading_font);
  layout->addWidget(heading_);

  canvas_ = new ImageCanvas(this);
  layout->addWidget(canvas_, 1);

  connect(this, &SingleImagePanel::imageReady,
          this, &SingleImagePanel::showImage, Qt::QueuedConnection);

  applySizePreference();
  subscribe();
}

SingleImagePanel::~SingleImagePanel() {
  subscriber_.shutdown();
}

void SingleImagePanel::imageCallback(
    const sensor_msgs::ImageConstPtr& message) {
  try {
    const cv_bridge::CvImageConstPtr converted = cv_bridge::toCvShare(
        message, sensor_msgs::image_encodings::RGB8);
    const QImage view(converted->image.data,
                      converted->image.cols,
                      converted->image.rows,
                      static_cast<int>(converted->image.step),
                      QImage::Format_RGB888);
    Q_EMIT imageReady(view.copy());
  } catch (const cv_bridge::Exception& error) {
    ROS_WARN_THROTTLE(2.0, "Unable to display image topic %s: %s",
                      topic_.toStdString().c_str(), error.what());
  }
}

void SingleImagePanel::showImage(const QImage& image) {
  if (image.isNull()) {
    return;
  }

  canvas_->setImage(image);
}

void SingleImagePanel::subscribe() {
  subscriber_.shutdown();

  subscriber_ = image_transport_.subscribe(
      topic_.toStdString(), 1, &SingleImagePanel::imageCallback, this);
}

void SingleImagePanel::applySizePreference() {
  if (prefer_square_) {
    canvas_->setMinimumSize(200, 120);
  } else {
    canvas_->setMinimumSize(200, 90);
  }
}

void SingleImagePanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);

  QString value;
  if (config.mapGetString("DisplayTitle", &value) &&
      !value.trimmed().isEmpty()) {
    display_title_ = value.trimmed();
  }
  if (config.mapGetString("Topic", &value) && !value.trimmed().isEmpty()) {
    topic_ = value.trimmed();
  }
  config.mapGetBool("PreferSquare", &prefer_square_);

  heading_->setText(display_title_);
  applySizePreference();
  subscribe();
}

void SingleImagePanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("DisplayTitle", display_title_);
  config.mapSetValue("Topic", topic_);
  config.mapSetValue("PreferSquare", prefer_square_);
}

}  // namespace vio_rviz_panel

PLUGINLIB_EXPORT_CLASS(vio_rviz_panel::SingleImagePanel, rviz::Panel)
