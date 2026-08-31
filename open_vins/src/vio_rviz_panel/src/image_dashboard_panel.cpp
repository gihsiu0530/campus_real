#include "vio_rviz_panel/image_dashboard_panel.h"

#include <algorithm>

#include <QDateTime>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPaintEvent>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include <boost/bind/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/config.h>
#include <sensor_msgs/image_encodings.h>

namespace vio_rviz_panel {
namespace {

constexpr int kTileCount = 3;
constexpr qint64 kDisconnectedAfterMs = 2500;

const char* const kConfigKeys[kTileCount] = {
    "ZedImageTopic",
    "SegmentationTopic",
    "TrajectoryPlotTopic",
};

const char* const kTitles[kTileCount] = {
    "ZED2i 左相機",
    "語意分割",
    "軌跡圖",
};

QString onlineStyle() {
  return "QLabel { color: #63d471; }";
}

QString waitingStyle() {
  return "QLabel { color: #e5b567; }";
}

QString errorStyle() {
  return "QLabel { color: #ff6b6b; }";
}

}  // namespace

ImageCanvas::ImageCanvas(QWidget* parent) : QWidget(parent) {
  setMinimumSize(200, 90);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void ImageCanvas::setImage(const QImage& image) {
  image_ = image;
  update();
}

void ImageCanvas::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);

  QPainter painter(this);
  painter.fillRect(rect(), QColor(18, 18, 18));

  if (image_.isNull()) {
    painter.setPen(QColor(155, 155, 155));
    painter.drawText(rect(), Qt::AlignCenter, tr("等待影像…"));
    return;
  }

  const QSize target_size = image_.size().scaled(size(), Qt::KeepAspectRatio);
  const QRect target_rect(
      QPoint((width() - target_size.width()) / 2,
             (height() - target_size.height()) / 2),
      target_size);

  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter.drawImage(target_rect, image_);
}

ImageDashboardPanel::ImageDashboardPanel(QWidget* parent)
    : rviz::Panel(parent), image_transport_(node_handle_) {
  topics_[0] = "/zed2i/zed_node/left/image_rect_color";
  topics_[1] = "/senpai/seg_cls4_full";
  topics_[2] = "/senpai/trajectory_plot";

  auto* root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(4, 4, 4, 4);
  root_layout->setSpacing(4);

  auto* heading = new QLabel(tr("VIO 影像監看"), this);
  QFont heading_font = heading->font();
  heading_font.setBold(true);
  heading_font.setPointSize(heading_font.pointSize() + 2);
  heading->setFont(heading_font);
  root_layout->addWidget(heading);

  auto* splitter = new QSplitter(Qt::Vertical, this);
  splitter->setChildrenCollapsible(false);
  for (int index = 0; index < kTileCount; ++index) {
    splitter->addWidget(createImageTile(index, tr(kTitles[index])));
  }
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  splitter->setStretchFactor(2, 1);
  root_layout->addWidget(splitter, 1);

  connect(this, &ImageDashboardPanel::imageReady,
          this, &ImageDashboardPanel::showImage, Qt::QueuedConnection);
  connect(this, &ImageDashboardPanel::imageError,
          this, &ImageDashboardPanel::showError, Qt::QueuedConnection);

  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout,
          this, &ImageDashboardPanel::updateConnectionStatus);
  status_timer_->start(1000);

  subscribeAll();
}

ImageDashboardPanel::~ImageDashboardPanel() {
  for (auto& subscriber : subscribers_) {
    subscriber.shutdown();
  }
}

QWidget* ImageDashboardPanel::createImageTile(int index, const QString& title) {
  auto* tile_frame = new QFrame(this);
  tile_frame->setFrameShape(QFrame::StyledPanel);
  auto* layout = new QVBoxLayout(tile_frame);
  layout->setContentsMargins(5, 5, 5, 5);
  layout->setSpacing(3);

  auto* title_label = new QLabel(title, tile_frame);
  QFont title_font = title_label->font();
  title_font.setBold(true);
  title_label->setFont(title_font);
  layout->addWidget(title_label);

  tiles_[index].topic_edit = new QLineEdit(topics_[index], tile_frame);
  tiles_[index].topic_edit->setToolTip(tr("輸入 sensor_msgs/Image 的基礎 topic，按 Enter 套用"));
  connect(tiles_[index].topic_edit, &QLineEdit::editingFinished,
          this, &ImageDashboardPanel::topicEdited);
  layout->addWidget(tiles_[index].topic_edit);

  tiles_[index].canvas = new ImageCanvas(tile_frame);
  layout->addWidget(tiles_[index].canvas, 1);

  tiles_[index].status = new QLabel(tr("等待 publisher…"), tile_frame);
  tiles_[index].status->setStyleSheet(waitingStyle());
  layout->addWidget(tiles_[index].status);

  return tile_frame;
}

void ImageDashboardPanel::imageCallback(
    const sensor_msgs::ImageConstPtr& message, int index) {
  try {
    const cv_bridge::CvImageConstPtr converted = cv_bridge::toCvShare(
        message, sensor_msgs::image_encodings::RGB8);
    const QImage view(converted->image.data,
                      converted->image.cols,
                      converted->image.rows,
                      static_cast<int>(converted->image.step),
                      QImage::Format_RGB888);
    Q_EMIT imageReady(index, view.copy());
  } catch (const cv_bridge::Exception& error) {
    Q_EMIT imageError(index, QString::fromStdString(error.what()));
  }
}

void ImageDashboardPanel::showImage(int index, const QImage& image) {
  if (index < 0 || index >= kTileCount || image.isNull()) {
    return;
  }

  Tile& tile = tiles_[index];
  tile.canvas->setImage(image);
  tile.image_width = image.width();
  tile.image_height = image.height();

  const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
  tile.last_frame_ms = now_ms;
  if (tile.fps_window_started_ms == 0) {
    tile.fps_window_started_ms = now_ms;
  }
  ++tile.frames_in_window;

  const qint64 elapsed_ms = now_ms - tile.fps_window_started_ms;
  if (elapsed_ms >= 1000) {
    tile.fps = 1000.0 * static_cast<double>(tile.frames_in_window) /
               static_cast<double>(elapsed_ms);
    tile.frames_in_window = 0;
    tile.fps_window_started_ms = now_ms;
  }

  tile.status->setText(
      tr("● 已連線  %1 × %2  |  %3 FPS")
          .arg(tile.image_width)
          .arg(tile.image_height)
          .arg(tile.fps, 0, 'f', 1));
  tile.status->setStyleSheet(onlineStyle());
}

void ImageDashboardPanel::showError(int index, const QString& message) {
  if (index < 0 || index >= kTileCount) {
    return;
  }

  tiles_[index].status->setText(tr("影像格式錯誤：%1").arg(message));
  tiles_[index].status->setStyleSheet(errorStyle());
}

void ImageDashboardPanel::topicEdited() {
  auto* edited = qobject_cast<QLineEdit*>(sender());
  if (edited == nullptr) {
    return;
  }

  for (int index = 0; index < kTileCount; ++index) {
    if (tiles_[index].topic_edit != edited) {
      continue;
    }

    const QString topic = edited->text().trimmed();
    if (topic.isEmpty() || topic == topics_[index]) {
      edited->setText(topics_[index]);
      return;
    }

    topics_[index] = topic;
    subscribe(index);
    Q_EMIT configChanged();
    return;
  }
}

void ImageDashboardPanel::subscribe(int index) {
  subscribers_[index].shutdown();
  tiles_[index].last_frame_ms = 0;
  tiles_[index].fps_window_started_ms = 0;
  tiles_[index].frames_in_window = 0;
  tiles_[index].fps = 0.0;
  setWaitingStatus(index, tr("等待 publisher…"));

  subscribers_[index] = image_transport_.subscribe(
      topics_[index].toStdString(), 1,
      boost::bind(&ImageDashboardPanel::imageCallback, this,
                  boost::placeholders::_1, index));
}

void ImageDashboardPanel::subscribeAll() {
  for (int index = 0; index < kTileCount; ++index) {
    subscribe(index);
  }
}

void ImageDashboardPanel::setWaitingStatus(int index, const QString& text) {
  tiles_[index].status->setText(text);
  tiles_[index].status->setStyleSheet(waitingStyle());
}

void ImageDashboardPanel::updateConnectionStatus() {
  const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
  for (int index = 0; index < kTileCount; ++index) {
    Tile& tile = tiles_[index];
    if (tile.last_frame_ms != 0 &&
        now_ms - tile.last_frame_ms > kDisconnectedAfterMs) {
      setWaitingStatus(index, tr("影像中斷，等待新畫面…"));
    } else if (tile.last_frame_ms == 0 &&
               subscribers_[index].getNumPublishers() == 0) {
      setWaitingStatus(index, tr("等待 publisher…"));
    }
  }
}

void ImageDashboardPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);

  for (int index = 0; index < kTileCount; ++index) {
    QString topic;
    if (config.mapGetString(kConfigKeys[index], &topic) &&
        !topic.trimmed().isEmpty()) {
      topics_[index] = topic.trimmed();
      tiles_[index].topic_edit->setText(topics_[index]);
    }
  }
  subscribeAll();
}

void ImageDashboardPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  for (int index = 0; index < kTileCount; ++index) {
    config.mapSetValue(kConfigKeys[index], topics_[index]);
  }
}

}  // namespace vio_rviz_panel

PLUGINLIB_EXPORT_CLASS(vio_rviz_panel::ImageDashboardPanel, rviz::Panel)
