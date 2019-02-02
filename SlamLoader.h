#ifndef SLAMLOADER_PLUGIN_HEADER
#define SLAMLOADER_PLUGIN_HEADER

// qCC
#include <CloudSamplingTools.h>
#include <ReferenceCloud.h>
#include <ccGBLSensor.h>
#include <ccHObject.h>
#include <ccLibAlgorithms.h>
#include <ccPointCloud.h>
#include <plugins/ccStdPluginInterface.h>

#include <ccEntityAction.h>
#include <ccScalarField.h>
#include <plugins/core/qPCL/PclUtils/filters/MLSSmoothingUpsampling.h>
#include <plugins/core/qPCL/PclUtils/utils/PCLConv.h>
#include <plugins/core/qPCL/PclUtils/utils/cc2sm.h>
#include <plugins/core/qPCL/PclUtils/utils/copy.h>
#include <plugins/core/qPCL/PclUtils/utils/sm2cc.h>

// Qt
#include <QFileDialog>
#include <QObject>
#include <QtGui>

// PCL
#include "custom_point_types.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <iostream>

#include "data_model.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

class SlamLoader : public QObject, public ccStdPluginInterface {
  Q_OBJECT
  Q_INTERFACES(ccStdPluginInterface)
  Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.SlamLoader")

public:
  explicit SlamLoader(QObject* parent = nullptr);
  virtual ~SlamLoader() = default;

  virtual void
      onNewSelection(const ccHObject::Container& selectedEntities) override;
  virtual QList<QAction*> getActions() override;

private:
  void doAction();
  QAction* m_action;
  QAction* load_slam_action;
  void DoActionLoadSlam();
  static void DownsampleCloud(ccPointCloud* scan, float resolution);
  void NoiseRemovalFilter(ccPointCloud* scan, float resolution);
  void RGBFilter(ccPointCloud*& scan, int lower_bound, int upper_bound);
  QString GetFirstAvailableSFName(const ccPointCloud* cloud,
                                  const QString& baseName);
  void AddScans(ccHObject* sub_folder_group, json& j);
  void RGBToSF(ccPointCloud* scan);
  void MLSFilter(ccPointCloud* scan, float search_radius);
  data_model workingModel;
};

#endif
