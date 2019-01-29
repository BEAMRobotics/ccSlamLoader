#ifndef SLAMLOADER_PLUGIN_HEADER
#define SLAMLOADER_PLUGIN_HEADER

//qCC
#include <plugins/ccStdPluginInterface.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <sm2cc.h>
#include <ReferenceCloud.h>
#include <CloudSamplingTools.h>

//Qt
#include <QObject>
#include <QtGui>
#include <QFileDialog>

//PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#include "data_model.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

class SlamLoader : public QObject, public ccStdPluginInterface {
 Q_OBJECT
  Q_INTERFACES(ccStdPluginInterface)
  Q_PLUGIN_METADATA(IID
                        "cccorp.cloudcompare.plugin.SlamLoader")

 public:

  explicit SlamLoader(QObject *parent = nullptr);
  virtual ~SlamLoader() = default;

  virtual void onNewSelection(const ccHObject::Container &selectedEntities) override;
  virtual QList<QAction *> getActions() override;

 private:

  void doAction();
  QAction *m_action;
  QAction *load_slam_action;
  void DoActionLoadSlam();

  void AddScans(ccHObject *sub_folder_group, json &j);

  data_model workingModel;
};

#endif
