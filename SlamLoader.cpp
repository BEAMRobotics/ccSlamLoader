#include "SlamLoader.h"

SlamLoader::SlamLoader(QObject* parent /*=0*/)
    : QObject(parent),
      ccStdPluginInterface(":/CC/plugin/SlamLoader/info.json"),
      m_action(nullptr) {
  load_slam_action = new QAction("Import SLAM results", this);
  connect(load_slam_action, SIGNAL(triggered()), this,
          SLOT(DoActionLoadSlam()));
}

void SlamLoader::onNewSelection(const ccHObject::Container& selectedEntities) {
  if (m_action == nullptr) { return; }
  m_action->setEnabled(!selectedEntities.empty());
}

QList<QAction*> SlamLoader::getActions() {
  if (!m_action) {
    m_action = new QAction(getName(), this);
    m_action->setToolTip(getDescription());
    m_action->setIcon(getIcon());
    connect(m_action, &QAction::triggered, this, &SlamLoader::doAction);
  }

  load_slam_action = new QAction("Import SLAM results", this);
  load_slam_action->setToolTip(getDescription());
  load_slam_action->setIcon(getIcon());
  connect(load_slam_action, &QAction::triggered, this,
          &SlamLoader::DoActionLoadSlam);

  return QList<QAction*>{m_action, load_slam_action};
}

void SlamLoader::DoActionLoadSlam() {
  m_app->dispToConsole("[load_slam_action] DoActionLoadSlam!",
                       ccMainAppInterface::STD_CONSOLE_MESSAGE);

  if (m_app == nullptr) {
    Q_ASSERT(false);
    return;
  }

  QString datasetName = QFileDialog::getOpenFileName(0, tr("Open Metamodel"),
                                                     "~", tr("XML (*.xml)"));

  m_app->dispToConsole("[SlamLoader] will load XML MetaModel " + datasetName,
                       ccMainAppInterface::STD_CONSOLE_MESSAGE);

  // Load dataset
  workingModel.loadFile(datasetName.toStdString());

  // JSON config
  boost::filesystem::path json_path =
      boost::filesystem::complete(datasetName.toStdString()).parent_path();
  json_path /= boost::filesystem::path("/CloudCompare.json");
  std::cout << "test = " << json_path << std::endl;
  json j;
  std::ifstream i(json_path.string());
  i >> j;

  // Loop through each group in JSON
  for (const auto& group : j["groups"]) {
    std::string group_name = group["name"];

    std::cout << "Creating folder group: " << group["name"] << std::endl;
    std::cout << "  Loading scans into folder: " << group["name"] << "/"
              << group["path"] << std::endl;
    workingModel.setDataSetPath(group["path"]);
    // Create parent folder group
    ccHObject* folder_group = new (ccHObject);
    folder_group->setName(group_name.c_str());

    for (const auto& sub_group : group["subgroups"]) {
      ccHObject* sub_folder_group = new (ccHObject);
      std::string sub_group_name = sub_group["name"];
      std::cout << "Creating sub-folder group: " << sub_group_name << std::endl;

      sub_folder_group->setName(sub_group_name.c_str());
      folder_group->addChild(sub_folder_group);
      json j_test = sub_group;
      AddScans(sub_folder_group, j_test);
    }

    m_app->addToDB(folder_group);
  }
}

void SlamLoader::AddScans(ccHObject* sub_folder_group, json& j) {
  std::vector<std::string> scan_ids;
  workingModel.getAllScansId(scan_ids);
  std::vector<ccPointCloud*> scans;

  // Loop through each scan
  for (auto& scan_id : scan_ids) {
    // Get path to point cloud
    std::string pc_file_path;
    pc_file_path = workingModel.getFullPathOfPointcloud(scan_id);

    // Load in transformation matrix
    Eigen::Affine3f tf_matrix;
    workingModel.getAffine(scan_id, tf_matrix.matrix());
    ccGLMatrix transform(tf_matrix.data());

    PCLCloud::Ptr pcl_pcd_cloud(new PCLCloud);

    // Load in point cloud from file
    if (pcl::io::loadPCDFile(pc_file_path, *pcl_pcd_cloud) < 0) {
      ccLog::Warning("[PCL] Failed to load point cloud from file");
    }
    // Populate the CC pc container
    ccPointCloud* cc_pc = sm2ccConverter(pcl_pcd_cloud).getCloud();
    cc_pc->setName(scan_id.c_str());
    cc_pc->applyRigidTransformation(transform);
    scans.push_back(cc_pc);
  }

  if (j["filters"].is_null()) {
    std::cout << "Inside !j[filters]" << std::endl;
    for (auto scan : scans) { sub_folder_group->addChild(scan); }
  }

  for (const auto& filter : j["filters"].items()) {
    if (filter.key() == "downsample") {
      std::cout << "  Applying downsampling filter with resolution: "
                << filter.value() << std::endl;

      CCLib::CloudSamplingTools::SFModulationParams modParams(false);

      for (auto scan : scans) {
        ccPointCloud* cloud = scan;
        CCLib::ReferenceCloud* sampledCloud =
            CCLib::CloudSamplingTools::resampleCloudSpatially(
                scan, filter.value(), modParams);

        int warnings = 0;
        ccPointCloud* newPointCloud =
            cloud->partialClone(sampledCloud, &warnings);
        delete sampledCloud;

        if (newPointCloud) {
          newPointCloud->setName(cloud->getName());
          newPointCloud->setGlobalShift(cloud->getGlobalShift());
          newPointCloud->setGlobalScale(cloud->getGlobalScale());
          newPointCloud->setDisplay(cloud->getDisplay());
          newPointCloud->prepareDisplayForRefresh();
          cloud->setEnabled(false);
          sub_folder_group->addChild(newPointCloud);
          newPointCloud->prepareDisplayForRefresh();
        }
      }
    } else if (filter.key() == "sor")
      std::cout << "  Applying statistical outlier filter with resolution"
                << filter.value() << std::endl;
    else
      (std::cout << "  Unknown filter: " << filter.key());
  }
}

void SlamLoader::doAction() {
  m_app->dispToConsole("[ExamplePlugin] doAction!",
                       ccMainAppInterface::STD_CONSOLE_MESSAGE);
  if (m_app == nullptr) {
    Q_ASSERT(false);
    return;
  }
  m_app->dispToConsole("[ExamplePlugin] Hello world!",
                       ccMainAppInterface::STD_CONSOLE_MESSAGE);
  m_app->dispToConsole(
      "[ExamplePlugin] Warning: example plugin shouldn't be used as is",
      ccMainAppInterface::WRN_CONSOLE_MESSAGE);
  m_app->dispToConsole(
      "Example plugin shouldn't be used - it doesn't do anything!",
      ccMainAppInterface::ERR_CONSOLE_MESSAGE);
}
