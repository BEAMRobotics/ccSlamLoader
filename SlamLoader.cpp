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
    std::cout << "Adding scan: " << scan_id.c_str() << std::endl;
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
    cc_pc->showNormals(false);
    cc_pc->showColors(true);
    cc_pc->showSF(false);
    cc_pc->setSelected(false);
    scans.push_back(cc_pc);
  }

  if (j["filters"].is_null()) {
    std::cout << "Inside !j[filters]" << std::endl;
    for (auto scan : scans) { sub_folder_group->addChild(scan); }
    return;
  }

  for (auto scan : scans) {
    for (auto& filter : j["filters"]) {
      if (filter[0] == "downsample") {
        std::cout << scan->getName().toStdString() << " - " << filter
                  << " - Downsampling point cloud" << std::endl;
        DownsampleCloud(scan, filter[1]);
      } else if (filter[0] == "noise filter") {
        std::cout << scan->getName().toStdString() << " - " << filter
                  << " - Noise filtering" << std::endl;
        NoiseRemovalFilter(scan, filter[1]);
      } else if (filter[0] == "rgb") {
        std::cout << scan->getName().toStdString() << " - " << filter
                  << " - RGB" << std::endl;
        RGBFilter(scan, filter[1], filter[2]);
      } else if (filter[0] == "mls") {
        std::cout << scan->getName().toStdString() << " - " << filter
                  << " - MLS" << std::endl;
        MLSFilter(scan, filter[1]);
      } else {
        std::cout << "Unknown filter" << std::endl;
      };
    }
    sub_folder_group->addChild(scan);
  }
}

void SlamLoader::DownsampleCloud(ccPointCloud* scan, float resolution) {
  CCLib::CloudSamplingTools::SFModulationParams modParams(false);
  // ccPointCloud *cloud = scan;
  CCLib::ReferenceCloud* sampled_cloud =
      CCLib::CloudSamplingTools::resampleCloudSpatially(scan, resolution,
                                                        modParams);

  int warnings = 0;
  ccPointCloud* new_pointcloud = scan->partialClone(sampled_cloud, &warnings);
  delete sampled_cloud;

  *scan = *new_pointcloud;
}

void SlamLoader::NoiseRemovalFilter(ccPointCloud* scan, float resolution) {
  static bool s_noiseFilterUseKnn = false;
  static int s_noiseFilterKnn = 6;
  static bool s_noiseFilterUseAbsError = false;
  static double s_noiseFilterAbsError = 1.0;
  static double s_noiseFilterNSigma = 1.0;
  static bool s_noiseFilterRemoveIsolatedPoints = false;
  PointCoordinateType kernel_radius =
      ccLibAlgorithms::GetDefaultCloudKernelSize(scan);
  CCLib::ReferenceCloud* sampled_cloud = CCLib::CloudSamplingTools::noiseFilter(
      scan, kernel_radius, s_noiseFilterNSigma,
      s_noiseFilterRemoveIsolatedPoints, s_noiseFilterUseKnn, s_noiseFilterKnn,
      s_noiseFilterUseAbsError, s_noiseFilterAbsError);
  int warnings = 0;
  ccPointCloud* new_pointcloud = scan->partialClone(sampled_cloud, &warnings);
  delete sampled_cloud;

  *scan = *new_pointcloud;
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

void SlamLoader::MLSFilter(ccPointCloud* scan, float search_radius) {
  // pointer to selected cloud
  ccPointCloud* cloud = scan;

  // get xyz in PCL format
  std::list<std::string> req_fields;
  try {
    req_fields.emplace_back("xyz");
    if (scan->getCurrentDisplayedScalarField())
      req_fields.emplace_back(
          scan->getCurrentDisplayedScalarField()->getName());
  } catch (const std::bad_alloc&) {
    return; // not enough memory
  }

  // take out the xyz info
  auto sm_cloud = cc2smReader(scan).getAsSM(req_fields);
  if (!sm_cloud) return;

  // get as pcl point cloud
  auto pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  FROM_PCL_CLOUD(*sm_cloud, *pcl_cloud);

  // create storage for outcloud
  auto normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> smoother;
  pcl::PointIndicesPtr mapping_indices;
  auto tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

  smoother.setInputCloud(pcl_cloud);
  smoother.setSearchMethod(tree);
  smoother.setSearchRadius(search_radius);
  smoother.setComputeNormals(false);
  smoother.setPolynomialFit(false);
  smoother.setUpsamplingMethod(
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::NONE);
  smoother.process(*normals);
  mapping_indices = smoother.getCorrespondingIndices();

  auto sm_normals = boost::make_shared<PCLCloud>();
  TO_PCL_CLOUD(*normals, *sm_normals);

  ccPointCloud* new_cloud = sm2ccConverter(sm_normals).getCloud();
  copyScalarFields(scan, new_cloud, mapping_indices, true);
  copyRGBColors(scan, new_cloud, mapping_indices, true);
  *scan = *new_cloud;
}

void SlamLoader::RGBFilter(ccPointCloud*& scan, int lower_bound,
                           int upper_bound) {
  RGBToSF(scan);
  auto minVal = static_cast<ScalarType>(lower_bound);
  auto maxVal = static_cast<ScalarType>(upper_bound);
  scan = scan->filterPointsByScalarValue(minVal, maxVal, false);
}

void SlamLoader::RGBToSF(ccPointCloud* cloud) {
  ccScalarField* field = new ccScalarField(
      qPrintable(GetFirstAvailableSFName(cloud, "Composite")));

  // try to instantiate memory for each field
  unsigned count = cloud->size();
  if (field && !field->reserve(count)) {
    ccLog::Warning(QString("[sfFromColor] Not enough memory to instantiate SF "
                           "'%1' on cloud '%2'")
                       .arg(field->getName(), cloud->getName()));
    field->release();
    field = nullptr;
  }

  // export points
  for (unsigned j = 0; j < cloud->size(); ++j) {
    const ColorCompType* rgb = cloud->getPointColor(j);

    if (field)
      field->addElement(static_cast<ScalarType>(rgb[0] + rgb[1] + rgb[2]) / 3);
  }

  QString field_str;
  field->computeMinAndMax();
  int sfIdx = cloud->getScalarFieldIndexByName(field->getName());
  if (sfIdx >= 0) cloud->deleteScalarField(sfIdx);
  sfIdx = cloud->addScalarField(field);
  if (sfIdx >= 0) {
    cloud->setCurrentDisplayedScalarField(sfIdx);
    cloud->showSF(true);
    cloud->prepareDisplayForRefresh();

    if (!field_str.isEmpty()) field_str.append(", ");
    field_str.append(field->getName());
  } else {
    ccLog::Warning(
        QString("[sfFromColor] Failed to add scalar field '%1' to cloud '%2'?!")
            .arg(field->getName(), cloud->getName()));
    field->release();
  }
}

QString SlamLoader::GetFirstAvailableSFName(const ccPointCloud* cloud,
                                            const QString& baseName) {
  if (cloud == nullptr) {
    Q_ASSERT(false);
    return QString();
  }

  QString name = baseName;
  int tries = 0;

  while (cloud->getScalarFieldIndexByName(qPrintable(name)) >= 0 || tries > 99)
    name = QString("%1 #%2").arg(baseName).arg(++tries);

  if (tries > 99) return QString();

  return name;
}
