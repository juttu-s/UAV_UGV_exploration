/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/occupancyMap.h>

namespace mapManager{
	occMap::occMap(){
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
	}

	occMap::occMap(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
	}

	void occMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
	}

	void occMap::initParam(){
		// sensor input mode
		if (not this->nh_.getParam(this->ns_ + "/sensor_input_mode", this->sensorInputMode_)){
			this->sensorInputMode_ = 0;
			cout << this->hint_ << ": No sensor input mode option. Use default: depth image" << endl;
		}
		else{
			cout << this->hint_ << ": Sensor input mode: depth image (0)/pointcloud (1). Your option: " << this->sensorInputMode_ << endl;
		}		

		// localization mode
		if (not this->nh_.getParam(this->ns_ + "/localization_mode", this->localizationMode_)){
			this->localizationMode_ = 0;
			cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
		}
		else{
			cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
		}

		// depth topic name
		if (not this->nh_.getParam(this->ns_ + "/depth_image_topic", this->depthTopicName_)){
			this->depthTopicName_ = "/camera/depth/image_raw";
			cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
		}
		else{
			cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
		}

		// pointcloud topic name
		if (not this->nh_.getParam(this->ns_ + "/point_cloud_topic", this->pointcloudTopicName_)){
			this->pointcloudTopicName_ = "/camera/depth/points";
			cout << this->hint_ << ": No poincloud topic name. Use default: /camera/depth/points" << endl;
		}
		else{
			cout << this->hint_ << ": Pointcloud topic: " << this->pointcloudTopicName_ << endl;
		}

		if (this->localizationMode_ == 0){
			// odom topic name
			if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
				this->poseTopicName_ = "/CERLAB/quadcopter/pose";
				cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
			}
			else{
				cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
			}			
		}

		if (this->localizationMode_ == 1){
			// pose topic name
			if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopicName_)){
				this->odomTopicName_ = "/CERLAB/quadcopter/odom";
				cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
			}
			else{
				cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
			}
		}

		std::vector<double> robotSizeVec (3);
		if (not this->nh_.getParam(this->ns_ + "/robot_size", robotSizeVec)){
			robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
		}
		else{
			cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
		}
		this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];

		std::vector<double> depthIntrinsics (4);
		if (not this->nh_.getParam(this->ns_ + "/depth_intrinsics", depthIntrinsics)){
			cout << this->hint_ << ": Please check camera intrinsics!" << endl;
			exit(0);
		}
		else{
			this->fx_ = depthIntrinsics[0];
			this->fy_ = depthIntrinsics[1];
			this->cx_ = depthIntrinsics[2];
			this->cy_ = depthIntrinsics[3];
			cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
		}

		// depth scale factor
		if (not this->nh_.getParam(this->ns_ + "/depth_scale_factor", this->depthScale_)){
			this->depthScale_ = 1000.0;
			cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
		}
		else{
			cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
		}

		// depth min value
		if (not this->nh_.getParam(this->ns_ + "/depth_min_value", this->depthMinValue_)){
			this->depthMinValue_ = 0.2;
			cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << endl;
		}

		// depth max value
		if (not this->nh_.getParam(this->ns_ + "/depth_max_value", this->depthMaxValue_)){
			this->depthMaxValue_ = 5.0;
			cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth depth max value: " << this->depthMaxValue_ << endl;
		}

		// depth filter margin
		if (not this->nh_.getParam(this->ns_ + "/depth_filter_margin", this->depthFilterMargin_)){
			this->depthFilterMargin_ = 0;
			cout << this->hint_ << ": No depth filter margin. Use default: 0." << endl;
		}
		else{
			cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam(this->ns_ + "/depth_skip_pixel", this->skipPixel_)){
			this->skipPixel_ = 1;
			cout << this->hint_ << ": No depth skip pixel. Use default: 1." << endl;
		}
		else{
			cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << endl;
		}

		// ------------------------------------------------------------------------------------
		// depth image columns
		if (not this->nh_.getParam(this->ns_ + "/image_cols", this->imgCols_)){
			this->imgCols_ = 640;
			cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam(this->ns_ + "/image_rows", this->imgRows_)){
			this->imgRows_ = 480;
			cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
		}
		this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
		// ------------------------------------------------------------------------------------


		// transform matrix: body to camera
		std::vector<double> body2CamVec (16);
		if (not this->nh_.getParam(this->ns_ + "/body_to_camera", body2CamVec)){
			ROS_ERROR("[OccMap]: Please check body to camera matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
				}
			}
			// cout << this->hint_ << ": from body to camera: " << endl;
			// cout << this->body2Cam_ << endl;
		}

		// Raycast max length
		if (not this->nh_.getParam(this->ns_ + "/raycast_max_length", this->raycastMaxLength_)){
			this->raycastMaxLength_ = 5.0;
			cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
		}
		else{
			cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
		}

		// p hit
		double pHit;
		if (not this->nh_.getParam(this->ns_ + "/p_hit", pHit)){
			pHit = 0.70;
			cout << this->hint_ << ": No p hit. Use default: 0.70." << endl;
		}
		else{
			cout << this->hint_ << ": P hit: " << pHit << endl;
		}
		this->pHitLog_ = this->logit(pHit);

		// p miss
		double pMiss;
		if (not this->nh_.getParam(this->ns_ + "/p_miss", pMiss)){
			pMiss = 0.35;
			cout << this->hint_ << ": No p miss. Use default: 0.35." << endl;
		}
		else{
			cout << this->hint_ << ": P miss: " << pMiss << endl;
		}
		this->pMissLog_ = this->logit(pMiss);

		// p min
		double pMin;
		if (not this->nh_.getParam(this->ns_ + "/p_min", pMin)){
			pHit = 0.12;
			cout << this->hint_ << ": No p min. Use default: 0.12." << endl;
		}
		else{
			cout << this->hint_ << ": P min: " << pMin << endl;
		}
		this->pMinLog_ = this->logit(pMin);

		// p max
		double pMax;
		if (not this->nh_.getParam(this->ns_ + "/p_max", pMax)){
			pMax = 0.97;
			cout << this->hint_ << ": No p max. Use default: 0.97." << endl;
		}
		else{
			cout << this->hint_ << ": P max: " << pMax << endl;
		}
		this->pMaxLog_ = this->logit(pMax);

		// p occ
		double pOcc;
		if (not this->nh_.getParam(this->ns_ + "/p_occ", pOcc)){
			pOcc = 0.80;
			cout << this->hint_ << ": No p occ. Use default: 0.80." << endl;
		}
		else{
			cout << this->hint_ << ": P occ: " << pOcc << endl;
		}
		this->pOccLog_ = this->logit(pOcc);


		// map resolution
		if (not this->nh_.getParam(this->ns_ + "/map_resolution", this->mapRes_)){
			this->mapRes_ = 0.1;
			cout << this->hint_ << ": No map resolution. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Map resolution: " << this->mapRes_ << endl;
		}

		// ground height
		if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.0;
			cout << this->hint_ << ": No ground height. Use default: 0.0." << endl;
		}
		else{
			cout << this->hint_ << ": Ground height: " << this->groundHeight_ << endl;
		}


		// map size
		std::vector<double> mapSizeVec (3);
		if (not this->nh_.getParam(this->ns_ + "/map_size", mapSizeVec)){
			mapSizeVec[0] = 20; mapSizeVec[1] = 20; mapSizeVec[2] = 3;
			cout << this->hint_ << ": No map size. Use default: [20, 20, 3]." << endl;
		}
		else{
			this->mapSize_(0) = mapSizeVec[0];
			this->mapSize_(1) = mapSizeVec[1];
			this->mapSize_(2) = mapSizeVec[2];

			// init min max
			this->mapSizeMin_(0) = -mapSizeVec[0]/2; this->mapSizeMax_(0) = mapSizeVec[0]/2;
			this->mapSizeMin_(1) = -mapSizeVec[1]/2; this->mapSizeMax_(1) = mapSizeVec[1]/2;
			this->mapSizeMin_(2) = this->groundHeight_; this->mapSizeMax_(2) = this->groundHeight_ + mapSizeVec[2];
			
			// min max for voxel
			this->mapVoxelMin_(0) = 0; this->mapVoxelMax_(0) = ceil(mapSizeVec[0]/this->mapRes_);
			this->mapVoxelMin_(1) = 0; this->mapVoxelMax_(1) = ceil(mapSizeVec[1]/this->mapRes_);
			this->mapVoxelMin_(2) = 0; this->mapVoxelMax_(2) = ceil(mapSizeVec[2]/this->mapRes_);

			// reserve vector for variables
			int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
			this->countHitMiss_.resize(reservedSize, 0);
			this->countHit_.resize(reservedSize, 0);
			this->occupancy_.resize(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
			this->occupancyInflated_.resize(reservedSize, false);
			this->flagTraverse_.resize(reservedSize, -1);
			this->flagRayend_.resize(reservedSize, -1);

			cout << this->hint_ << ": Map size: " << "[" << mapSizeVec[0] << ", " << mapSizeVec[1] << ", " << mapSizeVec[2] << "]" << endl;
		}

		// local update range
		std::vector<double> localUpdateRangeVec;
		if (not this->nh_.getParam(this->ns_ + "/local_update_range", localUpdateRangeVec)){
			localUpdateRangeVec = std::vector<double>{5.0, 5.0, 3.0};
			cout << this->hint_ << ": No local update range. Use default: [5.0, 5.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local update range: " << "[" << localUpdateRangeVec[0] << ", " << localUpdateRangeVec[1] << ", " << localUpdateRangeVec[2] << "]" << endl;
		}
		this->localUpdateRange_(0) = localUpdateRangeVec[0]; this->localUpdateRange_(1) = localUpdateRangeVec[1]; this->localUpdateRange_(2) = localUpdateRangeVec[2];


		// local bound inflate factor
		if (not this->nh_.getParam(this->ns_ + "/local_bound_inflation", this->localBoundInflate_)){
			this->localBoundInflate_ = 0.0;
			cout << this->hint_ << ": No local bound inflate. Use default: 0.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Local bound inflate: " << this->localBoundInflate_ << endl;
		}

		// whether to clean local map
		if (not this->nh_.getParam(this->ns_ + "/clean_local_map", this->cleanLocalMap_)){
			this->cleanLocalMap_ = true;
			cout << this->hint_ << ": No clean local map option. Use default: true." << endl;
		}
		else{
			cout << this->hint_ << ": Clean local map option is set to: " << this->cleanLocalMap_ << endl; 
		}

		// absolute dir of prebuilt map file (.pcd)
		if (not this->nh_.getParam(this->ns_ + "/prebuilt_map_directory", this->prebuiltMapDir_)){
			this->prebuiltMapDir_ = "";
			cout << this->hint_ << ": Not using prebuilt map." << endl;
		}
		else{
			cout << this->hint_ << ": the prebuilt map absolute dir is found: " << this->prebuiltMapDir_ << endl;
		}

		// local map size (visualization)
		std::vector<double> localMapSizeVec;
		if (not this->nh_.getParam(this->ns_ + "/local_map_size", localMapSizeVec)){
			localMapSizeVec = std::vector<double>{10.0, 10.0, 2.0};
			cout << this->hint_ << ": No local map size. Use default: [10.0, 10.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local map size: " << "[" << localMapSizeVec[0] << ", " << localMapSizeVec[1] << ", " << localMapSizeVec[2] << "]" << endl;
		}
		this->localMapSize_(0) = localMapSizeVec[0]/2; this->localMapSize_(1) = localMapSizeVec[1]/2; this->localMapSize_(2) = localMapSizeVec[2]/2;
		this->localMapVoxel_(0) = int(ceil(localMapSizeVec[0]/(2*this->mapRes_))); this->localMapVoxel_(1) = int(ceil(localMapSizeVec[1]/(2*this->mapRes_))); this->localMapVoxel_(2) = int(ceil(localMapSizeVec[2]/(2*this->mapRes_)));

		// max vis height
		if (not this->nh_.getParam(this->ns_ + "/max_height_visualization", this->maxVisHeight_)){
			this->maxVisHeight_ = 3.0;
			cout << this->hint_ << ": No max visualization height. Use default: 3.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Max visualization height: " << this->maxVisHeight_ << endl;
		}

		// visualize global map
		if (not this->nh_.getParam(this->ns_ + "/visualize_global_map", this->visGlobalMap_)){
			this->visGlobalMap_ = false;
			cout << this->hint_ << ": No visualize map option. Use default: visualize local map." << endl;
		}
		else{
			cout << this->hint_ << ": Visualize map option. local (0)/global (1): " << this->visGlobalMap_ << endl;
		}

		// verbose
		if (not this->nh_.getParam(this->ns_ + "/verbose", this->verbose_)){
			this->verbose_ = true;
			cout << this->hint_ << ": No verbose option. Use default: check update info." << endl;
		}
		else{
			if (not this->verbose_){
				cout << this->hint_ << ": Not display messages" << endl;
			}
			else{
				cout << this->hint_ << ": Display messages" << endl;
			}
		}


	}

	void occMap::initPrebuiltMap(){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (this->prebuiltMapDir_, *cloud) == -1) //* load the file
		{
			cout << this->hint_ << ": No prebuilt map found/not using the prebuilt map." << endl;
		}
		else {
			cout << this->hint_ << ": Map loaded with " << cloud->width * cloud->height << " data points. " << endl;
			int address;
			Eigen::Vector3i pointIndex;
			Eigen::Vector3d pointPos;
			Eigen::Vector3i inflateIndex;
			int inflateAddress;

			// update occupancy info
			int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
			int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
			int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

			Eigen::Vector3d currMapRangeMin (0.0, 0.0, 0.0);
			Eigen::Vector3d currMapRangeMax (0.0, 0.0, 0.0);

			const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
			for (const auto& point: *cloud)
			{
				address = this->posToAddress(point.x, point.y, point.z);
				pointPos(0) = point.x; pointPos(1) = point.y; pointPos(2) = point.z;
				this->posToIndex(pointPos, pointIndex);

				this->occupancy_[address] = this->pMaxLog_;
				// update map range
				if (pointPos(0) < currMapRangeMin(0)){
					currMapRangeMin(0) = pointPos(0);
				}

				if (pointPos(0) > currMapRangeMax(0)){
					currMapRangeMax(0) = pointPos(0);
				}

				if (pointPos(1) < currMapRangeMin(1)){
					currMapRangeMin(1) = pointPos(1);
				}

				if (pointPos(1) > currMapRangeMax(1)){
					currMapRangeMax(1) = pointPos(1);
				}

				if (pointPos(2) < currMapRangeMin(2)){
					currMapRangeMin(2) = pointPos(2);
				}

				if (pointPos(2) > currMapRangeMax(2)){
					currMapRangeMax(2) = pointPos(2);
				}

				for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
					for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
						for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
							inflateIndex(0) = pointIndex(0) + ix;
							inflateIndex(1) = pointIndex(1) + iy;
							inflateIndex(2) = pointIndex(2) + iz;
							inflateAddress = this->indexToAddress(inflateIndex);
							if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
								continue; // those points are not in the reserved map
							} 
							this->occupancyInflated_[inflateAddress] = true;
						}
					}
				}
			}
			this->currMapRangeMin_ = currMapRangeMin;
			this->currMapRangeMax_ = currMapRangeMax;
		}
	}

	void occMap::registerCallback(){
		if (this->sensorInputMode_ == 0){
			// depth pose callback
			this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
				this->depthPoseSync_->registerCallback(boost::bind(&occMap::depthPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
				this->depthOdomSync_->registerCallback(boost::bind(&occMap::depthOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else if (this->sensorInputMode_ == 1){
			// pointcloud callback
			this->pointcloudSub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->nh_, this->pointcloudTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->pointcloudPoseSync_.reset(new message_filters::Synchronizer<pointcloudPoseSync>(pointcloudPoseSync(100), *this->pointcloudSub_, *this->poseSub_));
				this->pointcloudPoseSync_->registerCallback(boost::bind(&occMap::pointcloudPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->pointcloudOdomSync_.reset(new message_filters::Synchronizer<pointcloudOdomSync>(pointcloudOdomSync(100), *this->pointcloudSub_, *this->odomSub_));
				this->pointcloudOdomSync_->registerCallback(boost::bind(&occMap::pointcloudOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else{
			ROS_ERROR("[OccMap]: Invalid sensor input mode!");
			exit(0);
		}

		// occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::inflateMapCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::visCB, this);
		this->visWorker_ = std::thread(&occMap::startVisualization, this);
		this->visWorker_.detach();
		// this->projPointsVisTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::projPointsVisCB, this);
		// this->mapVisTimer_ = this->nh_.createTimer(ros::Duration(0.15), &occMap::mapVisCB, this);
		// this->inflatedMapVisTimer_ = this->nh_.createTimer(ros::Duration(0.15), &occMap::inflatedMapVisCB, this);
		// this->map2DVisTimer_ = this->nh_.createTimer(ros::Duration(0.15), &occMap::map2DVisCB, this);
	}

	void occMap::registerPub(){
		this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
		this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/voxel_map", 10);
		this->inflatedMapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/inflated_voxel_map", 10);
		this->map2DPub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(this->ns_ + "/2D_occupancy_map", 10);
		this->mapExploredPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_+"/explored_voxel_map",10);
		// publish service
		this->collisionCheckServer_ = this->nh_.advertiseService(this->ns_ + "/check_pos_collision", &occMap::checkCollision, this);
		this->raycastServer_ = this->nh_.advertiseService(this->ns_ + "/raycast", &occMap::getRayCast, this);
	}

	bool occMap::checkCollision(map_manager::CheckPosCollision::Request& req, map_manager::CheckPosCollision::Response& res){
		if (req.inflated){
			res.occupied = this->isInflatedOccupied(Eigen::Vector3d (req.x, req.y, req.z));
		}
		else{
			res.occupied = this->isOccupied(Eigen::Vector3d (req.x, req.y, req.z));
		}

		return true;
	}

	bool occMap::getRayCast(map_manager::RayCast::Request& req, map_manager::RayCast::Response& res){
		double hres = req.hres * M_PI/180.0;
		int numHbeams = int(360/req.hres);
		double vres = double(((req.vfov_max - req.vfov_min)* M_PI/180.0)/(req.vbeams-1));
		double vStartAngle = req.vfov_min * M_PI/180.0;
		int numVbeams = req.vbeams;
		double range = req.range;
		Eigen::Vector3d start (req.position.x, req.position.y, req.position.z);


		double starthAngle = req.startAngle;
		for (int h=0; h<numHbeams; ++h){
			double hAngle = starthAngle + double(h) * hres;
			Eigen::Vector3d hdirection (cos(hAngle), sin(hAngle), 0.0); // horizontal direction 
			for (int v=0; v<numVbeams; ++v){
				// get hit points
				double vAngle = vStartAngle + double(v) * vres;
				double vup = tan(vAngle);
				Eigen::Vector3d direction = hdirection;
				direction(2) += vup;
				direction /= direction.norm();
				Eigen::Vector3d hitPoint;
				bool success = this->castRay(start, direction, hitPoint, range, true);
				if (not success){
					hitPoint = start + range * direction;
				}
				for (int i=0; i<3; ++i){
					res.points.push_back(hitPoint(i));
				}
			}
		}
		return true;
	}
	
	void occMap::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);


		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::updateOccupancyCB(const ros::TimerEvent& ){
		if (not this->occNeedUpdate_){
			return;
		}
		// cout << "update occupancy map" << endl;
		ros::Time startTime, endTime;
		
		startTime = ros::Time::now();
		if (this->sensorInputMode_ == 0){
			// project 3D points from depth map
			this->projectDepthImage();
		}
		else if (this->sensorInputMode_ == 1){
			// directly get pointcloud
			this->getPointcloud();
		}

		// raycasting and update occupancy
		this->raycastUpdate();


		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}
		
		// inflate map
		// this->inflateLocalMap();
		endTime = ros::Time::now();
		if (this->verbose_){
			cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}

	void occMap::inflateMapCB(const ros::TimerEvent& ){
		// inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap();
			this->mapNeedInflate_ = false;
			this->esdfNeedUpdate_ = true;
		}
	}


	void occMap::projectDepthImage(){
		this->projPointsNum_ = 0;

		int cols = this->depthImage_.cols;
		int rows = this->depthImage_.rows;
		uint16_t* rowPtr;

		Eigen::Vector3d currPointCam, currPointMap;
		double depth;
		const double inv_factor = 1.0 / this->depthScale_;
		const double inv_fx = 1.0 / this->fx_;
		const double inv_fy = 1.0 / this->fy_;


		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
			rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
				depth = (*rowPtr) * inv_factor;
				
				// if (*rowPtr == 0) {
				// 	depth = this->raycastMaxLength_ + 0.1;
				// 	rowPtr =  rowPtr + this->skipPixel_;
				// 	continue;
				// } else if (depth < this->depthMinValue_) {
				// 	continue;
				// } else if (depth > this->depthMaxValue_ and depth < 1.5 * this->depthMaxValue_) {
				// 	depth = this->raycastMaxLength_ + 0.1;
				// }
				// else if (depth >= 1.5 * this->depthMaxValue_){
				// 	rowPtr =  rowPtr + this->skipPixel_;
				// 	continue;
				// }

				if (*rowPtr == 0) {
					depth = this->raycastMaxLength_ + 0.1;
				} else if (depth < this->depthMinValue_) {
					continue;
				} else if (depth > this->depthMaxValue_ ) {
					depth = this->raycastMaxLength_ + 0.1;
				}

				rowPtr =  rowPtr + this->skipPixel_;

				// get 3D point in camera frame
				currPointCam(0) = (u - this->cx_) * depth * inv_fx;
				currPointCam(1) = (v - this->cy_) * depth * inv_fy;
				currPointCam(2) = depth;
				currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate

				if (this->useFreeRegions_){ // this region will not be updated and directly set to free
					if (this->isInHistFreeRegions(currPointMap)){
						continue;
					}
				}

				// store current point
				this->projPoints_[this->projPointsNum_] = currPointMap;
				this->projPointsNum_ = this->projPointsNum_ + 1;
			}
		} 
	}

	void occMap::getPointcloud(){
		this->projPointsNum_ = this->pointcloud_.size();
		this->projPoints_.resize(this->projPointsNum_);
		Eigen::Vector3d currPointCam, currPointMap;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPointCam(0) = this->pointcloud_.points[i].x;
			currPointCam(1) = this->pointcloud_.points[i].y;
			currPointCam(2) = this->pointcloud_.points[i].z;
			currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate
			if ((currPointMap-this->position_).norm()>=0.5){
				this->projPoints_[i] = currPointMap;
			}
		}
	}

	void occMap::raycastUpdate(){
		if (this->projPointsNum_ == 0){
			return;
		}
		this->raycastNum_ += 1;

		// record local bound of update
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = xmax = this->position_(0);
		ymin = ymax = this->position_(1);
		zmin = zmax = this->position_(2);

		// iterate through each projected points, perform raycasting and update occupancy
		Eigen::Vector3d currPoint;
		bool pointAdjusted;
		int rayendVoxelID, raycastVoxelID;
		const double MAX_MAPPING_HEIGHT = 3.0;  // Don't map above wall height
		double length;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPoint = this->projPoints_[i];
			if (std::isnan(currPoint(0)) or std::isnan(currPoint(1)) or std::isnan(currPoint(2))){
				continue; // nan points can happen when we are using pointcloud as input
			}

			// ADD: Skip points above maximum mapping height
			// ============================================================
			if (currPoint(2) > MAX_MAPPING_HEIGHT) {
				continue;  // Don't map ceiling/roof observations
			}

			pointAdjusted = false;
			// check whether the point is in reserved map range
			if (not this->isInMap(currPoint)){
				currPoint = this->adjustPointInMap(currPoint);
				pointAdjusted = true;
			}

			// check whether the point exceeds the maximum raycasting length
			length = (currPoint - this->position_).norm();
			if (length > this->raycastMaxLength_){
				currPoint = this->adjustPointRayLength(currPoint);
				pointAdjusted = true;
			}


			// update local bound
			if (currPoint(0) < xmin){xmin = currPoint(0);}
			if (currPoint(1) < ymin){ymin = currPoint(1);}
			if (currPoint(2) < zmin){zmin = currPoint(2);}
			if (currPoint(0) > xmax){xmax = currPoint(0);}
			if (currPoint(1) > ymax){ymax = currPoint(1);}
			if (currPoint(2) > zmax){zmax = currPoint(2);}

			// update occupancy itself update information
			rayendVoxelID = this->updateOccupancyInfo(currPoint, not pointAdjusted); // point adjusted is free, not is occupied

			// check whether the voxel has already been updated, so no raycasting needed
			// rayendVoxelID = this->posToAddress(currPoint);
			if (this->flagRayend_[rayendVoxelID] == this->raycastNum_){
				continue; // skip
			}
			else{
				this->flagRayend_[rayendVoxelID] = this->raycastNum_;
			}



			// raycasting for update occupancy
			this->raycaster_.setInput(currPoint/this->mapRes_, this->position_/this->mapRes_);
			Eigen::Vector3d rayPoint, actualPoint;
			while (this->raycaster_.step(rayPoint)){
				actualPoint = rayPoint;
				actualPoint(0) += 0.5;
				actualPoint(1) += 0.5;
				actualPoint(2) += 0.5;
				actualPoint *= this->mapRes_;
				raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				// raycastVoxelID = this->posToAddress(actualPoint);
				if (this->flagTraverse_[raycastVoxelID] == this->raycastNum_){
					break;
				}
				else{
					this->flagTraverse_[raycastVoxelID] = this->raycastNum_;
				}

			}
		}

		// store local bound and inflate local bound (inflate is for ESDF update)
		this->posToIndex(Eigen::Vector3d (xmin, ymin, zmin), this->localBoundMin_);
		this->posToIndex(Eigen::Vector3d (xmax, ymax, zmax), this->localBoundMax_);
		this->localBoundMin_ -= int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); // inflate in x y direction
		this->localBoundMax_ += int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); 
		this->boundIndex(this->localBoundMin_); // since inflated, need to bound if not in reserved range
		this->boundIndex(this->localBoundMax_);


		// update occupancy in the cache
		double logUpdateValue;
		int cacheAddress, hit, miss;
		while (not this->updateVoxelCache_.empty()){
			Eigen::Vector3i cacheIdx = this->updateVoxelCache_.front();
			this->updateVoxelCache_.pop();
			cacheAddress = this->indexToAddress(cacheIdx);

			hit = this->countHit_[cacheAddress];
			miss = this->countHitMiss_[cacheAddress] - hit;

			if (hit >= miss and hit != 0){
				logUpdateValue = this->pHitLog_;
			}
			else{
				logUpdateValue = this->pMissLog_;
			}
			this->countHit_[cacheAddress] = 0; // clear hit
			this->countHitMiss_[cacheAddress] = 0; // clear hit and miss

			// check whether point is in the local update range
			if (not this->isInLocalUpdateRange(cacheIdx)){
				continue; // do not update if not in the range
			}

			if (this->useFreeRegions_){ // current used in simulation, this region will not be updated and directly set to free
				Eigen::Vector3d pos;
				this->indexToPos(cacheIdx, pos);
				if (this->isInHistFreeRegions(pos)){
					this->occupancy_[cacheAddress] = this->pMinLog_;
					continue;
				}
			}

			// update occupancy info
			if ((logUpdateValue >= 0) and (this->occupancy_[cacheAddress] >= this->pMaxLog_)){
				continue; // not increase p if max clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] == this->pMinLog_)){
				continue; // not decrease p if min clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] < this->pMinLog_)){
				this->occupancy_[cacheAddress] = this->pMinLog_; // if unknown set it free (prior), 
				continue;
			}

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);

			// update the entire map range (if it is not unknown)
			if (not this->isUnknown(cacheIdx)){
				Eigen::Vector3d cachePos;
				this->indexToPos(cacheIdx, cachePos);
				if (cachePos(0) > this->currMapRangeMax_(0)){
					this->currMapRangeMax_(0) = cachePos(0);
				}
				else if (cachePos(0) < this->currMapRangeMin_(0)){
					this->currMapRangeMin_(0) = cachePos(0);
				}

				if (cachePos(1) > this->currMapRangeMax_(1)){
					this->currMapRangeMax_(1) = cachePos(1);
				}
				else if (cachePos(1) < this->currMapRangeMin_(1)){
					this->currMapRangeMin_(1) = cachePos(1);
				}

				if (cachePos(2) > this->currMapRangeMax_(2)){
					this->currMapRangeMax_(2) = cachePos(2);
				}
				else if (cachePos(2) < this->currMapRangeMin_(2)){
					this->currMapRangeMin_(2) = cachePos(2);
				}
			}
		}

	}

	void occMap::cleanLocalMap(){
		Eigen::Vector3i posIndex;
		this->posToIndex(this->position_, posIndex);
		Eigen::Vector3i innerMinBBX = posIndex - this->localMapVoxel_;
		Eigen::Vector3i innerMaxBBX = posIndex + this->localMapVoxel_;
		Eigen::Vector3i outerMinBBX = innerMinBBX - Eigen::Vector3i(5, 5, 5);
		Eigen::Vector3i outerMaxBBX = innerMaxBBX + Eigen::Vector3i(5, 5, 5);
		this->boundIndex(innerMinBBX);
		this->boundIndex(innerMaxBBX);
		this->boundIndex(outerMinBBX);
		this->boundIndex(outerMaxBBX);

		// clear x axis
		for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int x=outerMinBBX(0); x<=innerMinBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int x=innerMaxBBX(0); x<=outerMaxBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;					
				}
			}
		}

		// clear y axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int y=outerMinBBX(1); y<=innerMinBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int y=innerMaxBBX(1); y<=outerMaxBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}

		// clear z axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
				for (int z=outerMinBBX(2); z<=innerMinBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int z=innerMaxBBX(2); z<=outerMaxBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}
	}

	void occMap::getExplorationStatistics( 
										int& explored_voxels,
										int& free_voxels, 
										int& occupied_voxels) {
		
		explored_voxels = 0;
		free_voxels = 0;
		occupied_voxels = 0;
	
		
		Eigen::Vector3i minIdx, maxIdx;
		this->posToIndex(this->currMapRangeMin_, minIdx);
		this->posToIndex(this->currMapRangeMax_, maxIdx);
		this->boundIndex(minIdx);
		this->boundIndex(maxIdx);
		
		// ============================================================
		// FIX: Only count up to wall height (3.0m in your world)
		// ============================================================
		const double WALL_HEIGHT = 3.0;  // Actual wall height in Gazebo
		Eigen::Vector3d maxHeightPos = this->currMapRangeMax_;
		maxHeightPos(2) = WALL_HEIGHT;
		
		Eigen::Vector3i maxHeightIdx;
		this->posToIndex(maxHeightPos, maxHeightIdx);
		this->boundIndex(maxHeightIdx);
		
		// Override Z max to wall height
		maxIdx(2) = std::min(maxIdx(2), maxHeightIdx(2));
		// ============================================================
		
		for (int x = minIdx(0); x <= maxIdx(0); ++x) {
			for (int y = minIdx(1); y <= maxIdx(1); ++y) {
				for (int z = minIdx(2); z <= maxIdx(2); ++z) {
					Eigen::Vector3i idx(x, y, z);
					
					if (this->isUnknown(idx)) {
						// unknown_voxels++;
						continue; // skip unknown voxels
					} else {
						explored_voxels++;
						if (this->isOccupied(idx)) {
							occupied_voxels++;
						} else {
							free_voxels++;
						}
					}
				}
			}
		}
	}

	// void occMap::getExplorationStatistics(int& total_voxels, 
	// 									int& explored_voxels,
	// 									int& free_voxels, 
	// 									int& occupied_voxels,
	// 									int& unknown_voxels) {
	// 	total_voxels = 0;
	// 	explored_voxels = 0;
	// 	free_voxels = 0;
	// 	occupied_voxels = 0;
	// 	unknown_voxels = 0;
		
	// 	// Use current map range (only count voxels that are in the explored region)
	// 	Eigen::Vector3i minIdx, maxIdx;
	// 	this->posToIndex(this->currMapRangeMin_, minIdx);
	// 	this->posToIndex(this->currMapRangeMax_, maxIdx);
	// 	this->boundIndex(minIdx);
	// 	this->boundIndex(maxIdx);
	// 	std::cout << "Exploration statistics range: " << minIdx.transpose() << " to " << maxIdx.transpose() << std::endl;
		
	// 	for (int x = minIdx(0); x <= maxIdx(0); ++x) {
	// 		for (int y = minIdx(1); y <= maxIdx(1); ++y) {
	// 			for (int z = minIdx(2); z <= maxIdx(2); ++z) {
	// 				Eigen::Vector3i idx(x, y, z);
	// 				total_voxels++;
					
	// 				if (this->isUnknown(idx)) {
	// 					unknown_voxels++;
	// 				} else {
	// 					explored_voxels++;
						
	// 					if (this->isOccupied(idx)) {
	// 						occupied_voxels++;
	// 					} else {
	// 						free_voxels++;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	double occMap::getExploredVolume() {
		int explored, free, occupied;
		getExplorationStatistics(explored, free, occupied);
		return explored * pow(this->mapRes_, 3);
	}

	int occMap::getExploredVoxelCount() {
		int explored, free, occupied, unknown;
		getExplorationStatistics(explored, free, occupied);
		return explored;
	}

	void occMap::inflateLocalMap(){
		int xmin = this->localBoundMin_(0);
		int xmax = this->localBoundMax_(0);
		int ymin = this->localBoundMin_(1);
		int ymax = this->localBoundMax_(1);
		int zmin = this->localBoundMin_(2);
		int zmax = this->localBoundMax_(2);
		Eigen::Vector3i clearIndex;
		// clear previous data in current data range
		for (int x=xmin; x<=xmax; ++x){
			for (int y=ymin; y<=ymax; ++y){
				for (int z=zmin; z<=zmax; ++z){
					clearIndex(0) = x; clearIndex(1) = y; clearIndex(2) = z;
					this->occupancyInflated_[this->indexToAddress(clearIndex)] = false;
				}
			}
		}

		int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

		// inflate based on current occupancy
		Eigen::Vector3i pointIndex, inflateIndex;
		int inflateAddress;
		const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		for (int x=xmin; x<=xmax; ++x){
			for (int y=ymin; y<=ymax; ++y){
				for (int z=zmin; z<=zmax; ++z){
					pointIndex(0) = x; pointIndex(1) = y; pointIndex(2) = z;
					if (this->isOccupied(pointIndex)){
						for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
							for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
								for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
									inflateIndex(0) = pointIndex(0) + ix;
									inflateIndex(1) = pointIndex(1) + iy;
									inflateIndex(2) = pointIndex(2) + iz;
									inflateAddress = this->indexToAddress(inflateIndex);
									if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
										continue; // those points are not in the reserved map
									} 
									this->occupancyInflated_[inflateAddress] = true;
								}
							}
						}
					}
				}
			}
		}
	}



	void occMap::visCB(const ros::TimerEvent& ){
		// this->publishProjPoints();
		// this->publishMap();
		this->publishInflatedMap();
		// this->publish2DOccupancyGrid();
	}

	void occMap::projPointsVisCB(const ros::TimerEvent& ){
		this->publishProjPoints();
	}

	void occMap::mapVisCB(const ros::TimerEvent& ){
		this->publishMap();
	}
	void occMap::inflatedMapVisCB(const ros::TimerEvent& ){
		this->publishInflatedMap();
	}

	void occMap::map2DVisCB(const ros::TimerEvent& ){
		this->publish2DOccupancyGrid();
	}
	
	void occMap::startVisualization(){
		ros::Rate r (10);
		while (ros::ok()){
			// pcl::PointCloud<pcl::PointXYZ> mapCloud, inflatedMapCloud, exploredMapCloud, depthCloud;
			// this->getMapVisData(mapCloud, inflatedMapCloud, exploredMapCloud, depthCloud);
			// sensor_msgs::PointCloud2 mapCloudMsg,inflatedMapCloudMsg, exploredMapCloudMsg, depthCloudMsg;
			// pcl::toROSMsg(mapCloud, mapCloudMsg);
			// pcl::toROSMsg(inflatedMapCloud, inflatedMapCloudMsg);
			// pcl::toROSMsg(exploredMapCloud, exploredMapCloudMsg);
			// pcl::toROSMsg(depthCloud, depthCloudMsg);

			// this->inflatedMapVisPub_.publish(inflatedMapCloudMsg);
			// this->mapVisPub_.publish(mapCloudMsg);
			// this->mapExploredPub_.publish(exploredMapCloudMsg);
			// this->depthCloudPub_.publish(depthCloudMsg);
			this->publishProjPoints();
			this->publishMap();
			// this->publishInflatedMap();
			this->publish2DOccupancyGrid();
			r.sleep();	
		}
	}

	void occMap::getMapVisData(pcl::PointCloud<pcl::PointXYZ>& mapCloud, pcl::PointCloud<pcl::PointXYZ>& inflatedMapCloud, pcl::PointCloud<pcl::PointXYZ>& exploredMapCloud, pcl::PointCloud<pcl::PointXYZ>& depthCloud){
		pcl::PointXYZ pt;
		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			depthCloud.push_back(pt);
		}

		depthCloud.width = depthCloud.points.size();
		depthCloud.height = 1;
		depthCloud.is_dense = true;
		depthCloud.header.frame_id = "map";

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							mapCloud.push_back(pt);
						}
					}
					
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							inflatedMapCloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredMapCloud.push_back(pt);
					}
				}
			}
		}

		mapCloud.width = mapCloud.points.size();
		mapCloud.height = 1;
		mapCloud.is_dense = true;
		mapCloud.header.frame_id = "map";

		inflatedMapCloud.width = inflatedMapCloud.points.size();
		inflatedMapCloud.height = 1;
		inflatedMapCloud.is_dense = true;
		inflatedMapCloud.header.frame_id = "map";

		exploredMapCloud.width = exploredMapCloud.points.size();
		exploredMapCloud.height = 1;
		exploredMapCloud.is_dense = true;
		exploredMapCloud.header.frame_id = "map";
	}

	void occMap::publishProjPoints(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			cloud.push_back(pt);
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->depthCloudPub_.publish(cloudMsg);
	}

	void occMap::publishMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> exploredCloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredCloud.push_back(pt);
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		exploredCloud.width = exploredCloud.points.size();
		exploredCloud.height = 1;
		exploredCloud.is_dense = true;
		exploredCloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		sensor_msgs::PointCloud2 exploredCloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		pcl::toROSMsg(exploredCloud, exploredCloudMsg);
		this->mapVisPub_.publish(cloudMsg);
		this->mapExploredPub_.publish(exploredCloudMsg);
	}

	void occMap::publishInflatedMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);
					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->inflatedMapVisPub_.publish(cloudMsg);	
	}

	// void occMap::injectPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	// 	if (!cloud || cloud->data.empty()) {
	// 		ROS_WARN("[OccMap] injectPointCloud: Empty cloud received");
	// 		return;
	// 	}
		
	// 	// DEBUG: Print map bounds
	// 	ROS_INFO("[OccMap] DEBUG - Map bounds: min=[%.2f, %.2f, %.2f], max=[%.2f, %.2f, %.2f]",
	// 			this->mapSizeMin_(0), this->mapSizeMin_(1), this->mapSizeMin_(2),
	// 			this->mapSizeMax_(0), this->mapSizeMax_(1), this->mapSizeMax_(2));
		
	// 	// DEBUG: Print cloud frame
	// 	ROS_INFO("[OccMap] DEBUG - Cloud frame_id: %s", cloud->header.frame_id.c_str());
		
	// 	// Convert ROS PointCloud2 to PCL
	// 	pcl::PointCloud<pcl::PointXYZ> pclCloud;
	// 	pcl::fromROSMsg(*cloud, pclCloud);
		
	// 	ROS_INFO("[OccMap] DEBUG - Cloud has %zu points", pclCloud.points.size());
		
	// 	int xInflateSize = ceil(this->robotSize_(0) / (2 * this->mapRes_));
	// 	int yInflateSize = ceil(this->robotSize_(1) / (2 * this->mapRes_));
	// 	int zInflateSize = ceil(this->robotSize_(2) / (2 * this->mapRes_));
	// 	const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		
	// 	int injectedCount = 0;
	// 	int outOfBoundsCount = 0;
		
	// 	for (const auto& point : pclCloud.points) {
	// 		if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
	// 			continue;
	// 		}
			
	// 		Eigen::Vector3d pos(point.x, point.y, point.z);
			
	// 		// DEBUG: Print first 5 points
	// 		if (injectedCount + outOfBoundsCount < 5) {
	// 			Eigen::Vector3i testIdx;
	// 			this->posToIndex(pos, testIdx);
	// 			ROS_INFO("[OccMap] DEBUG - Point[%d]: pos=[%.2f, %.2f, %.2f] -> idx=[%d, %d, %d], inMap=%d",
	// 					injectedCount + outOfBoundsCount,
	// 					pos(0), pos(1), pos(2),
	// 					testIdx(0), testIdx(1), testIdx(2),
	// 					this->isInMap(pos) ? 1 : 0);
	// 		}
			
	// 		if (!this->isInMap(pos)) {
	// 			outOfBoundsCount++;
	// 			continue;
	// 		}
			
	// 		Eigen::Vector3i pointIndex;
	// 		this->posToIndex(pos, pointIndex);
	// 		int address = this->indexToAddress(pointIndex);
			
	// 		if (address < 0 || address >= maxIndex) {
	// 			outOfBoundsCount++;
	// 			continue;
	// 		}
			
	// 		this->occupancy_[address] = this->pMaxLog_;
			
	// 		for (int ix = -xInflateSize; ix <= xInflateSize; ++ix) {
	// 			for (int iy = -yInflateSize; iy <= yInflateSize; ++iy) {
	// 				for (int iz = -zInflateSize; iz <= zInflateSize; ++iz) {
	// 					Eigen::Vector3i inflateIndex;
	// 					inflateIndex(0) = pointIndex(0) + ix;
	// 					inflateIndex(1) = pointIndex(1) + iy;
	// 					inflateIndex(2) = pointIndex(2) + iz;
						
	// 					int inflateAddress = this->indexToAddress(inflateIndex);
	// 					if ((inflateAddress < 0) || (inflateAddress >= maxIndex)) {
	// 						continue;
	// 					}
	// 					this->occupancyInflated_[inflateAddress] = true;
	// 				}
	// 			}
	// 		}
			
	// 		if (pos(0) < this->currMapRangeMin_(0)) this->currMapRangeMin_(0) = pos(0);
	// 		if (pos(0) > this->currMapRangeMax_(0)) this->currMapRangeMax_(0) = pos(0);
	// 		if (pos(1) < this->currMapRangeMin_(1)) this->currMapRangeMin_(1) = pos(1);
	// 		if (pos(1) > this->currMapRangeMax_(1)) this->currMapRangeMax_(1) = pos(1);
	// 		if (pos(2) < this->currMapRangeMin_(2)) this->currMapRangeMin_(2) = pos(2);
	// 		if (pos(2) > this->currMapRangeMax_(2)) this->currMapRangeMax_(2) = pos(2);
			
	// 		injectedCount++;
	// 	}
		
	// 	ROS_INFO("[OccMap] Injected %d points, %d out of bounds", injectedCount, outOfBoundsCount);
	// }

	void occMap::injectPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                               const Eigen::Vector3d& excludeCenter,
                               double excludeRadius) {
		if (!cloud || cloud->data.empty()) {
			ROS_WARN("[OccMap] injectPointCloud: Empty cloud received");
			return;
		}
		
		// Convert ROS PointCloud2 to PCL
		pcl::PointCloud<pcl::PointXYZ> pclCloud;
		pcl::fromROSMsg(*cloud, pclCloud);
		
		int xInflateSize = ceil(this->robotSize_(0) / (2 * this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1) / (2 * this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2) / (2 * this->mapRes_));
		const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		
		int injectedCount = 0;
		int excludedCount = 0;
		int outOfBoundsCount = 0;
		
		// Check if exclusion is enabled (excludeRadius > 0)
		bool useExclusion = (excludeRadius > 0.0);
		
		for (const auto& point : pclCloud.points) {
			if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
				continue;
			}
			
			Eigen::Vector3d pos(point.x, point.y, point.z);
			
			// --- EXCLUSION ZONE: Skip points near the UAV ---
			if (useExclusion) {
				// Use XY distance (cylinder exclusion) - UAV can be at different heights
				double distXY = std::sqrt(std::pow(pos(0) - excludeCenter(0), 2) + 
										std::pow(pos(1) - excludeCenter(1), 2));
				
				// Also check Z within reasonable range (UAV height ± some margin)
				double distZ = std::abs(pos(2) - excludeCenter(2));
				
				if (distXY < excludeRadius && distZ < 2.0) {  // 2m vertical margin
					excludedCount++;
					continue;
				}
			}
			
			if (!this->isInMap(pos)) {
				outOfBoundsCount++;
				continue;
			}
			
			Eigen::Vector3i pointIndex;
			this->posToIndex(pos, pointIndex);
			int address = this->indexToAddress(pointIndex);
			
			if (address < 0 || address >= maxIndex) {
				outOfBoundsCount++;
				continue;
			}
			
			this->occupancy_[address] = this->pMaxLog_;
			
			for (int ix = -xInflateSize; ix <= xInflateSize; ++ix) {
				for (int iy = -yInflateSize; iy <= yInflateSize; ++iy) {
					for (int iz = -zInflateSize; iz <= zInflateSize; ++iz) {
						Eigen::Vector3i inflateIndex;
						inflateIndex(0) = pointIndex(0) + ix;
						inflateIndex(1) = pointIndex(1) + iy;
						inflateIndex(2) = pointIndex(2) + iz;
						
						int inflateAddress = this->indexToAddress(inflateIndex);
						if ((inflateAddress < 0) || (inflateAddress >= maxIndex)) {
							continue;
						}
						this->occupancyInflated_[inflateAddress] = true;
					}
				}
			}
			
			if (pos(0) < this->currMapRangeMin_(0)) this->currMapRangeMin_(0) = pos(0);
			if (pos(0) > this->currMapRangeMax_(0)) this->currMapRangeMax_(0) = pos(0);
			if (pos(1) < this->currMapRangeMin_(1)) this->currMapRangeMin_(1) = pos(1);
			if (pos(1) > this->currMapRangeMax_(1)) this->currMapRangeMax_(1) = pos(1);
			if (pos(2) < this->currMapRangeMin_(2)) this->currMapRangeMin_(2) = pos(2);
			if (pos(2) > this->currMapRangeMax_(2)) this->currMapRangeMax_(2) = pos(2);
			
			injectedCount++;
		}
		
		// ROS_INFO("[OccMap] Injected %d points, excluded %d (near UAV), %d out of bounds", 
		// 		injectedCount, excludedCount, outOfBoundsCount);
	}

	void occMap::injectPointCloudWithExclusions(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                                             const std::vector<std::pair<Eigen::Vector3d, double>>& exclusions) {
		if (!cloud || cloud->data.empty()) {
			ROS_WARN("[OccMap] injectPointCloudWithExclusions: Empty cloud received");
			return;
		}
		
		// Convert ROS PointCloud2 to PCL
		pcl::PointCloud<pcl::PointXYZ> pclCloud;
		pcl::fromROSMsg(*cloud, pclCloud);
		
		int xInflateSize = ceil(this->robotSize_(0) / (2 * this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1) / (2 * this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2) / (2 * this->mapRes_));
		const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		
		int injectedCount = 0;
		int excludedCount = 0;
		int outOfBoundsCount = 0;
		
		for (const auto& point : pclCloud.points) {
			if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
				continue;
			}
			
			Eigen::Vector3d pos(point.x, point.y, point.z);
			
			// --- Check all exclusion zones ---
			bool excluded = false;
			for (const auto& excl : exclusions) {
				const Eigen::Vector3d& center = excl.first;
				double radius = excl.second;
				
				// XY distance (cylindrical exclusion)
				double distXY = std::sqrt(std::pow(pos(0) - center(0), 2) + 
										std::pow(pos(1) - center(1), 2));
				
				// Check if within exclusion cylinder (XY radius, Z within 2m)
				if (distXY < radius && std::abs(pos(2) - center(2)) < 2.0) {
					excluded = true;
					break;
				}
			}
			
			if (excluded) {
				excludedCount++;
				continue;
			}
			
			if (!this->isInMap(pos)) {
				outOfBoundsCount++;
				continue;
			}
			
			Eigen::Vector3i pointIndex;
			this->posToIndex(pos, pointIndex);
			int address = this->indexToAddress(pointIndex);
			
			if (address < 0 || address >= maxIndex) {
				outOfBoundsCount++;
				continue;
			}
			
			// Mark as occupied
			this->occupancy_[address] = this->pMaxLog_;
			
			// Inflate
			for (int ix = -xInflateSize; ix <= xInflateSize; ++ix) {
				for (int iy = -yInflateSize; iy <= yInflateSize; ++iy) {
					for (int iz = -zInflateSize; iz <= zInflateSize; ++iz) {
						Eigen::Vector3i inflateIndex;
						inflateIndex(0) = pointIndex(0) + ix;
						inflateIndex(1) = pointIndex(1) + iy;
						inflateIndex(2) = pointIndex(2) + iz;
						
						int inflateAddress = this->indexToAddress(inflateIndex);
						if ((inflateAddress < 0) || (inflateAddress >= maxIndex)) {
							continue;
						}
						this->occupancyInflated_[inflateAddress] = true;
					}
				}
			}
			
			// Update map range
			if (pos(0) < this->currMapRangeMin_(0)) this->currMapRangeMin_(0) = pos(0);
			if (pos(0) > this->currMapRangeMax_(0)) this->currMapRangeMax_(0) = pos(0);
			if (pos(1) < this->currMapRangeMin_(1)) this->currMapRangeMin_(1) = pos(1);
			if (pos(1) > this->currMapRangeMax_(1)) this->currMapRangeMax_(1) = pos(1);
			if (pos(2) < this->currMapRangeMin_(2)) this->currMapRangeMin_(2) = pos(2);
			if (pos(2) > this->currMapRangeMax_(2)) this->currMapRangeMax_(2) = pos(2);
			
			injectedCount++;
		}
		
		// ROS_INFO("[OccMap] Injected %d points, excluded %d (near robots), %d out of bounds", 
		// 		injectedCount, excludedCount, outOfBoundsCount);
	}

	void occMap::clearFootprints(const std::vector<Eigen::Vector3d>& positions, double radius, double zMin, double zMax) {
		int totalCleared = 0;
		
		for (const auto& position : positions) {
			Eigen::Vector3d minPos(position(0) - radius, position(1) - radius, zMin);
			Eigen::Vector3d maxPos(position(0) + radius, position(1) + radius, zMax);
			
			Eigen::Vector3i minIdx, maxIdx;
			this->posToIndex(minPos, minIdx);
			this->posToIndex(maxPos, maxIdx);
			this->boundIndex(minIdx);
			this->boundIndex(maxIdx);
			
			for (int x = minIdx(0); x <= maxIdx(0); ++x) {
				for (int y = minIdx(1); y <= maxIdx(1); ++y) {
					for (int z = minIdx(2); z <= maxIdx(2); ++z) {
						Eigen::Vector3i idx(x, y, z);
						Eigen::Vector3d pos;
						this->indexToPos(idx, pos);
						
						double distXY = std::sqrt(std::pow(pos(0) - position(0), 2) + 
												std::pow(pos(1) - position(1), 2));
						
						if (distXY <= radius) {
							int address = this->indexToAddress(idx);
							
							if (this->occupancy_[address] >= this->pOccLog_) {
								this->occupancy_[address] = this->pMinLog_;
								this->occupancyInflated_[address] = false;
								totalCleared++;
							}
						}
					}
				}
			}
		}
		
		// ROS_INFO("[OccMap] Cleared %d voxels along UGV path (%zu positions)", totalCleared, positions.size());
	}



	// void occMap::inject2DFreeSpace(const nav_msgs::OccupancyGrid::ConstPtr& grid, double zMin, double zMax) {
	// 	if (!grid || grid->data.empty()) {
	// 		ROS_WARN("[OccMap] inject2DFreeSpace: Empty grid received");
	// 		return;
	// 	}
		
	// 	double resolution = grid->info.resolution;
	// 	double originX = grid->info.origin.position.x;
	// 	double originY = grid->info.origin.position.y;
	// 	int width = grid->info.width;
	// 	int height = grid->info.height;
		
	// 	ROS_INFO("[OccMap] 2D Map: %dx%d, res=%.3f, origin=(%.2f, %.2f), z=[%.2f, %.2f]",
	// 			width, height, resolution, originX, originY, zMin, zMax);
		
	// 	int freeCount = 0;
	// 	int skippedOutOfBounds = 0;
	// 	int skippedAlreadyKnown = 0;
	// 	int skippedNotFree = 0;
	// 	int totalFreeCells = 0;
		
	// 	double zStep = this->mapRes_;
	// 	const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		
	// 	for (int j = 0; j < height; ++j) {
	// 		for (int i = 0; i < width; ++i) {
	// 			int idx = j * width + i;
	// 			int8_t value = grid->data[idx];
				
	// 			// Count how many cells are free in 2D map
	// 			if (value == 0) {
	// 				totalFreeCells++;
	// 			} else {
	// 				skippedNotFree++;
	// 				continue;  // Skip non-free cells
	// 			}
				
	// 			double worldX = originX + (i + 0.5) * resolution;
	// 			double worldY = originY + (j + 0.5) * resolution;
				
	// 			for (double z = zMin; z <= zMax; z += zStep) {
	// 				Eigen::Vector3d pos(worldX, worldY, z);
					
	// 				if (!this->isInMap(pos)) {
	// 					skippedOutOfBounds++;
	// 					continue;
	// 				}
					
	// 				Eigen::Vector3i voxelIdx;
	// 				this->posToIndex(pos, voxelIdx);
	// 				int address = this->indexToAddress(voxelIdx);
					
	// 				if (address < 0 || address >= maxIndex) {
	// 					skippedOutOfBounds++;
	// 					continue;
	// 				}
					
	// 				// Check current state
	// 				bool isUnknown = (this->occupancy_[address] < this->pMinLog_);
					
	// 				if (isUnknown) {
	// 					// Mark unknown cells as free
	// 					this->occupancy_[address] = this->pMinLog_;
	// 					this->occupancyInflated_[address] = false;
	// 					freeCount++;
	// 				} else {
	// 					skippedAlreadyKnown++;
	// 				}
	// 			}
	// 		}
	// 	}
		
	// 	ROS_INFO("[OccMap] 2D map has %d free cells (of %d total)", totalFreeCells, width * height);
	// 	ROS_INFO("[OccMap] Injected %d, skipped: %d (already known), %d (out of bounds), %d (not free in 2D)", 
	// 			freeCount, skippedAlreadyKnown, skippedOutOfBounds, skippedNotFree);
	// }

	void occMap::inject2DFreeSpace(const nav_msgs::OccupancyGrid::ConstPtr& grid, double floorHeight) {
		if (!grid || grid->data.empty()) {
			ROS_WARN("[OccMap] inject2DFreeSpace: Empty grid received");
			return;
		}
		
		double resolution = grid->info.resolution;
		double originX = grid->info.origin.position.x;
		double originY = grid->info.origin.position.y;
		int width = grid->info.width;
		int height = grid->info.height;
		
		// ROS_INFO("[OccMap] 2D Map: %dx%d, res=%.3f, origin=(%.2f, %.2f), floor at z=%.2f",
		// 		width, height, resolution, originX, originY, floorHeight);
		
		int floorCount = 0;
		int skippedOutOfBounds = 0;
		int skippedAlreadyKnown = 0;
		int totalFreeCells = 0;
		
		const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		
		for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				int idx = j * width + i;
				int8_t value = grid->data[idx];
				
				// Only process FREE cells from 2D map (value == 0)
				// These are areas where UGV can drive = floor exists
				if (value != 0) {
					continue;
				}
				
				totalFreeCells++;
				
				double worldX = originX + (i + 0.5) * resolution;
				double worldY = originY + (j + 0.5) * resolution;
				
				// Mark the FLOOR at this XY location as OCCUPIED
				Eigen::Vector3d floorPos(worldX, worldY, floorHeight);
				
				if (!this->isInMap(floorPos)) {
					skippedOutOfBounds++;
					continue;
				}
				
				Eigen::Vector3i floorIdx;
				this->posToIndex(floorPos, floorIdx);
				int address = this->indexToAddress(floorIdx);
				
				if (address < 0 || address >= maxIndex) {
					skippedOutOfBounds++;
					continue;
				}
				
				// Only mark if currently UNKNOWN (don't overwrite UAV observations)
				if (this->occupancy_[address] < this->pMinLog_) {
					// Mark floor as OCCUPIED
					this->occupancy_[address] = this->pMaxLog_;
					floorCount++;
				} else {
					skippedAlreadyKnown++;
				}
			}
		}
		
		// Now inflate the newly added floor cells
		if (floorCount > 0) {
			this->mapNeedInflate_ = true;
		}
		
		// ROS_INFO("[OccMap] UGV explored %d floor cells, marked %d new floor voxels, skipped: %d (already known), %d (out of bounds)", 
		// 		totalFreeCells, floorCount, skippedAlreadyKnown, skippedOutOfBounds);
	}

	void occMap::publish2DOccupancyGrid(){
		Eigen::Vector3d minRange, maxRange;
		minRange = this->mapSizeMin_;
		maxRange = this->mapSizeMax_;
		minRange(2) = this->groundHeight_;
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		nav_msgs::OccupancyGrid mapMsg;
		for (int i=0; i<maxRangeIdx(0); ++i){
			for (int j=0; j<maxRangeIdx(1); ++j){
				mapMsg.data.push_back(0);
			}
		}

		double z = 0.5;
		int zIdx = int(z/this->mapRes_);
		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				Eigen::Vector3i pointIdx (x, y, zIdx);
				int map2DIdx = x  +  y * maxRangeIdx(0);
				if (this->isUnknown(pointIdx)){
					mapMsg.data[map2DIdx] = -1;
				}
				else if (this->isOccupied(pointIdx)){
					mapMsg.data[map2DIdx] = 100;
				}
				else{
					mapMsg.data[map2DIdx] = 0;
				}
			}
		}
		mapMsg.header.frame_id = "map";
		mapMsg.header.stamp = ros::Time::now();
		mapMsg.info.resolution = this->mapRes_;
		mapMsg.info.width = maxRangeIdx(0);
		mapMsg.info.height = maxRangeIdx(1);
		mapMsg.info.origin.position.x = minRange(0);
		mapMsg.info.origin.position.y = minRange(1);
		this->map2DPub_.publish(mapMsg);		
	}
}
