/*
*	File: dep.cpp
*	---------------
*   dynamic exploration planner implementation
*/

#include <global_planner/dep.h>
#include <random>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <limits>
#include <algorithm>
#include <std_msgs/Bool.h>


namespace globalPlanner{
	DEP::DEP(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "/DEP";
		this->hint_ = "[DEP]";

		// Initialize timing variables
		this->total_planning_time_ = 0.0;
		this->planning_iterations_ = 0;
		this->exploration_complete_ = false;
		// Initialize milestone tracking
		this->reached_80_percent_ = false;
		this->reached_95_percent_ = false;
    
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void DEP::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void DEP::loadVelocity(double vel, double angularVel){
		this->vel_ = vel;
		this->angularVel_ = angularVel;
	}

	void DEP::initParam(){
		// odom topic name
		if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopic_)){
			this->odomTopic_ = "/CERLAB/quadcopter/odom";
			cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
		}
		else{
			cout << this->hint_ << ": Odom topic: " << this->odomTopic_ << endl;
		}

		// local sample region min
		std::vector<double> localRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_min", localRegionMinTemp)){
			this->localRegionMin_(0) = -5.0;
			this->localRegionMin_(1) = -5.0;
			this->localRegionMin_(2) = -2.0;
			cout << this->hint_ << ": No local region min param. Use default: [-5 -5 -2]" <<endl;
		}
		else{
			this->localRegionMin_(0) = localRegionMinTemp[0];
			this->localRegionMin_(1) = localRegionMinTemp[1];
			this->localRegionMin_(2) = localRegionMinTemp[2];
			cout << this->hint_ << ": Local Region Min: " << this->localRegionMin_[0] <<" " <<this->localRegionMin_[1]<<" "<< this->localRegionMin_[2]<< endl;
		}

		// local sample region max
		std::vector<double> localRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_max", localRegionMaxTemp)){
			this->localRegionMax_(0) = 5.0;
			this->localRegionMax_(1) = 5.0;
			this->localRegionMax_(2) = 2.0;
			cout << this->hint_ << ": No local region max param. Use default: [5 5 2]" <<endl;
		}
		else{
			this->localRegionMax_(0) = localRegionMaxTemp[0];
			this->localRegionMax_(1) = localRegionMaxTemp[1];
			this->localRegionMax_(2) = localRegionMaxTemp[2];
			cout << this->hint_ << ": Local Region Max: " << this->localRegionMax_[0] <<" " <<this->localRegionMax_[1]<<" "<< this->localRegionMax_[2]<< endl;
		}

		// global sample region min
		std::vector<double> globalRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_min", globalRegionMinTemp)){
			this->globalRegionMin_(0) = -20.0;
			this->globalRegionMin_(1) = -20.0;
			this->globalRegionMin_(2) = 0.0;
			cout << this->hint_ << ": No global region min param. Use default: [-20 -20 0]" <<endl;
		}
		else{
			this->globalRegionMin_(0) = globalRegionMinTemp[0];
			this->globalRegionMin_(1) = globalRegionMinTemp[1];
			this->globalRegionMin_(2) = globalRegionMinTemp[2];
			cout << this->hint_ << ": Global Region Min: " << this->globalRegionMin_[0] <<" "<< this->globalRegionMin_[1]<<" "<< this->globalRegionMin_[2]<< endl;
		}

		// global sample region max
		std::vector<double> globalRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_max", globalRegionMaxTemp)){
			this->globalRegionMax_(0) = 20.0;
			this->globalRegionMax_(1) = 20.0;
			this->globalRegionMax_(2) = 3.0;
			cout << this->hint_ << ": No global region max param. Use default: [20 20 3]" <<endl;
		}
		else{
			this->globalRegionMax_(0) = globalRegionMaxTemp[0];
			this->globalRegionMax_(1) = globalRegionMaxTemp[1];
			this->globalRegionMax_(2) = globalRegionMaxTemp[2];
			cout << this->hint_ << ": Global Region Max: " << this->globalRegionMax_[0] <<" "<< this->globalRegionMax_[1]<<" "<< this->globalRegionMax_[2]<< endl;
		}

		// Local Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/local_sample_thresh", this->localSampleThresh_)){
			this->localSampleThresh_ = 50;
			cout << this->hint_ << ": No local sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Local sample Thresh: " << this->localSampleThresh_ << endl;
		}
		
		// Global Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/global_sample_thresh", this->globalSampleThresh_)){
			this->globalSampleThresh_ = 50;
			cout << this->hint_ << ": No global sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Global sample Thresh: " << this->globalSampleThresh_ << endl;
		}

		// Frontier Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/frontier_sample_thresh", this->frontierSampleThresh_)){
			this->frontierSampleThresh_ = 50;
			cout << this->hint_ << ": No frontier sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier sample Thresh: " << this->frontierSampleThresh_ << endl;
		}		

		// minimum distance for node sampling
		if (not this->nh_.getParam(this->ns_ + "/dist_thresh", this->distThresh_)){
			this->distThresh_ = 0.8;
			cout << this->hint_ << ": No distance thresh param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Distance Thresh: " << this->distThresh_ << endl;
		}

		// safety distance for random sampling in xy
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_xy", this->safeDistXY_)){
			this->safeDistXY_ = 1.0;
			cout << this->hint_ << ": No safe distance in XY param. Use default: 0.3" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in XY: " << this->safeDistXY_ << endl;
		}

		// safety distance for random sampling in z
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_z", this->safeDistZ_)){
			this->safeDistZ_ = 0.0;
			cout << this->hint_ << ": No safe distance in Z param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in Z: " << this->safeDistZ_ << endl;
		}

		// safety distance check unknown
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_check_unknown", this->safeDistCheckUnknown_)){
			this->safeDistCheckUnknown_ = true;
			cout << this->hint_ << ": No safe distance check unknown param. Use default: true" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance check unknown: " << this->safeDistCheckUnknown_ << endl;
		}

		//Camera Parameters	
		if (not this->nh_.getParam(this->ns_ + "/horizontal_FOV", this->horizontalFOV_)){
			this->horizontalFOV_ = 0.8;
			cout << this->hint_ << ": No Horizontal FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Horizontal FOV: " << this->horizontalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/vertical_FOV", this->verticalFOV_)){
			this->verticalFOV_ = 0.8;
			cout << this->hint_ << ": No Vertical FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Vertical FOV: " << this->verticalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmin", this->dmin_)){
			this->dmin_ = 0.0;
			cout << this->hint_ << ": No min depth param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Min Depth: " << this->dmin_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmax", this->dmax_)){
			this->dmax_ = 1.0;
			cout << this->hint_ << ": No max depth param. Use default: 1.0" << endl;
		}
		else{
			cout << this->hint_ << ": Max Depth: " << this->dmax_ << endl;
		}

		// nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/nearest_neighbor_number", this->nnNum_)){
			this->nnNum_ = 15;
			cout << this->hint_ << ": No nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Nearest neighbor number is set to: " << this->nnNum_ << endl;
		}

		// frontier nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/frontier_nearest_neighbor_number", this->nnNumFrontier_)){
			this->nnNumFrontier_ = 15;
			cout << this->hint_ << ": No frontier nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier nearest neighbor number is set to: " << this->nnNumFrontier_ << endl;
		}

		// node connection max distances
		if (not this->nh_.getParam(this->ns_ + "/max_connect_dist", this->maxConnectDist_)){
			this->maxConnectDist_ = 3.0;
			cout << this->hint_ << ": No max conect distance param. Use default: 1.5m." << endl;
		}
		else{
			cout << this->hint_ << ": Max connect distance is set to: " << this->maxConnectDist_ << endl;
		}

		// number of yaw angles
		int yawNum = 32;
		if (not this->nh_.getParam(this->ns_ + "/num_yaw_angles", yawNum)){
			for (int i=0; i<32; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}					
			cout << this->hint_ << ": No number of yaw angles param. Use default: 32." << endl;
		}
		else{
			for (int i=0; i<yawNum; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}	
			cout << this->hint_ << ": Number of yaw angles is set to: " << yawNum << endl;
		}

		// minimum threshold of voxels
		if (not this->nh_.getParam(this->ns_ + "/min_voxel_thresh", this->minVoxelThresh_)){
			this->minVoxelThresh_ = 0.1;
			cout << this->hint_ << ": No minimum threshold of voxels param. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum threshold of voxels is set to: " << this->minVoxelThresh_ << endl;
		}

		// minimum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/min_goal_candidates", this->minCandidateNum_)){
			this->minCandidateNum_ = 10;
			cout << this->hint_ << ": No minimum number of goal candidates param. Use default: 10." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum number of goal candidates is set to: " << this->minCandidateNum_ << endl;
		}

		// maximum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/max_goal_candidates", this->maxCandidateNum_)){
			this->maxCandidateNum_ = 30;
			cout << this->hint_ << ": No maximum number of goal candidates param. Use default: 30." << endl;
		}
		else{
			cout << this->hint_ << ": Maximum number of goal candidates is set to: " << this->maxCandidateNum_ << endl;
		}

		// Information gain update  distance
		if (not this->nh_.getParam(this->ns_ + "/information_gain_update_distance", this->updateDist_)){
			this->updateDist_ = 1.0;
			cout << this->hint_ << ": No information gain update distance param. Use default: 1.0m." << endl;
		}
		else{
			cout << this->hint_ << ": Information gain update distance is set to: " << this->updateDist_ << endl;
		}

		// yaw penalty weight
		if (not this->nh_.getParam(this->ns_ + "/yaw_penalty_weight", this->yawPenaltyWeight_)){
			this->yawPenaltyWeight_ = 1.0;
			cout << this->hint_ << ": No yaw penalty weight param. Use default: 1.0." << endl;
		}
		else{
			cout << this->hint_ << ": Yaw penalty weight is set to: " << this->yawPenaltyWeight_ << endl;
		}
	}

	void DEP::initModules(){
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void DEP::registerPub(){
		// roadmap visualization publisher
		this->roadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/roadmap", 10);
		this->roadmapUGVPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/roadmap_ugv", 10);

		// candidate paths publisher
		this->candidatePathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/candidate_paths", 10);

		// best path publisher
		this->bestPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/best_paths", 10);

		// fronteir vis publisher
		this->frontierVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/frontier_regions", 10);

		ugv_path_pub_ = nh_.advertise<nav_msgs::Path>("/ugv/op_path", 1, true);

		uav_path_pub_ = nh_.advertise<nav_msgs::Path>("/uav/op_path", 1, true);

		completion_pub_ = nh_.advertise<std_msgs::Bool>("/exploration/complete", 1, true);

	}

	void DEP::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &DEP::odomCB, this);
		ugv_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1, &DEP::ugvOdomCb, this);
	
		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &DEP::visCB, this);

	}

	void DEP::ugvOdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
		ugv_pose_ = msg->pose.pose;
		ugv_odom_received_ = true;
	}

	bool DEP::makePlan() {
		
		std::lock_guard<std::mutex> lock(this->data_mutex_);
		if (!this->odomReceived_) return false;
		// =================================================================================
		// CHECK EXPLORATION COMPLETION FIRST 
		// =================================================================================
		
		// Calculate total information gain across all existing nodes
		int total_info_gain = 0;
		int nodes_with_gain = 0;
		double exploration_percent = 97.0; 

		
		for (const auto& node : this->prmNodeVec_) {
			total_info_gain += node->numVoxels;
			if (node->numVoxels > 0) nodes_with_gain++;
		}
		
		// Get exploration volume statistics
		int explored_voxels, free_voxels, occupied_voxels;
		this->map_->getExplorationStatistics(explored_voxels, free_voxels, occupied_voxels);
		
		int total_voxels = 1240000; // For 20x20x3.1 area with 0.1m resolution
		// int total_voxels = 1440000; // For 20x25x3.1 area with 0.1m resolution (adjusted for larger global region)
		// int total_voxels = 15.5*15*3.1*1000; // For 15x15x3.1 area with 0.1m resolution (room)
		double coverage_percent = 100.0 * explored_voxels / (double)total_voxels;

		// ============================================================
		// CHECK AND RECORD MILESTONES
		// ============================================================
		if (!reached_80_percent_ && coverage_percent >= 80.0) {
			time_80_percent_ = ros::Time::now();
			double time_to_80 = (time_80_percent_ - exploration_start_time_).toSec();
			reached_80_percent_ = true;
			
			ROS_WARN("========================================");
			ROS_WARN("[DEP] MILESTONE: 80%% COVERAGE REACHED!");
			ROS_WARN("[DEP] Time to 80%%: %.2f s (%.2f min)", time_to_80, time_to_80 / 60.0);
			ROS_WARN("[DEP] Explored volume: %.2f m³", explored_voxels * pow(this->map_->getRes(), 3));
			ROS_WARN("========================================");
		}
		
		if (!reached_95_percent_ && coverage_percent >= 95.0) {
			time_95_percent_ = ros::Time::now();
			double time_to_95 = (time_95_percent_ - exploration_start_time_).toSec();
			reached_95_percent_ = true;
			
			ROS_WARN("========================================");
			ROS_WARN("[DEP] MILESTONE: 95%% COVERAGE REACHED!");
			ROS_WARN("[DEP] Time to 95%%: %.2f s (%.2f min)", time_to_95, time_to_95 / 60.0);
			ROS_WARN("[DEP] Explored volume: %.2f m³", explored_voxels * pow(this->map_->getRes(), 3));
			ROS_WARN("========================================");
		}
		// ============================================================
			
		// ROS_INFO("[DEP] Pre-check: Total IG=%d, Nodes with gain=%d/%zu, Coverage=%.1f%%", 
		// 		total_info_gain, nodes_with_gain, this->prmNodeVec_.size(), coverage_percent);
		
		// Stop if EITHER condition is met (after first iteration):
		// 1. Low information gain (< 100)
		// 2. High coverage (> 95%)
		
		bool low_info_gain = (total_info_gain < 150);
		bool high_coverage = (coverage_percent > exploration_percent);
		bool not_first_iteration = (planning_iterations_ > 0);
		
		if (not_first_iteration && (low_info_gain || high_coverage)) {
			ROS_WARN("========================================");
			ROS_WARN("[DEP] EXPLORATION COMPLETE!");
			if (low_info_gain) {
				ROS_WARN("[DEP] Reason: Total information gain (%d) < 100", total_info_gain);
			}
			if (high_coverage) {
				ROS_WARN("[DEP] Reason: Coverage (%.1f%%) > 97%%", coverage_percent);
			}
			ROS_WARN("========================================");
			
			double explored_volume = explored_voxels * pow(this->map_->getRes(), 3);
			double free_volume = free_voxels * pow(this->map_->getRes(), 3);
			double occupied_volume = occupied_voxels * pow(this->map_->getRes(), 3);
			
			ROS_WARN("========== FINAL EXPLORATION RESULTS ==========");
			ROS_WARN("[Final] Total planning time: %.2f s (%.2f min)", 
					total_planning_time_, total_planning_time_ / 60.0);
			ROS_WARN("[Final] Planning iterations: %d", planning_iterations_);
			ROS_WARN("[Final] Average planning time: %.2f s/iteration", 
					total_planning_time_ / planning_iterations_);
			ROS_WARN("-----------------------------------------------");
			ROS_WARN("[Final] Explored volume: %.2f m³ (%d voxels)", 
					explored_volume, explored_voxels);
			ROS_WARN("[Final] Free space: %.2f m³ (%d voxels)", 
					free_volume, free_voxels);
			ROS_WARN("[Final] Obstacles: %.2f m³ (%d voxels)", 
					occupied_volume, occupied_voxels);
			ROS_WARN("[Final] Coverage: %.1f%% of mapped region", coverage_percent);
			ROS_WARN("===============================================");
			
			std_msgs::Bool completion_msg;
			completion_msg.data = true;
			this->completion_pub_.publish(completion_msg);
			
			this->exploration_complete_ = true;
			return false; // Stop planning
		}
		// Start timing this planning iteration
    	ros::Time plan_start = ros::Time::now();

		ROS_INFO("========== PLANNING ITERATION %d START ==========", planning_iterations_ + 1);

		// =================================================================================
		// 1. BUILD UAV PRM & MAP
		// =================================================================================
		this->detectFrontierRegion(this->frontierPointPairs_);
		this->buildRoadMap();
		this->pruneNodes();

		double prm_time = (ros::Time::now() - plan_start).toSec();
    	ROS_INFO("[DEP Timing] UAV PRM build: %.3f s", prm_time);

		// 2. Project UGV nodes
		ros::Time ugv_proj_start = ros::Time::now();
		this->buildUGVFromUAVProjection();
		this->buildUGVRoadMap();

		double ugv_proj_time = (ros::Time::now() - ugv_proj_start).toSec();
    	ROS_INFO("[DEP Timing] UGV PRM build: %.3f s", ugv_proj_time);

		ros::Time info_gain_start = ros::Time::now();
		this->updateInformationGain();
		this->updateInformationGainUGV();
		double info_gain_time = (ros::Time::now() - info_gain_start).toSec();
    	ROS_INFO("[DEP Timing] Info gain calculation: %.3f s", info_gain_time);
		// this->printPRMNodesInformationGain();

		if (this->prmNodeVec_.empty()) {
			ROS_WARN("[DEP] PRM is empty; cannot plan.");
			return false;
		}

		// ROS_INFO("[DEP] PRM nodes: %zu, UGV nodes: %zu", 
        //  prmNodeVec_.size(), prmNodeVecUGV_.size());

		// --- Build UAV order + index map ---
		std::vector<std::shared_ptr<PRM::Node>> order(this->prmNodeVec_.begin(), this->prmNodeVec_.end());
		std::unordered_map<const PRM::Node*, int> id_of;
		id_of.reserve(order.size());
		for (int i = 0; i < (int)order.size(); ++i) id_of[order[i].get()] = i;

		// --- Nearest index helper (UAV) ---
		auto nearestIndex = [&](const Eigen::Vector3d& p) {
			int best = -1; double bestd = 1e18;
			for (int i = 0; i < (int)order.size(); ++i) {
				double d = (order[i]->pos - p).norm();
				if (d < bestd) { bestd = d; best = i; }
			}
			return best;
		};

		// --- Reachability check ---
		auto reachable = [&](int s, int t) {
			if (s < 0 || t < 0) return false;
			const int N = (int)order.size();
			std::vector<char> vis(N, 0);
			std::queue<int> q; q.push(s); vis[s] = 1;
			while (!q.empty()) {
				int u = q.front(); q.pop();
				if (u == t) return true;
				for (const auto& nb : order[u]->adjNodes) {
					auto it = id_of.find(nb.get());
					if (it != id_of.end()) {
						int v = it->second;
						if (!vis[v]) { vis[v] = 1; q.push(v); }
					}
				}
			}
			return false;
		};

		// --- Start Node (UAV Pose) ---
		const int start_idx = nearestIndex(this->position_);
		if (start_idx < 0) {
			ROS_WARN("[DEP] No valid start node found.");
			return false;
		}

		// =================================================================================
		// [STEP A] PREPARE UGV GRAPH & RUN DIJKSTRA (One-to-All)
		// =================================================================================
		
		// 1. Build UGV Node Vector
		std::vector<std::shared_ptr<PRM::Node>> ugv_order(this->prmNodeVecUGV_.begin(), this->prmNodeVecUGV_.end());
		
		// Map pointer -> index for UGV graph lookup
		std::unordered_map<const PRM::Node*, int> ugv_id_of;
		for (int i = 0; i < (int)ugv_order.size(); ++i) ugv_id_of[ugv_order[i].get()] = i;

		// 2. Find UGV Start Node (Nearest to current UGV Pose)
		// Project UGV actual pose to the Z-plane used by the PRM
		Eigen::Vector3d current_ugv_pos(ugv_pose_.position.x, ugv_pose_.position.y, z_proj_ugv_);

		// ROS_WARN("[DEP DEBUG] Current UGV Pose used for planning: [%.2f, %.2f]", 
        //  current_ugv_pos.x(), current_ugv_pos.y());

		int ugv_start_idx = -1;
		double min_start_dist = 1e9;
		
		for (int i = 0; i < (int)ugv_order.size(); ++i) {
			double d = (ugv_order[i]->pos - current_ugv_pos).norm();
			if (d < min_start_dist) { min_start_dist = d; ugv_start_idx = i; }
		}

		// 3. RUN DIJKSTRA
		// ugv_path_dists[i] will store the shortest GRAPH distance from UGV to Node i
		std::vector<double> ugv_path_dists(ugv_order.size(), 1e9);
		
		if (ugv_start_idx != -1) {
			ugv_path_dists[ugv_start_idx] = 0.0;
			
			// Priority Queue: <Distance, NodeIndex> (Min-Heap)
			typedef std::pair<double, int> PDI;
			std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq;
			pq.push({0.0, ugv_start_idx});

			while (!pq.empty()) {
				double d = pq.top().first;
				int u = pq.top().second;
				pq.pop();

				if (d > ugv_path_dists[u]) continue;

				// Explore Neighbors
				for (const auto& neighbor_ptr : ugv_order[u]->adjNodes) {
					if (ugv_id_of.find(neighbor_ptr.get()) == ugv_id_of.end()) continue;
					
					int v = ugv_id_of[neighbor_ptr.get()];
					// Edge weight = Euclidean distance between nodes
					double weight = (ugv_order[u]->pos - ugv_order[v]->pos).norm(); 

					if (ugv_path_dists[u] + weight < ugv_path_dists[v]) {
						ugv_path_dists[v] = ugv_path_dists[u] + weight;
						pq.push({ugv_path_dists[v], v});
					}
				}
			}
		} else {
			ROS_WARN("[DEP] UGV Start Node not found! Distances will be infinite.");
		}

		// =================================================================================
		// [STEP B] SELECT BEST TARGET NODE (Using Dijkstra Distances)
		// =================================================================================

		// int best_info_idx = -1;
		// double best_score = -1.0;

		// // --- Tuning Parameters ---
		// const double lambda = 0.001;       
		// const double ugv_avg_speed = 1.0; 

		// for (int i = 0; i < (int)order.size(); ++i) {
		// 	double info_gain = static_cast<double>(order[i]->numVoxels);
			
		// 	// Optimization: Skip nodes with negligible info gain
		// 	if (info_gain <= 50) continue;

		// 	if (projOf_.find(order[i].get()) == projOf_.end()) {
		// 		// This UAV node has no valid UGV projection (was rejected due to obstacles)
		// 		continue;
		// 	}

		// 	if (i == start_idx) {
		// 		continue;
		// 	}

		// 	// 1. Project UAV Node to Ground (to find matching UGV node)
		// 	Eigen::Vector3d target_proj = order[i]->pos;
		// 	target_proj.z() = z_proj_ugv_;

		// 	// 2. Find Nearest UGV PRM Node to this target location
		// 	// (If your PRMs are generated 1:1, this search might be replaced by direct indexing)
		// 	int nearest_ugv_node = -1;
		// 	double min_d = 1e9;
			
		// 	for (int k = 0; k < (int)ugv_order.size(); ++k) {
		// 		double d = (ugv_order[k]->pos - target_proj).norm();
		// 		if (d < min_d) { min_d = d; nearest_ugv_node = k; }
		// 	}

		// 	// 3. Get Shortest Path Distance
		// 	// Total Dist = (Path on Graph) + (Residual dist from graph node to exact target)
		// 	double path_dist = (nearest_ugv_node != -1) ? ugv_path_dists[nearest_ugv_node] : 1e9;
		// 	double total_dist = path_dist + min_d;

		// 	// Cap unreachable distance to prevent overflow/NaN issues
		// 	if (total_dist > 1e8) total_dist = 1000.0; 

		// 	double t_ugv_path = total_dist / ugv_avg_speed;
			
		// 	// 4. Calculate Score
		// 	double score = info_gain * std::exp(-lambda * t_ugv_path);

		// 	if (score > best_score) { 
		// 		best_score = score; 
		// 		best_info_idx = i; 
		// 	}
		// }

		// if (best_info_idx < 0) {
		// 	ROS_WARN("[DEP] No valid info node found.");
		// 	return false;
		// }

		// // Determine UAV End Node
		// int uav_end_idx = best_info_idx;

		// if (uav_end_idx == start_idx) {
		// 	ROS_ERROR("[DEP] CRITICAL: Selected end node is same as start! start_idx=%d", start_idx);
			
		// 	// Emergency fallback: Find any other node with gain
		// 	for (int i = 0; i < (int)order.size(); ++i) {
		// 		if (i != start_idx && order[i]->numVoxels > 0 && reachable(start_idx, i)) {
		// 			uav_end_idx = i;
		// 			ROS_WARN("[DEP] Emergency fallback: Using node %d instead", i);
		// 			break;
		// 		}
		// 	}
			
		// 	// If still same, abort
		// 	if (uav_end_idx == start_idx) {
		// 		ROS_ERROR("[DEP] Cannot find valid end node different from start. Aborting.");
		// 		return false;
		// 	}
		// }

		// // Verify Reachability (UAV Side)
		// if (!reachable(start_idx, uav_end_idx)) {
		// 	ROS_WARN("[DEP] Best Info Node is unreachable from start! Searching fallback...");
			
		// 	int fallback = -1;
		// 	double bestReachGain = -1.0;

		// 	for (int i = 0; i < (int)order.size(); ++i) {
		// 		double g = static_cast<double>(order[i]->numVoxels);
		// 		if (g > bestReachGain && reachable(start_idx, i)) {
		// 			bestReachGain = g; fallback = i;
		// 		}
		// 	}
			
		// 	if (fallback >= 0) {
		// 		uav_end_idx = fallback;
		// 		ROS_INFO("[DEP] Fallback successful. New end node idx: %d", fallback);
		// 	} else {
		// 		ROS_ERROR("[DEP] No reachable nodes found in PRM. Aborting.");
		// 		return false;
		// 	}


		int best_info_idx = -1;
		double best_score = -1.0;

		// --- Tuning Parameters ---
		const double lambda = 0.001;       
		const double ugv_avg_speed = 1.0; 

		// Rejection counters for debugging
		int rejected_low_gain = 0;
		int rejected_no_projection = 0;
		int rejected_ugv_unreachable = 0;
		int rejected_is_start = 0;
		int rejected_vertical_blocked = 0;


		for (int i = 0; i < (int)order.size(); ++i) {
			// Skip if this is the start node
			if (i == start_idx) {
				rejected_is_start++;
				continue;
			}
			
			double info_gain = static_cast<double>(order[i]->numVoxels);
			
			// Skip nodes with negligible info gain
			if (info_gain <= 50) {
				rejected_low_gain++;
				continue;
			}

			// ============================================================
			// CHECK 1: Does this UAV node have a valid UGV projection?
			// ============================================================
			auto proj_it = projOf_.find(order[i].get());
			if (proj_it == projOf_.end()) {
				rejected_no_projection++;
				continue;
			}

			const auto& proj_ugv_node = proj_it->second; 
			Eigen::Vector3d uav_pos = order[i]->pos;
			Eigen::Vector3d ugv_pos = proj_ugv_node->pos;
			
			// Raycast from UAV position DOWN to UGV position
			bool vertical_path_clear = this->map_->isInflatedFreeLine(uav_pos, ugv_pos);
			
			if (!vertical_path_clear) {
				rejected_vertical_blocked++;
				continue;  // Obstacle between UAV and UGV positions (e.g., scaffolding)
			}
			// ============================================================

			// ============================================================
			// CHECK 2: Find the projected UGV node's index
			// ============================================================
			const auto& projected_ugv_node = proj_it->second;
			int projected_ugv_idx = -1;
			
			// Find this projected node in ugv_order
			for (int k = 0; k < (int)ugv_order.size(); ++k) {
				if (ugv_order[k] == projected_ugv_node) {
					projected_ugv_idx = k;
					break;
				}
			}
			
			if (projected_ugv_idx < 0) {
				ROS_WARN_THROTTLE(5.0, "[DEP] Projected UGV node not found in ugv_order!");
				rejected_no_projection++;
				continue;
			}
			// ============================================================

			// ============================================================
			// CHECK 3: Is the projected UGV node reachable via graph?
			// Use Dijkstra distance - if >= 1e8, it means no path exists
			// ============================================================
			double ugv_graph_dist = ugv_path_dists[projected_ugv_idx];
			
			if (ugv_graph_dist >= 1e8) {
				rejected_ugv_unreachable++;
				continue;  // UGV cannot reach this target
			}
			// ============================================================

			// Now calculate score using the ACTUAL reachable distance
			double t_ugv_path = ugv_graph_dist / ugv_avg_speed;
			
			// Calculate Score
			double score = info_gain * std::exp(-lambda * t_ugv_path);

			if (score > best_score) { 
				best_score = score; 
				best_info_idx = i; 
			}
		}

		// ============================================================
		// LOG REJECTION STATISTICS
		// ============================================================
		ROS_INFO("[DEP Target] Evaluated %d UAV nodes:", (int)order.size());
		ROS_INFO("[DEP Target]   Rejected: %d (is start)", rejected_is_start);
		ROS_INFO("[DEP Target]   Rejected: %d (low gain < 50)", rejected_low_gain);
		ROS_INFO("[DEP Target]   Rejected: %d (no UGV projection)", rejected_no_projection);
		ROS_INFO("[DEP Target]   Rejected: %d (UGV unreachable)", rejected_ugv_unreachable);
		ROS_INFO("[DEP Target]   Valid candidates: %d", 
				(int)order.size() - rejected_is_start - rejected_low_gain - 
				rejected_no_projection - rejected_ugv_unreachable);
		// ============================================================

		if (best_info_idx < 0) {
			ROS_WARN("[DEP] No valid target found (must have: gain>50, UGV projection, UGV reachable).");
			return false;
		}

		// Determine UAV End Node
		int uav_end_idx = best_info_idx;

		// Sanity check
		if (uav_end_idx == start_idx) {
			ROS_ERROR("[DEP] CRITICAL: Selected end == start! This shouldn't happen.");
			return false;
		}

		ROS_INFO("[DEP Target] Selected UAV node %d: IG=%.0f, UGV dist=%.2fm, Score=%.1f", 
				uav_end_idx, 
				(double)order[uav_end_idx]->numVoxels,
				best_score);

		// Verify UAV Reachability
		if (!reachable(start_idx, uav_end_idx)) {
			ROS_WARN("[DEP] UAV: Best target unreachable from start! Searching fallback...");
			
			int fallback = -1;
			double bestReachGain = -1.0;

			for (int i = 0; i < (int)order.size(); ++i) {
				if (i == start_idx) continue;
				
				double g = static_cast<double>(order[i]->numVoxels);
				
				// Also check UGV reachability for fallback
				auto proj_it = projOf_.find(order[i].get());
				if (proj_it == projOf_.end()) continue;
				
				const auto& proj_node = proj_it->second;
				int proj_idx = -1;
				for (int k = 0; k < (int)ugv_order.size(); ++k) {
					if (ugv_order[k] == proj_node) {
						proj_idx = k;
						break;
					}
				}
				
				if (proj_idx < 0 || ugv_path_dists[proj_idx] >= 1e8) continue;
				
				if (g > bestReachGain && reachable(start_idx, i)) {
					bestReachGain = g; 
					fallback = i;
				}
			}
			
			if (fallback >= 0) {
				uav_end_idx = fallback;
				ROS_INFO("[DEP] Fallback successful. New target: node %d", fallback);
			} else {
				ROS_ERROR("[DEP] No reachable nodes found (UAV+UGV). Aborting.");
				return false;
			}
		}

		

		// =================================================================================
		// [STEP C] SOLVE UAV OP
		// =================================================================================

		ros::Time uav_op_start = ros::Time::now();
		OPParams opp;
		opp.GRASP_ITER     = 40;
		opp.ELITE_SET_SIZE = 10;
		opp.ALPHA          = 0.30;
		opp.T_MAX          = this->timeBudgetSec_;
		this->uavCruiseSpeed_ = std::max(0.1, this->uavCruiseSpeed_);

		double uav_path_time = 0.0;
		std::vector<int> best_idx_uav = this->selectOrienteeringPathGRASPPR(
			order, id_of, start_idx, uav_end_idx, opp, this->uavCruiseSpeed_, uav_path_time);

		double uav_op_time = (ros::Time::now() - uav_op_start).toSec();

		if (best_idx_uav.empty()) {
			ROS_WARN("[DEP] OP/GRASP returned empty path.");
			return false;
		}

		// ============================================================
		// CALCULATE PATH LENGTH: distance = speed × time
		// ============================================================
		// current_uav_path_length_ = this->uavCruiseSpeed_ * uav_path_time;
		// total_uav_path_length_ += current_uav_path_length_;
		// uav_path_lengths_history_.push_back(current_uav_path_length_);

		ROS_INFO("[DEP Timing] UAV OP solve: %.3f s", uav_op_time);
		// ROS_INFO("[DEP Path] UAV: %.2fs @ %.2fm/s = %.2fm (%zu nodes)", 
		// 		uav_path_time, this->uavCruiseSpeed_, current_uav_path_length_, best_idx_uav.size());

		// Publish UAV Path
		{
			this->bestPath_.clear();
			this->bestPath_.reserve(best_idx_uav.size());
			for (int idx : best_idx_uav) this->bestPath_.push_back(order[idx]);
		}
		nav_msgs::Path path_msg = this->getBestPath();
		if (this->uav_path_pub_) {
			this->uav_path_pub_.publish(path_msg);
		}

		// =================================================================================
		// [STEP D] SOLVE UGV OP
		// =================================================================================
		if (ugv_order.empty()) {
			ROS_WARN("[UGV_OP] No UGV nodes available.");
			return true; 
		}

		// 1. Define Start Node (Already calculated in Step A as ugv_start_idx)
		// but good to re-validate
		if (ugv_start_idx == -1) {
			ROS_WARN("[UGV_OP] Could not find valid start index.");
			return true;
		}

		// 2. Define End Node (Same as UAV End Node)
		std::shared_ptr<PRM::Node> uav_final_node = this->bestPath_.back();
		Eigen::Vector3d target_ugv_pos = uav_final_node->pos;
		target_ugv_pos.z() = z_proj_ugv_; 

		int ugv_end_idx = -1;
		double min_dist = std::numeric_limits<double>::max();

		// Reuse the lookup logic to find the closest UGV node to the goal
		for (int i = 0; i < (int)ugv_order.size(); ++i) {
			double d = (ugv_order[i]->pos - target_ugv_pos).norm();
			if (d < min_dist) {
				min_dist = d;
				ugv_end_idx = i;
			}
		}

		if (ugv_end_idx == -1) {
			ROS_WARN("[UGV_OP] Could not find valid end index.");
			return true;
		}

		// 3. Build UGV Cost & Score Matrices
		double ugv_speed = 1.0; // m/s
		std::vector<double> S_ugv;
		std::vector<std::vector<double>> T_ugv;
		this->buildScoresAndTimes(ugv_order, S_ugv, T_ugv, ugv_speed);

		ros::Time ugv_op_start = ros::Time::now();

		// 4. Setup Solver Parameters for UGV
		OPParams ugv_opp;
		ugv_opp.GRASP_ITER     = 40;
		ugv_opp.ELITE_SET_SIZE = 10;
		ugv_opp.ALPHA          = 0.2;
		
		// Calculate straight line distance (fallback min time)
		double straight_line_dist = (ugv_order[ugv_start_idx]->pos - ugv_order[ugv_end_idx]->pos).norm();
		
		// Use Dijkstra distance if available for better budget estimation
		double dijkstra_dist = ugv_path_dists[ugv_end_idx];
		double estimated_dist = (dijkstra_dist < 1e8) ? dijkstra_dist : straight_line_dist;
		
		double min_time = estimated_dist / ugv_speed;
		ugv_opp.T_MAX = std::max(15.0, min_time * 1.2);

		double ugv_path_time = 0.0;
		std::vector<int> best_idx_ugv = this->selectOrienteeringPathGRASPPR(
			ugv_order, ugv_id_of, ugv_start_idx, ugv_end_idx, ugv_opp, ugv_speed, ugv_path_time);


		double ugv_op_time = (ros::Time::now() - ugv_op_start).toSec();

		if (best_idx_ugv.empty()) {
			ROS_WARN("[UGV_OP] Solver failed to find path.");
			return true;
		}
		// current_ugv_path_length_ = ugv_speed * ugv_path_time;
		// total_ugv_path_length_ += current_ugv_path_length_;
		// ugv_path_lengths_history_.push_back(current_ugv_path_length_);

		ROS_INFO("[DEP Timing] UGV OP solve: %.3f s", ugv_op_time);
		// ROS_INFO("[DEP Path] UGV: %.2fs @ %.2fm/s = %.2fm (%zu nodes)", 
		// 		ugv_path_time, ugv_speed, current_ugv_path_length_, best_idx_ugv.size());

		// 7. Publish Result
		nav_msgs::Path ugv_path_msg;
		ugv_path_msg.header.stamp = ros::Time::now();
		ugv_path_msg.header.frame_id = "map";

		for (int idx : best_idx_ugv) {
			geometry_msgs::PoseStamped pose;
			pose.header = ugv_path_msg.header;
			pose.pose.position.x = ugv_order[idx]->pos.x();
			pose.pose.position.y = ugv_order[idx]->pos.y();
			pose.pose.position.z = ugv_order[idx]->pos.z();
			ugv_path_msg.poses.push_back(pose);
		}

		this->ugv_path_pub_.publish(ugv_path_msg);
		// ROS_INFO("[UGV_OP] Plan generated with %lu nodes. Shared End Goal: (%.2f, %.2f)", 
		// 		best_idx_ugv.size(), target_ugv_pos.x(), target_ugv_pos.y());

		// Record total planning time for this iteration
		double this_plan_time = (ros::Time::now() - plan_start).toSec();
		total_planning_time_ += this_plan_time;
		planning_iterations_++;

		ROS_INFO("========== PLANNING ITERATION %d COMPLETE ==========", planning_iterations_);
		ROS_WARN("========== CYCLE EXPLORATION RESULTS ==========");
		ROS_INFO("[DEP Timing] This planning iteration: %.3f s", this_plan_time);
		ROS_INFO("[DEP Timing] Average planning time: %.3f s", 
				total_planning_time_ / planning_iterations_);
		ROS_INFO("[DEP Timing] Total planning time so far: %.3f s", total_planning_time_);

		ROS_WARN(" Explored  (%d voxels)", explored_voxels);
		ROS_WARN(" Coverage: %.1f%% of mapped region", coverage_percent);
		ROS_WARN("-----------------------------------------------");

		return true;
	}

	nav_msgs::Path DEP::getBestPath(){
		nav_msgs::Path bestPath;
		for (int i=0; i<int(this->bestPath_.size()); ++i){
			std::shared_ptr<PRM::Node> currNode = this->bestPath_[i];
			geometry_msgs::PoseStamped p;
			p.pose.position.x = currNode->pos(0);
			p.pose.position.y = currNode->pos(1);
			p.pose.position.z = currNode->pos(2);
			if (i < int(this->bestPath_.size())-1){
				std::shared_ptr<PRM::Node> nextNode = this->bestPath_[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));
				p.pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, angle);
			}
			bestPath.poses.push_back(p);
		}
		
		// get the best yaw for the last pose
		double bestYaw = this->bestPath_.back()->getBestYaw();
		bestPath.poses.back().pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, bestYaw);

		return bestPath;
	}

	bool DEP::sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2){
		Eigen::Vector3d direction = n2->pos - n1->pos;
		Eigen::Vector3d projection;
		projection(0) = direction.x();
		projection(1) = direction.y();
		projection(2) = 0;
		double verticalAngle = angleBetweenVectors(direction, projection);
		if (verticalAngle < this->verticalFOV_/2){
			return true;
		}
		else{
			return false;
		}
	}

	// create sensor check for 
	// vert, horz FOV, collision, and sensor distance range
	// for yaw angles in vector3d:  cos(yaw), sin(yaw), 0
	// horz angle between yaw angle vector and direction (x y 0) vector for horz FOV
	// Vert angle yaw angle vector and yaw angle vector (c s z) z is direction.z()
	bool DEP::sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos){
		Eigen::Vector3d direction = sample - pos;
		double distance = direction.norm();
		if (distance > this->dmax_){
			return false;
		}
		bool hasCollision = this->map_->isInflatedOccupiedLine(sample, pos);
		if (hasCollision == true){
			return false;
		}
		return true;
	}


	static inline std::shared_ptr<PRM::Node> makeNodeAt(const Eigen::Vector3d& p)
	{
	auto n = std::make_shared<PRM::Node>(p); // or (p.x(), p.y(), p.z(), <other required args>)
	n->newNode   = false;
	n->numVoxels = 0;
	return n;
	}

	// void DEP::buildUGVFromUAVProjection()
	// {
	// projOf_.clear();
	// prmNodeVecUGV_.clear();
	// if (!roadmap_ugv_) roadmap_ugv_ = std::make_shared<PRM::KDTree>();
	// roadmap_ugv_->clear();

	// size_t made = 0;

	// const auto& UAV = prmNodeVec_;          

	// int rejected_nodes = 0;

	// for (const auto& uavPtr : UAV)
	// {
	// 	const PRM::Node* raw = uavPtr.get();
	// 	Eigen::Vector3d p = uavPtr->pos;  
	// 	p.z() = z_proj_ugv_; // Drop to ground height

	// 	// -------------------------------------------------------------
	// 	// FIX 1: VALIDITY CHECK
	// 	// Check if the robot body (at ground level) is in collision
	// 	// -------------------------------------------------------------
		
	// 	// Check base 
	// 	if (this->map_->isInflatedOccupied(p)) {
	// 		rejected_nodes++;
	// 		continue; // Do not create a UGV node inside a wall/floor
	// 	}

	// 	// Check Sensor Head (feet + sensor height)
	// 	Eigen::Vector3d sensorCheck = p;
	// 	sensorCheck(2) += this->ugvSensorHeight_; 
	// 	if (this->map_->isInflatedOccupied(sensorCheck)) {
	// 		rejected_nodes++;
	// 		continue; // Do not create a UGV node if sensor would be in collision
	// 	}
	// 	auto n = std::make_shared<PRM::Node>(p); 
	// 	n->newNode = false;
		
	// 	// Calculate gain (using the function we debugged earlier)
	// 	n->numVoxels = this->calculateUnknownUGV(n); // recalc info gain for UGV node
		
	// 	roadmap_ugv_->insert(n);
	// 	prmNodeVecUGV_.insert(n);
	// 	projOf_[raw] = n;
	// 	++made;
	// }

	// std::cout << "[DEP] Projection: Created " << made << " nodes. Rejected " << rejected_nodes << " nodes due to ground obstacles." << std::endl;

	// ROS_WARN("[UGV-PRM:DEBUG] forced 1:1 projections: UAV=%zu  UGV=%zu", UAV.size(), made);

	// for (const auto& nUAV : prmNodeVec_) {
	// 	auto itU = projOf_.find(nUAV.get());
	// 	if (itU == projOf_.end()) continue;
	// 	const auto& u = itU->second;

	// 	for (const auto& nbUAV : nUAV->adjNodes) {
	// 	auto itV = projOf_.find(nbUAV.get());
	// 	if (itV == projOf_.end()) continue;
	// 	const auto& v = itV->second;
	// 	if (u == v) continue;

	// 	// Range & collision checks (on ground plane)
	// 	double d = (u->pos - v->pos).norm();
	// 	bool inRange = (d < maxConnectDist_);
	// 	bool free = inRange && map_ && map_->isInflatedFreeLine(u->pos, v->pos);

	// 	if (true) {
	// 		u->adjNodes.insert(v);
	// 		v->adjNodes.insert(u);
	// 	}
	// 	}
	// }

	// // 3) (Optional) Add kNN stitching among UGV nodes to improve connectivity
	// const int k = nnNum_;
	// for (const auto& nUGV : prmNodeVecUGV_) {
	// 	auto knn = roadmap_ugv_->kNearestNeighbor(nUGV, k);
	// 	for (const auto& m : knn) {
	// 	if (m == nUGV) continue;
	// 	double d = (nUGV->pos - m->pos).norm();
	// 	if (d >= maxConnectDist_) continue;
	// 	bool free = map_ && map_->isInflatedFreeLine(nUGV->pos, m->pos);
	// 	if (free) {
	// 		nUGV->adjNodes.insert(m);
	// 		m->adjNodes.insert(nUGV);
	// 	}
	// 	}
	// }

	// ROS_INFO("[UGV PRM] nodes=%zu edges≈%zu",
	// 		prmNodeVecUGV_.size(),
	// 		[&](){
	// 			size_t e=0; for (auto& n:prmNodeVecUGV_) e += n->adjNodes.size();
	// 			return e/2;
	// 		}());
	// }

	void DEP::buildUGVFromUAVProjection()
	{
		// DON'T clear - keep existing nodes!
		// projOf_.clear();        // ← REMOVED
		// prmNodeVecUGV_.clear(); // ← REMOVED
		// roadmap_ugv_->clear();  // ← REMOVED
		
		if (!roadmap_ugv_) roadmap_ugv_ = std::make_shared<PRM::KDTree>();

		size_t made = 0;
		int rejected_nodes = 0;
		int skipped_existing = 0;
		int rejected_unsafe = 0;

		const auto& UAV = prmNodeVec_;

		// Only project NEW UAV nodes that haven't been projected yet
		for (const auto& uavPtr : UAV) {
			const PRM::Node* raw = uavPtr.get();
			
			// Check if already projected
			if (projOf_.find(raw) != projOf_.end()) {
				++skipped_existing;
				continue;  // Skip already-projected nodes
			}
			
			Eigen::Vector3d p = uavPtr->pos;  
			p.z() = z_proj_ugv_ + 0.4; // Drop to ground height

			if (!this->isPosValidUGV(p)) {
				rejected_unsafe++;
				continue;  // Reject if too close to obstacles
			}

			// Validity checks
			if (this->map_->isInflatedOccupied(p)) {
				rejected_nodes++;
				continue;
			}

			Eigen::Vector3d sensorCheck = p;
			sensorCheck(2) += this->ugvSensorHeight_; 
			if (this->map_->isInflatedOccupied(sensorCheck)) {
				rejected_nodes++;
				continue;
			}
			
			auto n = std::make_shared<PRM::Node>(p); 
			n->newNode = true;   // Mark as NEW (will be updated in updateInformationGainUGV)
			n->numVoxels = 0;    // Don't calculate yet - will be done in update phase
			
			roadmap_ugv_->insert(n);
			prmNodeVecUGV_.insert(n);
			projOf_[raw] = n;
			++made;
		}

		// std::cout << "[DEP] Projection: Created " << made << " NEW nodes. "
		// 		<< "Skipped " << skipped_existing << " existing. "
		// 		<< "Rejected " << rejected_nodes << " due to obstacles." << std::endl;

		// ROS_INFO("[UGV-PRM] Incremental projection: UAV=%zu  UGV=%zu  New=%zu  Existing=%zu", 
		// 		UAV.size(), prmNodeVecUGV_.size(), made, skipped_existing);
	}

	bool isNodeRequireUpdate(std::shared_ptr<PRM::Node> n, std::vector<std::shared_ptr<PRM::Node>> path, double& leastDistance){
		double distanceThresh = 2;
		leastDistance = std::numeric_limits<double>::max();
		for (std::shared_ptr<PRM::Node>& waypoint: path){
			double currentDistance = (n->pos - waypoint->pos).norm();
			if (currentDistance < leastDistance){
				leastDistance = currentDistance;
			}
		}
		if (leastDistance <= distanceThresh){
			return true;
		}
		else{
			return false;	
		}
		
	}

	void DEP::detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs){
		frontierPointPairs.clear();

		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		int numRow = (mapMax(1) - mapMin(1))/this->map_->getRes() + 1;
		int numCol = (mapMax(0) - mapMin(0))/this->map_->getRes() + 1;

		cv::SimpleBlobDetector::Params params;

		params.filterByColor = true;
		params.blobColor = 255;  // Blobs should be white
		params.filterByArea = true;
		params.minArea = pow(1/this->map_->getRes(), 2);
		params.maxArea = numRow * numCol;
		params.filterByCircularity = false;
		params.minCircularity = 1;
		params.filterByConvexity = true;
		params.minConvexity = 0.1;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		std::vector<cv::Mat> imgVec;
		// find height levels to slice the map
		double heightRes = 0.3;
		int col = 0;
		int row = 0;
		for (double h=this->globalRegionMin_(2); h<=this->globalRegionMax_(2); h+=heightRes){
			row = 0;
			cv::Mat im (numRow, numCol, CV_8UC1);
			for (double y=mapMin(1); y<=mapMax(1) and row < numRow; y+=this->map_->getRes()){
				col = 0;
				for (double x=mapMin(0); x<=mapMax(0) and col < numCol; x+=this->map_->getRes()){
					Eigen::Vector3d p (x, y, h);
					if (this->map_->isInflatedOccupied(p)){
						im.at<uchar>(row, col) = 0;
					}
					else if (this->map_->isInflatedFree(p)){
						// im.at<uchar>(row, col) = 255/2;
						// im.at<uchar>(row, col) = 255;
						im.at<uchar>(row, col) = 0;
					}
					else{
						// im.at<uchar>(row, col) = 255/2;
						im.at<uchar>(row, col) = 255;
					}
					++col;
				}
				++row;
			}
			imgVec.push_back(im);
		}



		// detect each image and find the corresponding 3D positions
		double height = this->globalRegionMin_(2);
		for (cv::Mat img : imgVec){
			std::vector<cv::KeyPoint> keypoints;

			cv::Rect rect(0, 0, numRow, numCol);
			cv::rectangle(img, rect, cv::Scalar(0, 0, 0), 3);
			detector->detect(img, keypoints);

			// convert keypoints back to the map coordinate
			for (cv::KeyPoint keypoint : keypoints){
				Eigen::Vector3d p (mapMin(0) + keypoint.pt.x * this->map_->getRes(), mapMin(1) + keypoint.pt.y * this->map_->getRes(), height);
				double dist = keypoint.size * this->map_->getRes();
				frontierPointPairs.push_back({p, dist});
			}

			height += heightRes;

			// cv::Mat imgWithKeypoints;
			// cv::drawKeypoints(img, keypoints, imgWithKeypoints,  cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// cv::imshow("Blobs with Keypoints", imgWithKeypoints);
			// cv::imwrite("/home/zhefan/Desktop/temp/test.jpg", img);
			// cv::waitKey(0);
		} 
	}

	namespace {
	inline double urand(double a, double b) {
	return a + (b - a) * (double)std::rand() / (double)RAND_MAX;
	}
	}

	// ==============================================================================
	// 1. HELPER: Enforce Min Height for Sampling (Z > 1.0)
	// ==============================================================================
	void DEP::computeEffectiveBox(const Eigen::Vector3d& reqMin,
								const Eigen::Vector3d& reqMax,
								Eigen::Vector3d& effMin,
								Eigen::Vector3d& effMax) const
	{
		effMin = reqMin;
		effMax = reqMax;

		if (this->clampToMapRange_) {
			Eigen::Vector3d mapMin, mapMax;
			this->map_->getCurrMapRange(mapMin, mapMax);
			effMin = effMin.cwiseMax(mapMin);
			effMax = effMax.cwiseMin(mapMax);
		}
		effMin = effMin.cwiseMax(this->globalRegionMin_);
		effMax = effMax.cwiseMin(this->globalRegionMax_);

		// --- CRITICAL CHANGE: UAV Sample Floor ---
		// Force the sampling floor to be at least 1.0m
		if (effMin.z() < 1.0) {
			effMin.z() = 1.0;
		}
		// Safety: ensure max is at least min
		if (effMax.z() < effMin.z()) effMax.z() = effMin.z() + 0.1;
	}

	// ==============================================================================
	// 2. HELPER: Random Config (Pure Air Sampling)
	// ==============================================================================
	std::shared_ptr<PRM::Node> DEP::randomConfigBBoxValid(const Eigen::Vector3d& bbMin,
														const Eigen::Vector3d& bbMax)
	{
		if ((bbMax.array() <= bbMin.array()).any()) return nullptr;

		for (int k = 0; k < 40 && ros::ok(); ++k) {
			Eigen::Vector3d p;
			p.x() = urand(bbMin.x(), bbMax.x());
			p.y() = urand(bbMin.y(), bbMax.y());
			p.z() = urand(bbMin.z(), bbMax.z());
			
			// Double check Z limit just in case
			if (p.z() < 1.0) continue; 

			if (this->isPosValid(p, this->safeDistXY_, this->safeDistZ_)) {
				return std::make_shared<PRM::Node>(p);
			}
		}
		return nullptr;
	}

	// Replaces the old "Mixed" function. Now it just calls the standard BBox sampler.
	std::shared_ptr<PRM::Node> DEP::randomConfigMixed(const Eigen::Vector3d& reqMin,
													const Eigen::Vector3d& reqMax)
	{
		Eigen::Vector3d effMin, effMax;
		computeEffectiveBox(reqMin, reqMax, effMin, effMax);
		// std::cout << "[DEP DEBUG] Sampling Box Min: [" << effMin.x() << ", " << effMin.y() << ", " << effMin.z() << "]" << std::endl;
		// std::cout << "[DEP DEBUG] Sampling Box Max: [" << effMax.x() << ", " << effMax.y() << ", " << effMax.z() << "]" << std::endl;
		return randomConfigBBoxValid(effMin, effMax);
	}


	// ==============================================================================
	// 3. MAIN: Build Road Map (Sampling Loop)
	// ==============================================================================
	void DEP::buildRoadMap(){
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodes;

		// --- A. Frontier Sampling ---
		std::vector<double> sampleWeights;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			double size = this->frontierPointPairs_[i].second;
			sampleWeights.push_back(pow(size, 2));
		}
		
		int countFrontierFailure = 0;
		while (ros::ok() and countFrontierFailure < this->frontierSampleThresh_ and !sampleWeights.empty()){
			std::shared_ptr<PRM::Node> fn = this->sampleFrontierPoint(sampleWeights);
			
			// Filter: Ignore frontier points that are too low
			if (fn->pos.z() < 1.0) {
				countFrontierFailure++; 
				continue;
			}

			std::vector<std::shared_ptr<PRM::Node>> fnNeighbors = this->roadmap_->kNearestNeighbor(fn, this->nnNumFrontier_);

			if (!fnNeighbors.empty()){
				int countSampleOnce = 0;
				for (std::shared_ptr<PRM::Node> fnNN : fnNeighbors){
					n = this->extendNode(fnNN, fn);
					
					// CRITICAL CHECK: Ensure new extended node is also > 1.0m
					if (n->pos.z() < 1.0) continue;

					if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
						std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						double distToNN = (n->pos - nn->pos).norm();
						if (distToNN >= this->distThresh_){
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;  
							++countSampleOnce;
						}
					}
				}
				if (countSampleOnce == 0){
					++countFrontierFailure;
				}
			}
			else{ 
				break;
			}
		}

		// --- B. Random Sampling (Global & Local) ---
		// Note: randomConfigBBox calls computeEffectiveBox, which we modified to enforce Z > 1.0
		while (ros::ok() and not saturate){
			if (regionSaturate){
				// Global Sampling
				int countFailureGlobal = 0;
				while (ros::ok()){
					if (countFailureGlobal > this->globalSampleThresh_){
						saturate = true;
						break;
					}
					// Use mixed/valid sampler which enforces 1.0m height
					n = this->randomConfigMixed(this->globalRegionMin_, this->globalRegionMax_);
					
					if (n == nullptr) { ++countFailureGlobal; continue; }

					double distToNN;
					if (this->roadmap_->getSize() != 0){
						shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						distToNN = (n->pos - nn->pos).norm();
					} else {
						distToNN = this->distThresh_;
					}

					if (distToNN < this->distThresh_){
						++countFailureGlobal;
					} else {
						this->roadmap_->insert(n);
						newNodes.push_back(n);
						this->prmNodeVec_.insert(n);
						++countSample;
					}
				}
			}
			else{
				// Local Sampling
				int countFailureLocal = 0;
				while (ros::ok()){
					if (countFailureLocal > this->localSampleThresh_){
						regionSaturate = true;
						break;
					}
					Eigen::Vector3d localSampleMin = this->position_ + this->localRegionMin_;
					Eigen::Vector3d localSampleMax = this->position_ + this->localRegionMax_;
					
					n = this->randomConfigMixed(localSampleMin, localSampleMax);

					if (n == nullptr) { ++countFailureLocal; continue; }

					double distToNN;
					if (this->roadmap_->getSize() != 0){
						std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						distToNN = (n->pos - nn->pos).norm();
					} else {
						distToNN = this->distThresh_;
					}

					if (distToNN < this->distThresh_){
						++countFailureLocal;
					} else {
						this->roadmap_->insert(n);
						newNodes.push_back(n);
						this->prmNodeVec_.insert(n);
						++countSample;
					}
				}
			}
		}

		// --- C. Connection ---
		for (std::shared_ptr<PRM::Node>& n : newNodes){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn){ 
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				bool rangeCondition = sensorRangeCondition(n, nearestNeighborNode) and sensorRangeCondition(nearestNeighborNode, n);
				if (distance2knn < this->maxConnectDist_ and rangeCondition == true){
					bool hasCollision = not this->map_->isInflatedFreeLine(n->pos, nearestNeighborNode->pos);
					if (hasCollision == false){
						n->adjNodes.insert(nearestNeighborNode);
						nearestNeighborNode->adjNodes.insert(n);
					}
				}
			}
			n->newNode = true;
		}
	}	 


	void DEP::printPRMNodesInformationGain() {
		std::cout << "\nPRM Node Information Gain List:" << std::endl;
		for (const auto& node : this->prmNodeVec_) {
			std::cout << "Node position: (" << node->pos(0) << ", "
					<< node->pos(1) << ", " << node->pos(2) << ")  "
					<< "Information Gain: " << node->numVoxels << std::endl;
		}
	}


	void DEP::pruneNodes(){
		// record the invalid nodes
		std::unordered_set<std::shared_ptr<PRM::Node>> invalidSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (not this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){// 1. new nodes
			// if (this->map_->isInflatedOccupied(n->pos)){// 1. new nodes
				invalidSet.insert(n);
			}	
		}

		// remove invalid nodes
		for (std::shared_ptr<PRM::Node> in : invalidSet){
			this->prmNodeVec_.erase(in);
			this->roadmap_->remove(in);
		}


		//  remove invalid edges
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			std::vector<std::shared_ptr<PRM::Node>> eraseVec;
			for (std::shared_ptr<PRM::Node> neighbor : n->adjNodes){
				if (invalidSet.find(neighbor) != invalidSet.end()){
					eraseVec.push_back(neighbor);
				}
			}

			for (std::shared_ptr<PRM::Node> en : eraseVec){
				n->adjNodes.erase(en);
			}
		}
	}

	void DEP::updateInformationGain(){
		// iterate through all current nodes (ignore update by path now)
		// two types of nodes need update:
		// 1. new nodes
		// 2. nodes close to the historical trajectory
		std::unordered_set<std::shared_ptr<PRM::Node>> updateSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (n->newNode == true){// 1. new nodes
				updateSet.insert(n);
			}	
		}

		for (Eigen::Vector3d& histPos : this->histTraj_){ // traj update nodes
			std::shared_ptr<PRM::Node> histN;
			histN.reset(new PRM::Node(histPos));
			std::vector<std::shared_ptr<PRM::Node>> nns = this->roadmap_->kNearestNeighbor(histN, 10);
			for (std::shared_ptr<PRM::Node>& nn : nns){
				if ((nn->pos - histN->pos).norm() <= this->updateDist_){
					updateSet.insert(nn);
				}
			}
		}

		for (std::shared_ptr<PRM::Node> updateN : updateSet){ // update information gain
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(updateN, yawNumVoxels);
			updateN->numVoxels = unknownVoxelNum;
			updateN->yawNumVoxels = yawNumVoxels;
			updateN->newNode = false;
		}
		this->histTraj_.clear(); // clear history
	}

	

	void DEP::updateInformationGainUGV() {
    // Calculate info gain for ALL UGV nodes with smart skipping
    
		ROS_INFO("[UGV-IG] Updating info gain for UGV nodes...");
		ros::Time ig_start = ros::Time::now();
		
		int updated_count = 0;
		int skipped_low_gain = 0;
		int skipped_zero_gain = 0;
		
		// Iterate through ALL UGV nodes
		for (std::shared_ptr<PRM::Node> node : this->prmNodeVecUGV_) {
			
			// Skip nodes that had zero gain in previous iteration (likely fully blocked)
			if (!node->newNode && node->numVoxels == 0) {
				skipped_zero_gain++;
				continue;
			}
			
			// Skip nodes that had very low gain (< 100) in previous iteration
			// These are likely in poor positions and won't improve
			if (!node->newNode && node->numVoxels > 0 && node->numVoxels < 100) {
				skipped_low_gain++;
				node->numVoxels = 0; // Set to zero to skip in future iterations
				continue;
			}
			
			// Calculate info gain for:
			// 1. New nodes (newNode == true)
			// 2. Nodes with previously good gain (>= 10)
			// 3. Nodes that weren't calculated yet (numVoxels might be uninitialized)
			node->numVoxels = this->calculateUnknownUGV(node);
			node->newNode = false;  // Mark as processed
			updated_count++;
		}
		
		double ig_time = (ros::Time::now() - ig_start).toSec();
		// ROS_INFO("[UGV-IG] Update complete: %d calculated, %d skipped (zero), %d skipped (low gain < 10)", 
		// 		updated_count, skipped_zero_gain, skipped_low_gain);
		// ROS_INFO("[UGV-IG] Time: %.3f s (avg: %.4f s/node calculated)", 
		// 		ig_time, updated_count > 0 ? ig_time / updated_count : 0.0);
		// ROS_INFO("[UGV-IG] Speedup: %.1fx (calculated %d/%zu nodes)", 
		// 		(double)prmNodeVecUGV_.size() / updated_count, 
		// 		updated_count, prmNodeVecUGV_.size());
	}



	void DEP::buildUGVRoadMap() {
		// This runs AFTER buildUGVFromUAVProjection()
		// Now we add independently sampled ground-level nodes
		
		bool saturate = false;
		bool regionSaturate = false;
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodesUGV;  // NEW nodes from sampling

		// ROS_INFO("[UGV-PRM] Starting independent sampling...");

		// --- A. Frontier Sampling (at ground level) ---
		std::vector<double> sampleWeights;
		for (int i = 0; i < int(this->frontierPointPairs_.size()); ++i) {
			double size = this->frontierPointPairs_[i].second;
			sampleWeights.push_back(pow(size, 2));
		}
		
		int countFrontierFailure = 0;
		while (ros::ok() && countFrontierFailure < this->frontierSampleThresh_ && !sampleWeights.empty()) {
			std::shared_ptr<PRM::Node> fn = this->sampleFrontierPointUGV(sampleWeights);
			
			std::vector<std::shared_ptr<PRM::Node>> fnNeighbors = this->roadmap_ugv_->kNearestNeighbor(fn, this->nnNumFrontier_);

			if (!fnNeighbors.empty()) {
				int countSampleOnce = 0;
				for (std::shared_ptr<PRM::Node> fnNN : fnNeighbors) {
					n = this->extendNodeUGV(fnNN, fn);
					
					if (this->isPosValidUGV(n->pos)) {
						std::shared_ptr<PRM::Node> nn = this->roadmap_ugv_->nearestNeighbor(n);
						double distToNN = (n->pos - nn->pos).norm();
						if (distToNN >= this->distThresh_) {
							n->newNode = true;   // Mark as new
							n->numVoxels = 0;    // Don't calculate yet
							this->roadmap_ugv_->insert(n);
							newNodesUGV.push_back(n);
							this->prmNodeVecUGV_.insert(n);
							++countSampleOnce;
						}
					}
				}
				if (countSampleOnce == 0) ++countFrontierFailure;
			} else {
				break;
			}
		}

		// --- B. Random Sampling (Global & Local at ground level) ---
		while (ros::ok() && !saturate) {
			if (regionSaturate) {
				// Global Sampling
				int countFailureGlobal = 0;
				while (ros::ok()) {
					if (countFailureGlobal > this->globalSampleThresh_) {
						saturate = true;
						break;
					}
					
					n = this->randomConfigUGV(this->globalRegionMin_, this->globalRegionMax_);
					
					if (n == nullptr) { ++countFailureGlobal; continue; }

					double distToNN = this->distThresh_;
					if (this->roadmap_ugv_->getSize() != 0) {
						std::shared_ptr<PRM::Node> nn = this->roadmap_ugv_->nearestNeighbor(n);
						distToNN = (n->pos - nn->pos).norm();
					}

					if (distToNN < this->distThresh_) {
						++countFailureGlobal;
					} else {
						n->newNode = true;   // Mark as new
						n->numVoxels = 0;    // Don't calculate yet
						this->roadmap_ugv_->insert(n);
						newNodesUGV.push_back(n);
						this->prmNodeVecUGV_.insert(n);
					}
				}
			} else {
				// Local Sampling (near UGV position)
				int countFailureLocal = 0;
				Eigen::Vector3d ugvPosition(ugv_pose_.position.x, ugv_pose_.position.y, z_proj_ugv_);
				
				while (ros::ok()) {
					if (countFailureLocal > this->localSampleThresh_) {
						regionSaturate = true;
						break;
					}
					
					Eigen::Vector3d localSampleMin = ugvPosition + this->localRegionMin_;
					Eigen::Vector3d localSampleMax = ugvPosition + this->localRegionMax_;
					
					n = this->randomConfigUGV(localSampleMin, localSampleMax);

					if (n == nullptr) { ++countFailureLocal; continue; }

					double distToNN = this->distThresh_;
					if (this->roadmap_ugv_->getSize() != 0) {
						std::shared_ptr<PRM::Node> nn = this->roadmap_ugv_->nearestNeighbor(n);
						distToNN = (n->pos - nn->pos).norm();
					}

					if (distToNN < this->distThresh_) {
						++countFailureLocal;
					} else {
						n->newNode = true;   // Mark as new
						n->numVoxels = 0;    // Don't calculate yet
						this->roadmap_ugv_->insert(n);
						newNodesUGV.push_back(n);
						this->prmNodeVecUGV_.insert(n);
					}
				}
			}
		}

		ROS_INFO("[UGV-PRM] Sampling complete. Added %zu new nodes", newNodesUGV.size());

		// --- C. Connection Phase - Connect ALL NEW nodes (projected + sampled) ---
		std::vector<std::shared_ptr<PRM::Node>> allNewNodes;
		
		// Collect all new nodes (from projection + sampling)
		for (const auto& node : prmNodeVecUGV_) {
			if (node->newNode) {
				allNewNodes.push_back(node);
			}
		}
		
		// ROS_INFO("[UGV-PRM] Connecting %zu new nodes...", allNewNodes.size());
		
		for (std::shared_ptr<PRM::Node>& n : allNewNodes) {
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_ugv_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn) {
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				if (distance2knn < this->maxConnectDist_) {
					bool hasCollision = not this->map_->isInflatedFreeLine(n->pos, nearestNeighborNode->pos);
					if (hasCollision == false) {
						n->adjNodes.insert(nearestNeighborNode);
						nearestNeighborNode->adjNodes.insert(n);
					}
				}
			}
			// Note: Don't set newNode = false yet - that happens in updateInformationGainUGV()
		}

		ROS_INFO("[UGV-PRM] Final: nodes=%zu, edges≈%zu",
				prmNodeVecUGV_.size(),
				[&](){
					size_t e=0; 
					for (auto& n : prmNodeVecUGV_) e += n->adjNodes.size();
					return e/2;
				}());
	}

	std::shared_ptr<PRM::Node> DEP::randomConfigUGV(const Eigen::Vector3d& minRegion, 
                                                 const Eigen::Vector3d& maxRegion) {
		Eigen::Vector3d effMin, effMax;
		
		// Clamp to map and global bounds (XY only, Z is fixed)
		Eigen::Vector3d mapMinRegion, mapMaxRegion;
		this->map_->getCurrMapRange(mapMinRegion, mapMaxRegion);
		
		effMin.x() = std::max({mapMinRegion.x(), minRegion.x(), globalRegionMin_.x()});
		effMin.y() = std::max({mapMinRegion.y(), minRegion.y(), globalRegionMin_.y()});
		effMax.x() = std::min({mapMaxRegion.x(), maxRegion.x(), globalRegionMax_.x()});
		effMax.y() = std::min({mapMaxRegion.y(), maxRegion.y(), globalRegionMax_.y()});
		
		if (effMax.x() <= effMin.x() || effMax.y() <= effMin.y()) return nullptr;

		for (int k = 0; k < 40 && ros::ok(); ++k) {
			Eigen::Vector3d p;
			p.x() = globalPlanner::randomNumber(effMin.x(), effMax.x());
			p.y() = globalPlanner::randomNumber(effMin.y(), effMax.y());
			p.z() = z_proj_ugv_;  // FIXED ground height
			
			if (this->isPosValidUGV(p)) {
				auto node = std::make_shared<PRM::Node>(p);
				node->newNode = true;   // Will be set to false after IG update
            	node->numVoxels = 0;    // Don't calculate yet
				// node->numVoxels = this->calculateUnknownUGV(node);
				return node;
			}
		}
		return nullptr;
	}

	bool DEP::isPosValidUGV(const Eigen::Vector3d& p) {
		double safeDistXY = this->safeDistXY_;  // 0.8m from your config
		
		// Check XY safety box around UGV position
		for (double x = p(0) - safeDistXY; x <= p(0) + safeDistXY; x += this->map_->getRes()) {
			for (double y = p(1) - safeDistXY; y <= p(1) + safeDistXY; y += this->map_->getRes()) {
				
				// Check ground level
				Eigen::Vector3d checkPoint(x, y, p(2));
				if (!this->map_->isInflatedFree(checkPoint)) {
					return false;
				}
				
				// Check sensor head height
				Eigen::Vector3d sensorPos(x, y, p(2) + this->ugvSensorHeight_);
				if (!this->map_->isInflatedFree(sensorPos)) {
					return false;
				}
			}
		}
		
		return true;
	}

	// bool DEP::isPosValidUGV(const Eigen::Vector3d& p) {
	// 	double safeDistXY = this->safeDistXY_;  // 0.8m from your config
		

	// 	for (double x = p(0) - safeDistXY; x <= p(0) + safeDistXY; x += this->map_->getRes()) {
	// 		for (double y = p(1) - safeDistXY; y <= p(1) + safeDistXY; y += this->map_->getRes()) {
				
	// 			Eigen::Vector3d checkPoint(x, y, p(2));  // Check at ground level
				
	// 			// Same strict logic as UAV
	// 			if (this->safeDistCheckUnknown_) {
	// 				// Reject if NOT confirmed free
	// 				if (!this->map_->isInflatedFree(checkPoint)) {
	// 					return false;
	// 				}
	// 			} else {
	// 				// Reject if occupied
	// 				if (this->map_->isInflatedOccupied(checkPoint)) {
	// 					return false;
	// 				}
	// 			}
				
	// 			// Also check sensor head at this XY position
	// 			Eigen::Vector3d sensorPos(x, y, p(2) + this->ugvSensorHeight_);
				
	// 			if (this->safeDistCheckUnknown_) {
	// 				if (!this->map_->isInflatedFree(sensorPos)) {
	// 					return false;
	// 				}
	// 			} else {
	// 				if (this->map_->isInflatedOccupied(sensorPos)) {
	// 					return false;
	// 				}
	// 			}
	// 		}
	// 	}
		
	// 	return true;
	// }
	// bool DEP::isPosValidUGV(const Eigen::Vector3d& p) {
	// 	// Ground level collision check
	// 	if (!this->map_->isInflatedFree(p)) {
	// 		// ROS_INFO("[UGV-VAL] REJECTED: Ground not free at (%.2f, %.2f, %.2f)", 
	// 		// 		p.x(), p.y(), p.z());
	// 		return false;
	// 	}
		
	// 	// Sensor head clearance check
	// 	Eigen::Vector3d sensorPos = p;
	// 	sensorPos.z() += this->ugvSensorHeight_;
	// 	if (this->map_->isInflatedOccupied(sensorPos)) {
	// 		// ROS_INFO("[UGV-VAL] REJECTED: Sensor collision at (%.2f, %.2f, %.2f)", 
	// 		// 		sensorPos.x(), sensorPos.y(), sensorPos.z());
	// 		return false;
	// 	}
		
	// 	// ============================================================
	// 	// DEBUG THE FOOTPRINT CHECK
	// 	// ============================================================
	// 	int total_checked = 0;
	// 	int blocked_count = 0;
	// 	double actual_safe_dist = 1.4* (this->safeDistXY_);
	// 	double resolution = this->map_->getRes();
		
	// 	// ROS_INFO("[UGV-VAL] Checking footprint: center=(%.2f,%.2f,%.2f), radius=%.2f, res=%.2f",
	// 	// 		p.x(), p.y(), p.z(), actual_safe_dist, resolution);
		
	// 	for (double dx = -actual_safe_dist; dx <= actual_safe_dist; dx += resolution) {
	// 		for (double dy = -actual_safe_dist; dy <= actual_safe_dist; dy += resolution) {
	// 			total_checked++;
				
	// 			Eigen::Vector3d checkPos(p.x() + dx, p.y() + dy, p.z());
				
	// 			bool is_blocked = this->map_->isInflatedOccupied(checkPos);
				
	// 			if (is_blocked) {
	// 				blocked_count++;
					
	// 				// // Log first few blocked positions
	// 				// if (blocked_count <= 3) {
	// 				// 	ROS_WARN("[UGV-VAL] BLOCKED at offset (%.2f, %.2f) -> pos (%.2f, %.2f, %.2f)",
	// 				// 			dx, dy, checkPos.x(), checkPos.y(), checkPos.z());
	// 				// }
	// 			}
				
	// 			// Also check sensor height
	// 			Eigen::Vector3d checkSensor = checkPos;
	// 			checkSensor.z() += this->ugvSensorHeight_;
	// 			if (this->map_->isInflatedOccupied(checkSensor)) {
	// 				blocked_count++;
	// 			}
	// 		}
	// 	}
		
	// 	// ROS_INFO("[UGV-VAL] Footprint check: %d positions checked, %d blocked", 
	// 	// 		total_checked, blocked_count);
		
	// 	if (blocked_count > 0) {
	// 		// ROS_WARN("[UGV-VAL] REJECTED: %d/%d footprint voxels blocked", 
	// 		// 		blocked_count, total_checked);
	// 		return false;
	// 	}
		
	// 	// ROS_INFO("[UGV-VAL] ACCEPTED: pos (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
	// 	return true;
	// 	// ============================================================
	// }
	std::shared_ptr<PRM::Node> DEP::sampleFrontierPointUGV(const std::vector<double>& sampleWeights) {
		int idx = weightedSample(sampleWeights);
		
		Eigen::Vector3d frontierCenter = this->frontierPointPairs_[idx].first;
		double frontierSize = this->frontierPointPairs_[idx].second;
		
		double xmin = std::max(frontierCenter.x() - frontierSize/sqrt(2), globalRegionMin_.x());
		double xmax = std::min(frontierCenter.x() + frontierSize/sqrt(2), globalRegionMax_.x());
		double ymin = std::max(frontierCenter.y() - frontierSize/sqrt(2), globalRegionMin_.y());
		double ymax = std::min(frontierCenter.y() + frontierSize/sqrt(2), globalRegionMax_.y());
		
		Eigen::Vector3d frontierPoint;
		frontierPoint.x() = globalPlanner::randomNumber(xmin, xmax);
		frontierPoint.y() = globalPlanner::randomNumber(ymin, ymax);
		frontierPoint.z() = z_proj_ugv_;  // FIXED ground height
		
		return std::make_shared<PRM::Node>(frontierPoint);
	}

	std::shared_ptr<PRM::Node> DEP::extendNodeUGV(const std::shared_ptr<PRM::Node>& n, 
                                               const std::shared_ptr<PRM::Node>& target) {
		double extendDist = randomNumber(this->distThresh_, this->maxConnectDist_);
		Eigen::Vector3d dir = target->pos - n->pos;
		dir.z() = 0;  // Keep extension horizontal
		dir.normalize();
		
		Eigen::Vector3d p = n->pos + dir * extendDist;
		
		// Replace std::clamp with std::max/std::min
		p.x() = std::max(globalRegionMin_.x(), std::min(p.x(), globalRegionMax_.x()));
		p.y() = std::max(globalRegionMin_.y(), std::min(p.y(), globalRegionMax_.y()));
		p.z() = z_proj_ugv_;  // FIXED ground height
		
		return std::make_shared<PRM::Node>(p);
	}

	// Helper: Finds the sequence of node indices from start to end using Dijkstra
	std::vector<int> DEP::findPathDijkstra(int start_idx, int end_idx, 
										const std::vector<std::shared_ptr<PRM::Node>>& nodes,
										const std::unordered_map<const PRM::Node*, int>& id_map) {
		
		// 1. Initialization
		int N = nodes.size();
		std::vector<double> dist(N, std::numeric_limits<double>::infinity());
		std::vector<int> parent(N, -1);
		
		// Priority Queue: <Distance, NodeIndex> (Min-Heap)
		typedef std::pair<double, int> PDI;
		std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq;

		// 2. Start Node Setup
		dist[start_idx] = 0.0;
		pq.push({0.0, start_idx});

		// 3. Main Loop
		while (!pq.empty()) {
			double d = pq.top().first;
			int u = pq.top().second;
			pq.pop();

			// Early Exit: If we reached the target, stop.
			if (u == end_idx) {
				std::vector<int> path;
				int curr = end_idx;
				while (curr != -1) {
					path.push_back(curr);
					if (curr == start_idx) break; // Stop at start
					curr = parent[curr];
				}
				std::reverse(path.begin(), path.end());
				return path;
			}

			// Optimization: If we found a shorter path to u already, skip
			if (d > dist[u]) continue;

			// Explore Neighbors
			for (const auto& neighbor_ptr : nodes[u]->adjNodes) {
				// Check if neighbor exists in our index map
				auto it = id_map.find(neighbor_ptr.get());
				if (it == id_map.end()) continue;
				
				int v = it->second;
				
				// Calculate distance (Euclidean weight)
				double weight = (nodes[u]->pos - nodes[v]->pos).norm();
				
				// Relaxation Step
				if (dist[u] + weight < dist[v]) {
					dist[v] = dist[u] + weight;
					parent[v] = u;
					pq.push({dist[v], v});
				}
			}
		}

		return {}; // Return empty if no path found (disconnected)
	}

	
	void DEP::getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates){
		goalCandidates.clear();
		bool firstNode = true;
		std::priority_queue<std::shared_ptr<PRM::Node>, std::vector<std::shared_ptr<PRM::Node>>, PRM::GainCompareNode> gainPQ;

		// iterate through all points in the roadmap
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			gainPQ.push(n);
		}

		// select candidates from the priority queue
		int maxNumVoxel = 0;
		while (ros::ok()){
			if (gainPQ.size() == 0){
				break;
			}


			std::shared_ptr<PRM::Node> n = gainPQ.top();
			
			if (firstNode){
				// if ((n->pos - this->position_).norm() >= 1.0){
				if ((n->pos - this->position_).norm() >= 0.0){
					maxNumVoxel = n->numVoxels;
					firstNode = false;
				}
			}

			if (double(n->numVoxels) < double(maxNumVoxel) * this->minVoxelThresh_){
				break;
			}
			// if ((n->pos - this->position_).norm() >= 1.0){
			if ((n->pos - this->position_).norm() >= 0.0){			
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}
			}
			gainPQ.pop();
			
			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}
		}

		// cout << "current pos: " << this->position_.transpose() << endl;
		while (int(goalCandidates.size()) < this->minCandidateNum_){
			if (gainPQ.size() == 0){
				break;
			}

			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}

			std::shared_ptr<PRM::Node> n = gainPQ.top();
			gainPQ.pop();
			// if ((n->pos - this->position_).norm() >= 1.0){ 
			if ((n->pos - this->position_).norm() >= 0.0){	
				// cout << "candidate goal: " << n->pos.transpose() << endl;	
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}			
			}
		}
	}

	bool DEP::findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates, std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths){
		bool findPath = false;
		// find nearest node of current location
		std::shared_ptr<PRM::Node> currPos;
		currPos.reset(new PRM::Node (this->position_));
		std::shared_ptr<PRM::Node> start = this->roadmap_->nearestNeighbor(currPos);

		candidatePaths.clear();
		for (std::shared_ptr<PRM::Node> goal : goalCandidates){
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal, this->map_);
			if (int(path.size()) != 0){
				findPath = true;
			}
			else{
				continue;
			}
			path.insert(path.begin(), currPos);
			std::vector<std::shared_ptr<PRM::Node>> pathSc;
			this->shortcutPath(path, pathSc);
			candidatePaths.push_back(pathSc);
		}
		return findPath;
	}

	static inline double euclid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
		return (a - b).norm();
	}

	inline bool DEP::edgeExists(int u, int v, 
                            const std::vector<std::vector<double>>& T) const 
	{
		return std::isfinite(T[u][v]);
	}

	bool DEP::validatePathUsesPRMEdges(const std::vector<int>& path,
									const std::vector<std::vector<double>>& T) const 
	{
		for (size_t i = 0; i + 1 < path.size(); ++i) {
			if (!std::isfinite(T[path[i]][path[i+1]])) {
				ROS_ERROR("[DEP] Path uses non-existent edge: %d -> %d", 
						path[i], path[i+1]);
				return false;
			}
		}
		return true;
	}

	void DEP::buildScoresAndTimes(const std::vector<std::shared_ptr<PRM::Node>>& order,
                              std::vector<double>& scores,
                              std::vector<std::vector<double>>& times,
                              double cruise_speed_mps) const
	{
		const int N = (int)order.size();

		// 1. Assign Scores
		scores.assign(N, 0.0);
		for (int i = 0; i < N; ++i) 
			scores[i] = (double)order[i]->numVoxels;

		// 2. Build Index Map
		std::unordered_map<const PRM::Node*, int> idx_of;
		idx_of.reserve(N);
		for (int i = 0; i < N; ++i) 
			idx_of[order[i].get()] = i;

		// 3. MODIFIED: Use INFINITY instead of 9999
		const double NO_EDGE = std::numeric_limits<double>::infinity();
		times.assign(N, std::vector<double>(N, NO_EDGE));

		// 4. Fill in ONLY Direct Edges
		for (int i = 0; i < N; ++i) {
			times[i][i] = 0.0;

			for (const auto& neighbor_ptr : order[i]->adjNodes) {
				auto it = idx_of.find(neighbor_ptr.get());
				if (it != idx_of.end()) {
					int j = it->second;
					double dist = (order[i]->pos - order[j]->pos).norm();
					double t_edge = dist / std::max(0.1, cruise_speed_mps);
					times[i][j] = t_edge;
					times[j][i] = t_edge;  // Ensure symmetry
				}
			}
		}
	}
	void DEP::dijkstraTimes(int src,
							const std::vector<std::shared_ptr<PRM::Node>>& order,
							const std::unordered_map<const PRM::Node*, int>&,
							const std::vector<std::vector<int>>& adj_idx,
							std::vector<double>& dist,
							double cruise_speed_mps) const
	{
	struct QN { int v; double d; bool operator<(const QN& o) const { return d > o.d; } };
	std::priority_queue<QN> pq;
	dist[src] = 0.0; pq.push({src, 0.0});
	const double sp = std::max(0.1, cruise_speed_mps);
	while (!pq.empty()) {
		auto cur = pq.top(); pq.pop();
		if (cur.d > dist[cur.v]) continue;
		for (int v : adj_idx[cur.v]) {
		double w = euclid(order[cur.v]->pos, order[v]->pos) / sp;
		double nd = cur.d + w;
		if (nd < dist[v]) { dist[v] = nd; pq.push({v, nd}); }
		}
	}
	}

	double DEP::pathTime(const std::vector<int>& P,
						const std::vector<std::vector<double>>& T) const {
	if (P.size() < 2) return 0.0;
	double s = 0.0; for (size_t i=0;i+1<P.size();++i) s += T[P[i]][P[i+1]];
	return s;
	}

	double DEP::pathScore(const std::vector<int>& P,
						const std::vector<double>& S) const {
	double s = 0.0; for (int v : P) s += S[v]; return s;
	}

	void DEP::twoOpt(std::vector<int>& path,
					const std::vector<std::vector<double>>& T) const 
	{
		if (path.size() < 4) return;
		
		bool improved = true;
		while (improved) {
			improved = false;
			for (size_t i = 1; i + 2 < path.size(); ++i) {
				for (size_t j = i + 1; j + 1 < path.size(); ++j) {
					// MODIFIED: Check if new edges would exist after reversal
					// After reversing [i,j], new edges are: (i-1 -> j) and (i -> j+1)
					if (!edgeExists(path[i-1], path[j], T) || 
						!edgeExists(path[i], path[j+1], T)) {
						continue;  // Skip - would create non-existent edge
					}
					
					double a = T[path[i-1]][path[i]] + T[path[j]][path[j+1]];
					double b = T[path[i-1]][path[j]] + T[path[i]][path[j+1]];
					
					if (b + 1e-12 < a) { 
						std::reverse(path.begin()+i, path.begin()+j+1); 
						improved = true; 
					}
				}
			}
		}
	}

	void DEP::localSearchSwapInsert(std::vector<int>& path,
                                const std::vector<double>& S,
                                const std::vector<std::vector<double>>& T,
                                double T_MAX) const
	{
		if (path.size() < 2) return;
		
		auto time_of  = [&](const std::vector<int>& p){ return pathTime(p, T); };
		auto score_of = [&](const std::vector<int>& p){ return pathScore(p, S); };

		bool improved = true;
		while (improved) {
			improved = false;
			std::unordered_set<int> in(path.begin(), path.end());

			// Exchange: swap a visited node with an unvisited
			for (size_t v_idx = 1; v_idx + 1 < path.size(); ++v_idx) {
				int v = path[v_idx];
				int u = path[v_idx-1];
				int z = path[v_idx+1];
				
				for (int w = 0; w < (int)S.size(); ++w) {
					// if (in.count(w) || S[w] <= 0.0) continue;
					if (in.count(w)) continue;
					
					// MODIFIED: Check if new edges (u,w) and (w,z) exist
					if (!edgeExists(u, w, T) || !edgeExists(w, z, T)) {
						continue;
					}
					
					double dt = (T[u][w] + T[w][z]) - (T[u][v] + T[v][z]);
					double curT = time_of(path);
					
					if (curT + dt <= T_MAX + 1e-9) {
						double gain = S[w] - S[v];
						if (gain > 0.0 || (gain == 0.0 && dt < 0.0)) {
							path[v_idx] = w; 
							in.erase(v); 
							in.insert(w);
							improved = true;

							// Greedy insert more nodes while feasible
							bool inserted = true;
							while (inserted) {
								inserted = false;
								int bestK = -1, bestPos = -1; 
								double bestEff = -1.0;
								
								for (int k = 0; k < (int)S.size(); ++k) {
									if (in.count(k) || S[k] <= 0.0) continue;
									
									for (size_t j = 0; j + 1 < path.size(); ++j) {
										int a = path[j], b = path[j+1];
										
										// MODIFIED: Check edges exist
										if (!edgeExists(a, k, T) || !edgeExists(k, b, T)) {
											continue;
										}
										
										double inc = T[a][k] + T[k][b] - T[a][b];
										double cur = time_of(path);
										
										if (cur + inc <= T_MAX + 1e-9) {
											double eff = S[k] / std::max(1e-9, inc);
											if (eff > bestEff) { 
												bestEff = eff; 
												bestK = k; 
												bestPos = (int)j + 1; 
											}
										}
									}
								}
								
								if (bestK != -1) { 
									path.insert(path.begin()+bestPos, bestK); 
									in.insert(bestK); 
									inserted = true; 
								}
							}
						}
					}
				}
			}
			
			double before = time_of(path);
			twoOpt(path, T);
			if (time_of(path) < before - 1e-9) improved = true;
		}
	}

	// HELPER: Find shortest TIME path using Dijkstra (respects infinity edges)
	// This is used ONLY to initialize the path from start to end
	// ============================================================================
	std::vector<int> DEP::findInitialPath(int start, int end,
										const std::vector<std::vector<double>>& T) const
	{
		const int N = (int)T.size();
		
		// If direct edge exists, use it (like the paper assumes)
		if (std::isfinite(T[start][end])) {
			return {start, end};
		}
		
		// Otherwise, find shortest path through the graph
		std::vector<double> dist(N, std::numeric_limits<double>::infinity());
		std::vector<int> parent(N, -1);
		
		using PQItem = std::pair<double, int>;
		std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
		
		dist[start] = 0.0;
		pq.push({0.0, start});
		
		while (!pq.empty()) {
			auto [d, u] = pq.top(); pq.pop();
			if (d > dist[u]) continue;
			if (u == end) break;
			
			for (int v = 0; v < N; ++v) {
				if (!std::isfinite(T[u][v])) continue;  // Skip non-edges
				double nd = dist[u] + T[u][v];
				if (nd < dist[v]) {
					dist[v] = nd;
					parent[v] = u;
					pq.push({nd, v});
				}
			}
		}
		
		// Reconstruct path
		if (!std::isfinite(dist[end])) {
			return {};  // No path exists
		}
		
		std::vector<int> path;
		for (int v = end; v != -1; v = parent[v]) {
			path.push_back(v);
		}
		std::reverse(path.begin(), path.end());
		return path;
	}

	std::vector<int> DEP::constructiveC4(const std::vector<double>& S,
                                     const std::vector<std::vector<double>>& T,
                                     int start_idx, int end_idx, 
                                     double ALPHA, double T_MAX) const
	{
		const int N = (int)S.size();
		std::mt19937 rng{std::random_device{}()};
		
		// Initialize path from start to end (paper assumes direct edge exists)
		std::vector<int> path = findInitialPath(start_idx, end_idx, T);
		
		if (path.empty()) {
			ROS_WARN("[DEP] No path exists from start %d to end %d!", start_idx, end_idx);
			return {start_idx};
		}
		
		// Calculate initial path time
		double curT = pathTime(path, T);
		
		// Check if initial path exceeds time budget
		if (curT > T_MAX) {
			ROS_WARN("[DEP] Initial path time (%.2f) exceeds T_MAX (%.2f)", curT, T_MAX);
			return {start_idx};
		}
		
		// Now follow the paper's C4 construction:
		// Insert nodes that maximize score/time_increment ratio
		while (true) {
			struct Cand { int v; double s; double inc; };
			std::vector<Cand> cand;
			std::unordered_set<int> in_path(path.begin(), path.end());

			// Find candidates that can be inserted
			for (int i = 0; i < N; ++i) {
				if (in_path.count(i)) continue;
				// if (S[i] <= 0.0) continue;
				
				// Find best insertion position for node i
				double best_inc = std::numeric_limits<double>::infinity();
				
				for (size_t j = 0; j + 1 < path.size(); ++j) {
					int u = path[j], v = path[j+1];
					
					// Check if insertion is possible (edges must exist)
					if (!std::isfinite(T[u][i]) || !std::isfinite(T[i][v])) continue;
					
					double inc = T[u][i] + T[i][v] - T[u][v];
					if (inc < best_inc) best_inc = inc;
				}
				
				// Only add as candidate if insertion is feasible within time budget
				if (std::isfinite(best_inc) && curT + best_inc <= T_MAX + 1e-9) {
					cand.push_back({i, S[i], best_inc});
				}
			}
			
			if (cand.empty()) break;

			// Sort by efficiency (score / time_increment) - highest first
			std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b){
				double ea = (a.inc > 1e-9) ? (a.s / a.inc) : (a.s * 1e12);
				double eb = (b.inc > 1e-9) ? (b.s / b.inc) : (b.s * 1e12);
				return ea > eb;
			});

			// GRASP: Build RCL and select randomly
			int rcl_size = std::max(1, (int)std::floor(ALPHA * cand.size()));
			std::uniform_int_distribution<int> pick(0, rcl_size - 1);
			Cand choice = cand[pick(rng)];

			// Find best position to insert the chosen node
			int bestPos = -1; 
			double bestInc = std::numeric_limits<double>::infinity();
			
			for (size_t j = 0; j + 1 < path.size(); ++j) {
				int u = path[j], v = path[j+1];
				
				if (!std::isfinite(T[u][choice.v]) || !std::isfinite(T[choice.v][v])) continue;
				
				double inc = T[u][choice.v] + T[choice.v][v] - T[u][v];
				if (inc < bestInc) { 
					bestInc = inc; 
					bestPos = (int)j + 1; 
				}
			}
			
			if (bestPos != -1 && curT + bestInc <= T_MAX + 1e-9) {
				path.insert(path.begin() + bestPos, choice.v);
				curT += bestInc;
			} else {
				break;
			}
		}
		
		return path;
	}

	std::vector<int> DEP::pathRelinking(const std::vector<int>& P, 
                                    const std::vector<int>& Q,
                                    const std::vector<double>& S,
                                    const std::vector<std::vector<double>>& T,
                                    double T_MAX) const
	{
		std::vector<int> init = P;
		std::unordered_set<int> Pset(P.begin(), P.end()), Qset(Q.begin(), Q.end());
		
		std::vector<int> to_add; 
		for (int v : Q) 
			if (!Pset.count(v)) to_add.push_back(v);
		std::sort(to_add.begin(), to_add.end(), [&](int a, int b){ 
			return S[a] > S[b]; 
		});

		auto time_of  = [&](const std::vector<int>& p){ return pathTime(p, T); };
		auto score_of = [&](const std::vector<int>& p){ return pathScore(p, S); };

		std::vector<int> best = init;
		double bestSc = score_of(best);

		std::unordered_set<int> remove_cands; 
		for (int v : P) 
			if (!Qset.count(v)) remove_cands.insert(v);

		for (int add : to_add) {
			if (S[add] <= 0.0) continue;
			
			// MODIFIED: Find best VALID insertion position
			int bestPos = -1; 
			double bestInc = std::numeric_limits<double>::infinity();
			
			for (size_t j = 0; j + 1 < init.size(); ++j) {
				int u = init[j], w = init[j+1];
				
				// Check edges exist
				if (!edgeExists(u, add, T) || !edgeExists(add, w, T)) {
					continue;
				}
				
				double inc = T[u][add] + T[add][w] - T[u][w];
				if (inc < bestInc) { 
					bestInc = inc; 
					bestPos = (int)j + 1; 
				}
			}
			
			// Only insert if valid position found
			if (bestPos != -1) {
				init.insert(init.begin()+bestPos, add);
			} else {
				continue;  // Can't add this node
			}

			// Remove nodes to make feasible
			while (time_of(init) > T_MAX + 1e-9) {
				int remIdx = -1; 
				double minSc = std::numeric_limits<double>::infinity();
				
				for (size_t i = 1; i + 1 < init.size(); ++i) {
					int v = init[i];
					if (remove_cands.count(v) && S[v] < minSc) {
						// MODIFIED: Check that removal keeps path valid
						int prev = init[i-1], next = init[i+1];
						if (edgeExists(prev, next, T)) {
							minSc = S[v]; 
							remIdx = (int)i;
						}
					}
				}
				
				if (remIdx != -1) {
					init.erase(init.begin()+remIdx);
				} else {
					break;
				}
			}
			
			double t = time_of(init), sc = score_of(init);
			if (t <= T_MAX + 1e-9 && sc > bestSc) { 
				bestSc = sc; 
				best = init; 
			}
		}
		return best;
	}


	std::vector<int> DEP::selectOrienteeringPathGRASPPR(
    const std::vector<std::shared_ptr<PRM::Node>>& order,
    const std::unordered_map<const PRM::Node*, int>& id_of, // Added variable name 'id_of' to match usage
    int start_idx, int end_idx, const OPParams& P, 
    double cruise_speed, double& path_time_out)
	{
		std::vector<double> S;
		std::vector<std::vector<double>> T;
		
		// USE THE PASSED PARAMETER HERE
		// Ensure we don't divide by zero by clamping min speed to 0.1
		buildScoresAndTimes(order, S, T, std::max(0.1, cruise_speed));

		std::vector<std::pair<std::vector<int>, double>> elite;
		elite.reserve(P.ELITE_SET_SIZE);
		
		auto consider_elite = [&](const std::vector<int>& path){
			double sc = pathScore(path, S);
			elite.push_back({path, sc});
			std::sort(elite.begin(), elite.end(), 
					[](auto& a, auto& b){ return a.second > b.second; });
			if ((int)elite.size() > P.ELITE_SET_SIZE) 
				elite.resize(P.ELITE_SET_SIZE);
		};

		for (int it = 0; it < P.GRASP_ITER; ++it) {
			auto path = constructiveC4(S, T, start_idx, end_idx, P.ALPHA, P.T_MAX);
			twoOpt(path, T);
			localSearchSwapInsert(path, S, T, P.T_MAX);
			twoOpt(path, T);
			consider_elite(path);
		}
		
		if (elite.empty()) return {start_idx};

		std::vector<int> best = elite.front().first;
		double bestSc = elite.front().second;
		
		for (size_t i = 0; i < elite.size(); ++i) {
			for (size_t j = i+1; j < elite.size(); ++j) {
				for (int flip = 0; flip < 2; ++flip) {
					const auto& A = (flip == 0 ? elite[i].first : elite[j].first);
					const auto& B = (flip == 0 ? elite[j].first : elite[i].first);
					auto rel = pathRelinking(A, B, S, T, P.T_MAX);
					twoOpt(rel, T);
					double sc = pathScore(rel, S);
					if (sc > bestSc) { 
						best = std::move(rel); 
						bestSc = sc; 
					}
				}
			}
		}
		
		// ADDED: Validate final path (optional - for debugging)
		#ifdef DEBUG
		if (!validatePathUsesPRMEdges(best, T)) {
			ROS_ERROR("[DEP] Final path contains invalid edges!");
		}
		#endif
		
		path_time_out = pathTime(best, T);
		// ADDED: Print path info
		std::stringstream ss;
		ss << "[DEP] OP Path (" << best.size() << " nodes): ";
		for (size_t i = 0; i < best.size(); ++i) {
			ss << best[i];
			if (i + 1 < best.size()) ss << " -> ";
		}
		ss << "\n[DEP] Total Score: " << bestSc 
		<< ", Total Time: " << path_time_out;
		ROS_INFO_STREAM(ss.str());
		
		// ADDED: Print detailed node info
		// ROS_INFO("[DEP] Node details:");
		// for (size_t i = 0; i < best.size(); ++i) {
		// 	int idx = best[i];
		// 	const auto& node = order[idx];
		// 	ROS_INFO("  [%zu] Node %d: pos=(%.2f, %.2f, %.2f), score=%.1f", 
		// 			i, idx, 
		// 			node->pos.x(), node->pos.y(), node->pos.z(), 
		// 			S[idx]);
		// }
		
		return best;
	}

	void DEP::findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath){
		// find path highest unknown
		bestPath.clear();
		double highestScore = -1;
		for (int n=0; n<int(candidatePaths.size()); ++n){
			std::vector<std::shared_ptr<PRM::Node>> path = candidatePaths[n]; 
			if (int(path.size()) == 0) continue;
			double yawDist = 0;
			double prevYaw = this->currYaw_;
			int unknownVoxel = 0;
			for (int i=0; i<int(path.size())-1; ++i){
				std::shared_ptr<PRM::Node> currNode = path[i];
				std::shared_ptr<PRM::Node> nextNode = path[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));

				// reevaluate the unknowns for intermediate points
				std::unordered_map<double, int> yawNumVoxels;
				int unknownVoxelNum = this->calculateUnknown(currNode, yawNumVoxels);
				currNode->numVoxels = unknownVoxelNum;
				currNode->yawNumVoxels = yawNumVoxels;


				unknownVoxel += currNode->getUnknownVoxels(angle);
				yawDist += globalPlanner::angleDiff(prevYaw, angle);
				prevYaw = angle;
			}
			// reevaluate the goal node
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(path.back(), yawNumVoxels);
			path.back()->numVoxels = unknownVoxelNum;
			path.back()->yawNumVoxels = yawNumVoxels;
			unknownVoxel += path.back()->getBestYawVoxel();
			yawDist += globalPlanner::angleDiff(prevYaw, path.back()->getBestYaw());

			double distance = this->calculatePathLength(path);
			// cout << "total is distance is: " << distance << " total yaw distance is: " << yawDist << " voxel: " << path.back()->numVoxels << endl;
			double pathTime = distance/this->vel_ + this->yawPenaltyWeight_ * yawDist/this->angularVel_;
			double score = double(unknownVoxel)/pathTime; 
			// cout << "unknown for path: " << n <<  " is: " << unknownVoxel << " score: " << score << " distance: " << distance << " Time: " << pathTime <<  " Last total unknown: " << path.back()->numVoxels << " last best: " << path.back()->getBestYawVoxel() << endl;
			if (score > highestScore){
				highestScore = score;
				bestPath = path;
			}
		}
		if (highestScore == 0){
			cout << "[DEP]: Current score is 0. The exploration might complete." << endl;
		}
	}


	void DEP::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->position_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->currYaw_ = globalPlanner::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		this->odomReceived_ = true;

		if (this->histTraj_.size() == 0){
			this->histTraj_.push_back(this->position_);
		}
		else{
			Eigen::Vector3d lastPos = this->histTraj_.back();
			double dist = (this->position_ - lastPos).norm();
			if (dist >= 0.5){
				if (this->histTraj_.size() >= 100){
					this->histTraj_.pop_front();
					this->histTraj_.push_back(this->position_);
				}
				else{
					this->histTraj_.push_back(this->position_);
				}
			}
		}
	}

	void DEP::visCB(const ros::TimerEvent&){
		std::lock_guard<std::mutex> lock(this->data_mutex_);
		if (this->prmNodeVec_.size() != 0){
			this->publishRoadmap();
		}
		if (!prmNodeVecUGV_.empty())   publishRoadmapUGV();

		if (this->candidatePaths_.size() != 0){
			this->publishCandidatePaths();
		}

		if (this->bestPath_.size() != 0){
			this->publishBestPath();
		}

		if (this->frontierPointPairs_.size() != 0){
			this->publishFrontier();
		}
	}


	bool DEP::isPosValid(const Eigen::Vector3d& p){
		for (double x=p(0)-this->safeDistXY_; x<=p(0)+this->safeDistXY_; x+=this->map_->getRes()){
			for (double y=p(1)-this->safeDistXY_; y<=p(1)+this->safeDistXY_; y+=this->map_->getRes()){
				for (double z=p(2)-this->safeDistZ_; z<=p(2)+this->safeDistZ_; z+=this->map_->getRes()){
					if (this->safeDistCheckUnknown_){
						if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
					else{
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
				}
			}
		}
		return true;			
	}


	bool DEP::isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ){
		for (double x=p(0)-safeDistXY; x<=p(0)+safeDistXY; x+=this->map_->getRes()){
			for (double y=p(1)-safeDistXY; y<=p(1)+safeDistXY; y+=this->map_->getRes()){
				for (double z=p(2)-safeDistZ; z<=p(2)+safeDistZ; z+=this->map_->getRes()){
					if (this->safeDistCheckUnknown_){
						if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
					else{
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
				}
			}
		}
		return true;		
	}

	std::shared_ptr<PRM::Node> DEP::randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion){
		Eigen::Vector3d mapMinRegion, mapMaxRegion, minSampleRegion, maxSampleRegion;
		this->map_->getCurrMapRange(mapMinRegion, mapMaxRegion);
		// cout << "current map range is: " << mapMinRegion.transpose() << ", " << mapMaxRegion.transpose() << endl;
		minSampleRegion(0) = std::max(mapMinRegion(0), minRegion(0));
		minSampleRegion(1) = std::max(mapMinRegion(1), minRegion(1));


		minSampleRegion(2) = std::max(mapMinRegion(2), minRegion(2));
		maxSampleRegion(0) = std::min(mapMaxRegion(0), maxRegion(0));
		maxSampleRegion(1) = std::min(mapMaxRegion(1), maxRegion(1));
		maxSampleRegion(2) = std::min(mapMaxRegion(2), maxRegion(2));

		minSampleRegion(0) = std::max(minSampleRegion(0), this->globalRegionMin_(0));
		minSampleRegion(1) = std::max(minSampleRegion(1), this->globalRegionMin_(1));
		minSampleRegion(2) = std::max(minSampleRegion(2), this->globalRegionMin_(2));
		maxSampleRegion(0) = std::min(maxSampleRegion(0), this->globalRegionMax_(0));
		maxSampleRegion(1) = std::min(maxSampleRegion(1), this->globalRegionMax_(1));
		maxSampleRegion(2) = std::min(maxSampleRegion(2), this->globalRegionMax_(2));

		// std::cout << "Sample z bounds: " << minSampleRegion(2) << " to " << maxSampleRegion(2) << std::endl;


		bool valid = false;
		Eigen::Vector3d p;
		while (valid == false){	
			p(0) = globalPlanner::randomNumber(minSampleRegion(0), maxSampleRegion(0));
			p(1) = globalPlanner::randomNumber(minSampleRegion(1), maxSampleRegion(1));
			p(2) = globalPlanner::randomNumber(minSampleRegion(2), maxSampleRegion(2));

			valid = this->isPosValid(p, this->safeDistXY_, this->safeDistZ_);

			// valid = this->map_->isInflatedFree(p);
		}

		std::shared_ptr<PRM::Node> newNode (new PRM::Node(p));
		return newNode;
	}

	int DEP::calculateUnknownUGV(const std::shared_ptr<PRM::Node>& n){
		Eigen::Vector3d p = n->pos;
		p(2) += this->ugvSensorHeight_;
		
		double range_limit = this->ugvMaxRange_; 
		double effective_vertical_fov = this->verticalFOV_;
		double zRange = range_limit * tan(effective_vertical_fov / 2.0);
		int countTotalUnknown = 0;
		
		// DEBUG: Count blocked vs successful rays
		int countBlocked = 0;
		int countChecked = 0;
		
		for (double z = p(2) - zRange; z <= p(2) + zRange; z += this->map_->getRes()){
			for (double y = p(1) - range_limit; y <= p(1) + range_limit; y += this->map_->getRes()){
				for (double x = p(0) - range_limit; x <= p(0) + range_limit; x += this->map_->getRes()){
					
					Eigen::Vector3d nodePoint(x, y, z);
					
					if (nodePoint(0) < globalRegionMin_(0) || nodePoint(0) > globalRegionMax_(0) ||
						nodePoint(1) < globalRegionMin_(1) || nodePoint(1) > globalRegionMax_(1) ||
						nodePoint(2) < globalRegionMin_(2) || nodePoint(2) > globalRegionMax_(2)) {
						continue;
					}
					
					if (this->map_->isUnknown(nodePoint) && !this->map_->isInflatedOccupied(nodePoint)){
						Eigen::Vector3d direction = nodePoint - p;
						double distance = direction.norm();
						if (distance > range_limit) continue; 
						if (distance < 0.6) continue;
						
						countChecked++;
						bool hasCollision = this->map_->isInflatedOccupiedLine(nodePoint, p);
						
						if (hasCollision) {
							countBlocked++;
						} else {
							countTotalUnknown++;
						}
					}
				}
			}
		}
		
		// Print debug for extreme cases
		if (countTotalUnknown == 0 && countChecked > 100) {
			// ROS_WARN("[UGV-IG] Node at (%.1f, %.1f): ALL BLOCKED! Checked=%d, Blocked=%d",
			// 		n->pos.x(), n->pos.y(), countChecked, countBlocked);
		}
		if (countTotalUnknown > 10000) {
			// ROS_INFO("[UGV-IG] Node at (%.1f, %.1f): HIGH GAIN! Visible=%d, Blocked=%d",
			// 		n->pos.x(), n->pos.y(), countTotalUnknown, countBlocked);
		}
		
		return countTotalUnknown;
	}
	int DEP::calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels){
		for (double yaw : this->yaws_){
			yawNumVoxels[yaw] = 0;
		}
		// Position:
		Eigen::Vector3d p = n->pos;

		double zRange = this->dmax_ * tan(this->verticalFOV_/2.0);
		int countTotalUnknown = 0;
		for (double z = p(2) - zRange; z <= p(2) + zRange; z += this->map_->getRes()){
			for (double y = p(1) - this->dmax_; y <= p(1)+ this->dmax_; y += this->map_->getRes()){
				for (double x = p(0) - this->dmax_; x <= p(0) + this->dmax_; x += this->map_->getRes()){
					Eigen::Vector3d nodePoint (x, y, z);
					if (nodePoint(0) < this->globalRegionMin_(0) or nodePoint(0) > this->globalRegionMax_(0) or
						nodePoint(1) < this->globalRegionMin_(1) or nodePoint(1) > this->globalRegionMax_(1) or
						nodePoint(2) < this->globalRegionMin_(2) or nodePoint(2) > this->globalRegionMax_(2)){
						// not in global range
						continue;
					}

					if (this->map_->isUnknown(nodePoint) and not this->map_->isInflatedOccupied(nodePoint)){
						if (this->sensorFOVCondition(nodePoint, p)){
							++countTotalUnknown;
							for (double yaw: this->yaws_){
								Eigen::Vector3d yawDirection (cos(yaw), sin(yaw), 0);
								Eigen::Vector3d direction = nodePoint - p;
								Eigen::Vector3d face (direction(0), direction(1), 0);
								double angleToYaw = angleBetweenVectors(face, yawDirection);
								if (angleToYaw <= this->horizontalFOV_/2){
									yawNumVoxels[yaw] += 1;
								}
							}
						}
					}
				}
			}
		}
		return countTotalUnknown;
	}
	double DEP::calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path){
		int idx1 = 0;
		double length = 0;
		for (size_t idx2=1; idx2<=path.size()-1; ++idx2){
			length += (path[idx2]->pos - path[idx1]->pos).norm();
			++idx1;
		}
		return length;
	}

	void DEP::shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc){
		size_t ptr1 = 0; size_t ptr2 = 2;
		pathSc.push_back(path[ptr1]);

		if (path.size() == 1){
			return;
		}

		if (path.size() == 2){
			pathSc.push_back(path[1]);
			return;
		}

		while (ros::ok()){
			if (ptr2 > path.size()-1){
				break;
			}
			std::shared_ptr<PRM::Node> p1 = path[ptr1];
			std::shared_ptr<PRM::Node> p2 = path[ptr2];
			Eigen::Vector3d pos1 = p1->pos;
			Eigen::Vector3d pos2 = p2->pos;
			bool lineValidCheck;
			// lineValidCheck = not this->map_->isInflatedOccupiedLine(pos1, pos2);
			lineValidCheck = this->map_->isInflatedFreeLine(pos1, pos2);
			// double maxDistance = std::numeric_limits<double>::max();
			// double maxDistance = 3.0;
			// if (lineValidCheck and (pos1 - pos2).norm() <= maxDistance){
			if (lineValidCheck){
				if (ptr2 == path.size()-1){
					pathSc.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				pathSc.push_back(path[ptr2-1]);
				if (ptr2 == path.size()-1){
					pathSc.push_back(p2);
					break;
				}
				ptr1 = ptr2-1;
				ptr2 = ptr1+2;
			}
		}		
	}

	int DEP::weightedSample(const std::vector<double>& weights){
		double total = std::accumulate(weights.begin(), weights.end(), 0.0);
		std::vector<double> normalizedWeights;

		 for (const double weight : weights){
		 	normalizedWeights.push_back(weight/total);
		 }

		std::random_device rd;
		std::mt19937 gen(rd());
		std::discrete_distribution<int> distribution(normalizedWeights.begin(), normalizedWeights.end());
		return distribution(gen);
	}


	std::shared_ptr<PRM::Node> DEP::sampleFrontierPoint(const std::vector<double>& sampleWeights){
		// choose the frontier region (random sample by frontier area) 
		int idx = weightedSample(sampleWeights);

		// sample a frontier point in the region
		Eigen::Vector3d frontierCenter = this->frontierPointPairs_[idx].first;
		double frontierSize = this->frontierPointPairs_[idx].second;
		double xmin = std::max(frontierCenter(0) - frontierSize/sqrt(2), this->globalRegionMin_(0));
		double xmax = std::min(frontierCenter(0) + frontierSize/sqrt(2), this->globalRegionMax_(0));
		double ymin = std::max(frontierCenter(1) - frontierSize/sqrt(2), this->globalRegionMin_(1));
		double ymax = std::min(frontierCenter(1) + frontierSize/sqrt(2), this->globalRegionMax_(1));
		double zmin = frontierCenter(2);
		double zmax = frontierCenter(2);
		Eigen::Vector3d frontierPoint;
		frontierPoint(0) = globalPlanner::randomNumber(xmin, xmax);
		frontierPoint(1) = globalPlanner::randomNumber(ymin, ymax);
		frontierPoint(2) = globalPlanner::randomNumber(zmin, zmax);
		std::shared_ptr<PRM::Node> frontierNode (new PRM::Node(frontierPoint));
		return frontierNode;
	}

	std::shared_ptr<PRM::Node> DEP::extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target){
		double extendDist = randomNumber(this->distThresh_, this->maxConnectDist_);
		Eigen::Vector3d p = n->pos + (target->pos - n->pos)/(target->pos - n->pos).norm() * extendDist;
		p(0) = std::max(this->globalRegionMin_(0), std::min(p(0), this->globalRegionMax_(0)));
		p(1) = std::max(this->globalRegionMin_(1), std::min(p(1), this->globalRegionMax_(1)));
		p(2) = std::max(this->globalRegionMin_(2), std::min(p(2), this->globalRegionMax_(2)));
		std::shared_ptr<PRM::Node> extendedNode (new PRM::Node(p));
		return extendedNode;
	}

	void DEP::publishRoadmap(){
		visualization_msgs::MarkerArray roadmapMarkers;

		// PRM nodes and edges
		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			// std::shared_ptr<PRM::Node> n = this->prmNodeVec_[i];

			// Node point
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "prm_point";
			point.id = countPointNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.1;
			point.scale.y = 0.1;
			point.scale.z = 0.1;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 0.0;
			point.color.b = 0.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);

			// number of voxels for each node
			visualization_msgs::Marker voxelNumText;
			voxelNumText.ns = "num_voxel_text";
			voxelNumText.header.frame_id = "map";
			voxelNumText.id = countVoxelNumText;
			voxelNumText.header.stamp = ros::Time::now();
			voxelNumText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			voxelNumText.action = visualization_msgs::Marker::ADD;
			voxelNumText.pose.position.x = n->pos(0);
			voxelNumText.pose.position.y = n->pos(1);
			voxelNumText.pose.position.z = n->pos(2)+0.1;
			voxelNumText.scale.x = 0.1;
			voxelNumText.scale.y = 0.1;
			voxelNumText.scale.z = 0.1;
			voxelNumText.color.a = 1.0;
			voxelNumText.text = std::to_string(n->numVoxels);
			voxelNumText.lifetime = ros::Duration(0.5);
			++countVoxelNumText;
			roadmapMarkers.markers.push_back(voxelNumText);


			// Edges
			visualization_msgs::Marker line;
			line.ns = "edge";
			line.header.frame_id = "map";
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.header.stamp = ros::Time::now();
			for (std::shared_ptr<PRM::Node> adjNode : n->adjNodes){
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = adjNode->pos(0);
				p2.y = adjNode->pos(1);
				p2.z = adjNode->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countEdgeNum;
				line.scale.x = 0.05;
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.r = 0.0;
				line.color.g = 1.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.5);
				++countEdgeNum;
				roadmapMarkers.markers.push_back(line);
			}
		}

		int countGoalCandidateNum = 0;
		for (size_t i=0; i<this->goalCandidates_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->goalCandidates_[i];

			// Goal candidates
			visualization_msgs::Marker goalCandidatePoint;
			goalCandidatePoint.ns = "goal_candidate";
			goalCandidatePoint.header.frame_id = "map";
			goalCandidatePoint.header.stamp = ros::Time::now();
			goalCandidatePoint.id = countGoalCandidateNum;
			goalCandidatePoint.type = visualization_msgs::Marker::SPHERE;
			goalCandidatePoint.action = visualization_msgs::Marker::ADD;
			goalCandidatePoint.pose.position.x = n->pos(0);
			goalCandidatePoint.pose.position.y = n->pos(1);
			goalCandidatePoint.pose.position.z = n->pos(2);
			goalCandidatePoint.lifetime = ros::Duration(0.5);
			goalCandidatePoint.scale.x = 0.2;
			goalCandidatePoint.scale.y = 0.2;
			goalCandidatePoint.scale.z = 0.2;
			goalCandidatePoint.color.a = 1.0;
			goalCandidatePoint.color.r = 1.0;
			goalCandidatePoint.color.g = 0.0;
			goalCandidatePoint.color.b = 1.0;
			++countGoalCandidateNum;
			roadmapMarkers.markers.push_back(goalCandidatePoint);
		}

		this->roadmapPub_.publish(roadmapMarkers);
	}
	void DEP::publishRoadmapUGV() {
		visualization_msgs::MarkerArray arr;

		int id_pt = 0, id_edge = 0, id_text = 0;

		for (const auto& n : prmNodeVecUGV_) {
			// --- Node point (blue) ---
			visualization_msgs::Marker pt;
			pt.header.frame_id = "map";
			pt.header.stamp    = ros::Time::now();
			pt.ns   = "ugv_prm_point";
			pt.id   = id_pt++;
			pt.type = visualization_msgs::Marker::SPHERE;
			pt.action = visualization_msgs::Marker::ADD;
			pt.pose.position.x = n->pos(0);
			pt.pose.position.y = n->pos(1);
			pt.pose.position.z = n->pos(2);        // likely your z_proj_ugv_ (~0.40)
			pt.scale.x = pt.scale.y = pt.scale.z = 0.10;
			pt.color.a = 1.0; pt.color.r = 0.0; pt.color.g = 0.2; pt.color.b = 1.0;
			pt.lifetime = ros::Duration(0.5);
			arr.markers.push_back(pt);

			// --- Info gain text (optional) ---
			visualization_msgs::Marker txt;
			txt.header.frame_id = "map";
			txt.header.stamp    = ros::Time::now();
			txt.ns   = "ugv_num_voxel_text";
			txt.id   = id_text++;
			txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			txt.action = visualization_msgs::Marker::ADD;
			txt.pose.position.x = n->pos(0);
			txt.pose.position.y = n->pos(1);
			txt.pose.position.z = n->pos(2) + 0.1;
			txt.scale.z = 0.12;
			txt.color.a = 1.0; txt.color.r = txt.color.g = txt.color.b = 1.0;
			txt.text = std::to_string(n->numVoxels);
			txt.lifetime = ros::Duration(0.5);
			arr.markers.push_back(txt);

			// --- Edges (cyan) ---
			visualization_msgs::Marker line;
			line.header.frame_id = "map";
			line.header.stamp    = ros::Time::now();
			line.ns   = "ugv_edge";
			line.id   = id_edge++;
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.action = visualization_msgs::Marker::ADD;
			line.scale.x = 0.05;
			line.color.a = 1.0; line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0;
			line.lifetime = ros::Duration(0.5);

			for (const auto& adj : n->adjNodes) {
			geometry_msgs::Point p1, p2;
			p1.x = n->pos(0);      p1.y = n->pos(1);      p1.z = n->pos(2);
			p2.x = adj->pos(0);    p2.y = adj->pos(1);    p2.z = adj->pos(2);
			line.points.push_back(p1);
			line.points.push_back(p2);
			}
			if (!line.points.empty()) arr.markers.push_back(line);
		}

		roadmapUGVPub_.publish(arr);
	}

	void DEP::publishCandidatePaths(){
		visualization_msgs::MarkerArray candidatePathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (std::vector<std::shared_ptr<PRM::Node>>& path : this->candidatePaths_){
			for (size_t i=0; i<path.size(); ++i){
				std::shared_ptr<PRM::Node> n = path[i];
				visualization_msgs::Marker point;
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "candidate_path_node";
				point.id = countNodeNum;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = n->pos(0);
				point.pose.position.y = n->pos(1);
				point.pose.position.z = n->pos(2);
				point.lifetime = ros::Duration(0.1);
				point.scale.x = 0.15;
				point.scale.y = 0.15;
				point.scale.z = 0.15;
				point.color.a = 1.0;
				point.color.r = 1.0;
				point.color.g = 1.0;
				point.color.b = 0.0;
				++countNodeNum;
				candidatePathMarkers.markers.push_back(point);

				if (i<path.size()-1){
					std::shared_ptr<PRM::Node> nNext = path[i+1];
					visualization_msgs::Marker line;
					line.ns = "candidate_path";
					line.header.frame_id = "map";
					line.type = visualization_msgs::Marker::LINE_LIST;
					line.header.stamp = ros::Time::now();
					geometry_msgs::Point p1, p2;
					p1.x = n->pos(0);
					p1.y = n->pos(1);
					p1.z = n->pos(2);
					p2.x = nNext->pos(0);
					p2.y = nNext->pos(1);
					p2.z = nNext->pos(2);				
					line.points.push_back(p1);
					line.points.push_back(p2);
					line.id = countLineNum;
					line.scale.x = 0.1;
					line.scale.y = 0.1;
					line.scale.z = 0.1;
					line.color.r = 0.0;
					line.color.g = 0.0;
					line.color.b = 0.0;
					line.color.a = 1.0;
					line.lifetime = ros::Duration(0.5);
					++countLineNum;
					candidatePathMarkers.markers.push_back(line);				
				}
			}
		}
		this->candidatePathPub_.publish(candidatePathMarkers);		
	}
	
	void DEP::publishBestPath(){
		visualization_msgs::MarkerArray bestPathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (size_t i=0; i<this->bestPath_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->bestPath_[i];
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "best_path_node";
			point.id = countNodeNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.5);
			point.scale.x = 0.2;
			point.scale.y = 0.2;
			point.scale.z = 0.2;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 1.0;
			point.color.b = 1.0;
			++countNodeNum;
			bestPathMarkers.markers.push_back(point);

			if (i<this->bestPath_.size()-1){
				std::shared_ptr<PRM::Node> nNext = this->bestPath_[i+1];
				visualization_msgs::Marker line;
				line.ns = "best_path";
				line.header.frame_id = "map";
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.header.stamp = ros::Time::now();
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = nNext->pos(0);
				p2.y = nNext->pos(1);
				p2.z = nNext->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countLineNum;
				line.scale.x = 0.2;
				line.scale.y = 0.2;
				line.scale.z = 0.2;
				line.color.r = 1.0;
				line.color.g = 0.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.5);
				++countLineNum;
				bestPathMarkers.markers.push_back(line);				
			}
		}
		this->bestPathPub_.publish(bestPathMarkers);		
	}

	void DEP::publishFrontier(){
		visualization_msgs::MarkerArray frontierMarkers;
		int frontierRangeCount = 0;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			visualization_msgs::Marker range;

			Eigen::Vector3d p = this->frontierPointPairs_[i].first;
			double dist = this->frontierPointPairs_[i].second;

			range.header.frame_id = "map";
			range.header.stamp = ros::Time::now();
			range.ns = "frontier range";
			range.id = frontierRangeCount;
			range.type = visualization_msgs::Marker::SPHERE;
			range.action = visualization_msgs::Marker::ADD;
			range.pose.position.x = p(0);
			range.pose.position.y = p(1);
			range.pose.position.z = p(2);
			range.lifetime = ros::Duration(0.5);
			range.scale.x = dist;
			range.scale.y = dist;
			range.scale.z = 0.1;
			range.color.a = 0.4;
			range.color.r = 0.0;
			range.color.g = 0.0;
			range.color.b = 1.0;
			++frontierRangeCount;
			frontierMarkers.markers.push_back(range);			
		}
		this->frontierVisPub_.publish(frontierMarkers);
	}

}