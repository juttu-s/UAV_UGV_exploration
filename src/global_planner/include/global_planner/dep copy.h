/*
*	File: dep.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef DEP_H
#define DEP_H

#include <map_manager/dynamicMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <global_planner/PRMKDTree.h>
#include <global_planner/PRMAstar.h>
#include <global_planner/utils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <queue>

namespace globalPlanner{
	class DEP{
	private:
		std::string ns_;
		std::string hint_;
		std::mutex data_mutex_; // Add this mutex

		ros::NodeHandle nh_;
		ros::Publisher roadmapPub_;
		ros::Publisher candidatePathPub_;
		ros::Publisher bestPathPub_;
		ros::Publisher frontierVisPub_;
		ros::Subscriber odomSub_;
		ros::Timer visTimer_;
		ros::Publisher roadmapUGVPub_;

		nav_msgs::Odometry odom_;
		std::shared_ptr<mapManager::occMap> map_; 
		std::shared_ptr<PRM::KDTree> roadmap_;
		ros::Publisher ground_goal_pub_;
  		std::string ground_goal_topic_;
		ros::Publisher ugv_goal_pub_;
		ros::Subscriber ugv_odom_sub_;
		ros::Publisher ugv_path_pub_;
		ros::Publisher uav_path_pub_;
		geometry_msgs::Pose ugv_pose_;
		bool ugv_odom_received_ = false;
		void ugvOdomCb(const nav_msgs::Odometry::ConstPtr& msg);

		// parameters
		bool clampToMapRange_ = true; // intersect with current map bbox

		double vel_ = 1.0;
		double angularVel_ = 1.0;
		std::string odomTopic_;
		Eigen::Vector3d globalRegionMin_, globalRegionMax_;
		Eigen::Vector3d localRegionMin_, localRegionMax_;
		int localSampleThresh_;
		int globalSampleThresh_;
		int frontierSampleThresh_;
		double distThresh_;
		double safeDistXY_ =0.8;
		double safeDistZ_;
		bool safeDistCheckUnknown_;
		double horizontalFOV_;
		double verticalFOV_;
		double dmin_;
		double dmax_;
		int nnNum_;
		int nnNumFrontier_;
		double maxConnectDist_;
		std::vector<double> yaws_;
		double minVoxelThresh_;
		int minCandidateNum_;
		int maxCandidateNum_;
		double updateDist_;
		double yawPenaltyWeight_;

		// data
		bool odomReceived_ = false;
		Eigen::Vector3d position_;
		double currYaw_;
		std::deque<Eigen::Vector3d> histTraj_; // historic trajectory for information gain update 
		// std::vector<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes		
		std::unordered_set<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes
		std::vector<std::shared_ptr<PRM::Node>> goalCandidates_;
		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths_;
		std::vector<std::shared_ptr<PRM::Node>> bestPath_;
		std::vector<std::pair<Eigen::Vector3d, double>> frontierPointPairs_;
		inline bool edgeExists(int u, int v, const std::vector<std::vector<double>>& T) const;
		bool validatePathUsesPRMEdges(const std::vector<int>& path,
                                  const std::vector<std::vector<double>>& T) const;
		std::vector<int> findInitialPath(int start, int end,
                                     const std::vector<std::vector<double>>& T) const;


	public:
		DEP(const ros::NodeHandle& nh);

		// Orienteering (GRASP + Path Relinking) parameters
		struct OPParams {
		int    GRASP_ITER     = 30;
		int    ELITE_SET_SIZE = 8;
		double ALPHA          = 0.30;   // RCL fraction
		double T_MAX          = 30.0;   // seconds budget
		};

		// Speed/budget you’ll use in OP
		double uavCruiseSpeed_  = 1.0;    // m/s (set from cfg)
		double timeBudgetSec_   = 30.0;   // set from cfg/battery

		// Main entry: returns PRM indices of the chosen tour
		std::vector<int> selectOrienteeringPathGRASPPR(
		const std::vector<std::shared_ptr<PRM::Node>>& order,
		const std::unordered_map<const PRM::Node*, int>& id_of,
		int start_idx, int end_idx, const OPParams& params, double cruise_speed);


		// --- Helpers used by the selector (declare; implemented in dep.cpp) ---
		void buildScoresAndTimes(const std::vector<std::shared_ptr<PRM::Node>>& order,
								std::vector<double>& scores,
								std::vector<std::vector<double>>& times,
								double cruise_speed_mps) const;

		void dijkstraTimes(int src,
						const std::vector<std::shared_ptr<PRM::Node>>& order,
						const std::unordered_map<const PRM::Node*, int>& id_of,
						const std::vector<std::vector<int>>& adjacency_idx,
						std::vector<double>& dist,
						double cruise_speed_mps) const;

		double pathTime (const std::vector<int>& path,
						const std::vector<std::vector<double>>& T) const;
		double pathScore(const std::vector<int>& path,
						const std::vector<double>& S) const;

		void twoOpt(std::vector<int>& path,
					const std::vector<std::vector<double>>& T) const;

		void localSearchSwapInsert(std::vector<int>& path,
								const std::vector<double>& S,
								const std::vector<std::vector<double>>& T,
								double T_MAX) const;

		std::vector<int> constructiveC4(const std::vector<double>& S,
										const std::vector<std::vector<double>>& T,
										int start_idx, int end_idx,
										double ALPHA, double T_MAX) const;

		std::vector<int> pathRelinking(const std::vector<int>& P, const std::vector<int>& Q,
									const std::vector<double>& S,
									const std::vector<std::vector<double>>& T,
									double T_MAX) const;

		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void loadVelocity(double vel, double angularVel);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		// --- UGV PRM (projection of UAV) ---
		std::shared_ptr<PRM::KDTree> roadmap_ugv_;
		std::unordered_set<std::shared_ptr<PRM::Node>> prmNodeVecUGV_;

		// Map: original UAV node raw pointer -> projected UGV node
		std::unordered_map<const PRM::Node*, std::shared_ptr<PRM::Node>> projOf_;

		double z_proj_ugv_ = 0.30;   // projection plane height

		double ugvMaxRange_ = 5.0;      // UGV sensor max range
		double ugvVerticalFOV_ = 0.52;  // Standard VLP-16 Vertical FOV (+/- 15 deg)
		double ugvSensorHeight_ = 0.2;  // UGV sensor height from ground

		void buildUGVFromUAVProjection();

		static inline Eigen::Vector3d projectToPlane(const Eigen::Vector3d& p, double z)
		{
		return Eigen::Vector3d(p.x(), p.y(), z);
		}


		std::shared_ptr<PRM::Node>
		randomConfigUAV(const Eigen::Vector3d& reqMin,
						const Eigen::Vector3d& reqMax);

		void buildRoadMap();
		nav_msgs::Path getBestPath();
		bool makePlan();
		void detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs);

		void computeEffectiveBox(const Eigen::Vector3d& reqMin,
								const Eigen::Vector3d& reqMax,
								Eigen::Vector3d& effMin,
								Eigen::Vector3d& effMax) const;

		std::shared_ptr<PRM::Node>
		randomConfigBBoxValid(const Eigen::Vector3d& bbMin,
								const Eigen::Vector3d& bbMax);

		std::shared_ptr<PRM::Node>
		randomConfigMixed(const Eigen::Vector3d& reqMin,
							const Eigen::Vector3d& reqMax);
		void pruneNodes();
		void updateInformationGain();
		void printPRMNodesInformationGain();
		std::vector<int> findPathDijkstra(int start_idx, int end_idx, 
                                      const std::vector<std::shared_ptr<PRM::Node>>& nodes,
                                      const std::unordered_map<const PRM::Node*, int>& id_map);
		void getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);
		bool findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates,  std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths);
		void findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath);
		

		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);



		// help function
		bool isPosValid(const Eigen::Vector3d& p);
		bool isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ);
		std::shared_ptr<PRM::Node> randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion);
		bool sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2);
		bool sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos);
		int calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels);
		int calculateUnknownUGV(const std::shared_ptr<PRM::Node>& n);
		double calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path);
		void shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc);
		int weightedSample(const std::vector<double>& weights);
		std::shared_ptr<PRM::Node> sampleFrontierPoint(const std::vector<double>& sampleWeights);
		std::shared_ptr<PRM::Node> extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target);

		// visualization functions
		void publishRoadmap();
		void publishRoadmapUGV();
		void publishCandidatePaths();
		void publishBestPath();
		void publishFrontier();
	};
}


#endif


