#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <memory>
#include <thread>
#include <cmath>

// AutoFlight (we use the flightBase helpers via dynamicExploration)
#include <autonomous_flight/simulation/dynamicExploration.h>

enum class State {
  INITIAL_TAKEOFF,
  INITIAL_SCAN,
  TRIGGER_PLANNER,      // Request new paths from DEP
  WAIT_FOR_PATHS,       // Wait for BOTH UAV and UGV paths
  EXECUTE_DUAL_PATHS,   // Run both robots simultaneously
  RENDEZVOUS_LAND,      // UAV lands on UGV
  LOOP_RESET            // Takeoff again to restart cycle
};

class Coordinator {
public:
  Coordinator(ros::NodeHandle& nh)
  : nh_(nh),
    mb_("/move_base", true)
  {
    ROS_INFO("[MAIN] Constructing Coordinator...");

    // --- Parameters ---
    nh_.param("map_frame",            map_frame_,            std::string("map"));
    nh_.param("scan_hold_sec",        scan_hold_sec_,        1.0);
    nh_.param("ugv_nav_timeout_sec",  ugv_nav_timeout_sec_,  100.0); // Increased for OP paths
    nh_.param("reach_tol_xyz",        reach_tol_xyz_,        0.3);

    // Topics
    nh_.param("request_op_topic",     request_op_topic_,     std::string("/global_planner/request_op"));
    nh_.param("uav_path_topic",       uav_path_topic_,       std::string("/uav/op_path")); // UAV OP Path
    nh_.param("ugv_path_topic",       ugv_path_topic_,       std::string("/ugv/op_path"));    // UGV OP Path
    nh_.param("ext_waypoints_topic",  ext_waypoints_topic_,  std::string("/uav_ugv_coordinator/waypoints")); // To AutoFlight

    // --- Subscribers ---
    uav_pose_sub_     = nh_.subscribe("/CERLAB/quadcopter/pose", 1, &Coordinator::uavPoseCb, this);
    uav_path_sub_     = nh_.subscribe(uav_path_topic_, 1, &Coordinator::uavPathCb, this);
    ugv_path_sub_     = nh_.subscribe(ugv_path_topic_, 1, &Coordinator::ugvPathCb, this);

    // --- Publishers ---
    // Sends UAV path to dynamicExploration for execution
    ext_waypoints_pub_ = nh_.advertise<nav_msgs::Path>(ext_waypoints_topic_, 1, true);
    // Triggers the DEP planner to calculate new OP paths
    request_op_pub_    = nh_.advertise<std_msgs::Empty>(request_op_topic_, 1, true);

    ROS_INFO("[Coordinator] Waiting for move_base server...");
    mb_.waitForServer();

    // ---- Construct dynamicExploration Helper ----
    ROS_INFO("[Coordinator] Initializing AutoFlight...");
    ros::NodeHandle nh_global;
    de_ = std::make_shared<AutoFlight::dynamicExploration>(nh_global);

    // Start target publisher thread (required for AutoFlight control)
    if (!de_->targetPubWorker_.joinable()) {
      de_->targetPubWorker_ = std::thread(&AutoFlight::flightBase::publishTarget, de_.get());
    }

    

    ROS_INFO("[Coordinator] Ready. Starting Mission.");
    change(State::INITIAL_TAKEOFF);
  }

  ~Coordinator(){
    if (de_) {
      try { de_->stop(); } catch(...) {}
      if (de_->targetPubWorker_.joinable()) de_->targetPubWorker_.detach();
    }
  }

  void spinOnce() {
    switch (state_) {
      // ---------------------------------------------------------
      // 1. INITIAL TAKEOFF & SCAN
      // ---------------------------------------------------------
      case State::INITIAL_TAKEOFF: {
        if (!waitForUavPose(5.0)) return;
        
        ROS_INFO("[Coordinator] Initial Takeoff...");
        de_->takeoff();
        ros::Duration(5.0).sleep(); 
        
        change(State::INITIAL_SCAN);
      } break;

      case State::INITIAL_SCAN: {
        ROS_INFO("[Coordinator] Performing 360 Scan...");
        const double yaw_rate = 0.5; 
        de_->moveToOrientation(-M_PI/2, yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation(-M_PI,   yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( M_PI/2, yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( 0.0,    yaw_rate); de_->waitTime(scan_hold_sec_);

        // Initialize AutoFlight Internals
        de_->initExplore(); 
        de_->registerCallback();
        
        change(State::TRIGGER_PLANNER);
      } break;

      // ---------------------------------------------------------
      // 2. PLANNING PHASE
      // ---------------------------------------------------------
      case State::TRIGGER_PLANNER: {
        ROS_INFO("[Coordinator] Requesting New Plans (UAV + UGV)...");
        
        // Reset flags
        have_uav_path_ = false;
        have_ugv_path_ = false;
        
        // Trigger DEP planner
        std_msgs::Empty e;
        request_op_pub_.publish(e);
        
        // Also enable internal flag if accessible, otherwise the pub above should handle it
        // de_->explorationReplan_ = true; 

        change(State::WAIT_FOR_PATHS);
      } break;

      case State::WAIT_FOR_PATHS: {
        if (have_uav_path_ && have_ugv_path_) {
            ROS_INFO("[Coordinator] Received BOTH paths. Starting Dual Execution.");
            change(State::EXECUTE_DUAL_PATHS);
        } else {
            ROS_INFO_THROTTLE(2.0, "[Coordinator] Waiting for paths... UAV: %d, UGV: %d", 
                              have_uav_path_, have_ugv_path_);
        }
      } break;

      // ---------------------------------------------------------
      // 3. EXECUTION PHASE (SIMULTANEOUS)
      // ---------------------------------------------------------
      case State::EXECUTE_DUAL_PATHS: {
        // --- A. START (First Run Only) ---
        if (!execution_started_) {
            // 1. Send UAV Path to AutoFlight
            ext_waypoints_pub_.publish(uav_path_);
            
            // 2. Start UGV Path (Send first waypoint)
            ugv_wp_idx_ = 0;
            sendNextUgvWaypoint();
            
            execution_started_ = true;
            uav_finished_ = false;
            ugv_finished_ = false;
            ROS_INFO("[Coordinator] dual execution STARTED.");
        }

        // --- B. MONITOR UGV ---
        if (!ugv_finished_) {
            auto st = mb_.getState();
            
            // Check if current waypoint is done
            if (st.isDone()) {
                if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
                    // Send next waypoint
                    ugv_wp_idx_++;
                    sendNextUgvWaypoint();
                } else {
                    ROS_INFO("[Coordinator] UGV finished path.");
                    ugv_finished_ = true;
                }
            } 
            // Timeout Check
            else if (timedOut(ros::Duration(ugv_nav_timeout_sec_))) {
                 ROS_WARN("[Coordinator] UGV Waypoint Timeout. Skipping...");
                 mb_.cancelGoal();
                 // Force next waypoint or finish
                 if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
                    ugv_wp_idx_++;
                    sendNextUgvWaypoint();
                 } else {
                    ugv_finished_ = true;
                 }
            }
        }

        // --- C. MONITOR UAV ---
        if (!uav_finished_) {
            // Check distance to final goal in UAV path
            geometry_msgs::Point target = uav_path_.poses.back().pose.position;
            if (distXYZ(last_pose_.pose.position, target) < reach_tol_xyz_) {
                ROS_INFO("[Coordinator] UAV finished path.");
                uav_finished_ = true;
            }
        }

        // --- D. CHECK COMPLETION ---
        if (uav_finished_ && ugv_finished_) {
            ROS_INFO("[Coordinator] Both robots reached rendezvous. Landing...");
            execution_started_ = false; // Reset for next time
            landing_triggered_ = false;
            change(State::RENDEZVOUS_LAND);
        }

      } break;

      // ---------------------------------------------------------
      // 4. LANDING & LOOP
      // ---------------------------------------------------------
      case State::RENDEZVOUS_LAND: {
        // 1. Ensure UGV is stopped
        mb_.cancelGoal(); 

        // 2. Send Land Command
        if (!landing_triggered_) {
             ROS_INFO("[Coordinator] Sending Land Command...");
             de_->land(); // Try to capture return value if possible
             landing_triggered_ = true; 
        }
        
        // 3. Monitor Height
        // Ensure we have a valid pose before checking Z
        if (have_pose_) {
            if (last_pose_.pose.position.z < 0.6) {
                 ROS_INFO("[Coordinator] Touchdown Confirmed.");
                 ros::Duration(2.0).sleep(); 
                 change(State::LOOP_RESET);
            }
        }
      } break;

      case State::LOOP_RESET: {
        ROS_INFO("[Coordinator] Taking off for next cycle...");
        de_->takeoff();
        ros::Duration(3.0).sleep();
        
        // Skip scan, go straight to planning
        change(State::TRIGGER_PLANNER);
      } break;
    }
  }

private:
  // ----------------- Callbacks -----------------
  void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_pose_ = *msg; have_pose_ = true;
  }

  void uavPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    uav_path_ = *msg; have_uav_path_ = true;
    ROS_INFO("[Coordinator] Received UAV path (%zu nodes).", uav_path_.poses.size());
  }

  void ugvPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    ugv_path_ = *msg; have_ugv_path_ = true;
    ROS_INFO("[Coordinator] Received UGV path (%zu nodes).", ugv_path_.poses.size());
  }

  // ----------------- Helpers -----------------
  bool waitForUavPose(double timeout_sec){
    ros::Time t0 = ros::Time::now(); ros::Rate r(10);
    while (ros::ok() && (ros::Time::now()-t0).toSec() < timeout_sec){
      if (have_pose_) return true;
      ros::spinOnce(); r.sleep();
    }
    return have_pose_;
  }

  void sendNextUgvWaypoint() {
    if (ugv_path_.poses.empty() || ugv_wp_idx_ >= ugv_path_.poses.size()) return;

    move_base_msgs::MoveBaseGoal g;
    g.target_pose = ugv_path_.poses[ugv_wp_idx_];
    g.target_pose.header.frame_id = map_frame_;
    g.target_pose.header.stamp = ros::Time::now();
    g.target_pose.pose.position.z = 0.0; // Force ground

    // --- ORIENTATION LOGIC ---
    
    // CASE A: Intermediate Waypoint 
    // We calculate the natural heading to the NEXT point so the robot drives smoothly forward.
    if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
        const auto& current_pt = ugv_path_.poses[ugv_wp_idx_].pose.position;
        const auto& next_pt    = ugv_path_.poses[ugv_wp_idx_ + 1].pose.position;
        
        // Point towards the next node. This is the fastest way to travel.
        double yaw = std::atan2(next_pt.y - current_pt.y, next_pt.x - current_pt.x);
        g.target_pose.pose.orientation = yawToQuat(yaw);
    } 
    
    // CASE B: Final Waypoint (Rendezvous Mode)
    else {
        if (!uav_path_.poses.empty()) {
          
            g.target_pose.pose.orientation = uav_path_.poses.back().pose.orientation;
            ROS_INFO("[Coordinator] Final Approach: Aligning UGV with UAV.");
        } else {
            // Fallback if UAV path is missing
            g.target_pose.pose.orientation.w = 1.0; 
        }
    }

    mb_.sendGoal(g);
    state_ts_ = ros::Time::now(); // Reset timeout timer
    ROS_INFO("[Coordinator] UGV Waypoint %zu/%zu sent.", ugv_wp_idx_+1, ugv_path_.poses.size());
}

  static geometry_msgs::Quaternion yawToQuat(double yaw) {
    geometry_msgs::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static double distXYZ(const geometry_msgs::Point& a, const geometry_msgs::Point& b){
    const double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  void change(State s){ state_=s; state_ts_=ros::Time::now(); }
  bool timedOut(const ros::Duration& d) const { return (ros::Time::now()-state_ts_)>d; }

private:
  ros::NodeHandle nh_;
  std::string map_frame_{"map"};
  double scan_hold_sec_, ugv_nav_timeout_sec_, reach_tol_xyz_;

  // Topics
  std::string request_op_topic_, uav_path_topic_, ugv_path_topic_, ext_waypoints_topic_;

  // Objects
  ros::Subscriber uav_pose_sub_, uav_path_sub_, ugv_path_sub_;
  ros::Publisher  ext_waypoints_pub_, request_op_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_;
  std::shared_ptr<AutoFlight::dynamicExploration> de_;

  // State
  State state_;
  ros::Time state_ts_;
  bool landing_triggered_{false};

  // Data
  geometry_msgs::PoseStamped last_pose_; bool have_pose_{false};
  nav_msgs::Path uav_path_; bool have_uav_path_{false}; bool uav_finished_{false};
  nav_msgs::Path ugv_path_; bool have_ugv_path_{false}; bool ugv_finished_{false};
  size_t ugv_wp_idx_{0};
  bool execution_started_{false};
};

int main(int argc, char** argv){
  ros::init(argc, argv, "uav_ugv_coordinator");
  ros::NodeHandle nh("~");
  Coordinator c(nh);
  ros::Rate r(20);
  while (ros::ok()){
    c.spinOnce();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
