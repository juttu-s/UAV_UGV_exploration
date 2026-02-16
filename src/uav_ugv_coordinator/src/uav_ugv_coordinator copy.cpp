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
  UAV_TAKEOFF_SCAN,
  UAV_LAND_ON_UGV,
  PICK_GROUND_TARGET,
  UGV_NAV_GOAL,
  UAV_TAKEOFF_FOR_OP,
  TRIGGER_OP_AND_WAIT,
  EXECUTING_OP,
  UAV_LAND_FINAL
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
    nh_.param("takeoff_alt",          takeoff_alt_,          1.2);
    nh_.param("scan_hold_sec",        scan_hold_sec_,        1.0);
    nh_.param("ugv_nav_timeout_sec",  ugv_nav_timeout_sec_,  50.0);
    nh_.param("reach_tol_xyz",        reach_tol_xyz_,        0.5);

    // nh_.param("ground_goal_topic",    ground_goal_topic_,    std::string("/global_planner/highest_gain_xy"));
    nh_.param("request_op_topic",     request_op_topic_,     std::string("/global_planner/request_op"));
    nh_.param("op_path_topic",        op_path_topic_,        std::string("/dep/best_paths"));
    nh_.param("ext_waypoints_topic",  ext_waypoints_topic_,  std::string("/uav_ugv_coordinator/waypoints"));
    nh_.param("ugv_path_topic",       ugv_path_topic_,       std::string("/ugv/op_path")); // NEW

    // --- I/O ---
    uav_pose_sub_     = nh_.subscribe("/CERLAB/quadcopter/pose", 1, &Coordinator::uavPoseCb, this);
    // ground_goal_sub_  = nh_.subscribe(ground_goal_topic_, 1, &Coordinator::groundGoalCb, this);
    op_path_sub_      = nh_.subscribe(op_path_topic_, 1, &Coordinator::opPathCb, this);
    ugv_path_sub_     = nh_.subscribe(ugv_path_topic_, 1, &Coordinator::ugvPathCb, this);   // NEW

    // To forward a path to AutoFlight (external waypoints) and to ping OP request
    ext_waypoints_pub_ = nh_.advertise<nav_msgs::Path>(ext_waypoints_topic_, 1, true);
    request_op_pub_    = nh_.advertise<std_msgs::Empty>(request_op_topic_, 1, true);

    ROS_INFO("[Coordinator] Waiting for move_base server...");
    mb_.waitForServer();

    // ---- Construct dynamicExploration and start its target publisher thread ----
    ROS_INFO("[Coordinator] Constructing dynamicExploration helper (no run())...");
    ros::NodeHandle nh_global;
    de_ = std::make_shared<AutoFlight::dynamicExploration>(nh_global);

    // Start the low-level target publisher the same way run() would
    if (!de_->targetPubWorker_.joinable()) {
      de_->targetPubWorker_ = std::thread(&AutoFlight::flightBase::publishTarget, de_.get());
    }

    ROS_INFO("[MAIN] Coordinator constructed. Entering loop.");
    change(State::UAV_TAKEOFF_SCAN);
  }

  ~Coordinator(){
    if (de_) {
      try { de_->stop(); } catch(...) {}
      if (de_->targetPubWorker_.joinable()) {
        // Avoid blocking on timers; let process shutdown tear it down.
        de_->targetPubWorker_.detach();
      }
    }
  }

  void spinOnce() {
    switch (state_) {
      case State::UAV_TAKEOFF_SCAN: {
        if (!waitForUavPose(5.0)) {
          ROS_INFO_THROTTLE(2.0,"[Coord] waiting UAV pose...");
          return;
        }

        // Use AutoFlight helper: takeoff (uses its configured height)
        ROS_INFO("[Coordinator] Calling AutoFlight::takeoff()");
        de_->takeoff();

        // 360° scan using yaw helper
        const double yaw_rate = 0.5; // rad/s
        ROS_INFO("[Coordinator] 360° scan");
        de_->moveToOrientation(-M_PI/2, yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation(-M_PI,   yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( M_PI/2, yaw_rate); de_->waitTime(scan_hold_sec_);
        de_->moveToOrientation( 0.0,    yaw_rate); de_->waitTime(scan_hold_sec_);

        // Initialize DEP internals without running the interactive flow
        de_->initExplore();
        de_->registerCallback();

        change(State::PICK_GROUND_TARGET);
      } break;

      case State::PICK_GROUND_TARGET: {
        // Prefer a full UGV OP path if available; otherwise fallback to single ground goal
        if (have_ugv_path_) {
          if (sendNextUgvWaypoint()) {
            change(State::UGV_NAV_GOAL);
            break;
          } else {
            ROS_WARN_THROTTLE(2.0, "[Coordinator] Have UGV path but failed to send first waypoint.");
          }
        }

        if (!have_ground_goal_) {
          ROS_INFO_THROTTLE(2.0,"[Coordinator] Waiting ground goal on %s", ground_goal_topic_.c_str());
          return;
        }
        sendUgvGoal(latest_ground_goal_);
        change(State::UGV_NAV_GOAL);
      } break;

      case State::UGV_NAV_GOAL: {
        // If no current goal is active (e.g., we just arrived), try next waypoint
        if (!ugv_goal_active_) {
          if (have_ugv_path_ && ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
            ++ugv_wp_idx_;
            sendNextUgvWaypoint();
            return;
          }
          // Path complete -> proceed to UAV OP
          ROS_INFO("[Coordinator] UGV completed OP path. UAV takeoff for OP.");
          change(State::UAV_TAKEOFF_FOR_OP);
          return;
        }

        // Check current goal state
        auto st = mb_.getState();
        if (st.isDone()) {
          ugv_goal_active_ = false;
          ROS_INFO("[Coordinator] UGV waypoint %zu done: %s",
                   ugv_wp_idx_ + 1, st.toString().c_str());
          return; // next loop iteration will send next waypoint (top of this case)
        }

        // Timeout handling
        if (timedOut(ros::Duration(ugv_nav_timeout_sec_))) {
          ROS_WARN("[Coordinator] UGV nav timeout on waypoint %zu. Cancelling and trying next.",
                   ugv_wp_idx_ + 1);
          mb_.cancelGoal();
          ugv_goal_active_ = false;
          if (have_ugv_path_ && ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
            ++ugv_wp_idx_;
            sendNextUgvWaypoint();
          } else {
            change(State::UAV_TAKEOFF_FOR_OP);
          }
        }
      } break;

      case State::UAV_TAKEOFF_FOR_OP: {
        de_->takeoff();
        // If you need a very specific height, you can also nudge with external waypoints:
        // publishVerticalPath(takeoff_alt_);
        // (void)waitReachZ(takeoff_alt_, 0.10, 20.0);

        std_msgs::Empty e;
        request_op_pub_.publish(e);  // Tell DEP to compute OP tour
        ROS_INFO("[Coordinator] OP requested, waiting for tour on %s", op_path_topic_.c_str());
        change(State::TRIGGER_OP_AND_WAIT);
      } break;

      case State::TRIGGER_OP_AND_WAIT: {
        if (!have_op_path_) {
          ROS_INFO_THROTTLE(2.0,"[Coordinator] Waiting OP path...");
          return;
        }
        // Forward to AutoFlight (its externalPathCb)
        ext_waypoints_pub_.publish(op_path_);
        ROS_INFO("[Coordinator] OP path (%zu poses) sent to AutoFlight.", op_path_.poses.size());
        change(State::EXECUTING_OP);
      } break;

      case State::EXECUTING_OP: {
        if (!have_pose_ || op_path_.poses.empty()) return;
        const auto& goal = op_path_.poses.back().pose.position;
        if (distXYZ(last_pose_.pose.position, goal) <= reach_tol_xyz_) {
          ROS_INFO("[Coordinator] OP finished.");
          have_op_path_ = false;
          change(State::UAV_LAND_FINAL);
        }
      } break;

      case State::UAV_LAND_FINAL: {
        publishExternalPathToZ(0.10);
        (void)waitReachZ(0.10, 0.10, 20.0);
        // Loop back for next task, or exit if you want
        change(State::PICK_GROUND_TARGET);
      } break;
    }
  }

private:
  // ----------------- Callbacks -----------------
  void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_pose_ = *msg; have_pose_ = true;
  }

  void groundGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    latest_ground_goal_ = *msg; have_ground_goal_ = true;
  }

  void opPathCb(const nav_msgs::Path::ConstPtr& msg){
    if (!msg || msg->poses.empty()) return;
    op_path_ = *msg; have_op_path_ = true;
  }

  void ugvPathCb(const nav_msgs::Path::ConstPtr& msg){ // NEW
    if (!msg || msg->poses.empty()) return;
    ugv_path_ = *msg;
    have_ugv_path_ = true;
    ugv_wp_idx_ = 0;
    ROS_INFO("[Coordinator] Received UGV OP path with %zu poses on %s",
             ugv_path_.poses.size(), ugv_path_topic_.c_str());
  }

  // ----------------- Helpers -----------------
  void publishVerticalPath(double z_target){
    if (!have_pose_) return;
    nav_msgs::Path path; path.header.frame_id = map_frame_; path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p0 = last_pose_;
    geometry_msgs::PoseStamped p1 = p0;
    p1.header.frame_id = map_frame_;
    p1.pose.position.z = z_target;
    p1.pose.orientation.w = 1.0;
    path.poses = {p0, p1};
    ext_waypoints_pub_.publish(path);
  }

  void publishExternalPathToZ(double z_target) {
    if (!have_pose_) return;

    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp    = ros::Time::now();

    geometry_msgs::PoseStamped p0 = last_pose_;
    p0.header.frame_id = map_frame_;
    p0.header.stamp    = ros::Time::now();

    geometry_msgs::PoseStamped p1 = p0;   // keep x/y & yaw, change z
    p1.pose.position.z = z_target;

    path.poses = {p0, p1};
    ext_waypoints_pub_.publish(path);
  }

  bool waitForUavPose(double timeout_sec){
    ros::Time t0 = ros::Time::now(); ros::Rate r(50);
    while (ros::ok() && (ros::Time::now()-t0).toSec() < timeout_sec){
      if (have_pose_) return true;
      ros::spinOnce(); r.sleep();
    }
    return have_pose_;
  }

  bool waitReachZ(double z_target, double tol, double timeout_sec){
    ros::Time t0 = ros::Time::now(); ros::Rate r(30);
    while (ros::ok() && (ros::Time::now()-t0).toSec() < timeout_sec){
      if (have_pose_ && std::fabs(last_pose_.pose.position.z - z_target) <= tol) return true;
      ros::spinOnce(); r.sleep();
    }
    return false;
  }

  void sendUgvGoal(const geometry_msgs::PoseStamped& goal){
    move_base_msgs::MoveBaseGoal g; g.target_pose = goal;
    if (g.target_pose.header.frame_id.empty()) g.target_pose.header.frame_id = map_frame_;
    g.target_pose.header.stamp = ros::Time::now();

    // Force Z=0 for UGV
    g.target_pose.pose.position.z = 0.0;

    mb_.sendGoal(g);
    ugv_goal_active_ = true;
    ugv_goal_sent_ts_ = ros::Time::now();
    ROS_INFO("[Coordinator] UGV goal sent (%.2f, %.2f).",
             g.target_pose.pose.position.x, g.target_pose.pose.position.y);
  }

  // Send the current ugv_path_ waypoint (index ugv_wp_idx_) to move_base
  bool sendNextUgvWaypoint() {
    if (!have_ugv_path_ || ugv_wp_idx_ >= ugv_path_.poses.size()) return false;

    move_base_msgs::MoveBaseGoal g;
    g.target_pose = ugv_path_.poses[ugv_wp_idx_];

    if (g.target_pose.header.frame_id.empty()) g.target_pose.header.frame_id = map_frame_;
    g.target_pose.header.stamp = ros::Time::now();

    // Enforce Z=0 for UGV
    g.target_pose.pose.position.z = 0.0;

    // Optional: face next waypoint
    if (ugv_wp_idx_ + 1 < ugv_path_.poses.size()) {
      const auto& A = g.target_pose.pose.position;
      const auto& B = ugv_path_.poses[ugv_wp_idx_ + 1].pose.position;
      const double yaw = std::atan2(B.y - A.y, B.x - A.x);
      g.target_pose.pose.orientation = yawToQuat(yaw);
    } else {
      g.target_pose.pose.orientation.w = 1.0;
    }

    mb_.sendGoal(g);
    ugv_goal_active_ = true;
    ugv_goal_sent_ts_ = ros::Time::now();

    ROS_INFO("[Coordinator] UGV waypoint %zu/%zu sent: (%.2f, %.2f).",
             ugv_wp_idx_ + 1, ugv_path_.poses.size(),
             g.target_pose.pose.position.x, g.target_pose.pose.position.y);
    return true;
  }

  static geometry_msgs::Quaternion yawToQuat(double yaw) {
    geometry_msgs::Quaternion q;
    q.x = 0.0; q.y = 0.0;
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
  // --- Node / Params ---
  ros::NodeHandle nh_;
  std::string map_frame_{"map"};
  double takeoff_alt_{1.2}, scan_hold_sec_{5.0}, ugv_nav_timeout_sec_{300.0}, reach_tol_xyz_{0.25};

  // --- Topics ---
  std::string ground_goal_topic_;
  std::string request_op_topic_;
  std::string op_path_topic_;
  std::string ext_waypoints_topic_;
  std::string ugv_path_topic_; // NEW

  // --- I/O ---
  ros::Subscriber uav_pose_sub_, op_path_sub_, ugv_path_sub_; // NEW
  ros::Publisher  ext_waypoints_pub_, request_op_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_;

  // --- State machine ---
  State state_{State::UAV_TAKEOFF_SCAN};
  ros::Time state_ts_;

  // --- UAV status ---
  geometry_msgs::PoseStamped last_pose_; bool have_pose_{false};
  nav_msgs::Path op_path_; bool have_op_path_{false};

  // --- UGV status ---
  geometry_msgs::PoseStamped latest_ground_goal_; bool have_ground_goal_{false};
  bool ugv_goal_active_{false};
  ros::Time ugv_goal_sent_ts_;
  nav_msgs::Path ugv_path_; bool have_ugv_path_{false}; size_t ugv_wp_idx_{0}; // NEW

  // --- AutoFlight glue ---
  std::shared_ptr<AutoFlight::dynamicExploration> de_;
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
