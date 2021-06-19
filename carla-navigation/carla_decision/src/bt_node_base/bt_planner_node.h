
#ifndef CARLA_DECISION_BT_PLANNER_NODE
#define CARLA_DECISION_BT_PLANNER_NODE

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "common.h"

namespace BT {

/** Helper Node to call an actionlib::SimpleActionClient<>
 * inside a BT::ActionNode.
 *
 * Note that the user must implement the methods:
 *
 *  - sendGoal
 *  - onResult
 *  - onFailedRequest
 *  - halt (optionally)
 *
 */
template<class GlobalPlannerActionT, class LocalPlannerActionT>
class RosPlanningActionNode : public BT::ActionNodeBase {
 public:
  using GlobalPlannerActionClientT = actionlib::SimpleActionClient<GlobalPlannerActionT>;
  using LocalPlannerActionClientT = actionlib::SimpleActionClient<LocalPlannerActionT>;
  using GlobalPlannerActionGoalT = typename GlobalPlannerActionT::_action_goal_type::_goal_type;
  using GlobalPlannerActionFeedbackT = typename GlobalPlannerActionT::_action_feedback_type::_feedback_type;
  using LocalPlannerActionGoalT = typename LocalPlannerActionT::_action_goal_type::_goal_type;
  using LocalPlannerActionResultT = typename LocalPlannerActionT::_action_result_type::_result_type;
  using ActionSimpleClientGoalState = actionlib::SimpleClientGoalState;

 public:
  RosPlanningActionNode() = delete;

  RosPlanningActionNode(const std::string& bt_node_name,
                        const std::string &global_action_client_name,
                        const std::string &local_action_client_name,
                        const BT::NodeConfiguration & conf) : BT::ActionNodeBase(bt_node_name, conf){
    global_action_client_name_ = global_action_client_name;
    local_action_client_name_ = local_action_client_name;
    nh_ = config().blackboard->template get<ros::NodeHandlePtr>("node_handler");
    global_planner_action_client_ = std::make_shared<GlobalPlannerActionClientT>(*nh_, global_action_client_name_, true);
    local_planner_action_client_ = std::make_shared<LocalPlannerActionClientT>(*nh_, local_action_client_name_, true);

    WaitActionServer(global_action_client_name_, global_planner_action_client_);
    WaitActionServer(local_action_client_name_, local_planner_action_client_);

  }

  virtual ~RosPlanningActionNode() = default;

  static PortsList providedPorts() {
    return {};
  }

  virtual void on_send_global_planner_goal() {};

  virtual void on_send_local_planner_goal() {};

  virtual void on_wait_for_result() {};

  virtual bool IsGlobalPlanValid(){
    return true;
  };

  virtual NodeStatus on_result(const LocalPlannerActionResultT &res) {
    setStatus(BT::NodeStatus::IDLE);
    return BT::NodeStatus::SUCCESS;
  };

  virtual BT::NodeStatus on_success() {
    return BT::NodeStatus::SUCCESS;
  }

  enum FailureCause {
    MISSING_SERVER = 0,
    ABORTED_BY_SERVER = 1,
    REJECTED_BY_SERVER = 2,
    NOT_VALID_PATH = 3,
    PREEMPTED = 4,
    WAIT_FIRST_GOAL = 5
  };

  virtual NodeStatus on_failed_request(FailureCause failure) {
    switch (failure) {
      case FailureCause::ABORTED_BY_SERVER:
        std::cout << "Error: ABORTED_BY_SERVER!" << std::endl;
        break;
      case FailureCause::REJECTED_BY_SERVER:
        std::cout << "Error: REJECTED_BY_SERVER!" << std::endl;
        break;
      case FailureCause::NOT_VALID_PATH:
        std::cout << "Error: NOT_VALID_PATH!" << std::endl;
        break;
      case FailureCause::PREEMPTED:
        std::cout << "Error: PREEMPTED!" << std::endl;
        break;
      default:std::cout << "Error: Aborted" << std::endl;
    }
    setStatus(NodeStatus::IDLE);
    return NodeStatus::FAILURE;
  }

  void halt() override {
    if (status() == NodeStatus::RUNNING) {
      global_planner_action_client_->cancelGoal();
      local_planner_action_client_->cancelGoal();
    }
    setStatus(NodeStatus::IDLE);
  }

  virtual void on_check_action_state(ActionSimpleClientGoalState state){};

  BT::NodeStatus tick() override {
    // Timeout 100ms
//    ros::Duration timeout(static_cast<double>(300.0) * 1e-3);
    bool connected = IsServerConnected();
    //action_client_->waitForServer(timeout);
    if (!connected) {
      std::cerr << "[MISSING_SERVER]: " << "action server connected failed!" << std::endl;
      return on_failed_request(MISSING_SERVER);
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);
      on_send_global_planner_goal();
      if (status() == BT::NodeStatus::FAILURE) {
        return BT::NodeStatus::FAILURE;
      } else {
        global_planner_action_client_->sendGoal(goal_,
                                               typename GlobalPlannerActionClientT::SimpleDoneCallback(),
                                               typename GlobalPlannerActionClientT::SimpleActiveCallback(),
                                               boost::bind(&RosPlanningActionNode::GlobalPlannerFeedbackCallback, this, _1));
      }
    }
    // RUNNING
    auto action_state = local_planner_action_client_->getState();

    // Please refer to these states
    if (action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE) {
      return NodeStatus::RUNNING;
    } else if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return on_result(*local_planner_action_client_->getResult());
    } else if (action_state == actionlib::SimpleClientGoalState::ABORTED) {
      return on_failed_request(ABORTED_BY_SERVER);
    } else if (action_state == actionlib::SimpleClientGoalState::REJECTED) {
      return on_failed_request(REJECTED_BY_SERVER);
    } else if (action_state == actionlib::SimpleClientGoalState::PREEMPTED) {
      return on_failed_request(PREEMPTED);
    } else {
      // FIXME: is there any other valid state we should consider?
      throw std::logic_error("Unexpected state in RosActionNode::tick()");
    }
  }

  virtual void GlobalPlannerFeedbackCallback(const typename GlobalPlannerActionFeedbackT::ConstPtr &global_planner_feedback) {}

 private:
  bool IsServerConnected() {
    return (local_planner_action_client_->isServerConnected() && global_planner_action_client_->isServerConnected());
  };

  template<class ActionClientT>
  void WaitActionServer(const std::string &action_client_name, const ActionClientT &action_client){
    std::cout << "[BT ROS Planner Action Node]: " << "Waiting action server (" << action_client_name << ")..." << std::endl;
    action_client->waitForServer();
    std::cout << "[BT ROS Planner Action Node]: Action Server (" << action_client_name << ") connected success!" << std::endl;
  };



 protected:
  ros::NodeHandlePtr nh_;

  std::string global_action_client_name_;
  std::string local_action_client_name_;
  std::shared_ptr<GlobalPlannerActionClientT> global_planner_action_client_;
  std::shared_ptr<LocalPlannerActionClientT> local_planner_action_client_;
  GlobalPlannerActionGoalT goal_;
  LocalPlannerActionGoalT local_goal_;
  bool first_goal_received_ = false;


};

}

#endif //CARLA_DECISION_BT_PLANNER_NODE
