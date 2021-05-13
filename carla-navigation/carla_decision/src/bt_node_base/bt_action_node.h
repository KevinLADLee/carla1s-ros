//
// Created by kevinlad on 2021/4/21.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_BASE_BT_ACTION_NODE_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_BASE_BT_ACTION_NODE_H_

// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "common.h"

namespace BT
{

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
template<class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
 protected:

  RosActionNode(const std::string& name, const std::string & action_client_name, const BT::NodeConfiguration & conf):
      BT::ActionNodeBase(name, conf), action_client_name_(action_client_name)
{
    nh_ = config().blackboard->template get<ros::NodeHandlePtr>("node_handler");
    action_client_ = std::make_shared<ActionClientType>( *nh_, action_client_name_, true );

    std::cout << "[BT ROS Action Node]: " << "Waiting action server (" << action_client_name_ << ")..." << std::endl;
    action_client_->waitForServer();
    std::cout << "[BT ROS Action Node]: Action Server (" << action_client_name_ << ") connected success!" << std::endl;
}

 public:

  using BaseClass  = RosActionNode<ActionT>;
  using ActionClientType = actionlib::SimpleActionClient<ActionT>;
  using ActionType = ActionT;
  using GoalType   = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;

  RosActionNode() = delete;

  virtual ~RosActionNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
    };
  }

  /// Method called when the Action makes a transition from IDLE to RUNNING.
  /// If it return false, the entire action is immediately aborted, it returns
  /// FAILURE and no request is sent to the server.
  //  virtual bool sendGoal(GoalType& goal) = 0;

  /**
 * @brief Function to perform some user-defined operation on tick
 * Could do dynamic checks, such as getting updates to values on the blackboard
 */
  virtual void on_tick(){};

  virtual void on_wait_for_result(){};

  virtual NodeStatus on_result( const ResultType& res){
    setStatus(BT::NodeStatus::IDLE);
    return BT::NodeStatus::SUCCESS;
  };

  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  enum FailureCause{
    MISSING_SERVER = 0,
    ABORTED_BY_SERVER = 1,
    REJECTED_BY_SERVER = 2,
    NOT_VALID_PATH = 3,
    PREEMPTED = 4,
    WAIT_FIRST_GOAL = 5
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus on_failed_request(FailureCause failure)
  {
    switch (failure) {
      case FailureCause::ABORTED_BY_SERVER:
        std::cout << "Error: " << action_client_name_ << " ABORTED_BY_SERVER!" << std::endl;
        break;
      case FailureCause::REJECTED_BY_SERVER:
        std::cout << "Error: " << action_client_name_ << " REJECTED_BY_SERVER!" << std::endl;
        break;
      case FailureCause::NOT_VALID_PATH:
        std::cout << "Error: " << action_client_name_ << " NOT_VALID_PATH!" << std::endl;
        break;
      case FailureCause::PREEMPTED:
        std::cout << "Error: " << action_client_name_ << " PREEMPTED!" << std::endl;
        break;
      default:
        std::cout << "Error: Aborted" << std::endl;
    }
    setStatus(NodeStatus::IDLE);
    return NodeStatus::FAILURE;
  }

  /// If you override this method, you MUST call this implementation invoking:
  ///
  ///    BaseClass::halt()
  ///
  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      action_client_->cancelGoal();
    }
    setStatus(NodeStatus::IDLE);
  }

  BT::NodeStatus tick() override
  {
    // Timeout 100ms
//    ros::Duration timeout(static_cast<double>(300.0) * 1e-3);
    bool connected = action_client_->isServerConnected();
        //action_client_->waitForServer(timeout);
    if( !connected ){
      std::cerr << "[MISSING_SERVER] " << action_client_name_ << " connected failed!" << std::endl;
      return on_failed_request(MISSING_SERVER);
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);
      on_tick();
      std::cout << action_client_name_ << " send goal" << std::endl;
      if (status() == BT::NodeStatus::FAILURE) {
        return BT::NodeStatus::FAILURE;
      } else{
        action_client_->sendGoal(goal_);
      }
    }
    // RUNNING
    auto action_state = action_client_->getState();

    // Please refer to these states
    if( action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE )
    {
      return NodeStatus::RUNNING;
    }
    else if( action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return on_result( *action_client_->getResult() );
    }
    else if( action_state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return on_failed_request( ABORTED_BY_SERVER );
    }
    else if( action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      return on_failed_request( REJECTED_BY_SERVER );
    }
    else if( action_state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      return on_failed_request( PREEMPTED );
    }
    else
    {
      // FIXME: is there any other valid state we should consider?
      throw std::logic_error("Unexpected state in RosActionNode::tick()");
    }
  }

 protected:


 protected:
  std::shared_ptr<ActionClientType> action_client_;
  ros::NodeHandlePtr nh_;
  std::string action_client_name_;
  GoalType goal_;
  ResultType goal_result_;
  bool goal_result_available_ = false;
  bool first_goal_received_ = false;

};

}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_BASE_BT_ACTION_NODE_H_
