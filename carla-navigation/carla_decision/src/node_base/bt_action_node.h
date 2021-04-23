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
        InputPort<std::string>("server_name", "name of the Action Server"),
        InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
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

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResult( const ResultType& res) {
    return NodeStatus::SUCCESS;
  };

  /**
 * @brief Function to perform some user-defined operation upon successful
 * completion of the action. Could put a value on the blackboard.
 * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
 */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  enum FailureCause{
    MISSING_SERVER = 0,
    ABORTED_BY_SERVER = 1,
    REJECTED_BY_SERVER = 2
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
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
    ros::Duration timeout(static_cast<double>(100.0) * 1e-3);

    bool connected = action_client_->waitForServer(timeout);
    if( !connected ){
      return onFailedRequest(MISSING_SERVER);
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      on_tick();

      action_client_->sendGoal(goal_);
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
      return onResult( *action_client_->getResult() );
    }
    else if( action_state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return onFailedRequest( ABORTED_BY_SERVER );
    }
    else if( action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      return onFailedRequest( REJECTED_BY_SERVER );
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
  bool goal_updated_ = false;

};

}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_BASE_BT_ACTION_NODE_H_
