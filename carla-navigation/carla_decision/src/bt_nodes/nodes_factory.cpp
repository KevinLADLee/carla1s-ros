
#include "nodes_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  factory.registerNodeType<MoveToGoal>("MoveToGoal");
  factory.registerNodeType<StopAndWait>("StopAndWait");
  factory.registerNodeType<CheckTrafficLight>("CheckTrafficLight");
}
