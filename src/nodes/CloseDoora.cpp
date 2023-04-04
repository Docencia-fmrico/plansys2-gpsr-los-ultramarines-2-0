// Copyright 2022 L4ROS2

#include <memory>

#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "bt_include/CloseDoor.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2_gpsr
{
    CloseDoor()
    : plansys2::ActionExecutorClient("closedoor", 500ms)
    {
      progress_ = 0.0;
    }

  private:
    void do_work()
    {
      if (progress_ < 1.0) {
        progress_ += 0.1;
        send_feedback(progress_, "close running");
      } else {
        finish(true, 1.0, "Door was closed");

        progress_ = 0.0;
        std::cout << std::endl;
      }

      std::cout << "\r\e[K" << std::flush;
      std::cout << "Closing door ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
        std::flush;
    }

    float progress_;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_gpsr::CloseDoor>("CloseDoor");
}
