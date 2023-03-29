// Copyright 2019 Intelligent Robotics Lab
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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GpsrController : public rclcpp::Node
{
public:
  GpsrController()
  : rclcpp::Node("gpsr_controller"), state_(PROBLEM1), last_problem_(PROBLEM2)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"Bano", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Cocina", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Salon", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Dormitorio", "room"});
    problem_expert_->addInstance(plansys2::Instance{"Pasillo", "room"});

    problem_expert_->addInstance(plansys2::Instance{"Robot", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"PBano", "door"});
    problem_expert_->addInstance(plansys2::Instance{"PCocina", "door"});
    problem_expert_->addInstance(plansys2::Instance{"PSalon", "door"});
    problem_expert_->addInstance(plansys2::Instance{"PDormitorio", "door"});
    problem_expert_->addInstance(plansys2::Instance{"PPasillo", "door"});

    problem_expert_->addInstance(plansys2::Instance{"Medicina", "object"});
    problem_expert_->addInstance(plansys2::Instance{"Toalla", "object"});
    problem_expert_->addInstance(plansys2::Instance{"Plato", "object"});
    problem_expert_->addInstance(plansys2::Instance{"Cubiertos", "object"});


    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Pasillo Bano PBano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PBano Bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Bano Pasillo PBano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PBano Pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Pasillo Cocina PCocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PCocina Cocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Cocina Pasillo PCocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PCocina Pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Pasillo Salon PSalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PSalon Salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Salon Pasillo PSalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PSalon Pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Pasillo Dormitorio PDormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PDormitorio Dormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(ConnectedTo Dormitorio Pasillo PDormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorAt PDormitorio Pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(doorOpen PBano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorOpen PCocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorOpen PSalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorOpen PDormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorOpen PPasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robotAt Robot Pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectAt Medicina Bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt Plato Salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt Cubiertos Salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectAt Toalla Dormitorio)"));

  }

  void step() { 

    bool new_plan = false;

    switch (state_) {

      case WORKING:
      {
        // Lets see feedback
        auto my_feedback = executor_client_->getFeedBack();
        
        for (const auto & action_feedback : my_feedback.action_execution_status) {
          if (action_feedback.status == 2 && action_feedback.action == "move") {
            std::cout << "Moving from " << action_feedback.arguments[1] << " to " << 
              action_feedback.arguments[2] << std::endl;
          } else if (action_feedback.status == 2 && action_feedback.action == "cross_door") {
            std::cout << "Crossing door " << action_feedback.arguments[3] << " from " <<
              action_feedback.arguments[1] << " to " << action_feedback.arguments[2] << std::endl;
          } else if (action_feedback.status == 2) {
            std::cout << action_feedback.message_status << " " <<
              action_feedback.completion * 100.0 << "%" << std::endl;
          } 
        }

        // Check if has finished the plan
        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;
          } else {
            std::cout << "Failure finished " << std::endl;
          }

          if (last_problem_ == PROBLEM1) {
            state_ = PROBLEM2;
          } else {
            state_ = PROBLEM1;
          }
        }
      }
      break;
    
      case PROBLEM1:
      {
        problem_expert_->clearGoal();
        problem_expert_->setGoal(plansys2::Goal("(and(objectAt Medicina Dormitorio) (objectAt Plato Cocina) (objectAt Cubiertos Cocina) (objectAt Toalla Bano)))"));
        new_plan = true;
        last_problem_ = PROBLEM1;

      }
      break;
      
      case PROBLEM2:
      {
        problem_expert_->clearGoal();
        problem_expert_->setGoal(plansys2::Goal("(and(robotAt Robot Salon)))"));
        new_plan = true;
        last_problem_ = PROBLEM2;
      }
      break;
    }


    if (new_plan) {

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (!plan.has_value()) {
        std::cout << "[ERROR] Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        return;
      }

      if (executor_client_->start_plan_execution(plan.value())) {
        std::cout << "Plan started succesfully!" << std::endl;
      } else {
        std::cout << "[ERROR] Plan failed at start!" << std::endl;
      }
      state_ = WORKING;
    }

  }

private:
  typedef enum {WORKING, PROBLEM1, PROBLEM2} StateType;
  StateType state_;
  StateType last_problem_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsrController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}