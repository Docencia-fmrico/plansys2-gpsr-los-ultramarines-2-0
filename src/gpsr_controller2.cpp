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
  bool finished_;

  GpsrController()
  : rclcpp::Node("gpsr_controller"), state_(GOAL_0)
  {
    finished_ = false;
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
    problem_expert_->addInstance(plansys2::Instance{"bano", "room"});
    problem_expert_->addInstance(plansys2::Instance{"cocina", "room"});
    problem_expert_->addInstance(plansys2::Instance{"salon", "room"});
    problem_expert_->addInstance(plansys2::Instance{"dormitorio", "room"});
    problem_expert_->addInstance(plansys2::Instance{"pasillo", "room"});

    problem_expert_->addInstance(plansys2::Instance{"paco", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"pbano", "door"});
    problem_expert_->addInstance(plansys2::Instance{"pcocina", "door"});
    problem_expert_->addInstance(plansys2::Instance{"psalon", "door"});
    problem_expert_->addInstance(plansys2::Instance{"pdormitorio", "door"});
    problem_expert_->addInstance(plansys2::Instance{"ppasillo", "door"});

    problem_expert_->addInstance(plansys2::Instance{"medicina", "object"});
    problem_expert_->addInstance(plansys2::Instance{"toalla", "object"});
    problem_expert_->addInstance(plansys2::Instance{"plato", "object"});
    problem_expert_->addInstance(plansys2::Instance{"cubiertos", "object"});


    problem_expert_->addPredicate(plansys2::Predicate("(connectedto pasillo bano pbano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pbano bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connectedto bano pasillo pbano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pbano pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connectedto pasillo cocina pcocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pcocina cocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connectedto cocina pasillo pcocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pcocina pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connectedto pasillo salon psalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat psalon salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connectedto salon pasillo psalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat psalon pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connectedto pasillo dormitorio pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pdormitorio dormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connectedto dormitorio pasillo pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pdormitorio pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pbano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pcocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen psalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen ppasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robotat paco pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat medicina bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat plato salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat cubiertos salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat toalla dormitorio)"));

    
  }

  void step()
  {
    switch (state_) {
      case GOAL_0:
        {
          // Set the goal for next state TO DO 
          problem_expert_->setGoal(plansys2::Goal("(and(doorclosed psalon))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = GOAL_1;
          }
        }
        break;
      case GOAL_1:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Set the goal for next state TO DO 
              problem_expert_->setGoal(plansys2::Goal("(and(robotat paco dormitorio))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = GOAL_3;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;

      case GOAL_3:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Set the goal for next state TO DO
              problem_expert_->setGoal(plansys2::Goal("(and(doorclosed pdormitorio))"));

              // Compute the plan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Could not find plan to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              if (executor_client_->start_plan_execution(plan.value())) {
                state_ = FINISHED;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      
      case FINISHED:
        std::cout << "CONTROLLER: PLAN FINISHED" << std::endl;
        finished_ = true;
        return;
        break;

      default:
        break;
    }
  }

private:
  typedef enum {GOAL_0, GOAL_1, GOAL_2, GOAL_3, FINISHED} StateType;
  StateType state_;


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
  while (rclcpp::ok() && !node->finished_) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}