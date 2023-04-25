// Copyright 2019 Intelligent robotics Lab
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

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GpsrController : public rclcpp::Node
{
public:
  GpsrController()
  : rclcpp::Node("gpsr_controller"), state_(PROBLEM1) {}

  bool init(int problem_knowledge)
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge(problem_knowledge);
    if (problem_knowledge == 1) {
      state_ = PROBLEM1;
    } else if (problem_knowledge == 2) {
      state_ = PROBLEM2;
      execute_times = 2;
      problem = 2;
    } else if (problem_knowledge == 3) {
      state_ = PROBLEM3;
      execute_times = 3;
      problem = 3;
    }

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal "
                << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }

  void static_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"bano", "room"});
    problem_expert_->addInstance(plansys2::Instance{"cocina", "room"});
    problem_expert_->addInstance(plansys2::Instance{"salon", "room"});
    problem_expert_->addInstance(plansys2::Instance{"dormitorio", "room"});
    problem_expert_->addInstance(plansys2::Instance{"terraza", "room"});
    problem_expert_->addInstance(plansys2::Instance{"pasillo", "room"});

    problem_expert_->addInstance(plansys2::Instance{"paco", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"pbano", "door"});
    problem_expert_->addInstance(plansys2::Instance{"pcocina", "door"});
    problem_expert_->addInstance(plansys2::Instance{"psalon", "door"});
    problem_expert_->addInstance(plansys2::Instance{"pdormitorio", "door"});
    problem_expert_->addInstance(plansys2::Instance{"pterraza", "door"});
    problem_expert_->addInstance(plansys2::Instance{"ppasillo", "door"});

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

    problem_expert_->addPredicate(
      plansys2::Predicate("(connectedto pasillo dormitorio pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pdormitorio dormitorio)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connectedto dormitorio pasillo pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pdormitorio pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connectedto pasillo terraza pterraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pterraza terraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connectedto terraza pasillo pterraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doorat pterraza pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(doorat ppasillo pasillo)"));
  }

  void init_knowledge1()
  {
    problem_expert_->addInstance(plansys2::Instance{"medicina", "object"});
    problem_expert_->addInstance(plansys2::Instance{"toalla", "object"});
    problem_expert_->addInstance(plansys2::Instance{"plato", "object"});
    problem_expert_->addInstance(plansys2::Instance{"cubiertos", "object"});
    problem_expert_->addInstance(plansys2::Instance{"planta", "object"});

    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pbano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pcocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen psalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pterraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen ppasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robotat paco terraza)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat medicina bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat plato salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat cubiertos salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat toalla dormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat planta terraza)"));
  }

  void init_knowledge2()
  {
    problem_expert_->addInstance(plansys2::Instance{"pastilla_verde", "object"});
    problem_expert_->addInstance(plansys2::Instance{"pastilla_roja", "object"});
    problem_expert_->addInstance(plansys2::Instance{"pastilla_amarilla", "object"});
    problem_expert_->addInstance(plansys2::Instance{"pastilla_azul", "object"});
    problem_expert_->addInstance(plansys2::Instance{"pastilla_naranja", "object"});
    problem_expert_->addInstance(plansys2::Instance{"agua", "object"});

    problem_expert_->addInstance(plansys2::Instance{"abuela", "object"});

    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pbano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pcocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen psalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pterraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen ppasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robotat paco pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat pastilla_verde bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat pastilla_roja salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat pastilla_amarilla salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat pastilla_azul pasillo)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat pastilla_naranja terraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat agua cocina)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat abuela terraza)"));
  }

  void init_knowledge3()
  {
    problem_expert_->addInstance(plansys2::Instance{"regadera_vacia", "object"});
    problem_expert_->addInstance(plansys2::Instance{"regadera_llena", "object"});
    problem_expert_->addInstance(plansys2::Instance{"aspiradora", "object"});
    problem_expert_->addInstance(plansys2::Instance{"polvo1", "object"});
    problem_expert_->addInstance(plansys2::Instance{"polvo2", "object"});
    problem_expert_->addInstance(plansys2::Instance{"polvo3", "object"});
    problem_expert_->addInstance(plansys2::Instance{"polvo4", "object"});
    problem_expert_->addInstance(plansys2::Instance{"polvo5", "object"});

    problem_expert_->addInstance(plansys2::Instance{"perro_sucio", "object"});
    problem_expert_->addInstance(plansys2::Instance{"perro_mojado", "object"});
    problem_expert_->addInstance(plansys2::Instance{"perro_seco", "object"});

    problem_expert_->addInstance(plansys2::Instance{"abuela", "object"});

    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pbano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pcocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen psalon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pdormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen pterraza)"));
    problem_expert_->addPredicate(plansys2::Predicate("(dooropen ppasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robotat paco pasillo)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat regadera_llena bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat aspiradora dormitorio)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat regadera_vacia salon)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat polvo1 salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat polvo2 bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat polvo3 pasillo)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat polvo4 cocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat polvo5 dormitorio)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat perro_sucio salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat perro_mojado bano)"));
    problem_expert_->addPredicate(plansys2::Predicate("(objectat perro_seco terraza)"));

    problem_expert_->addPredicate(plansys2::Predicate("(objectat abuela salon)"));
  }

  void init_knowledge(int problem_knowledge)
  {
    static_knowledge();
    if (problem_knowledge == 1) {
      init_knowledge1();
    } else if (problem_knowledge == 2) {
      init_knowledge2();
    } else if (problem_knowledge == 3) {
      init_knowledge3();
    }
  }

  void step()
  {
    bool new_plan = false;

    switch (state_) {
      case WORKING: {
          // Lets see feedback
          auto my_feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : my_feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0
                      << "%]";
          }
          std::cout << std::endl;

          for (const auto & action_feedback : my_feedback.action_execution_status) {
            if (action_feedback.status == 2 && action_feedback.action == "move") {
              std::cout << "Moving from " << action_feedback.arguments[1] << " to "
                        << action_feedback.arguments[2] << std::endl;
            } else if (action_feedback.status == 2) {
              std::cout << action_feedback.message_status << " " <<
                action_feedback.completion * 100.0
                        << "%" << std::endl;
            }
          }

          // Check if has finished the plan
          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;
              if (problem == 2) {
                std::cout << "CHANGING PROBLEM " << std::endl;
                state_ = PROBLEM2;
              }
              if (problem == 3) {
                std::cout << "CHANGING PROBLEM " << std::endl;
                state_ = PROBLEM3;
              }
            } else {
              std::cout << "Failure finished " << std::endl;
            }
          }
        } break;

      case PROBLEM1: {
          problem_expert_->clearGoal();
          problem_expert_->setGoal(
            plansys2::Goal(
              "(and (objectat medicina dormitorio) (objectat plato cocina) (objectat "
              "cubiertos cocina) (objectat toalla bano) (objectat planta salon))"));

          new_plan = true;
        } break;
      case PROBLEM2: {
          if (execute_times == 2) {
            problem_expert_->clearGoal();
            problem_expert_->setGoal(
              plansys2::Goal(
                "(and (objectat abuela dormitorio) (robotat paco pasillo) "
                "(doorclosed pdormitorio))"));
            execute_times = 1;
          } else if (execute_times == 1) {
            problem_expert_->clearGoal();
            problem_expert_->setGoal(
              plansys2::Goal(
                "(and (objectatRobot paco pastilla_verde) (objectatRobot paco pastilla_roja) "
                "(objectatRobot paco pastilla_azul) (objectatRobot paco pastilla_amarilla) "
                "(objectatRobot paco pastilla_naranja) (objectatRobot paco agua)  (robotat paco "
                "dormitorio) (doorclosed pdormitorio) )"));

            execute_times = 0;
          }

          new_plan = true;
        } break;
      case PROBLEM3: {
          if (execute_times == 3) {
            problem_expert_->clearGoal();
            problem_expert_->setGoal(
              plansys2::Goal(
                "(and (objectat regadera_vacia bano) (objectatRobot paco "
                "regadera_llena) (robotat paco terraza) )"));
            execute_times = 2;
          } else if (execute_times == 2) {
            problem_expert_->clearGoal();
            problem_expert_->setGoal(
              plansys2::Goal(
                "(and (objectatRobot paco polvo1) (objectatRobot paco polvo2) "
                "(objectatRobot paco polvo3) (objectatRobot paco polvo4) (objectatRobot "
                "paco polvo5) (robotat paco terraza) )"));
            execute_times = 1;
          } else if (execute_times == 1) {
            problem_expert_->clearGoal();
            problem_expert_->setGoal(
              plansys2::Goal(
                "(and (objectat perro_sucio bano) (objectat perro_mojado terraza) "
                "(objectat perro_seco salon))"));

            execute_times = 0;
          }

          new_plan = true;
        } break;
    }

    if (new_plan) {
      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (!plan.has_value()) {
        std::cout << "[ERROR] Could not find plan to reach goal "
                  << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
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
  typedef enum { WORKING, PROBLEM1, PROBLEM2, PROBLEM3 } StateType;
  StateType state_;
  int execute_times;
  int problem;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsrController>();
  std::cout << "argv: " << argv[1] << std::endl;

  node->init(atoi(argv[1]));

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
