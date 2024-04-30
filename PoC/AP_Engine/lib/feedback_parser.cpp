// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#include "feedback_parser.h"

/**** PRIVATE ****/
void add_to_input_status_map(feedback::FeedbackData data) {
  // lock function
  std::lock_guard<std::mutex> lg(RobotStatesMutex);

  struct RobotStatus status = {.pos_x                 = data.pos_x,
                              .pos_y                  = data.pos_y,
                              .orient_w               = data.orient_w,
                              .battery_percent_charge = data.battery_percent_charge,
                              .goal_status            = data.goal_status,
                              .goal_code              = data.goal_code,
                              .goal_current_step      = data.goal_current_step,
                              .dock_status            = data.dock_status,
                              .system_status          = data.system_status
  };

  // create element or append data if the map it's already present
  if (RobotStatesMap.find(data.robot) == RobotStatesMap.end()) {
    RobotStatesMap[data.robot] = {status};
  } else {
    RobotStatesMap[data.robot].push_back(status);
  }  
  // std::cout << "Status added and sent to AP about robot: "   << data.robot   << endl;
}

/**** PUBLIC ****/
//! @brief String representation of a GoalStatus.
std::string feedback::to_string(feedback::GoalStatus s) {
    switch (s) {
        case feedback::GoalStatus::REACHED:
            return "REACHED";

        case feedback::GoalStatus::ABORTED:
            return "ABORTED";

        case feedback::GoalStatus::FAILED:
            return "FAILED";

        case feedback::GoalStatus::RUNNING:
            return "RUNNING";

        case feedback::GoalStatus::UNKNOWN:
            return "UNKNOWN";  

        case feedback::GoalStatus::ILLEGAL:
            return "ILLEGAL";

        case feedback::GoalStatus::NO_GOAL:
            return "NO_GOAL";       

        default:
            return "";
    }
}

//! @brief String representation of a DockStatus.
std::string feedback::to_string(feedback::DockStatus s) {
    switch (s) {
        case feedback::DockStatus::REACHED:
            return "REACHED";

        case feedback::DockStatus::ABORTED:
            return "ABORTED";

        case feedback::DockStatus::FAILED:
            return "FAILED";

        case feedback::DockStatus::RUNNING:
            return "RUNNING";

        case feedback::DockStatus::UNKNOWN:
            return "UNKNOWN";  

        case feedback::DockStatus::NO_DOCK:
            return "NO_DOCK";

        default:
            return "";
    }
}

//! @brief String representation of a SystemStatus.
std::string feedback::to_string(feedback::SystemStatus s) {
    switch (s) {
        case feedback::SystemStatus::OK:
            return "OK";

        case feedback::SystemStatus::KO:
            return "KO";

        case feedback::SystemStatus::NONE:
            return "NONE";

        default:
            return "";
    }
}

void feedback::manager::FeedbackManager::new_line_feedback(std::string content) {
    // std::cout << content << endl;
    auto data = feedback::parser::FeedbackParser::parse_line(content);
    // std::cout << "robot:"              << data.robot                          << endl;
    // std::cout << "pos_x:"              << data.pos_x                          << endl;
    // std::cout << "pos_y:"              << data.pos_y                          << endl;
    // std::cout << "orient_w:"           << data.orient_w                       << endl;
    // std::cout << "bat_charge:"         << data.battery_percent_charge         << endl;
    // std::cout << "goal_status:"        << static_cast<int>(data.goal_status)  << endl;
    // std::cout << "goal_code:"          << data.goal_code                      << endl;
    // std::cout << "goal_current_step:"  << goal_current_step.goal_code         << endl;
    // std::cout << "dock_status:"        << static_cast<int>(data.dock_status)  << endl;
    // std::cout << "system_status:"      << static_cast<int>(data.system_status)<< endl;
    // std::cout << endl;

    add_to_input_status_map(data);
}

feedback::FeedbackData feedback::parser::FeedbackParser::parse_line(string line) {
  std::istringstream ss(line);
  std::vector<std::string> line_elements;
  int idx = 0;
  string robot;
  float pos_x                     = NULL_FLOAT_VALUE;
  float pos_y                     = NULL_FLOAT_VALUE;
  float orient_w                  = NULL_FLOAT_VALUE;
  float battery_percent_charge    = NULL_FLOAT_VALUE;
  GoalStatus goal_status          = feedback::GoalStatus::UNKNOWN;
  string goal_code;
  int goal_current_step           = NULL_INT_VALUE;
  DockStatus dock_status          = feedback::DockStatus::UNKNOWN;
  SystemStatus system_status      = feedback::SystemStatus::NONE;

  for (std::string value; getline(ss, value, COLUMN_DELIMITER);idx++)
  {
    switch (idx)
    {
      case ROBOT_FEEDBACK_LINE_POSITION: {
        robot = std::move(value);
        break;
      }
      case POS_X_FEEDBACK_LINE_POSITION: {
        std::string val = std::move(value);
        pos_x = (val != "") ? std::stof(val) : NULL_FLOAT_VALUE;
        break;
      }
      case POS_Y_FEEDBACK_LINE_POSITION: {
        std::string val = std::move(value);
        pos_y = (val != "") ? std::stof(val) : NULL_FLOAT_VALUE;
        break;
      }
      case ORIENT_W_FEEDBACK_LINE_POSITION: {
        std::string val = std::move(value);
        orient_w = (val != "") ? std::stof(val) : NULL_FLOAT_VALUE;
        break;
      }
      case BATTERY_PERCENT_CHARGE_FEEDBACK_LINE_POSITION: {
        std::string val = std::move(value);
        battery_percent_charge = (val != "") ? std::stof(val) : NULL_FLOAT_VALUE;
        break;
      }
      case GOAL_STATUS_POSITION: {
        std::string val = std::move(value);
        goal_status = (val != "" && std::stoi(val) >= -1) ? feedback::GoalStatus(std::stoi(val)) : feedback::GoalStatus::NO_GOAL;
        break;
      }
      case GOAL_CODE_POSITION: {
        goal_code = std::move(value);
        break;
      }
      case GOAL_CURRENT_STEP_POSITION: {
        std::string val = std::move(value);
        goal_current_step = (val != "") ? std::stoi(val) : NULL_INT_VALUE;
        break;
      }
      case DOCK_STATUS_POSITION: {
        std::string val = std::move(value);
        dock_status = (val != "" && std::stoi(val) >= -1) ? feedback::DockStatus(std::stoi(val)) : feedback::DockStatus::REACHED;
        break;
      }
      case SYSTEM_STATUS_POSITION: {
        std::string val = std::move(value);
        system_status = (val != "" && std::stoi(val) >= -1) ? feedback::SystemStatus(std::stoi(val)) : feedback::SystemStatus::NONE;
        break;
      }
      
      default:
        break;
    }
  }
  struct FeedbackData data = {.robot                    = robot,
                              .pos_x                    = pos_x,
                              .pos_y                    = pos_y,
                              .orient_w                 = orient_w,
                              .battery_percent_charge   = battery_percent_charge,
                              .goal_code                = goal_code,
                              .goal_status              = goal_status,
                              .goal_current_step        = goal_current_step,
                              .dock_status              = dock_status,
                              .system_status            = system_status
  };
  return data;
}
