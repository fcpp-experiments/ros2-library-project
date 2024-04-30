// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file feedback_parser.h
 * @brief Parse feedback files
 */

#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <vector>
#include <sstream>
#include "common_data.hpp"
#include "poc_config.hpp"

#ifndef FEEDBACK_PARSER_H
#define FEEDBACK_PARSER_H

// CSV positions
#define ROBOT_FEEDBACK_LINE_POSITION                    0
#define POS_X_FEEDBACK_LINE_POSITION                    1
#define POS_Y_FEEDBACK_LINE_POSITION                    2
#define ORIENT_W_FEEDBACK_LINE_POSITION                 3
#define BATTERY_PERCENT_CHARGE_FEEDBACK_LINE_POSITION   4
#define GOAL_STATUS_POSITION                            5
#define GOAL_CODE_POSITION                              6
#define GOAL_CURRENT_STEP_POSITION                      7
#define DOCK_STATUS_POSITION                            8
#define SYSTEM_STATUS_POSITION                          9

using namespace std;

namespace feedback
{

  enum class GoalStatus
    {
        NO_GOAL = -1,
        REACHED,
        ABORTED,
        FAILED,
        RUNNING,
        UNKNOWN,
        ILLEGAL,
    };

  enum class DockStatus
    {
        NO_DOCK = -1,
        REACHED,
        ABORTED,
        FAILED,
        RUNNING,
        UNKNOWN,
    };

  enum class SystemStatus
    {
        NONE = -1,
        KO,
        OK
    };

  //! @brief String representation of a GoalStatus.
  std::string to_string(feedback::GoalStatus s);

  //! @brief Printing GoalStatus.
  template <typename O>
  O& operator<<(O& o, feedback::GoalStatus s) {
      o << to_string(s);
      return o;
  }

  //! @brief String representation of a DockStatus.
  std::string to_string(feedback::DockStatus s);

  //! @brief Printing DockStatus.
  template <typename O>
  O& operator<<(O& o, feedback::DockStatus s) {
      o << to_string(s);
      return o;
  }

  //! @brief String representation of a SystemStatus.
  std::string to_string(feedback::SystemStatus s);

  //! @brief Printing SystemStatus.
  template <typename O>
  O& operator<<(O& o, feedback::SystemStatus s) {
      o << to_string(s);
      return o;
  }

  struct FeedbackData {       
    string robot;        
    float pos_x;
    float pos_y;
    float orient_w;
    float battery_percent_charge; //TODO: add other battery fields
    string goal_code;
    GoalStatus goal_status;
    int goal_current_step;    
    DockStatus dock_status;
    SystemStatus system_status;
  };

  namespace manager
  {
    class FeedbackManager {
      private:

      public:
        static void new_line_feedback(std::string content);
    };
  }

  namespace parser
  {
    class FeedbackParser {
      private:

      public:
        static feedback::FeedbackData parse_line(string line);
    };
  }
}

#endif
