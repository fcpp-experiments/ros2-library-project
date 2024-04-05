// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>

#include "common_data.hpp"

//! @brief String representation of a ProcessingStatus.
std::string to_string(ProcessingStatus s) {
    switch (s) {
        case ProcessingStatus::IDLE:
            return "IDLE";

        case ProcessingStatus::TERMINATING:
            return "TERMINATING";

        case ProcessingStatus::SELECTED:
            return "SELECTED";

        default:
            return "";
    }
}

//! @brief String representation of a node_type.
std::string to_string(node_type s) {
    switch (s) {
        case node_type::ROBOT:
            return "ROBOT";

        case node_type::KIOSK:
            return "KIOSK";

        default:
            return "";
    }
}

std::unordered_map<std::string,std::vector<RobotStatus>> RobotStatesMap{};
std::mutex RobotStatesMutex{};

std::vector<InputGoal> InputGoalList{};
std::mutex GoalMutex{};
