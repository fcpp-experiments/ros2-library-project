// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#ifndef POC_UTILS_H
#define POC_UTILS_H

#include <string>
#include <limits>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <iostream>

using namespace std;

/* ENUM, CLASSES */
enum class ElectionAlgorithm
{
    /*! @brief The node with better rank starts to reach the goal, although there are any other nodes running. 
    *   The node with worst rank terminates his run.
    */
    GREEDY, 

    /*! @brief The node with better rank starts to reach the goal, but if and only if there aren't any nodes running (in the known network)
     */
    LAZY
};

class ElectionAlgorithmMapper
{
public:
    ElectionAlgorithmMapper();
    ElectionAlgorithm get_algorithm(const std::string& algorithm_name) const;
private:
    std::unordered_map<std::string, ElectionAlgorithm> algorithm_map;
};

//! @brief String representation of a node_type.
std::string to_string(ElectionAlgorithm ea);

//! @brief Printing node_type.
template <typename O>
O& operator<<(O& o, ElectionAlgorithm ea) {
    o << to_string(ea);
    return o;
}

/* FUNCTIONS */
std::string get_env_var(const char* env_key, const std::string& default_value);

std::vector<std::string> generate_robot_names(const std::string& robot_prefix, const int start, const int robots_count);

std::string read_string_env(const std::string& env_key, const std::string& default_str);

int read_int_env(const std::string& env_key, const int default_int);

double read_double_env(const std::string& env_key, const double default_double);

std::string  get_robot_name(std::vector<std::string> robots, int node_uid);

/* STD OVERRIDES */
namespace std {
    template <typename Rep, typename Period>
    std::string to_string(const std::chrono::duration<Rep, Period>& duration) {
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        return std::to_string(milliseconds);
    }

    template <typename O, typename Clock, typename Duration>
    O& operator<<(O& os, const std::chrono::time_point<Clock, Duration>& time_point) {
        auto duration_since_epoch = time_point.time_since_epoch();
        os << to_string(duration_since_epoch);
        return os;
    }
}

#endif
