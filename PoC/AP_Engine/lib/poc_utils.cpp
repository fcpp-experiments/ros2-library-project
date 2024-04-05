// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#include "lib/poc_utils.hpp"

using namespace std;

//! @brief String representation of a ElectionAlgorithm.
std::string to_string(ElectionAlgorithm ea) {
    switch (ea) {
        case ElectionAlgorithm::GREEDY:
            return "GREEDY";

        case ElectionAlgorithm::LAZY:
            return "LAZY";

        default:
            return "";
    }
}

ElectionAlgorithmMapper::ElectionAlgorithmMapper()
{
    algorithm_map[to_string(ElectionAlgorithm::GREEDY)] = ElectionAlgorithm::GREEDY;
    algorithm_map[to_string(ElectionAlgorithm::LAZY)] = ElectionAlgorithm::LAZY;
}

ElectionAlgorithm ElectionAlgorithmMapper::get_algorithm(const std::string& algorithm_name) const
{
    auto it = algorithm_map.find(algorithm_name);
    if (it != algorithm_map.end())
    {
        return it->second;
    }
    else
    {
        std::cerr << "Unknown algorithm " << algorithm_name << " to be used: so use default GREEDY alg" << endl;
        // default
        return ElectionAlgorithm::LAZY; 
    }
}

std::string get_env_var(const char* env_key, const std::string& default_value) {
    const char* env_value = std::getenv(env_key);
    if (env_value != nullptr) {
        std::string env_value_str(env_value);
        if (!env_value_str.empty() && env_value_str != "\n") {
            return env_value_str;
        }
    }
    return default_value;
}

std::vector<std::string> generate_robot_names(const std::string& robot_prefix, const int start, const int robots_count) {
    std::vector<std::string> prefixes;

    for (int i = start; i < start+robots_count; ++i) {
        prefixes.push_back(robot_prefix + std::to_string(i));
    }

    return prefixes;
}

std::string read_string_env(const std::string& env_key, const std::string& default_str) {
    return get_env_var(env_key.c_str(), default_str);
}

int read_int_env(const std::string& env_key, const int default_int) {
    std::string env_str = get_env_var(env_key.c_str(), std::to_string(default_int));
    int env_int = stoi(env_str);
    return env_int; 
}

double read_double_env(const std::string& env_key, const double default_double) {
    std::string env_str = get_env_var(env_key.c_str(), std::to_string(default_double));
    double env_double = stod(env_str);
    return env_double; 
}

// TODO: use a map instead of a vector
std::string  get_robot_name(std::vector<std::string> robots, int node_uid) {
    std::string robot_name = "";
    if (robots.size() > 1){
        return robots[node_uid-1]; // convert node.uid to index of array
    } else {
        return robots[0]; // there is only a robot set in embedded mode
    }
    return robot_name;
}