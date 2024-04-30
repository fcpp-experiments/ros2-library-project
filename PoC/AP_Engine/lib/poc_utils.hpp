// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#ifndef POC_UTILS_H
#define POC_UTILS_H

#include <string>
#include <limits>
#include <vector>
#include <unordered_map>
#include <iostream>

using namespace std;

/* FUNCTIONS */
std::string get_env_var(const char* env_key, const std::string& default_value);

std::vector<std::string> generate_robot_names(const std::string& robot_prefix, const int start, const int robots_count);

std::string read_string_env(const std::string& env_key, const std::string& default_str);

int read_int_env(const std::string& env_key, const int default_int);

double read_double_env(const std::string& env_key, const double default_double);

std::string  get_robot_name(std::vector<std::string> robots, int node_uid);

#endif
