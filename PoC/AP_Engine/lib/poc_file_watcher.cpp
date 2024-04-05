// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <regex>
#include "poc_config.hpp"
#include "goal_parser.h"
#include "feedback_parser.h"

#include "poc_file_watcher.hpp"
#include "file_watcher.hpp"

#include "common_data.hpp"

/**** PRIVATE ****/
void watch_file_fn(string path, string str_expr, void (*callback)(std::string)) {
  // Create a FileWatcher instance that will check the current folder for changes every POLLING_MS milliseconds
    FileWatcher fw{path, std::chrono::milliseconds(POLLING_MS)}; 

    // Start monitoring a folder for changes and (in case of changes)
    // run a user provided lambda function
    fw.start([&] (std::string path_to_watch, FileStatus status) -> void {

        try {
          // Process only regular files, all other file types are ignored
          if(!std::filesystem::is_regular_file(std::filesystem::path(path_to_watch))
              && status != FileStatus::erased) {
              return;
          }
          // Skip file not matching regex
          if(!regex_match (path_to_watch,regex(str_expr)) ) {
              return;
          } 
        } 
        catch(...) {
          std::cout << "warning: discarded file " << path_to_watch << endl;
          return;
        }

        switch(status) {
            case FileStatus::created:
                (*callback)(path_to_watch);
                break;
            case FileStatus::modified:
            //     std::cout << "File modified: " << path_to_watch << endl;
                break;
            case FileStatus::erased:
            //     std::cout << "File erased: " << path_to_watch << endl;
                break;
            default:
                std::cout << "Error! Unknown file status.\n";
        }
    });
}

void new_file_callback(std::string file_path, void (*line_cb)(std::string)) {
  std::ifstream file_stream;
  file_stream.open(file_path);
  std::string file_line;
  if ( file_stream.is_open() ) {
    while ( file_stream ) {
      std::getline (file_stream, file_line);
      // skip empty char
      if (file_line != "\n" && file_line != "\n\r" && file_line != "\r" && file_line != "")
        (*line_cb)(file_line);
    }
  }
}

// GOAL MANAGEMENT
void new_file_goal_callback(std::string file_path) {
  std::cout << "Goal File created: " << file_path << endl;
  new_file_callback(file_path, goal::manager::GoalManager::new_line_goal);
  // delete file
  std::filesystem::remove(file_path);
}

// FEEDBACK FROM ROBOT MANAGEMENT
void new_file_feedback_callback(std::string file_path) {
  // std::cout << "Feedback File created: " << file_path << endl;
  new_file_callback(file_path, feedback::manager::FeedbackManager::new_line_feedback);
  // delete file
  std::filesystem::remove(file_path);
}

// OTHERS FILE MANAGEMENT
void create_other_test(std::string path_to_watch) {
  std::cout << "Test File created: " << path_to_watch << endl;
}

/**** PUBLIC ****/
void poc_file_watcher::run_watch_files() {
  auto goal = [&]()
  {
      std::string input_folder_goals = string(INPUT_FOLDER_BASE_PATH) + INPUT_FOLDER_FROM_USER_GOALS;
      std::cout << "Watching goals from folder: " << input_folder_goals << endl;
      std::cout << endl;
      watch_file_fn(input_folder_goals, ".*.txt$", new_file_goal_callback);
  };

  auto feedback_robots = [&](std::string robot_name)
  {
      std::string input_folder_state = string(INPUT_FOLDER_BASE_PATH) + std::regex_replace(INPUT_FOLDER_FROM_ROBOTS_FEEDBACK, std::regex(ROBOTS_PLACEHOLDER), robot_name);
      std::cout << "Watching feedback information from folder: " << input_folder_state << endl;
      std::cout << endl;
      watch_file_fn(input_folder_state, ".*.txt$", new_file_feedback_callback);
  };

  // run watching threads
  thread th1(goal);
  std::vector<thread> th_states;
  for (std::string robot_name: ROBOTS) {
    th_states.push_back(thread(feedback_robots, robot_name));
  }

  th1.detach();
  for (auto it = begin (th_states); it != end (th_states); ++it) {
    it->detach();
  }
}

