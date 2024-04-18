// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file action_writer.cpp
 * @brief Writer of the action sent from AP
 * 
 */

#include "action_writer.hpp"

/**** PRIVATE ****/

/**** PUBLIC ****/
void action::manager::ActionManager::new_action(action::ActionData data) {
  std::cout << "Action sent to robots and to web: "   << data.robot   << endl;
  std::string output_folder_action_to_robots = string(OUTPUT_FOLDER_BASE_PATH) + std::regex_replace(OUTPUT_FOLDER_TO_ROBOTS_ACTION, std::regex(ROBOTS_PLACEHOLDER), data.robot);

  auto now = std::chrono::high_resolution_clock::now();
  auto duration = now.time_since_epoch();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

  std::stringstream ss;
  ss << "action-" << milliseconds.count() << "-reliable.txt";
  std::string filename = ss.str();
  // set milliseconds to data
  data.milliseconds = milliseconds;

  action::writer::ActionWriter::write_dto(data, output_folder_action_to_robots, filename);
}

void action::writer::ActionWriter::write_dto(action::ActionData data, string path, string filename) {
  create_folder_if_not_exists(path);
  
  int maxAttempts = 5;

  std::ofstream file;

  for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
      std::string uniqueFilename = path + "/" + filename + "_" + std::to_string(attempt) + ".lock";

      file.open(uniqueFilename, std::ios::app);

      if (file.is_open()) {
          file  << data.action                  << COLUMN_DELIMITER 
                << data.goal_code               << COLUMN_DELIMITER 
                << data.robot                   << COLUMN_DELIMITER 
                << data.pos_start_x             << COLUMN_DELIMITER
                << data.pos_start_y             << COLUMN_DELIMITER
                << data.orient_start_w          << COLUMN_DELIMITER
                << data.pos_end_x               << COLUMN_DELIMITER
                << data.pos_end_y               << COLUMN_DELIMITER
                << data.orient_end_w            << COLUMN_DELIMITER
                << data.goal_step               << COLUMN_DELIMITER
                << data.milliseconds.count()  
                << std::endl;

          file.close();

          if (std::rename(uniqueFilename.c_str(), (path + "/" + filename).c_str()) == 0) {
              std::cout << "Attempt #" << attempt << " writing file " << (path + "/" + filename).c_str() << " is ok" << std::endl;
              break;  
          } else {
              std::cout << "Error renaming action file. Attempt #" << attempt << std::endl;
          }
      } else {
          std::cout << "Error opening file. Attempt #" << attempt << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
