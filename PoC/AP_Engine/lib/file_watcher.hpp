// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#pragma once

#include <filesystem>
#include <chrono>
#include <thread>
#include <string>
#include <functional>
#include <map>
#include <unordered_map>
#include "file_utils.hpp"

// Define available file changes
enum class FileStatus {created, modified, erased};

class FileWatcher {
public:
    std::string path_to_watch;
    // Time interval at which we check the base folder for changes
    std::chrono::duration<int, std::milli> delay;

    // Keep a record of files from the base directory and their last modification time
    FileWatcher(std::string path_to_watch, std::chrono::duration<int, std::milli> delay) : path_to_watch{path_to_watch}, delay{delay} {
        create_folder_if_not_exists(path_to_watch);

        for(auto &file : std::filesystem::recursive_directory_iterator(path_to_watch)) {
            paths_[file.path().string()] = std::filesystem::last_write_time(file);
        }
    }

    // Monitor "path_to_watch" for changes and in case of a change execute the user supplied "action" function
    void start(const std::function<void (std::string, FileStatus)> &action) {
        while(running_) {
            try {
                // Wait for "delay" milliseconds
                std::this_thread::sleep_for(delay);

                auto it = paths_.begin();
                while (it != paths_.end()) {
                    if (!std::filesystem::exists(it->first)) {
                        action(it->first, FileStatus::erased);
                        it = paths_.erase(it);
                    }
                    else {
                        it++;
                    }                    
                }

                // Check if a file was created or modified
                // use sort map by file_time_type to order files
                std::map<std::filesystem::file_time_type, std::filesystem::directory_entry> files_sort_by_time;
                for(auto &file : std::filesystem::recursive_directory_iterator(path_to_watch)) {
                    auto current_file_last_write_time = std::filesystem::last_write_time(file);
                    files_sort_by_time[current_file_last_write_time] = file;
                }
                // loop on ordered files
                for(auto const &[time, file] : files_sort_by_time) {
                    // File creation
                    if(!contains(file.path().string())) {
                        paths_[file.path().string()] = time;
                        action(file.path().string(), FileStatus::created);
                    // File modification
                    } else {
                        if(paths_[file.path().string()] != time) {
                            paths_[file.path().string()] = time;
                            action(file.path().string(), FileStatus::modified);
                        }
                    }
                }
                // clear paths to avoid memory leak
                paths_.clear();
            }
            catch(const std::filesystem::filesystem_error& ex) {
                std::cerr << "warning: filesystem error, " << ex.what() << std::endl;
                std::cerr << "error code: " << ex.code() << std::endl;
            }
        }
    }
private:
    std::unordered_map<std::string, std::filesystem::file_time_type> paths_;
    bool running_ = true;

    // Check if "paths_" contains a given key
    bool contains(const std::string &key) {
        auto el = paths_.find(key);
        return el != paths_.end();
    }
};
