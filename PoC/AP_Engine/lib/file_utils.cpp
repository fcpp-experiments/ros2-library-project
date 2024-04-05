// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#include "file_utils.hpp"

void create_folder_if_not_exists(std::string path) {
    struct stat st = {0};
    if (stat(path.c_str(), &st) == -1) {
        std::filesystem::path p = path.c_str();
        std::cout <<  "Create path: " << std::filesystem::absolute(p).lexically_normal().c_str() << std::endl;
        std::filesystem::create_directories(std::filesystem::absolute(p).lexically_normal().c_str());
    }
}
