// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <filesystem>
#include <sys/stat.h>
#include <fstream>
#include <string>
#include <iostream>

void create_folder_if_not_exists(std::string path);

#endif
