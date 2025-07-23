/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once
#include <vector>
#include <string>
#include <filesystem>
#include <utility>

void PrintToTerminal(const std::string str);
std::vector<std::pair<std::string, std::vector<double>>> ReadCSV(
    const std::filesystem::path &path);
void PrintToCSV(
    std::filesystem::path path,
    const std::vector<std::pair<std::string, std::vector<double>>> &data,
    bool print_message = false);
std::vector<std::pair<std::string, std::vector<double>>>
PackVectorsToNameValuePairs(const std::vector<std::string> &names,
                            const std::vector<std::vector<double>> &vectors);
void ListFilesInDirectory(const std::filesystem::path &path);
void PrintCurrentPath();
std::string WhatsTheTimeString();
PreciseSeconds WhatsTheTimeSeconds();
