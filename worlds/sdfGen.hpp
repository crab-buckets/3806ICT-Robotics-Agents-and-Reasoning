#ifndef SDF_GENERATOR_HPP
#define SDF_GENERATOR_HPP

#include <vector>
#include <string>
#include <fstream>
std::string generateSDF(const std::vector<std::vector<char>>& grid, char wall = '#',
char goal = 'G', char openSpace = ' ');

#endif