#ifndef SAVEDATA_H
#define SAVEDATA_H
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include "../map/point.h"

void saveToCSV(const std::vector<double>& data, const std::string& filename);
void saveLaneToCSV(const std::vector<point>& data, const std::string& filename);

#endif // SAVEDATA_H