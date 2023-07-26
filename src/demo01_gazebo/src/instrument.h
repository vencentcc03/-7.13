#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#ifndef INSTRUMENT_H
#define INSTRUMENT_H
void read_file(const std::string &filepath, std::vector<std::vector<double>> &coordinates);
bool isCoordinateExists(const std::vector<std::vector<double>> &coordinates, double x, double y);
void removeCoordinate(std::vector<std::vector<double>> &coordinates, double x, double y);
void printDoubleVector(const std::vector<std::vector<double>> &data);
void printVector(const std::vector<double> &data);
int findCoordIndex(const std::vector<std::vector<double>> &coords, double x, double y);
void RandomCoordinates(int x, int y, double m, const char *file_path);
#endif