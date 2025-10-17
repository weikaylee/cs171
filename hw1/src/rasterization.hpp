#ifndef RASTERIZATION_H 
#define RASTERIZATION_H 

#include <vector>

using namespace std; 

void bresenham(int x1, int y1, int x2, int y2, int xres, int yres, vector<vector<int>> &grid);

#endif