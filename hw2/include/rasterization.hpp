#ifndef RASTERIZATION_H 
#define RASTERIZATION_H 

#include <vector>
#include "image.hpp"

using namespace std; 

void bresenham(int x1, int y1, int x2, int y2, int xres, int yres, vector<vector<int>>& grid);

void gouraud_shading(Image& img);

void phong_shading(Image& img);

#endif