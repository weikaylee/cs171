#ifndef IMAGE_H 
#define IMAGE_H

#include "object.hpp"
#include "scene.hpp"

struct Image { 
    int xres; 
    int yres; 
    vector<vector<float>> buffer; 
    vector<vector<Color>> grid; 
    Scene scene;
};

void get_ppm(int xres, int yres, vector<vector<Color>> grid);

#endif