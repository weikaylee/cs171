#ifndef SCENE_H 
#define SCENE_H

#include "object.hpp"
#include <Eigen/Dense>

struct Scene { 
    Eigen::Matrix4d world_to_camera, camera_to_ndc; // ndc in camera_to_ndc is homo ndc, NOT cartesian ndc
    vector<Object> copies;  
    vector<Light> lights; 
    Vertex camera; 
};

#endif