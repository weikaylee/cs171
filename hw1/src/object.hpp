#ifndef OBJECT_H 
#define OBJECT_H 

#include <string> 
#include <vector> 
#include <unordered_map>
#include <Eigen/Dense>

using namespace std; 

struct Vertex { 
    float v1; 
    float v2; 
    float v3; 
};

struct Face { 
    int f1; 
    int f2; 
    int f3;  
}; 

struct Screen { 
    int x; 
    int y; 
}; 

struct Object {
    string fname; 
    vector<Vertex> verts; 
    vector<Face> faces; 
    vector<Screen> screen_coords; 
}; 

/**
 * @brief Create an object
 * 
 * @param filename File name of object
 * @param objname Corresponding name of object
 * @param objmap Unordered map mapping objname to object
 */
Object create_obj(string filename, string objname);

#endif
