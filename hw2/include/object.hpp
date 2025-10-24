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
    int v1; 
    int v2; 
    int v3;  
    int n1; 
    int n2; 
    int n3; 
}; 

struct Screen {
    int x; 
    int y;
}; 

struct Color { 
    float r; 
    float g; 
    float b; 
};

struct Material { 
    Color ambient; 
    Color diffuse; 
    Color specular; 
    float shininess;  
};

struct Light {
    Vertex position; 
    Color color; 
    float k; 
};

struct Object {
    string fname; 
    vector<Vertex> verts; 
    vector<Face> faces; 
    vector<Vertex> normals; 
    Material material; 
}; 

/**
 * @brief Create an object
 * 
 * @param filename File name of object
 * @param objname Corresponding name of object
 * @param objmap Unordered map mapping objname to object
 */
Object create_obj(string filename, string objname);

Eigen::Vector3d to_vector3d(Vertex vert);

Eigen::Vector3d to_vector3d(Color color);


#endif
