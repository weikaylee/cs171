#ifndef QUATERNION_H 
#define QUATERNION_H

#include <Eigen/Dense>

struct Quaternion {
    float s; 
    Eigen::Vector3d v; 
};

Quaternion multiply(Quaternion q1, Quaternion q2); 

Eigen::Matrix4d get_rotation_matrix(Quaternion q); 

Quaternion get_identity_quaternion();

#endif