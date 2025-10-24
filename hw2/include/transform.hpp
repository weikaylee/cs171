#ifndef TRANSFORM_H 
#define TRANSFORM_H 

#include <Eigen/Dense>
#include <vector>

#include "object.hpp"

using namespace std; 

Eigen::Matrix4d get_translation(float x, float y, float z);

Eigen::Matrix4d get_rotation(float x, float y, float z, float theta);

Eigen::Matrix4d get_scaling(float x, float y, float z);

/** 
 * @brief get a single transformation matrix by multipling individual 
 * transformation matrices in reverse order. 
 * 
 * @param matrices: vector of indivudal transformation matrices in the order
 * they are given in the .obj file
 * @return a tuple consisting of (a single transformation matrix with all 
 * transofrmations applied, same but without translation transforms) 
 */
tuple<Eigen::Matrix4d, Eigen::Matrix3d> get_transformation_matrix(vector<Eigen::Matrix4d> matrices);

/**
 * @brief Get matrix for converting from world coords to camera coords
 * 
 * @param p_x position x
 * @param p_y position y
 * @param p_z position z
 * @param o_x orientation x
 * @param o_y orientation y
 * @param o_z orientation z
 * @param theta orientation theta 
 * @return Eigen::Matrix4d matrix to convert from world to camera coords 
 */
Eigen::Matrix4d get_world_to_camera(float p_x, float p_y, float p_z, float o_x, float o_y, float o_z, float theta);

/**
 * @brief Get matrix for converting from camera to homogenous ndc coords
 * 
 * @param n near
 * @param f far
 * @param l left
 * @param r right
 * @param t top
 * @param b bottom
 * @return Eigen::Matrix4d matrix to convert from camera to homogenous nds coords 
 */
Eigen::Matrix4d get_homo_ndc(float n, float f, float l, float r, float t, float b);

Vertex world_to_ndc(Vertex vert, Eigen::Matrix4d world_to_camera, Eigen::Matrix4d camera_to_ndc);

Screen ndc_to_screen(Vertex vert, int xres, int yres);

bool in_cube(float x, float y, float z);

#endif
