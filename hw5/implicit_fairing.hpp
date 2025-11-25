#ifndef IMPLICIT_FAIRING_H 
#define IMPLICIT_FAIRING_H

#include "structs.h"
#include "halfedge.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace std; 

Eigen::Vector3f calc_face_normal(HEF *face);
Vec3f calc_vertex_normal(HEV *vertex);

Eigen::SparseMatrix<float> build_F_operator(vector<HEV*> *hevs, float h);

#endif 