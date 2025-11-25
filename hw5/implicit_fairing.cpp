#include "implicit_fairing.hpp"

using namespace std;

// helper funcs 
Eigen::Vector3f hev_to_vec(HEV *hev) {
    return Eigen::Vector3f{hev->x, hev->y, hev->z};
}

void index_vertices(vector<HEV*> *hevs) {
    for(int i = 1; i < hevs->size(); i++) // start at 1 because obj files are 1-indexed
        hevs->at(i)->index = i; // assign each vertex an index
}

float calc_cot(Eigen::Vector3f v1, Eigen::Vector3f v2) {
    return v1.dot(v2) / v1.cross(v2).norm();
}

// main funcs 
Eigen::Vector3f calc_face_normal(HEF *face) {
    Eigen::Vector3f v1 = hev_to_vec(face->edge->vertex);
    Eigen::Vector3f v2 = hev_to_vec(face->edge->next->vertex);
    Eigen::Vector3f v3 = hev_to_vec(face->edge->next->next->vertex);
    return (v2 - v1).cross(v3 - v1);
}

Vec3f calc_vertex_normal(HEV *v) {
    Eigen::Vector3f normal{0, 0, 0};
    HE *he = v->out;
    do {
        // compute face normal 
        HEF *f = he->face; 
        Eigen::Vector3f face_normal = calc_face_normal(f);
        float face_area = 0.5 * face_normal.norm(); 
        normal += face_normal * face_area; 

        he = he->flip->next;
    } while (he != v->out);

    return Vec3f{normal(0), normal(1), normal(2)};
}


Eigen::SparseMatrix<float> build_F_operator(vector<HEV*> *hevs, float h) {
    index_vertices(hevs); // assign each vertex an index

    // recall that due to 1-indexing of obj files, index 0 of our list doesnt actually contain a vertex
    int num_vertices = hevs->size() - 1;

    Eigen::SparseMatrix<float> F(num_vertices, num_vertices);
    F.reserve( Eigen::VectorXi::Constant( num_vertices, 7 ) );

    for (int i = 1; i < hevs->size(); i++) {
        HEV *hev_i = hevs->at(i); 

        // calculate neighbor area sum A
        float A = 0;
        HE *he = hev_i->out;
        do {
            HEF *f = he->face; 
            Eigen::Vector3f face_normal = calc_face_normal(f);
            float face_area = 0.5 * face_normal.norm(); 
            A += face_area; 

            he = he->flip->next;
        } while (he != hev_i->out);

        // handle degenerate region 
        if (A < 1e-8f) {
            continue;
        }
        
        // get vertices adjacent to v_i
        Eigen::Vector3f v_i = hev_to_vec(hev_i);
        he = hev_i->out;
        do {
            HEV *hev_j = he->next->vertex;  
            HEV *hev_k = he->next->next->vertex;
            HEV *hev_l = he->flip->next->next->vertex;

            Eigen::Vector3f v_j = hev_to_vec(hev_j);
            Eigen::Vector3f v_k = hev_to_vec(hev_k);
            Eigen::Vector3f v_l = hev_to_vec(hev_l);

            float cot_a = calc_cot(v_j - v_k, v_i - v_k);
            float cot_b = calc_cot(v_j - v_l, v_i - v_l);

            int j = hev_j->index;
            F.insert(i - 1, j - 1) = (cot_a + cot_b) / (2 * A);

            he = he->flip->next;
        } while (he != hev_i->out);

        F.coeffRef(i - 1, i - 1) -= F.row(i - 1).sum(); // laplacian property
    }

    Eigen::SparseMatrix<float> I(num_vertices, num_vertices); 
    I.setIdentity(); 
    F = I - h * F;
    F.makeCompressed();

    return F;
}
