#include "transform.hpp"

Eigen::Matrix4d get_translation(float x, float y, float z) {
    Eigen::Matrix4d m;
    m << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d get_rotation(float x, float y, float z, float theta) {
    Eigen::Matrix4d m;
    float v11 = x * x + (1 - x * x) * cos(theta);
    float v12 = x * y * (1 - cos(theta)) - z * sin(theta);
    float v13 = x * z * (1 - cos(theta)) + y * sin(theta);

    float v21 = y * x * (1 - cos(theta)) + z * sin(theta);
    float v22 = y * y + (1 - y * y) * cos(theta);
    float v23 = y * z * (1 - cos(theta)) - x * (sin(theta));

    float v31 = z * x * (1 - cos(theta)) - y * sin(theta);
    float v32 = z * y * (1 - cos(theta)) + x * sin(theta);
    float v33 = z * z + (1 - z * z) * cos(theta);

    m << v11, v12, v13, 0,
        v21, v22, v23, 0,
        v31, v32, v33, 0,
        0, 0, 0, 1;
    
    return m; 
}

Eigen::Matrix4d get_scaling(float x, float y, float z) {
    Eigen::Matrix4d m;
    m << x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d get_transformation_matrix(vector<Eigen::Matrix4d> matrices) {
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();

    while (!matrices.empty()) {
        Eigen::Matrix4d curr = matrices.back(); 
        res = res * curr; 
        matrices.pop_back(); 
    }
    return res;
}

Eigen::Matrix4d get_world_to_camera(float p_x, float p_y, float p_z, float o_x, float o_y, float o_z, float theta) {
    Eigen::Matrix4d translate = get_translation(p_x, p_y, p_z); 
    Eigen::Matrix4d rotate = get_rotation(o_x, o_y, o_z, theta); 
    
    Eigen::Matrix4d camera_to_world = translate * rotate; 
    Eigen::Matrix4d world_to_camera = camera_to_world.inverse(); 

    return world_to_camera;
}

Eigen::Matrix4d get_homo_ndc(float n, float f, float l, float r, float t, float b) {
    Eigen::Matrix4d homo_ndc; 

    float v11 = 2 * n / (r - l); 
    float v13 = (r + l) / (r - l); 
    float v22 = 2 * n / (t - b); 
    float v23 = (t + b) / (t - b); 
    float v33 = -(f + n) / (f - n); 
    float v34 = -2 * f * n / (f - n); 

    homo_ndc << v11, 0, v13, 0,
        0, v22, v23, 0,
        0, 0, v33, v34,
        0, 0, -1, 0;

    return homo_ndc;
}
