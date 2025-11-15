#include "quaternion.hpp" 

// struct Quaternion {
//     float s; 
//     Eigen::Vector3d v; 
// };

Quaternion normalize(Quaternion q) {
    float magnitude = sqrt(q.s * q.s + q.v.dot(q.v));
    
    // Divide each component by the magnitude
    float new_s = q.s / magnitude;
    Eigen::Vector3d new_v = q.v / magnitude;
    
    return Quaternion{new_s, new_v};
}

Quaternion multiply(Quaternion q1, Quaternion q2) {
    float new_s = q1.s * q2.s - (q1.v).dot(q2.v); 
    Eigen::Vector3d new_v = q1.s * q2.v + q2.s * q1.v + (q1.v).cross(q2.v); 

    Quaternion new_q = {new_s, new_v}; 
    return new_q; 
}

Eigen::Matrix4d get_rotation_matrix(Quaternion q) {
    Eigen::Matrix4d m; 
    float x = q.v[0];
    float y = q.v[1]; 
    float z = q.v[2];
    float s = q.s; 

    float v11 = 1 - 2 * y * y - 2 * z * z; 
    float v12 = 2 * (x * y - z * s); 
    float v13 = 2 * (x * z + y * s); 
    float v21 = 2 * (x * y + z * s); 
    float v22 = 1 - 2 * x * x - 2 * z * z; 
    float v23 = 2 * (y * z - x * s); 
    float v31 = 2 * (x * z - y * s); 
    float v32 = 2 * (y * z + x * s) ;
    float v33 = 1 - 2 * x * x - 2 * y * y; 

    m << v11, v12, v13, 0, 
    v21, v22, v23, 0, 
    v31, v32, v33, 0, 
    0, 0, 0, 1; 

    return m; 
}

Quaternion get_identity_quaternion() { 
    Eigen::Vector3d v; 
    v << 0, 0, 0; 

    return Quaternion{1, v};
}
