#include <stdlib.h>
#include <assert.h>
#include <string>
#include <iostream> 
#include <vector> 
#include <sstream>
#include <fstream>
#include <Eigen/Dense>

using namespace std; 

int main(int argc, char *argv[]) {
    vector<Eigen::Matrix4d> matrices; 
    ifstream file(argv[1]);
    string line; 

    while (getline(file, line)) {
        istringstream ss(line);
        string type;
        ss >> type; 
        
        Eigen::Matrix4d m; 
        float x, y, z;
        ss >> x >> y >> z;
        if (type == "t") {
            // create translation matrix 
            m << 1, 0, 0, x,
                0, 1, 0, y,
                0, 0, 1, z,
                0, 0, 0, 1;
        }
        else if (type == "r") {
            // create rotation matrix 
            float theta;
            ss >> theta;

            float v11 = x * x + (1 - x * x) * cos(theta);
            float v12 = x * y * (1 - cos(theta)) - z * sin(theta); 
            float v13 = x * z * (1 - cos(theta)) + y * sin(theta); 
            
            float v21 = y * x * (1 - cos(theta)) + z * sin(theta); 
            float v22 = y * y + (1 - y * y) * cos(theta);
            float v23 = y * z * (1 - cos(theta)) - x  * (sin(theta)); 

            float v31 = z * x * (1 - cos(theta)) - y * sin(theta); 
            float v32 = z * y * (1 - cos(theta)) + x * sin(theta); 
            float v33 = z * z + (1 - z * z) * cos(theta);

            m << v11, v12, v13, 0,
                v21, v22, v23, 0,
                v31, v32, v33, 0,
                0, 0, 0, 1;
        }
        else {
            // create scaling matrix 
            m << x, 0, 0, 0, 
                0, y, 0, 0, 
                0, 0, z, 0, 
                0, 0, 0, 1;
        }
        matrices.push_back(m);
    }

    // left multiply all matrices 
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();

    while (!matrices.empty()) {
        Eigen::Matrix4d curr = matrices.back(); 
        res = res * curr; 
        matrices.pop_back(); 
    }

    Eigen::Matrix4d inv_res = res.inverse(); 

    // iterate and clean up values close to zero
    for (int i = 0; i < res.rows(); i++) {
        for (int j = 0; j < res.cols(); j++) {
            if (abs(inv_res(i, j)) < 1e-12) {
                inv_res(i, j) = 0.0;
            }
        }
    }

    cout << inv_res << endl; 
}
