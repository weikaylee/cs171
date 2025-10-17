#include <stdlib.h>
#include <assert.h>
#include <string>
#include <iostream> 
#include <vector> 
#include <sstream>
#include <fstream>
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

struct Object {
    string fname; 
    int num_copies; 
    vector<Vertex> verts; 
    vector<Face> faces; 
}; 


/**
 * @brief Update mapping (in place) of object name to object defined by object file
 * 
 * @param filename File name of object
 * @param objname Corresponding name of object
 * @param objmap Unordered map mapping objname to object
 */
void update_objmap(string filename, string objname, unordered_map<string, Object>& objmap) {
    Object obj; 
    vector<Vertex> verts; 
    vector<Face> faces; 

    // account for one-indexing of vertices 
    Vertex null_v = {0, 0, 0}; 
    verts.push_back(null_v); 

    // read file 
    ifstream file(filename);
    string line;     
    while (getline(file, line)) {
        istringstream ss(line);
        string type;
        ss >> type; 

        if (type == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            verts.push_back({x, y, z});
        } 
        else if (type == "f") {
            int a, b, c;
            ss >> a >> b >> c;
            faces.push_back({a, b, c});
        }
    }

    obj.fname = objname;
    obj.verts = verts; 
    obj.faces = faces; 
    obj.num_copies = 0; 

    objmap[objname] =  obj; 
}


/** 
 * @brief a single transformation matrix by multipling individual 
 * transformation matrices in reverse order. 
 * 
 * @param matrices: vector of indivudal transformation matrices in the order
 * they are given in the .obj file
 * @return a single transformation matrix 
 */
Eigen::Matrix4d get_transformation_matrix(vector<Eigen::Matrix4d> matrices) {
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();

    while (!matrices.empty()) {
        Eigen::Matrix4d curr = matrices.back(); 
        res = res * curr; 
        matrices.pop_back(); 
    }
    return res;
}


/**
 * @brief transform all vertices corresponding to an object
 * 
 * @param objname The name of the object given in the input file
 * @param trans The transformation matrix from get_trans_matrix to apply 
 * @param objmap Hashmap of all objects given in the input file 
 * @return Object tranformed Object (a copy is returned; original is not changed)
 */
Object transform_obj(string objname, Eigen::Matrix4d trans, unordered_map<string, Object>& objmap) {
    Object copy = objmap[objname]; 
    vector<Vertex> verts = copy.verts; 
    vector<Face> faces = copy.faces; 

    for (int i = 1; i < int(verts.size()); i++) {
        Eigen::Vector4d v; 
        v << verts[i].v1, verts[i].v2, verts[i].v3, 1; 

        Eigen::Vector4d v_trans = trans * v; 
        Vertex v_trans_3d = {float(v_trans[0]), float(v_trans[1]), float(v_trans[2])};
        copy.verts[i] = v_trans_3d; 
    }

    objmap[objname].num_copies += 1;
    copy.num_copies = objmap[objname].num_copies;
    copy.fname = copy.fname + "_copy" + to_string(copy.num_copies); 
    return copy; 
}


/**
 * @brief Create a transformation matrix from params
 * 
 * @param first Type of transformation
 * @param x param 
 * @param y param
 * @param z param
 * @param theta param (for rotation matrix only)
 * @return Eigen::Matrix4d Transfromation matrix (4x4)
 */
Eigen::Matrix4d get_matrix(string first, float x, float y, float z, float theta) {
    Eigen::Matrix4d m; 
    if (first == "t") {
        // create translation matrix 
        m << 1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;
    }
    else if (first == "r") {
        // create rotation matrix 
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
    else if (first == "s") {
        // create scaling matrix 
        m << x, 0, 0, 0, 
            0, y, 0, 0, 
            0, 0, z, 0, 
            0, 0, 0, 1;
    }
    return m; 
}


int main(int argc, char *argv[]) {
    unordered_map<string, Object> objmap; 
    vector<Object> objs;
    vector<Object> copies; 
    vector<Eigen::Matrix4d> matrices; 
    string objname; 

    ifstream file(argv[1]); 
    string line; 
    while (getline(file, line)) {
        istringstream ss(line); 
        string first; 
        ss >> first; 
        if (line.find(".obj") != string::npos) {
            string filename; 
            ss >> filename; 
            update_objmap(filename, first, objmap);
        }
        else if (first == "r" || first == "s" || first == "t") {
            float x, y, z, theta; 
            ss >> x >> y >> z >> theta; 
            Eigen::Matrix4d m = get_matrix(first, x, y, z, theta); 
            matrices.push_back(m); 
        }
        else if (line == "" && objname != "") {
            Eigen::Matrix4d trans = get_transformation_matrix(matrices); 
            Object transformed = transform_obj(objname, trans, objmap);
            copies.push_back(transformed);
        }
        else {
            // reset after finding a new object 
            objname = first; 
            matrices.clear(); 
        }
    }

    // doens't count last empty line, so need to do this again
    Eigen::Matrix4d trans = get_transformation_matrix(matrices); 
    Object transformed = transform_obj(objname, trans, objmap);
    copies.push_back(transformed);

    // print transformed vertices
    for (int i = 0; i < int(copies.size()); i++) {
        cout << copies[i].fname << ":" << endl; 
        vector<Vertex> verts = copies[i].verts;
        for (int j = 1; j < int(verts.size()); j++) {
            cout << verts[j].v1 << " " << verts[j].v2 << " " << verts[j].v3 << endl;
        }
    }  
}
