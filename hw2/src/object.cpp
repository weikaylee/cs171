#include <fstream> 
#include <iostream>

#include "object.hpp" 

Object create_obj(string filename, string objname) {
    Object obj; 
    vector<Vertex> verts; 
    vector<Face> faces; 
    vector<Vertex> normals; 

    // account for one-indexing of vertices 
    Vertex null_v = {0, 0, 0};
    verts.push_back(null_v);
    normals.push_back(null_v);

    // read file 
    ifstream file("./data/" + filename);
    string line;     
    while (getline(file, line)) {
        istringstream ss(line);
        string type;
        ss >> type; 

        if (type == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            verts.push_back(Vertex{x, y, z});
        } 
        else if (type == "f") {
            vector<int> idxs; 
            string token; 
            while (ss >> token) { 
                // replace // with spaces 
                for (char&c : token) { 
                    if (c == '/') {
                        c = ' ';
                    }
                }

                stringstream subtok(token); 
                int value; 
                while (subtok >> value) { 
                    idxs.push_back(value); 
                }
            }
            faces.push_back(Face{idxs[0], idxs[2], idxs[4], idxs[1], idxs[3], idxs[5]});
        }
        else if (type == "vn") {
            float x, y, z;
            ss >> x >> y >> z;
            normals.push_back(Vertex{x, y, z});
        }
    }

    obj.fname = objname;
    obj.verts = verts; 
    obj.faces = faces; 
    obj.normals = normals; 

    return obj; 
}

Eigen::Vector3d to_vector3d(Vertex vert) {
    return Eigen::Vector3d(vert.v1, vert.v2, vert.v3);
}

// Overload 2: Convert a Color to Eigen::Vector3d
Eigen::Vector3d to_vector3d(Color color) {
    return Eigen::Vector3d(color.r, color.g, color.b);
}
