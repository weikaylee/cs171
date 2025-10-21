#include <fstream> 
#include <iostream>

#include "object.hpp" 

// TODO include option for passing in dataroot
Object create_obj(string filename, string objname) {
    Object obj; 
    vector<Vertex> verts; 
    vector<Face> faces; 

    // account for one-indexing of vertices 
    Vertex null_v = {0, 0, 0}; 
    verts.push_back(null_v); 

    // read file 
    ifstream file("../data/" + filename);
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

    return obj; 
}
