#include <stdlib.h>
#include <assert.h>
#include <string>
#include <iostream> 
#include <vector> 
#include <sstream>
#include <fstream>

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
    vector<Vertex> verts; 
    vector<Face> faces; 
}; 

int main(int argc, char *argv[]) {
    vector<Object> objs;
    
    // iterate through each input, store in objs 
    for (int i = 1; i < argc; i++) {
        Object obj; 
        vector<Vertex> verts; 
        vector<Face> faces; 

        // account for one-indexing of vertices 
        Vertex null_v = {0, 0, 0}; 
        verts.push_back(null_v); 

        // read file 
        ifstream file(argv[i]);
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

        string fname = argv[i];
        obj.fname = fname.substr(0, fname.length() - 4);
        obj.verts = verts; 
        obj.faces = faces; 
        objs.push_back(obj);
    }

    // print out objects 
    for (int i = 0; i < int(objs.size()); i++) {
        cout << objs[i].fname << ":" << endl; 

        vector<Vertex> verts = objs[i].verts;
        // account for one-indexing here
        for (int j = 1; j < int(verts.size()); j++) {
            cout << "v " << verts[j].v1 << " " << verts[j].v2 << " " << verts[j].v3 << endl;
        }

        vector<Face> faces = objs[i].faces; 
        for (int j = 0; j < int(faces.size()); j++) {
            cout << "f " << faces[j].f1 << " " << faces[j].f2 << " " << faces[j].f3 << endl;
        }
    }  
}

