#include <stdlib.h>
#include <string>
#include <iostream> 
#include <algorithm>
#include <cmath>
#include <vector> 
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <limits> 
#include <Eigen/Dense>

#include "object.hpp"
#include "rasterization.hpp"
#include "transform.hpp"
#include "scene.hpp"
#include "image.hpp"

/**
 * @brief transform all vertices corresponding to an object
 * 
 * @param objname The name of the object given in the input file
 * @param with_trans The transformation matrix from get_trans_matrix, with translations
 * @param without_trans The transformation matrix from get_trans_matrix, without translations
 * @param objmap Hashmap of all objects given in the input file 
 * @return Object tranformed Object (a copy is returned; original is not changed)
 */
Object transform_obj(string objname, Eigen::Matrix4d with_trans, Eigen::Matrix3d without_trans, unordered_map<string, Object>& objmap) {
    Object copy = objmap[objname]; 
    vector<Vertex> verts = copy.verts; 
    vector<Face> faces = copy.faces; 
    vector<Vertex> normals = copy.normals; 

    for (int i = 1; i < int(verts.size()); i++) {
        Eigen::Vector4d v; 
        v << verts[i].v1, verts[i].v2, verts[i].v3, 1; 

        Eigen::Vector4d v_trans = with_trans * v; 
        Vertex v_trans_3d = {float(v_trans[0]), float(v_trans[1]), float(v_trans[2])};
        copy.verts[i] = v_trans_3d; 

        Eigen::Vector3d vn; 
        vn << normals[i].v1, normals[i].v2, normals[i].v3;
        
        Eigen::Vector3d vn_trans = without_trans * vn; 
        vn_trans.normalize();
        Vertex v_trans_3d_normal = {float(vn_trans[0]), float(vn_trans[1]), float(vn_trans[2])};
        copy.normals[i] = v_trans_3d_normal; 
    }

    return copy; 
}

Eigen::Matrix4d get_matrix(string first, float x, float y, float z, float theta) {
    Eigen::Matrix4d m; 
    if (first == "t") {
        m = get_translation(x, y, z); 
    }
    else if (first == "r") {
        m = get_rotation(x, y, z, theta); 
    }
    else if (first == "s") {
        m = get_scaling(x, y, z); 
    }
    return m; 
}

// create copies of objects (copies are geometrically transformed)
vector<Object> get_objects(ifstream& file) { 
    unordered_map<string, Object> objmap; 
    vector<Object> objs;
    vector<Object> copies; 
    vector<Eigen::Matrix4d> matrices; 
    Material material; 
    string objname; 

    string line; 
    while (getline(file, line)) {
        istringstream ss(line); 
        string first; 
        ss >> first; 
        if (line.find(".obj") != string::npos) {
            string filename; 
            ss >> filename; 
            objmap[first] =  create_obj(filename, first); 
        }
        else if (first == "r" || first == "s" || first == "t") {
            float x, y, z, theta; 
            ss >> x >> y >> z >> theta; 
            Eigen::Matrix4d m = get_matrix(first, x, y, z, theta); 
            matrices.push_back(m); 
        }
        else if (line == "" && objname != "") {
            tuple<Eigen::Matrix4d, Eigen::Matrix3d> trans = get_transformation_matrix(matrices); 
            Object transformed = transform_obj(objname, get<0>(trans), get<1>(trans), objmap);
            transformed.material = material; 
            copies.push_back(transformed);
        }
        else if (first == "ambient") { 
            float r, g, b; 
            ss >> r >> g >> b; 
            material.ambient = Color{r, g, b};             
        }
        else if (first == "diffuse") { 
            float r, g, b; 
            ss >> r >> g >> b; 
            material.diffuse = Color{r, g, b};             
        }
        else if (first == "specular") { 
            float r, g, b; 
            ss >> r >> g >> b; 
            material.specular = Color{r, g, b};             
        }
        else if (first == "shininess") { 
            float k;
            ss >> k;
            material.shininess = k;            
        }
        else {
            // reset after finding a new object 
            objname = first; 
            matrices.clear(); 
        }
    }

    // doens't count last empty line, so need to do this again
    tuple<Eigen::Matrix4d, Eigen::Matrix3d> trans = get_transformation_matrix(matrices); 
    Object transformed = transform_obj(objname, get<0>(trans), get<1>(trans), objmap);
    transformed.material = material; 
    copies.push_back(transformed);

    return copies; 
}

/**
 * @brief transforms vertices of all copied objects (after they are geometrically
 * transformed) into cartesian ndc
 * 
 * @param copies vector of copied objects after geo transform
 * @param world_to_camera matrix to convert from world to camera space
 * @param homo_ndc matrix to convert from camera to homo ndc space
 */
void transform_to_ndc(vector<Object>& copies, Eigen::Matrix4d world_to_camera, Eigen::Matrix4d homo_ndc) { 
    for (int i = 0; i < int(copies.size()); i++) {
        vector<Vertex>& verts = copies[i].verts; 
        for (int j = 1; j < int(verts.size()); j++) {
            Eigen::Vector4d v;   
            v << verts[j].v1, verts[j].v2, verts[j].v3, 1;

            // convert to camera space 
            Eigen::Vector4d v_camera = world_to_camera * v; 
            
            // convert to homo ndc and then cartesian ndc 
            Eigen::Vector4d v_ndc = homo_ndc * v_camera; 
            float w_ndc = v_ndc[3];
            Vertex v_ndc_cartesian = {float(v_ndc[0] / w_ndc), float(v_ndc[1]  / w_ndc), float(v_ndc[2] / w_ndc)};
            verts[j] = v_ndc_cartesian; 
        }
    } 
}

tuple<Vertex, Eigen::Matrix4d, Eigen::Matrix4d> get_camera(ifstream& file) { 
    float n, f, l, r, t, b; 
    float p_x, p_y, p_z; 
    float o_x, o_y, o_z, theta; 
    string position, orientation; 

    string line; 
    while (getline(file, line)) { 
        istringstream ss(line);
        string type; 
        ss >> type; 

        if (line.empty()) { 
            break; 
        }

        if (type == "position") {
            ss >> p_x >> p_y >> p_z;
        }
        else if (type == "orientation") {
            ss >> o_x >> o_y >> o_z >> theta;
        }
        else if (type == "near") { 
            ss >> n;  
        }
        else if (type == "far") { 
            ss >> f; 
        }
        else if (type == "left") { 
            ss >> l; 
        }
        else if (type == "right") { 
            ss >> r; 
        }
        else if (type == "top") { 
            ss >> t; 
        }
        else if (type == "bottom") { 
            ss >> b; 
        }
    }

    Eigen::Matrix4d world_to_camera = get_world_to_camera(p_x, p_y, p_z, o_x, o_y, o_z, theta); 
    Eigen::Matrix4d homo_ndc = get_homo_ndc(n, f, l, r, t, b); 
    Vertex camera = Vertex{p_x, p_y, p_z};
    return make_tuple(camera, world_to_camera, homo_ndc); 
}

vector<Light> get_lights(ifstream& file) { 
    vector<Light> lights; 
    string line; 
    while (getline(file, line)) {
        if (line.empty()) { 
            break; 
        }
        
        istringstream ss(line); 
        float x, y, z, k;
        float r, g, b; 
        char comma; 
        string token; 
        ss >> token >> x >> y >> z >> comma >> r >> g >> b >> comma >> k;
        Light light  = {Vertex{x, y, z}, Color{r, g, b}, k}; 
        lights.push_back(light); 
    }
    return lights;    
}

int main(int argc, char *argv[]) {
    ifstream file(argv[1]);
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);
    int mode = atoi(argv[4]);

    Scene scene; 
    string line; 
    while (getline(file, line)) {
        if (line == "camera:") { 
            tuple<Vertex, Eigen::Matrix4d, Eigen::Matrix4d> matrix_tuple = get_camera(file); 
            scene.camera = get<0>(matrix_tuple);
            scene.world_to_camera = get<1>(matrix_tuple);
            scene.camera_to_ndc = get<2>(matrix_tuple); 
        }
        else if (line == "objects:") { 
            scene.copies = get_objects(file); // already transforms everything (normals + vertices)
        }
        else if (line.find("light") != string::npos) {
            vector<Light> lights; 

            istringstream ss(line); 
            float x, y, z, k;
            float r, g, b; 
            char comma; 
            string token; 
            ss >> token >> x >> y >> z >> comma >> r >> g >> b >> comma >> k;
            Light light  = {Vertex{x, y, z}, Color{r, g, b}, k}; 
            lights.push_back(light); 

            vector<Light> all_lights = get_lights(file);
            lights.insert(lights.end(), all_lights.begin(), all_lights.end());  
            scene.lights = lights;
        }
    }

    // create image 
    vector<vector<Color>> grid(yres, vector<Color>(xres, Color{0, 0, 0}));
    vector<vector<float>> buffer(yres, vector<float>(xres, numeric_limits<float>::max()));
    Image img = Image{xres, yres, buffer, grid, scene};

    // rasterize all objects in scene in img
    if (mode == 0) {
        gouraud_shading(img); 
    }
    else if (mode == 1) { 
        phong_shading(img); 
    }
    
    get_ppm(xres, yres, img.grid); 
}
