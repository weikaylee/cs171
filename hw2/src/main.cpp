#include <stdlib.h>
#include <string>
#include <iostream> 
#include <vector> 
#include <sstream>
#include <fstream>
#include <unordered_map> 
#include <Eigen/Dense>

#include "object.hpp"
#include "rasterization.hpp"
#include "transform.hpp"

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

vector<Object> get_objects(ifstream& file) { 
    unordered_map<string, Object> objmap; 
    vector<Object> objs;
    vector<Object> copies; 
    vector<Eigen::Matrix4d> matrices; 
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

bool in_cube(float x, float y) { 
    return (x >= -1.0 && x <= 1.0) && (y >= -1.0 && y <= 1.0);
}

/**
 * @brief Convert from cartesian ndc to screen coords. return (-1, -1) if invalid point
 * NOTE THAT Y GROWS DOWNWARDS!!!
 * 
 * @param copies vector of objects in cartesian ndc space 
 * @param xres num cols
 * @param yres num rows 
 */
void get_screen_coords(vector<Object>& copies, int xres, int yres) { 
    for (int i = 0; i < int(copies.size()); i++) {
        vector<Vertex> verts = copies[i].verts; 
        vector<Screen> screen_coords; 

        // account for one-indexing of vertices 
        Screen null_screen = {-1, -1}; 
        screen_coords.push_back(null_screen);
        
        for (int j = 1; j < int(verts.size()); j++) {
            int new_v1 = int((verts[j].v1 + 1) / 2 * xres); 
            int new_v2 = yres - int((verts[j].v2 + 1) / 2 * yres);
            if (!in_cube(verts[j].v1, verts[j].v2)) { 
                new_v1 = -1; 
                new_v2 = -1; 
            }    
            Screen coord = {new_v1, new_v2}; 
            screen_coords.push_back(coord); 
        }
        copies[i].screen_coords = screen_coords;
    } 
}

tuple<Eigen::Matrix4d, Eigen::Matrix4d> get_camera(ifstream& file) { 
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

    return make_tuple(world_to_camera, homo_ndc); 
}

void get_ppm(int xres, int yres, vector<vector<int>> grid) { 
    cout << "P3" << endl;
    cout << xres << " " << yres << endl;
    cout << "255" << endl;

    string color_bg = "0 0 0";       
    string color = "255 255 255"; 
    for (int i = 0; i < yres; i++) {        
        for (int j = 0; j < xres; j++) {
            if (grid[i][j] == 1) {
                cout << color << endl;
            } 
            else {
                cout << color_bg << endl;
            }
        }
    }
}

int main(int argc, char *argv[]) {
    ifstream file(string("../data/") + argv[1]);
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);

    Eigen::Matrix4d world_to_camera, homo_ndc; 
    vector<Object> copies;  
    string line; 
    while (getline(file, line)) {
        if (line == "camera:") { 
            tuple<Eigen::Matrix4d, Eigen::Matrix4d> matrix_tuple = get_camera(file); 
            world_to_camera = get<0>(matrix_tuple);
            homo_ndc = get<1>(matrix_tuple); 
        }
        else if (line == "objects:") { 
            copies = get_objects(file); 
        }
    }

    transform_to_ndc(copies, world_to_camera, homo_ndc); 
    get_screen_coords(copies, xres, yres);

    vector<vector<int>> grid(yres, vector<int>(xres, 0));
    for (int i = 0; i < int(copies.size()); i++) { 
        vector<Screen> screens = copies[i].screen_coords;

        vector<Face> faces = copies[i].faces; 

        for (int j = 0; j < int(faces.size()); j++) {
            int idx1 = faces[j].f1; 
            int idx2 = faces[j].f2; 
            int idx3 = faces[j].f3; 

            bresenham(screens[idx1].x, screens[idx1].y,screens[idx2].x, screens[idx2].y, xres, yres, grid);
            bresenham(screens[idx1].x, screens[idx1].y,screens[idx3].x, screens[idx3].y,  xres, yres, grid);
            bresenham(screens[idx2].x, screens[idx2].y,screens[idx3].x, screens[idx3].y,  xres, yres, grid);
        }
    }

    get_ppm(xres, yres, grid); 
}
