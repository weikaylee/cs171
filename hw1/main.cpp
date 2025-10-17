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

struct Screen { 
    int x; 
    int y; 
}; 

struct Object {
    string fname; 
    vector<Vertex> verts; 
    vector<Face> faces; 
    vector<Screen> screen_coords; 
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

Eigen::Matrix4d get_world_to_camera(string position, string orientation) { 
    // create translation matrix for world 
    istringstream ss0(position); 
    string discard0; 
    int x0, y0, z0; 
    ss0 >> discard0 >> x0 >> y0 >> z0;

    Eigen::Matrix4d translate;
    translate << 1, 0, 0, x0,
        0, 1, 0, y0,
        0, 0, 1, z0,
        0, 0, 0, 1;

    // create rotation matrix for camera 
    istringstream ss(orientation);
    string discard; 
    int x, y, z, theta; 
    ss >> discard >> x >> y >> z >> theta; 

    Eigen::Matrix4d rotate;
    float v11 = x * x + (1 - x * x) * cos(theta);
    float v12 = x * y * (1 - cos(theta)) - z * sin(theta); 
    float v13 = x * z * (1 - cos(theta)) + y * sin(theta); 
    
    float v21 = y * x * (1 - cos(theta)) + z * sin(theta); 
    float v22 = y * y + (1 - y * y) * cos(theta);
    float v23 = y * z * (1 - cos(theta)) - x  * (sin(theta)); 

    float v31 = z * x * (1 - cos(theta)) - y * sin(theta); 
    float v32 = z * y * (1 - cos(theta)) + x * sin(theta); 
    float v33 = z * z + (1 - z * z) * cos(theta);

    rotate << v11, v12, v13, 0,
        v21, v22, v23, 0,
        v31, v32, v33, 0,
        0, 0, 0, 1;
    
    Eigen::Matrix4d camera_to_world = translate * rotate; 
    Eigen::Matrix4d world_to_camera = camera_to_world.inverse(); 
    return world_to_camera;
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

    return copies; 
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

// modifies copies in place
void transform_to_ndc(vector<Object>& copies, Eigen::Matrix4d world_to_camera, Eigen::Matrix4d homo_ndc) { 
    // cout << "TRANFORMING POINTS TO CARTESIAN NDC" << endl; 
    // cout << "matrix" << endl; 
    // cout << world_to_camera << endl; 

    // cout << "homo ndc" << endl; 
    // cout << homo_ndc << endl;

    for (int i = 0; i < int(copies.size()); i++) {
        vector<Vertex>& verts = copies[i].verts; 
        for (int j = 1; j < int(verts.size()); j++) {
            Eigen::Vector4d v; // these are already geometrically tansformed  
            v << verts[j].v1, verts[j].v2, verts[j].v3, 1;

            // convert to camera space 
            Eigen::Vector4d v_camera = world_to_camera * v; 
            // float w_ndc = v_camera[2];
            
            // convert to homo ndc and then cartesian ndc 
            Eigen::Vector4d v_ndc = homo_ndc * v_camera; 
            float w_ndc = v_ndc[3];
            Vertex v_ndc_cartesian = {float(v_ndc[0] / w_ndc), float(v_ndc[1]  / w_ndc), float(v_ndc[2] / w_ndc)};
            verts[j] = v_ndc_cartesian; 

            // cout << verts[j].v1 << " " << verts[j].v2 << " " << verts[j].v3 << endl;
        }
    } 
}

bool in_cube(float x, float y) { 
    return (x >= -1.0 && x <= 1.0) && (y >= -1.0 && y <= 1.0);
}

// converts ndc vertices to screen coordinates 
void get_screen_coords(vector<Object>& copies, int xres, int yres) { 
    for (int i = 0; i < int(copies.size()); i++) {
        vector<Vertex> verts = copies[i].verts; 
        vector<Screen> screen_coords; 
        Screen null_screen = {-1, -1}; // account for one indexing 
        screen_coords.push_back(null_screen);

        for (int j = 1; j < int(verts.size()); j++) {
            int new_v1 = int((verts[j].v1 + 1) / 2 * xres); 
            int new_v2 = yres - int((verts[j].v2 + 1) / 2 * yres); // Y GROWS DOWNWARDS!!!!!
            if (!in_cube(verts[j].v1, verts[j].v2)) { 
                new_v1 = -1; 
                new_v2 = -1; // return -1, -1 if point not in cube!!!
            }    
            Screen coord = {new_v1, new_v2}; 
            screen_coords.push_back(coord); 
        }
        copies[i].screen_coords = screen_coords;
    } 
}

tuple<Eigen::Matrix4d, Eigen::Matrix4d> get_camera(ifstream& file) { 
    float n, f, l, r, t, b; 
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
            position = line; 
        }
        else if (type == "orientation") {
            orientation = line; 
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

    // get stuff 
    Eigen::Matrix4d world_to_camera = get_world_to_camera(position, orientation); 
    Eigen::Matrix4d homo_ndc = get_homo_ndc(n, f, l, r, t, b); 

    return make_tuple(world_to_camera, homo_ndc); 
}

// return a list of points that should be colored 

// Works for first and eigth octants.
void low_slope_bresenham(int x1, int y1, int x2, int y2, int xres, int yres,
                        vector<vector<int>>& grid) {
    int eps = 0;
    int y = y1;
    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    for (size_t x = x1; x <= x2; x++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) { // ignore anythign off screeen
            grid[y][x] = 1;
        }

        if (2 * (eps + dy) < dx) {
            eps += dy;
        }
        else {
            eps += dy - dx;
            y += (y2 >= y1) ? 1 : -1;
        }
    }
}

// Works for 2nd and 7th octants.
void high_slope_bresenham(int x1, int y1, int x2, int y2, int xres, int yres,
                        vector<vector<int>>& grid) {
    int eps = 0;
    int x = x1;
    int dx = abs(x2 - x1);
    int dy = y2 - y1;

    for (size_t y = y1; y <= y2; y++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            grid[y][x] = 1;
        }

        if (2 * (eps + dx) < dy) {
            eps += dx;
        }
        else {
            eps += dx - dy;
            x += (x2 >= x1) ? 1 : -1;
        }
    }
}

// make different classes for scenes, for rendering stuff (drawing lines), for porcessing objects
void bresenham(int x1, int y1, int x2, int y2,int xres, int yres,
                    vector<vector<int>>& grid) {
    if (x1 == -1 || x2 == -1) {
        return;
    }

    if (abs(y2 - y1) <= abs(x2 - x1)) {
        if (x1 <= x2) {
            low_slope_bresenham(x1, y1, x2, y2, xres, yres,grid);
        }
        else {
            low_slope_bresenham(x2, y2, x1, y1,  xres,  yres,grid);
        }
    }
    else {
        if (y1 <= y2) {
            high_slope_bresenham(x1, y1, x2, y2,  xres,  yres,grid);
        }
        else {
            high_slope_bresenham(x2, y2, x1, y1, xres,  yres, grid);
        }
    }
}

int main(int argc, char *argv[]) {
    ifstream file(argv[1]);
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

    // get screen coordinates for each object 
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

    // iterate over every pixel
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
