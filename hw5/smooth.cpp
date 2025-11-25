#include <GL/glew.h>
#include <GL/glut.h>

#include <math.h>
#define _USE_MATH_DEFINES

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
#include <tuple> 
#include <Eigen/Sparse>

#include "structs.h" 
#include "halfedge.h"
#include "quaternion.hpp"
#include "implicit_fairing.hpp"

using namespace std;

void init(void);
void reshape(int width, int height);
void display(void);

void init_lights();
void set_lights();
void draw_objects();
 
void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);

struct Point_Light
{
    /* Index 0 has the x-coordinate
     * Index 1 has the y-coordinate
     * Index 2 has the z-coordinate
     * Index 3 has the w-coordinate
     */
    float position[4];
    
    /* Index 0 has the r-component
     * Index 1 has the g-component
     * Index 2 has the b-component
     */
    float color[3];
    
    /* This is our 'k' factor for attenuation as discussed in the lecture notes
     * and extra credit of Assignment 2.
     */
    float attenuation_k;
};

struct Triple
{
    float x;
    float y;
    float z;
};

struct Transform
{
    string type; // either s, r, or t
    float params[3]; 
    
    /* Angle in degrees.
     */
    float rotation_angle; // use -1 if not relevant
};

struct Object
{
    vector<HEV*> *hevs;
    vector<HEF*> *hefs;

    vector<Triple> vertex_buffer;
    vector<Triple> normal_buffer;
    
    vector<Transform> transform_sets;
    
    /* Index 0 has the r-component
     * Index 1 has the g-component
     * Index 2 has the b-component
     */
    float ambient_reflect[3];
    float diffuse_reflect[3];
    float specular_reflect[3];
    
    float shininess;
};

float cam_position[3];
float cam_orientation_axis[3];

float cam_orientation_angle;

float near_param, far_param,
      left_param, right_param,
      top_param, bottom_param;

vector<Point_Light> lights;
vector<Object> objects;

int xres, yres; 
int mouse_start_x, mouse_start_y;
int mouse_current_x, mouse_current_y;
float mouse_scale_x, mouse_scale_y;

const float step_size = 0.2;
const float x_view_step = 90.0, y_view_step = 90.0;
float x_view_angle = 0, y_view_angle = 0;

bool is_pressed = false;
bool wireframe_mode = false;

unordered_map<string, Object> objmap; // map the obj name to its file

void parse_obj_file(); 
void create_lights();
void create_objects();

Quaternion last_rotation; 
Quaternion current_rotation; 
Eigen::Matrix4d get_current_rotation();

// smoothing time step 
float h; 

void init(void)
{
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    glEnable(GL_DEPTH_TEST);
    
    glEnable(GL_NORMALIZE);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);
    glMatrixMode(GL_MODELVIEW);
    
    init_lights();

    last_rotation = get_identity_quaternion(); 
    current_rotation = get_identity_quaternion(); 
}

void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;
    glViewport(0, 0, width, height);
    
    mouse_scale_x = (float) (right_param - left_param) / (float) width;
    mouse_scale_y = (float) (top_param - bottom_param) / (float) height;
    glutPostRedisplay();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();
    glRotatef(-cam_orientation_angle * 180.0 / M_PI,  // Convert radians to degrees
              cam_orientation_axis[0], cam_orientation_axis[1], cam_orientation_axis[2]);
    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);

    // quaternion stuff 
    glMultMatrixd(get_current_rotation().data());

    set_lights();
    draw_objects();
    
    glutSwapBuffers();
}

Eigen::Vector3f screen_to_ndc(int x, int y) {
    float x_ndc = (((float) x) / xres) * 2 - 1;
    float y_ndc = (((float) (yres - y)) / yres) * 2 - 1;
    float z_ndc = 0;

    if (pow(x_ndc, 2) + pow(y_ndc, 2) <= 1) {
        z_ndc = sqrt(1 - pow(x_ndc, 2) - pow(y_ndc, 2));
    }

    return {x_ndc, y_ndc, z_ndc};
}

Quaternion compute_rotation_quaternion(float start_x, float start_y, float current_x, float current_y) {
    Eigen::Vector3f start_ndc = screen_to_ndc(start_x, start_y);
    Eigen::Vector3f current_ndc = screen_to_ndc(current_x, current_y);

    float angle = acos(min(1.f, start_ndc.dot(current_ndc) / (start_ndc.norm() * current_ndc.norm())));
    Eigen::Vector3f u = start_ndc.cross(current_ndc);
    u = u.normalized();

    float s = cos(angle / 2);
    float i = u(0) * sin(angle / 2);
    float j = u(1) * sin(angle / 2);
    float k = u(2) * sin(angle / 2);

    Eigen::Vector3d v; 
    v << i, j, k; 

    Quaternion q = Quaternion{s, v};
    
    // Normalize the quaternion
    return normalize(q);
}

Eigen::Matrix4d get_current_rotation() {
    Quaternion q = normalize(multiply(current_rotation, last_rotation));
    
    return get_rotation_matrix(q);
}

void init_lights()
{
    glEnable(GL_LIGHTING);
    
    int num_lights = lights.size();
    
    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        
        glEnable(light_id);
        
        glLightfv(light_id, GL_AMBIENT, lights[i].color);
        glLightfv(light_id, GL_DIFFUSE, lights[i].color);
        glLightfv(light_id, GL_SPECULAR, lights[i].color);
        
    }
}

void set_lights()
{
    int num_lights = lights.size();
    
    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        
        glLightfv(light_id, GL_POSITION, lights[i].position);
    }
}

void draw_objects()
{
    int num_objects = objects.size();
    
    for(int i = 0; i < num_objects; ++i)
    {
        glPushMatrix();
        {
            int num_transform_sets = objects[i].transform_sets.size();
            
            for(int j = num_transform_sets - 1; j >= 0; --j)
            {
                Transform t = objects[i].transform_sets[j]; 
                string type = t.type; 
                
                if (type == "s") { 
                    glScalef(t.params[0], t.params[1], t.params[2]);
                }
                else if (type == "t") {
                    glTranslatef(t.params[0], t.params[1], t.params[2]);
                }
                else if (type == "r") {
                    glRotatef(t.rotation_angle * 180.0 / M_PI, t.params[0], t.params[1], t.params[2]);
                }
            }
            
            glMaterialfv(GL_FRONT, GL_AMBIENT, objects[i].ambient_reflect);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, objects[i].diffuse_reflect);
            glMaterialfv(GL_FRONT, GL_SPECULAR, objects[i].specular_reflect);
            glMaterialf(GL_FRONT, GL_SHININESS, objects[i].shininess);
            
            glVertexPointer(3, GL_FLOAT, 0, &objects[i].vertex_buffer[0]);
            glNormalPointer(GL_FLOAT, 0, &objects[i].normal_buffer[0]);
            
            int buffer_size = objects[i].vertex_buffer.size();
            
            if(!wireframe_mode)
                glDrawArrays(GL_TRIANGLES, 0, buffer_size);
            else
                for(int j = 0; j < buffer_size; j += 3)
                    glDrawArrays(GL_LINE_LOOP, j, 3);
        }
        glPopMatrix();
    }
}

void mouse_pressed(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        mouse_start_x = x;
        mouse_start_y = y;
        mouse_current_x = x; 
        mouse_current_y = y; 
        
        is_pressed = true;
    }
    else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        is_pressed = false;
        last_rotation = normalize(multiply(current_rotation, last_rotation)); 
        
        current_rotation = get_identity_quaternion();  

        mouse_start_x = x; 
        mouse_start_y = y; 
    }
}

void mouse_moved(int x, int y)
{
    if(is_pressed)
    {
        mouse_current_x = x;
        mouse_current_y = y;

        current_rotation = compute_rotation_quaternion(mouse_start_x, mouse_start_y, mouse_current_x, mouse_current_y);
        
        glutPostRedisplay();
    }
}

// Constructs an n x n sparse identity matrix.
Eigen::SparseMatrix<float> get_identity_matrix(int n) {
    Eigen::SparseMatrix<float> I{n, n};

    I.reserve(Eigen::VectorXi::Constant(n, 1));

    for (int i = 0; i < n; i++) {
        I.insert(i, i) = 1.f;
    }

    I.makeCompressed();

    return I;
}

void implicit_fairing() {
    for (Object &obj : objects) {
        // Solve for new x, y, z coordinates using matrix operator F
        Eigen::SparseMatrix<float> F = build_F_operator(obj.hevs, h);
        Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>>
            solver;
        solver.analyzePattern(F);
        solver.factorize(F);

        int num_vertices = obj.hevs->size() - 1;
        Eigen::VectorXf x_0{num_vertices};
        Eigen::VectorXf y_0{num_vertices};
        Eigen::VectorXf z_0{num_vertices};

        for (int i = 1; i < obj.hevs->size(); i++) {
            HEV *hev = obj.hevs->at(i);
            x_0(i - 1) = hev->x;
            y_0(i - 1) = hev->y;
            z_0(i - 1) = hev->z;
        }

        Eigen::VectorXf x_h{num_vertices};
        Eigen::VectorXf y_h{num_vertices};
        Eigen::VectorXf z_h{num_vertices};

        x_h = solver.solve(x_0);
        y_h = solver.solve(y_0);
        z_h = solver.solve(z_0);

        // set old coords x_0 to new x_h coords 
        for (int i = 1; i < obj.hevs->size(); i++) {
            HEV *hev = obj.hevs->at(i);
            hev->x = x_h(i - 1);
            hev->y = y_h(i - 1);
            hev->z = z_h(i - 1);
        }

        // recompute vertex normals 
        obj.vertex_buffer.clear();
        obj.normal_buffer.clear();

        for (HEF *hef : *obj.hefs) {
            HE *he = hef->edge;

            for (int i = 0; i < 3; i++) {
                HEV *hev = he->vertex;

                Vec3f n = calc_vertex_normal(hev);
                hev->normal = n;

                obj.vertex_buffer.push_back(Triple{hev->x, hev->y, hev->z});
                obj.normal_buffer.push_back(Triple{n.x, n.y, n.z});

                he = he->next;
            }
        }
    }
}

float deg2rad(float angle)
{
    return angle * M_PI / 180.0;
}

void key_pressed(unsigned char key, int x, int y)
{
    if(key == 'q')
    {
        exit(0);
    }
    else if(key == 't')
    {
        wireframe_mode = !wireframe_mode;
        glutPostRedisplay();
    }
    else
    {
        float x_view_rad = deg2rad(x_view_angle);
        
        if(key == 'w')
        {
            cam_position[0] += step_size * sin(x_view_rad);
            cam_position[2] -= step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        else if(key == 'a')
        {
            cam_position[0] -= step_size * cos(x_view_rad);
            cam_position[2] -= step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
        else if(key == 's')
        {
            cam_position[0] -= step_size * sin(x_view_rad);
            cam_position[2] += step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        else if(key == 'd')
        {
            cam_position[0] += step_size * cos(x_view_rad);
            cam_position[2] += step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
        else if (key == 'i') {
            implicit_fairing();
            glutPostRedisplay();
        }
    }
}

void parse_obj_file(string filename, string objname) {
    Object obj; 
    Mesh_Data *mesh = new Mesh_Data;
    mesh->vertices = new vector<Vertex*>;
    mesh->faces = new vector<Face*>;

    // account for 1-indexing
    mesh->vertices->push_back(NULL);

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
            mesh->vertices->push_back(new Vertex{x, y, z});
        } 
        else if (type == "f") {
            int x, y, z;
            ss >> x >> y >> z;
            mesh->faces->push_back(new Face{x, y, z});
        }
    }

    obj.hevs = new vector<HEV*>();
    obj.hefs = new vector<HEF*>();
    build_HE(mesh, obj.hevs, obj.hefs);

    // fill vertex and normal buffers using halfedge structures 
    for (HEF *hef : *obj.hefs) {
        HE *he = hef->edge;

        for (int i = 0; i < 3; i++) {
            HEV *hev = he->vertex;

            Vec3f n = calc_vertex_normal(hev);
            hev->normal = n;

            obj.vertex_buffer.push_back(Triple{hev->x, hev->y, hev->z});
            obj.normal_buffer.push_back(Triple{n.x, n.y, n.z});

            he = he->next;
        }
    }

    objmap[objname] = obj; 
}

Object create_obj(ifstream& file, string objname) { 
    Object obj;
    vector<Transform> transforms; 

    string line; 
    while (getline(file, line)) {
        string trimmed = line;
        trimmed.erase(0, trimmed.find_first_not_of(" \t\r\n"));
        trimmed.erase(trimmed.find_last_not_of(" \t\r\n") + 1);
        
        if (trimmed.empty()) { // check for empty line
            break; 
        }
        
        istringstream ss(line); 
        string first; 
        ss >> first; 

        if (first == "ambient") { 
            float r, g, b; 
            ss >> r >> g >> b; 
            obj.ambient_reflect[0] = r;
            obj.ambient_reflect[1] = g;
            obj.ambient_reflect[2] = b;
        }
        else if (first == "diffuse") { 
            float r, g, b; 
            ss >> r >> g >> b; 
            obj.diffuse_reflect[0] = r;
            obj.diffuse_reflect[1] = g;
            obj.diffuse_reflect[2] = b;
        }
        else if (first == "specular") { 
            float r, g, b; 
            ss >> r >> g >> b; 
            obj.specular_reflect[0] = r;
            obj.specular_reflect[1] = g;
            obj.specular_reflect[2] = b;
        }
        else if (first == "shininess") { 
            float k;
            ss >> k;
            obj.shininess = k;            
        }
        else if (first == "r") {
            float x, y, z, theta; 
            ss >> x >> y >> z >> theta; 
            Transform t; 
            t.type = "r"; 
            t.params[0] = x;
            t.params[1] = y;
            t.params[2] = z;
            t.rotation_angle = theta; 
            transforms.push_back(t); 
        }
        else if (first == "s") {
            float x, y, z; 
            ss >> x >> y >> z; 
            Transform t; 
            t.type = "s";
            t.params[0] = x;
            t.params[1] = y;
            t.params[2] = z;
            t.rotation_angle = -1; 
            transforms.push_back(t); 
        }
        else if (first == "t") {
            float x, y, z; 
            ss >> x >> y >> z; 
            Transform t; 
            t.type = "t";
            t.params[0] = x;
            t.params[1] = y;
            t.params[2] = z;
            t.rotation_angle = -1; 
            transforms.push_back(t); 
        }
    }

    obj.vertex_buffer = objmap[objname].vertex_buffer; 
    obj.normal_buffer = objmap[objname].normal_buffer; 
    obj.transform_sets = transforms; 

    obj.hevs = objmap[objname].hevs;  
    obj.hefs = objmap[objname].hefs;

    return obj; 
}

void create_objects(ifstream& file) {
    string line; 
    while (getline(file, line)) {
        istringstream ss(line); 
        string first; 
        ss >> first; 
        if (line.find(".obj") != string::npos) {
            string filename; 
            ss >> filename; 
            parse_obj_file(filename, first); 
        }
        else if (objmap.find(first) != objmap.end()) { // check if we want to create new object
            objects.push_back(create_obj(file, first));
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc != 5) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }
    
    ifstream file(argv[1]);
    xres = atoi(argv[2]);
    yres = atoi(argv[3]);
    h = atof(argv[4]); 

    string line; 
    while (getline(file, line)) {
        istringstream ss(line); 
        string type; 
        ss >> type; 

        if (type == "position") {
            float p_x, p_y, p_z; 
            ss >> p_x >> p_y >> p_z;
            cam_position[0] = p_x; 
            cam_position[1] = p_y; 
            cam_position[2] = p_z; 
        }
        else if (type == "orientation") {
            float o_x, o_y, o_z, theta; 
            ss >> o_x >> o_y >> o_z >> theta;
            cam_orientation_axis[0] = o_x; 
            cam_orientation_axis[1] = o_y; 
            cam_orientation_axis[2] = o_z; 
            cam_orientation_angle = theta; 
        }
        else if (type == "near") { 
            ss >> near_param;  
        }
        else if (type == "far") { 
            ss >> far_param; 
        }
        else if (type == "left") { 
            ss >> left_param; 
        }
        else if (type == "right") { 
            ss >> right_param; 
        }
        else if (type == "top") { 
            ss >> top_param; 
        }
        else if (type == "bottom") { 
            ss >> bottom_param; 
        }
        else if (type == "objects:") { 
            create_objects(file);
        }
        else if (type == "light") {
            Point_Light light1; 
            float x, y, z, k;
            float r, g, b; 
            char comma;
            ss >> x >> y >> z >> comma >> r >> g >> b >> comma >> k;

            light1.position[0] = x;
            light1.position[1] = y;
            light1.position[2] = z;
            light1.position[3] = 1; // w coord
            
            light1.color[0] = r;
            light1.color[1] = g;
            light1.color[2] = b;
            light1.attenuation_k = k;
            
            lights.push_back(light1);
        }
    }

    /* 'glutInit' intializes the GLUT (Graphics Library Utility Toolkit) library.
     * This is necessary, since a lot of the functions we used above and below
     * are from the GLUT library.
     *
     * 'glutInit' takes the 'main' function arguments as parameters. This is not
     * too important for us, but it is possible to give command line specifications
     * to 'glutInit' by putting them with the 'main' function arguments.
     */
    glutInit(&argc, argv);
    /* The following line of code tells OpenGL that we need a double buffer,
     * a RGB pixel buffer, and a depth buffer.
     */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    /* The following line tells OpenGL to create a program window of size
     * 'xres' by 'yres'.
     */
    glutInitWindowSize(xres, yres);
    /* The following line tells OpenGL to set the program window in the top-left
     * corner of the computer screen (0, 0).
     */
    glutInitWindowPosition(0, 0);
    /* The following line tells OpenGL to name the program window "Test".
     */
    glutCreateWindow("Test");

    GLenum err = glewInit();
    if (err != GLEW_OK) {
        cerr << "GLEW Error: " << glewGetErrorString(err) << endl;
        return 1;
    }
    
    /* Call our 'init' function...
     */
    init();
    /* Specify to OpenGL our display function.
     */
    glutDisplayFunc(display);
    /* Specify to OpenGL our reshape function.
     */
    glutReshapeFunc(reshape);
    /* Specify to OpenGL our function for handling mouse presses.
     */
    glutMouseFunc(mouse_pressed);
    /* Specify to OpenGL our function for handling mouse movement.
     */
    glutMotionFunc(mouse_moved);
    /* Specify to OpenGL our function for handling key presses.
     */
    glutKeyboardFunc(key_pressed);
    /* The following line tells OpenGL to start the "event processing loop". This
     * is an infinite loop where OpenGL will continuously use our display, reshape,
     * mouse, and keyboard functions to essentially run our program.
     */
    glutMainLoop();
    
    // free all hes
    for (Object &obj : objects) {
        delete_HE(obj.hevs, obj.hefs);
    }
}
