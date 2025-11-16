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
#include <Eigen/Dense>

#include "quaternion.hpp"

using namespace std;

void init(int mode);
void read_shaders(); 
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

// shader vars 
GLenum shaderProgram;
string vert_prog_filename, frag_prog_filename;

void init(int mode)
{
    // glewInit();
    if (mode == 0) { 
        glShadeModel(GL_SMOOTH); // gouraud shading
    }
    else {
        vert_prog_filename = "./vertex_program.glsl";
        frag_prog_filename = "./fragment_program.glsl";
        read_shaders();
    }
    
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

// Read shaders here!!! 
void read_shaders() {
   string vertProgramSource, fragProgramSource;
   
   ifstream vertProgFile(vert_prog_filename.c_str());
   if (! vertProgFile)
      cerr << "Error opening vertex shader program\n";
   ifstream fragProgFile(frag_prog_filename.c_str());
   if (! fragProgFile)
      cerr << "Error opening fragment shader program\n";

   getline(vertProgFile, vertProgramSource, '\0');
   const char* vertShaderSource = vertProgramSource.c_str();

   getline(fragProgFile, fragProgramSource, '\0');
   const char* fragShaderSource = fragProgramSource.c_str();

   char buf[1024];
   GLsizei blah;

   // Initialize shaders
   GLenum vertShader, fragShader;

   shaderProgram = glCreateProgram();
   if (shaderProgram == 0) {
      cerr << "Failed to create shader program" << endl;
      exit(1);
   }

   vertShader = glCreateShader(GL_VERTEX_SHADER);
   glShaderSource(vertShader, 1, &vertShaderSource, NULL);
   glCompileShader(vertShader);
    
   GLint isCompiled = 0;
   glGetShaderiv(vertShader, GL_COMPILE_STATUS, &isCompiled);
   if(isCompiled == GL_FALSE)
   {
      GLint maxLength = 0;
      glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &maxLength);
    
      // The maxLength includes the NULL character
      std::vector<GLchar> errorLog(maxLength);
      glGetShaderInfoLog(vertShader, maxLength, &maxLength, &errorLog[0]);
    
      // Provide the infolog in whatever manor you deem best.
      // Exit with failure.
      cerr << "Vertex shader compilation failed:" << endl;
      for (int i = 0; i < errorLog.size(); i++)
         cout << errorLog[i];
      glDeleteShader(vertShader); // Don't leak the shader.
      return;
   }

   fragShader = glCreateShader(GL_FRAGMENT_SHADER);
   glShaderSource(fragShader, 1, &fragShaderSource, NULL);
   glCompileShader(fragShader);

   isCompiled = 0;
   glGetShaderiv(fragShader, GL_COMPILE_STATUS, &isCompiled);
   if(isCompiled == GL_FALSE)
   {
      GLint maxLength = 0;
      glGetShaderiv(fragShader, GL_INFO_LOG_LENGTH, &maxLength);
    
      // The maxLength includes the NULL character
      std::vector<GLchar> errorLog(maxLength);
      glGetShaderInfoLog(fragShader, maxLength, &maxLength, &errorLog[0]);
    
      // Provide the infolog in whatever manor you deem best.
      // Exit with failure.
      cerr << "Fragment shader compilation failed:" << endl;
      for (int i = 0; i < errorLog.size(); i++)
         cout << errorLog[i];
      glDeleteShader(fragShader); // Don't leak the shader.
      return;
   }

   glAttachShader(shaderProgram, vertShader);
   glAttachShader(shaderProgram, fragShader);
   glLinkProgram(shaderProgram);

   GLint isLinked = 0;
   glGetProgramiv(shaderProgram, GL_LINK_STATUS, &isLinked);
   if (isLinked == GL_FALSE) {
      GLint maxLength = 0;
      glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &maxLength);
      
      vector<GLchar> errorLog(maxLength);
      glGetProgramInfoLog(shaderProgram, maxLength, &maxLength, &errorLog[0]);
      
      cerr << "Shader linking failed:" << endl;
      for (int i = 0; i < errorLog.size(); i++)
         cerr << errorLog[i];
      cerr << endl;
      
      glDeleteProgram(shaderProgram);
      glDeleteShader(vertShader);
      glDeleteShader(fragShader);
      return; 
   }

    // Use the program
    glUseProgram(shaderProgram);
    
    // Clean up shaders (no longer needed after linking)
    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

//    cerr << "Enabling fragment program: " << gluErrorString(glGetError()) << endl;
//    glGetProgramInfoLog(shaderProgram, 1024, &blah, buf);
//    cerr << buf;

//    cerr << "Enabling program object" << endl;
//    glUseProgram(shaderProgram);
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

        // for fragment program!!! 
        glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
        
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
    }
}

// read .obj file and return vertex_buffer and normal_buffer that define the faces of the obj
void parse_obj_file(string filename, string objname) {
    Object obj; 
    vector<Triple> verts; 
    vector<Triple> normals; 

    vector<Triple> vertex_buffer; 
    vector<Triple> normal_buffer;

    // account for one-indexing of vertices 
    Triple null_v = {0, 0, 0};
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
            verts.push_back({x, y, z});
        } 
        else if (type == "vn") {
            float x, y, z;
            ss >> x >> y >> z;
            normals.push_back({x, y, z});
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
            Triple v1 = verts[idxs[0]];
            Triple v2 = verts[idxs[2]];
            Triple v3 = verts[idxs[4]];

            Triple n1 = normals[idxs[1]];
            Triple n2 = normals[idxs[3]];
            Triple n3 = normals[idxs[5]];

            vertex_buffer.push_back(v1);
            vertex_buffer.push_back(v2);
            vertex_buffer.push_back(v3);

            normal_buffer.push_back(n1);
            normal_buffer.push_back(n2);
            normal_buffer.push_back(n3);
        }
    }
    obj.vertex_buffer = vertex_buffer; 
    obj.normal_buffer = normal_buffer; 
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
    obj.normal_buffer =  objmap[objname].normal_buffer; 
    obj.transform_sets = transforms; 

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
        cout << "Invalid num arguments" << endl;
        return 1;
    }

    ifstream file(argv[1]);
    xres = atoi(argv[2]);
    yres = atoi(argv[3]);
    int mode = atoi(argv[4]);

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
    init(mode); // set which shading (gouraud or phong) in here

    /* Specify to OpenGL our display function.
     */
    glutDisplayFunc(display);
    /* Specify to OpenGL our reshape function.
     */
    // glutReshapeFunc(reshape);
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
}
