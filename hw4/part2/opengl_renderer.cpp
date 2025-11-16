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

extern GLenum readpng(const char *filename);

void init(string color_texture, string normal_map);
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
    vector<Triple> tangent_buffer;
    vector<Triple> texture_buffer;
    
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
static GLenum color_tex, normal_tex;

void init(char * color_texture, char * normal_map)
{
    // glewInit();
    cerr << "Loading textures" << endl;
    if(!(color_tex = readpng(color_texture)))
        exit(1);
    if(!(normal_tex = readpng(normal_map)))
        exit(1);
    
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

    // enable texture and normal mapping
    GLint color_uniform_pos = glGetUniformLocation(shaderProgram, "color_texture");
    GLint normal_uniform_pos = glGetUniformLocation(shaderProgram, "normal_map_texture");

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glUniform1i(color_uniform_pos, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normal_tex);
    glUniform1i(normal_uniform_pos, 1);
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
            
            // send tangent, texture info to vertex_program as attribute 
            GLint tangent_attribute_loc = glGetAttribLocation(
                                                    shaderProgram, "tangent");
            GLint texture_attribute_loc = glGetAttribLocation(
                                                shaderProgram, "texture");

            glVertexAttribPointer(tangent_attribute_loc, 3, GL_FLOAT, GL_FALSE,
                                    0, &objects[i].tangent_buffer[0]);
            glVertexAttribPointer(texture_attribute_loc, 3, GL_FLOAT, GL_FALSE,
                                    0, &objects[i].texture_buffer[0]);

            glEnableVertexAttribArray(tangent_attribute_loc);
            glEnableVertexAttribArray(texture_attribute_loc);

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

void create_lights()
{
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Light 1 Below
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    Point_Light light1;
    
    light1.position[0] = -0.8;
    light1.position[1] = 0;
    light1.position[2] = 1;
    light1.position[3] = 1;
    
    light1.color[0] = 1;
    light1.color[1] = 1;
    light1.color[2] = 0;
    light1.attenuation_k = 0.2;
    
    lights.push_back(light1);
}

void create_square()
{
    Object square1;
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Reflectances
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    square1.ambient_reflect[0] = 0.2;
    square1.ambient_reflect[1] = 0.2;
    square1.ambient_reflect[2] = 0.2;
    
    square1.diffuse_reflect[0] = 0.6;
    square1.diffuse_reflect[1] = 0.6;
    square1.diffuse_reflect[2] = 0.6;
    
    square1.specular_reflect[0] = 1;
    square1.specular_reflect[1] = 1;
    square1.specular_reflect[2] = 1;
    
    square1.shininess = 5.0;
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Points
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    Triple point1;
    point1.x = -1;
    point1.y = -1;
    point1.z = 1;
    
    Triple point2;
    point2.x = 1;
    point2.y = -1;
    point2.z = 1;
    
    Triple point3;
    point3.x = 1;
    point3.y = 1;
    point3.z = 1;
    
    Triple point4;
    point4.x = -1;
    point4.y = 1;
    point4.z = 1;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Texture points
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    Triple tex1;
    tex1.x = -1;
    tex1.y = -1;
    tex1.z = 1;
    
    Triple tex2;
    tex2.x = 1;
    tex2.y = -1;
    tex2.z = 1;
    
    Triple tex3;
    tex3.x = 1;
    tex3.y = 1;
    tex3.z = 1;
    
    Triple tex4;
    tex4.x = -1;
    tex4.y = 1;
    tex4.z = 1;

    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Normals, tangents
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    Triple normal;
    normal.x = 0;
    normal.y = 0;
    normal.z = 1;

    Triple tangent;
    tangent.x = 1;
    tangent.y = 0;
    tangent.z = 0;

    // TODO binomral?
        
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Vertex and Normal Arrays
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    /* We are rendering our cubes with triangles, so each cube has 12 (triangle) faces
     * in all.
     */
    
    /* Face 1: */
    
    square1.vertex_buffer.push_back(point1);
    square1.normal_buffer.push_back(normal);
    square1.tangent_buffer.push_back(tangent);
    square1.texture_buffer.push_back(tex1);

    square1.vertex_buffer.push_back(point2);
    square1.normal_buffer.push_back(normal);
    square1.tangent_buffer.push_back(tangent);
    square1.texture_buffer.push_back(tex2);

    square1.vertex_buffer.push_back(point3);
    square1.normal_buffer.push_back(normal);
    square1.tangent_buffer.push_back(tangent);
    square1.texture_buffer.push_back(tex3);

    /* Face 2: */

    square1.vertex_buffer.push_back(point1);
    square1.normal_buffer.push_back(normal);
    square1.tangent_buffer.push_back(tangent);
    square1.texture_buffer.push_back(tex1);

    square1.vertex_buffer.push_back(point3);
    square1.normal_buffer.push_back(normal);
    square1.tangent_buffer.push_back(tangent);
    square1.texture_buffer.push_back(tex3);

    square1.vertex_buffer.push_back(point4);
    square1.normal_buffer.push_back(normal);
    square1.tangent_buffer.push_back(tangent);
    square1.texture_buffer.push_back(tex4);

    objects.push_back(square1);
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        cout << "Invalid num arguments" << endl;
        return 1;
    }

    char *color_texture = argv[1];
    char *normal_map = argv[2];

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

    // GLenum err = glewInit();
    // if (err != GLEW_OK) {
    //     cerr << "GLEW Error: " << glewGetErrorString(err) << endl;
    //     return 1;
    // }
    vert_prog_filename = "vertex_program.glsl";
    frag_prog_filename = "fragment_program.glsl";
    read_shaders();

    /* Call our 'init' function...
     */
    init(color_texture, normal_map); // load textures in here

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
