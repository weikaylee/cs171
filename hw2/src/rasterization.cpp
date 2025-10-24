#include <cmath>
#include <iostream> 

#include "rasterization.hpp"
#include "object.hpp"
#include "transform.hpp"
#include "image.hpp"

// first and eigth octants
void low_slope(int x1, int y1, int x2, int y2, int xres, int yres, vector<vector<int>>& grid) {
    int error = 0;
    int y = y1;
    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    for (size_t x = x1; x <= x2; x++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) { // ignore anythign off screeen
            grid[y][x] = 1;
        }

        if (2 * (error + dy) < dx) {
            error += dy;
        }
        else {
            error += dy - dx;
            y += (y2 >= y1) ? 1 : -1;
        }
    }
}

// second and seventh octants
void high_slope(int x1, int y1, int x2, int y2, int xres, int yres, vector<vector<int>>& grid) {
    int error = 0;
    int x = x1;
    int dx = abs(x2 - x1);
    int dy = y2 - y1;

    for (size_t y = y1; y <= y2; y++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            grid[y][x] = 1;
        }

        if (2 * (error + dx) < dy) {
            error += dx;
        }
        else {
            error += dx - dy;
            x += (x2 >= x1) ? 1 : -1;
        }
    }
}

// invalid points (x, y) are (-1, -1)
void bresenham(int x1, int y1, int x2, int y2,int xres, int yres, vector<vector<int>>& grid) {
    if (x1 == -1 || x2 == -1) {
        return;
    }

    if (abs(y2 - y1) <= abs(x2 - x1)) {
        if (x1 <= x2) {
            low_slope(x1, y1, x2, y2, xres, yres, grid);
        }
        else {
            low_slope(x2, y2, x1, y1, xres, yres, grid);
        }
    }
    else {
        if (y1 <= y2) {
            high_slope(x1, y1, x2, y2, xres, yres, grid);
        }
        else {
            high_slope(x2, y2, x1, y1, xres, yres, grid);
        }
    }
}

// params are screen coordinates 
float compute_alpha(int x_a, int y_a, int x_b, int y_b, int x_c, int y_c, int x, int y) { 
    int num = (y_b - y_c) * x  + (x_c - x_b) * y + x_b * y_c - x_c * y_b; 
    int denom = (y_b - y_c) * x_a  + (x_c - x_b) * y_a + x_b * y_c - x_c * y_b; 
    return (float) num / denom; 
}

float compute_beta(int x_a, int y_a, int x_b, int y_b, int x_c, int y_c, int x, int y) { 
    int num = (y_a - y_c) * x + (x_c - x_a) * y + x_a * y_c - x_c * y_a; 
    int denom = (y_a - y_c) * x_b + (x_c - x_a) * y_b + x_a * y_c - x_c * y_a; 
    return (float) num / denom; 
}

float compute_gamma(int x_a, int y_a, int x_b, int y_b, int x_c, int y_c, int x, int y) { 
    int num = (y_a - y_b) * x + (x_b - x_a) * y + x_a * y_b - x_b * y_a; 
    int denom = (y_a - y_b) * x_c + (x_b - x_a) * y_c + x_a * y_b - x_b * y_a; 
    return (float) num / denom; 
}

// all coords are in world space
Color lighting(Vertex point, Vertex normal, Material mat, vector<Light> lights, Vertex camera) {
    Eigen::Vector3d cd = to_vector3d(mat.diffuse); 
    Eigen::Vector3d ca = to_vector3d(mat.ambient); 
    Eigen::Vector3d cs = to_vector3d(mat.specular); 
    float p = mat.shininess; 

    Eigen::Vector3d diffuse_sum = {0.0, 0.0, 0.0}; 
    Eigen::Vector3d specular_sum = {0.0, 0.0, 0.0}; 

    Eigen::Vector3d point_vec = to_vector3d(point);
    Eigen::Vector3d e_dir = (to_vector3d(camera) - point_vec).normalized(); 


    for (Light l : lights) {
        Eigen::Vector3d lp = to_vector3d(l.position); 
        
        float d = (point_vec - lp).norm(); 
        Eigen::Vector3d lc = to_vector3d(l.color) / (1 + l.k * d * d); // attenuation
        
        Eigen::Vector3d l_dir = (lp - point_vec).normalized(); 
        Eigen::Vector3d l_diff = lc * max(0.0, to_vector3d(normal).dot(l_dir)); 
        diffuse_sum += l_diff; 

        Eigen::Vector3d l_spec = lc * pow(max(0.0, to_vector3d(normal).dot((e_dir + l_dir).normalized())), p); 
        specular_sum += l_spec; 
    }

    Eigen::Vector3d color = ca + diffuse_sum.cwiseProduct(cd) + specular_sum.cwiseProduct(cs); 
    for (int i = 0; i < color.size(); i++) {
        color[i] = min(1.0, color[i]);
    }
    
    return Color{float(color[0]), float(color[1]), float(color[2])};
} 

// vertices are in carteisn andc 
void raster_colored_triangle(Face f, vector<Vertex> verts, Color ca, Color cb, Color cc, int xres, int yres, Eigen::Matrix4d world_to_camera, Eigen::Matrix4d camera_to_ndc, vector<vector<Color>>& grid, vector<vector<float>>& buffer) { 
    // get ndc coords 
    Vertex va = world_to_ndc(verts[f.v1], world_to_camera, camera_to_ndc); 
    Vertex vb = world_to_ndc(verts[f.v2], world_to_camera, camera_to_ndc); 
    Vertex vc = world_to_ndc(verts[f.v3], world_to_camera, camera_to_ndc); 

    if ((to_vector3d(vc) - to_vector3d(vb)).cross(to_vector3d(va) - to_vector3d(vb))[2] < 0) {
        return;
    }
    
    Screen a = ndc_to_screen(va, xres, yres);
    Screen b = ndc_to_screen(vb, xres, yres);
    Screen c = ndc_to_screen(vc, xres, yres);

    int x_min = min({a.x, b.x, c.x}); 
    int x_max = max({a.x, b.x, c.x});
    int y_min = min({a.y, b.y, c.y}); 
    int y_max = max({a.y, b.y, c.y}); 

    for (int x = x_min; x <= x_max; x++) { 
        for (int y = y_min; y <= y_max; y++) { 
            float alpha = compute_alpha(a.x, a.y, b.x, b.y, c.x, c.y, x, y); 
            float beta = compute_beta(a.x, a.y, b.x, b.y, c.x, c.y, x, y); 
            float gamma = compute_gamma(a.x, a.y, b.x, b.y, c.x, c.y, x, y); 
            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1) {
                float cube_x = alpha * va.v1 + beta * vb.v1 + gamma * vc.v1; 
                float cube_y = alpha * va.v2 + beta * vb.v2 + gamma * vc.v2; 
                float cube_z = alpha * va.v3 + beta * vb.v3 + gamma * vc.v3; 
                
                
                if (in_cube(cube_x, cube_y, cube_z) && !(cube_z > buffer[y][x])) {
                    buffer[y][x] = cube_z; 
                    float new_r = alpha * ca.r + beta * cb.r + gamma * cc.r; 
                    float new_g = alpha * ca.g + beta * cb.g + gamma * cc.g;
                    float new_b = alpha * ca.b + beta * cb.b + gamma * cc.b;  
                    grid[y][x] = Color{new_r, new_g, new_b}; 
                }
            }
        }
    }
}

void gouraud_shading(Image& img) {
    Scene scene = img.scene; 
    vector<Light> lights = scene.lights; 
    Vertex camera = scene.camera; 

    for (Object& obj : img.scene.copies) {
        Material mat = obj.material; 
        for (Face& f : obj.faces) {
            Color color_a = lighting(obj.verts[f.v1], obj.normals[f.n1], mat, lights, camera); 
            Color color_b = lighting(obj.verts[f.v2], obj.normals[f.n2], mat, lights, camera); 
            Color color_c = lighting(obj.verts[f.v3], obj.normals[f.n3], mat, lights, camera); 
            
            raster_colored_triangle(f, obj.verts, color_a, color_b, color_c, img.xres, img.yres, scene.world_to_camera, scene.camera_to_ndc, img.grid, img.buffer);
        }
    }
}

void phong_shading(Image& img) {
    Scene scene = img.scene; 
    for (Object obj : scene.copies) {
        vector<Vertex> verts = obj.verts;
        vector<Vertex> normals = obj.normals;
        for (Face f : obj.faces) {
            Vertex world_va = obj.verts[f.v1];
            Vertex world_vb = obj.verts[f.v2];
            Vertex world_vc = obj.verts[f.v3];

            Vertex na = normals[f.n1]; 
            Vertex nb = normals[f.n2]; 
            Vertex nc = normals[f.n3]; 

            Vertex va = world_to_ndc(world_va, scene.world_to_camera, scene.camera_to_ndc); 
            Vertex vb = world_to_ndc(world_vb, scene.world_to_camera, scene.camera_to_ndc); 
            Vertex vc = world_to_ndc(world_vc, scene.world_to_camera, scene.camera_to_ndc);

            if ((to_vector3d(vc) - to_vector3d(vb)).cross(to_vector3d(va) - to_vector3d(vb))[2] < 0) {
                continue;
            }

            Screen a = ndc_to_screen(va, img.xres, img.yres);
            Screen b = ndc_to_screen(vb, img.xres, img.yres);
            Screen c = ndc_to_screen(vc, img.xres, img.yres);

            int x_min = min({a.x, b.x, c.x}); 
            int x_max = max({a.x, b.x, c.x});
            int y_min = min({a.y, b.y, c.y}); 
            int y_max = max({a.y, b.y, c.y}); 

            for (int x = x_min; x <= x_max; x++) { 
                for (int y = y_min; y <= y_max; y++) { 
                    float alpha = compute_alpha(a.x, a.y, b.x, b.y, c.x, c.y, x, y); 
                    float beta = compute_beta(a.x, a.y, b.x, b.y, c.x, c.y, x, y); 
                    float gamma = compute_gamma(a.x, a.y, b.x, b.y, c.x, c.y, x, y); 
                    
                    if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1) {
                        float cube_x = alpha * va.v1 + beta * vb.v1 + gamma * vc.v1; 
                        float cube_y = alpha * va.v2 + beta * vb.v2 + gamma * vc.v2; 
                        float cube_z = alpha * va.v3 + beta * vb.v3 + gamma * vc.v3; 

                        if (in_cube(cube_x, cube_y, cube_z) && !(cube_z > img.buffer[y][x])) {
                            img.buffer[y][x] = cube_z; 
                            Vertex n = {alpha * na.v1 + beta * nb.v1 + gamma * nc.v1, alpha * na.v2 + beta * nb.v2 + gamma * nc.v2, alpha * na.v3 + beta * nb.v3 + gamma * nc.v3};
 
                            // Normalize the interpolated normal
                            Eigen::Vector3d n_vec = to_vector3d(n).normalized();
                            n = Vertex{float(n_vec[0]), float(n_vec[1]), float(n_vec[2])};
                            
                            Vertex v = {alpha * world_va.v1 + beta * world_vb.v1 + gamma * world_vc.v1, alpha * world_va.v2 + beta * world_vb.v2 + gamma * world_vc.v2, alpha * world_va.v3 + beta * world_vb.v3 + gamma * world_vc.v3};
                            Color color = lighting(v, n, obj.material, scene.lights, scene.camera); 
                            img.grid[y][x] = color; 
                        }
                    }
                }
            }
        }
    }
}

