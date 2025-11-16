/* Vertex shader
 */

attribute vec3 tangent, tex_coord;
varying vec3 light_dir;

void main()
{
    // set gl_TexCoord[0] using tex_coord
    gl_TexCoord[0] = vec4(tex_coord, 1.0);

    vec3 view_pos = vec3(gl_ModelViewMatrix * gl_Vertex);
    light_dir = normalize(vec3(gl_LightSource[0].position) - view_pos);

    // Convert light_dir from camera space to texture surface space
    vec3 n = normalize(vec3(gl_NormalMatrix * gl_Normal));
    vec3 t = normalize(vec3(gl_ModelViewMatrix * vec4(tangent, 1.0)));// tangent
    vec3 b = normalize(cross(n, t));                                 // binormal

    float light_dir_x = dot(t, light_dir);
    float light_dir_y = dot(b, light_dir);
    float light_dir_z = dot(n, light_dir);

    light_dir = vec3(light_dir_x, light_dir_y, light_dir_z);

    gl_Position = ftransform();
}
