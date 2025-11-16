attribute vec3 tangent, texture;
varying vec3 s;

void main() {
    gl_TexCoord[0] = vec4(texture, 1.0);

    vec3 view_pos = vec3(gl_ModelViewMatrix * gl_Vertex);
    s = normalize(vec3(gl_LightSource[0].position) - view_pos);

    // construct tbn matrix
    vec3 n = normalize(vec3(gl_NormalMatrix * gl_Normal));
    vec3 t = normalize(vec3(gl_NormalMatrix * tangent));
    vec3 b = normalize(cross(n, t));                           

    float s_x = dot(t, s);
    float s_y = dot(b, s);
    float s_z = dot(n, s);

    s = normalize(vec3(s_x, s_y, s_z));

    gl_Position = ftransform();
}
