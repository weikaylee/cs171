uniform sampler2D color_texture, normal_map_texture;
varying vec3 s;

void main() {
    vec4 color = texture2D(color_texture, gl_TexCoord[0].st);
    vec3 normal = texture2D(normal_map_texture, gl_TexCoord[0].st).rgb;

    // map to [-1, 1] then normalize 
    normal[0] = normal[0] * 2.0 - 1.0;
    normal[1] = normal[1] * 2.0 - 1.0;
    normal[2] = normal[2] * 2.0 - 1.0;

    normal = normalize(normal);
    
    // get ambieny color, if facing camera 
    vec4 out_color = gl_FrontMaterial.ambient * gl_LightModel.ambient;
    float n_dot_l = dot(normal, s);

    if (n_dot_l > 0.0) {
        out_color += color * gl_LightSource[0].diffuse * n_dot_l;
    }

    gl_FragColor = out_color;
}
