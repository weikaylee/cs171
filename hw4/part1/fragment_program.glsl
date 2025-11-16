varying vec3 normal, point;

void main()
{
    vec3 d = vec3(gl_FrontMaterial.diffuse); // diffuse color
    vec3 a = vec3(gl_FrontMaterial.ambient); // ambient color 
    vec3 s = vec3(gl_FrontMaterial.specular); // specular color
    float p = gl_FrontMaterial.shininess; // shininess factor

    // accumulate from all lights 
    vec3 diffuse_sum = vec3(0.0, 0.0, 0.0);
    vec3 specular_sum = vec3(0.0, 0.0, 0.0);

    vec3 e_direction = normalize(-point);

    for (int i = 0; i < gl_MaxLights; i++) {
        vec3 light_pos = vec3(gl_LightSource[i].position);
        vec3 light_color = vec3(gl_LightSource[i].ambient); 

        // attentuation (linear comp negligble)
        float d = length(point - light_pos);
        float atten = 1.0 + gl_LightSource[i].quadraticAttenuation * d * d;
        light_color = light_color / atten;

        vec3 light_dir = normalize(light_pos - point);

        // accumulate diffuse lighting
        vec3 light_diffuse = light_color * max(0.0, dot(normal, light_dir));
        diffuse_sum += light_diffuse;

        // accumulate spec lighting
        vec3 light_spec = light_color * pow(max(0.0, dot(normal,
                                normalize(e_direction + light_dir))), p); // blinn-phong
        specular_sum += light_spec;
    }

    vec3 final_color = min(vec3(1, 1, 1),
                a + d * diffuse_sum + s * specular_sum); // rgb max is (1, 1, 1)

    gl_FragColor = vec4(final_color, 1.0);
}
