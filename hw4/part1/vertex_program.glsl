varying vec3 normal, point;

void main()
{
    normal = normalize(gl_NormalMatrix * gl_Normal);
    point = vec3(gl_ModelViewMatrix * gl_Vertex);
    gl_Position = ftransform();
}