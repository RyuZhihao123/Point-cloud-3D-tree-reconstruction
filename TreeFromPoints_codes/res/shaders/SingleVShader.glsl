#version 120
uniform mat4 mat_projection;
uniform mat4 mat_view;
uniform mat4 mat_model;

void main()
{
    gl_Position = mat_projection* mat_view * mat_model * gl_Vertex;

}
