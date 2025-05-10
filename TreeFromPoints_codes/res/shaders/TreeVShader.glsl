#version 120
uniform mat4 mat_projection;
uniform mat4 mat_view;
uniform mat4 mat_model;

varying vec3 v_position;

void main()
{
    gl_Position = mat_projection* mat_view * mat_model * vec4(gl_Vertex,1.0);
}
