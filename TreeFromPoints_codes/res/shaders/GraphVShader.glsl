#version 120
uniform mat4 mat_projection;
uniform mat4 mat_view;
uniform mat4 mat_model;

attribute vec3 a_position;
attribute vec3 a_color;
varying vec4 v_position;
varying vec4 v_color;
void main()
{
    gl_Position = mat_projection* mat_view * mat_model * vec4(a_position,1.0);
    v_position =  mat_model * vec4(a_position,1.0) * vec4(a_position,1.0);
    v_color = vec4(a_color,1.0);
}
