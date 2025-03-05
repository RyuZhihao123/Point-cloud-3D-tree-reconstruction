#version 120
uniform vec3 u_color;

void main()
{
    gl_FragColor = vec4(u_color.r,u_color.g,u_color.b,1.0);

}




