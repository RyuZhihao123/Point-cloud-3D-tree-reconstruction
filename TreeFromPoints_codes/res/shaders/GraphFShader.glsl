#version 120
varying vec4 v_position;
varying vec4 v_color;
float far = 100000.0;
float near = 0.1;

void main()
{
    gl_FragColor = v_color;
    //gl_FragColor= vec4(0,0.6,0,1.0);
}



