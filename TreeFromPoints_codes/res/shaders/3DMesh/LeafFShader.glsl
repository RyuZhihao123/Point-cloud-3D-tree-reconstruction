#version 120
uniform vec3 viewPos;
uniform int isTextured;
varying vec3 v_position;
varying vec3 v_normal;
varying vec3 v_texCoord;

uniform sampler2D texture;
uniform bool isLighting;
uniform bool isBark;
uniform bool isShowSeg;

void main()
{
   gl_FragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
}
