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

vec3 CalLight(vec3 lightPos,vec3 lightColor,float ambient,float specular)
{
    vec3 norm = normalize(v_normal);

    vec3 ambientLight = ambient*lightColor;

    vec3 lightDir = normalize(lightPos - v_position);

    float diffuse = max(dot(norm, lightDir),0.0);

    vec3 diffuseLight = diffuse*lightColor;

    vec3 viewDir = normalize(viewPos - v_position);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);

    vec3 specularLight = specular * spec * lightColor;

    return ambientLight+diffuseLight+specularLight;
}

void main()
{
    //vec3 objectColor = vec3(0.658824,0.564706,0.423529); // 物体表面的颜色
    if(texture2D(texture,vec2(v_texCoord)).a <0.5)
        discard;

    if(!isLighting)
    {
        gl_FragColor = texture2D(texture,vec2(v_texCoord));
        return;
    }

    vec3 objectColor;
    if(isTextured == 0)
        //objectColor = vec3(0.447059,0.405098,0.401569); // 物体表面的颜色、
        objectColor = vec3(0.564706,0.490196,0.443137);
    else
        objectColor = texture2D(texture,vec2(v_texCoord)).xyz; // 物体表面的颜色

    vec3 lColor1 = vec3(1.0,1.0,1.0);                // 光照的颜色
    //vec3 lPos1 = vec3(0,10000.0,10000.0);     // 光源的位置
    vec3 lPos1 = viewPos*100;          // 光源的位置
    vec3 light = CalLight(lPos1,lColor1,0.2,0.0);

    vec4 color = vec4(light*objectColor,1.0);


    if(isShowSeg)
    {
        if(isBark)
            gl_FragColor = vec4(1.0,0.0,0.0,1.0);
        else
            gl_FragColor = vec4(0.0,0.0,1.0,1.0);
    }
    else
        gl_FragColor = color;
}





