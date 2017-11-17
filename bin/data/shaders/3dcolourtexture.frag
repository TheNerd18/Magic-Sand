#version 120

// #extension GL_ARB_texture_rectangle : enable
uniform sampler3D myTexture;

void main()
{
    gl_FragColor = texture3D(myTexture, gl_TexCoord[0].xyz);
}