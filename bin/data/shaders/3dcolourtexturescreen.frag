#version 120

// #extension GL_ARB_texture_rectangle : enable
uniform sampler3D colourTexture;

void main()
{
    gl_FragColor = texture3D(colourTexture, gl_TexCoord[0].xyz);
}