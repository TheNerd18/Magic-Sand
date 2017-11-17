#version 120

uniform sampler2DRect tex0; // Sampler for the depth image-space elevation texture
uniform float maxHeight;
uniform vec2 meshDim;

void main() {
    gl_TexCoord[0] = gl_MultiTexCoord0 / vec4(meshDim, 1.0, 1.0);
    
    vec4 pos = gl_Vertex;

    // Set the vertex' depth image-space z coordinate from the texture:
    vec4 texel0 = texture2DRect(tex0, pos.xy);  
    
    float depth = texel0.x;

    // The texture is floating point 0.0-1.0
    pos.z = depth * maxHeight;
    pos.w = 1.0;
    
    gl_TexCoord[0].z = depth;

    gl_Position = gl_ModelViewProjectionMatrix * pos;
}