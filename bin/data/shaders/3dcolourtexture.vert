#version 120

uniform sampler2DRect tex0; // Sampler for the depth image-space elevation texture
uniform float maxHeight;
uniform vec2 meshDim;

uniform vec2 meshOffset;
uniform mat4 kinectProjMatrix; // Transformation from kinect world space to proj image space
uniform mat4 kinectWorldMatrix; // Transformation from kinect image space to kinect world space
uniform vec2 heightColorMapTransformation; // Transformation from elevation to height color map texture coordinate factor and offset
uniform vec2 depthTransformation; // Normalisation factor and offset applied by openframeworks
uniform vec4 basePlaneEq; // Base plane equation

void main() {
    gl_TexCoord[0] = (gl_MultiTexCoord0 - vec4(meshOffset.x, meshOffset.y, 0, 0)) / vec4(meshDim, 1.0, 1.0);

    vec4 position = gl_Vertex;
    
    // copy position so we can work with it.
    vec4 pos = position;

    /* Set the vertex' depth image-space z coordinate from the texture: */
    vec4 texel0 = texture2DRect(tex0, pos.xy);
    float depth1 = texel0.r;
    float depth = depth1 * depthTransformation.x + depthTransformation.y;

    // pos.z = depth;
    pos.z = depth;
    pos.w = 1;
    
    /* Transform the vertex from depth image space to world space: */
    vec4 vertexCc = kinectWorldMatrix * pos;  // Transposed multiplication (Row-major order VS col major order
    vec4 vertexCcx = vertexCc * depth;
    vertexCcx.w = 1;

    /* Transform elevation to height color map texture coordinate: */
    float elevation = dot(basePlaneEq,vertexCcx);///vertexCc.w;
    
    /* Transform vertex to proj coordinates: */
    vec4 screenPos = kinectProjMatrix * vertexCcx;
    vec4 projectedPoint = screenPos / screenPos.z;

    projectedPoint.z = 0;
    projectedPoint.w = 1;
    
    gl_Position = gl_ModelViewProjectionMatrix * (vec4(meshDim.x, meshDim.y, 0.0, 2*position.w) - (position - vec4(meshOffset.x, meshOffset.y, 0, 0)));
    
    gl_TexCoord[0].z = (elevation * heightColorMapTransformation.x + heightColorMapTransformation.y) / (maxHeight);
}