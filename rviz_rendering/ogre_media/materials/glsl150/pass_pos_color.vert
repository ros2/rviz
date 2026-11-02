#version 150 compatibility

// Passes position and color through unchanged; the geometry shaders
// expand each point into a billboard or a box.

void main()
{
  gl_Position = gl_Vertex;
  gl_FrontColor = gl_Color;
}
