#version 150 compatibility

// Geometry shader counterpart of glsl120/include/pass_depth.vert.

uniform mat4 worldview_matrix;

out float depth;

void passDepth( vec4 pos )
{
  vec4 pos_rel_view = worldview_matrix * pos;
  depth = -pos_rel_view.z;
}
