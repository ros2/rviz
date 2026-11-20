#version 150 compatibility

// Generates a camera-facing square (billboard) for each input vertex.
// Uses the same orientation and texture coords as nogp/billboard.vert,
// so both techniques render identically.

#ifdef WITH_DEPTH
  //include:
  void passDepth( vec4 pos );
#endif

uniform mat4 worldviewproj_matrix;
uniform vec4 camera_pos;
uniform vec4 size;
uniform vec4 auto_size;

// max_vertices must match max_output_vertices in glsl150.program.
layout(points) in;
layout(triangle_strip, max_vertices=4) out;

// corner is in [-0.5, 0.5], like the per-vertex texture coords of the nogp technique.
void emitVertex(vec4 center, vec3 right, vec3 up, vec2 size_factor, vec2 corner)
{
  vec2 s = corner * size_factor;
  vec4 pos = center + vec4(s.x * right + s.y * up, 0.0);
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = vec4(corner + vec2(0.5), 0.0, 0.0);
  gl_FrontColor = gl_in[0].gl_FrontColor;
#ifdef WITH_DEPTH
  passDepth( pos );
#endif
  EmitVertex();
}

void main()
{
  vec4 center = gl_in[0].gl_Position;

  vec3 at = normalize(camera_pos.xyz - center.xyz);
  vec3 right = cross(vec3(0.0, 1.0, 0.0), at);
  // Looking along the Y axis makes right zero; use the Z axis instead.
  if (dot(right, right) < 1e-12) {
    right = cross(vec3(0.0, 0.0, 1.0), at);
  }
  right = normalize(right);
  vec3 up = normalize(cross(at, right));

  // if auto_size == 1, then size_factor == size*gl_Vertex.z
  // if auto_size == 0, then size_factor == size
  vec2 size_factor = (1.0 - auto_size.x + (auto_size.x * center.z)) * size.xy;

  emitVertex(center, right, up, size_factor, vec2(-0.5,  0.5));
  emitVertex(center, right, up, size_factor, vec2(-0.5, -0.5));
  emitVertex(center, right, up, size_factor, vec2( 0.5,  0.5));
  emitVertex(center, right, up, size_factor, vec2( 0.5, -0.5));
  EndPrimitive();
}
