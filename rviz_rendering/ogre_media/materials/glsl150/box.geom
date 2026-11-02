#version 150 compatibility

// Generates an axis-aligned box with a given size
// for each input vertex.

#ifdef WITH_DEPTH
  uniform mat4 worldview_matrix;
  out float depth;
#endif

uniform mat4 worldviewproj_matrix;
uniform vec4 size;
uniform vec4 auto_size;

layout(points) in;
layout(triangle_strip, max_vertices=24) out;

const vec4 axes[3] = vec4[](
  vec4(1.0, 0.0, 0.0, 0.0),
  vec4(0.0, 1.0, 0.0, 0.0),
  vec4(0.0, 0.0, 1.0, 0.0));

// Texture coords follow nogp/box.vert (corner offset = -texcoord * size), so nogp/box.frag
// shades the faces exactly like the nogp technique.
void emitVertex(vec4 x, vec4 y, vec4 z, vec3 tex)
{
  // if auto_size == 1, then size_factor == size*gl_Vertex.z
  // if auto_size == 0, then size_factor == size
  vec4 size_factor = (1.0 - auto_size.x + (auto_size.x * gl_in[0].gl_Position.z)) * size;

  vec4 pos_rel = tex.x * x + tex.y * y + tex.z * z;
  vec4 pos = gl_in[0].gl_Position + vec4(pos_rel * size_factor * 0.5);
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = vec4(-0.5 * pos_rel.xyz, 0.0);
  gl_FrontColor = gl_in[0].gl_FrontColor;

#ifdef WITH_DEPTH
  depth = -((worldview_matrix * pos).z);
#endif

  EmitVertex();
}

void main()
{
  for (int side = 0; side < 3; side++) {
    vec4 x = axes[side];
    vec4 y = axes[(side + 1) % 3];
    vec4 z = axes[(side + 2) % 3];

    // face for +z
    emitVertex(x, y, z, vec3(-1, -1, +1));
    emitVertex(x, y, z, vec3(+1, -1, +1));
    emitVertex(x, y, z, vec3(-1, +1, +1));
    emitVertex(x, y, z, vec3(+1, +1, +1));
    EndPrimitive();

    // face for -z
    emitVertex(x, y, z, vec3(-1, -1, -1));
    emitVertex(x, y, z, vec3(-1, +1, -1));
    emitVertex(x, y, z, vec3(+1, -1, -1));
    emitVertex(x, y, z, vec3(+1, +1, -1));
    EndPrimitive();
  }
}
