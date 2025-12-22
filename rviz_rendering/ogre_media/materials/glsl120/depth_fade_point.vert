#version 120

// Vertex shader for point sprites with depth-based fading
// sets position, point size and alpha.

// This shader employs three techniques to simulate depth fading:
// 1. Scaling the rendered point shape based on world size and distance to the camera.
// 2. Reducing alpha based on distance to the camera for points smaller than a pixel.
// 3. Stochastic point removal for very small alphas to prevent alpha-related rendering artifacts.

uniform mat4 worldviewproj_matrix;
uniform mat4 worldview_matrix;
uniform mat4 projection_matrix;

uniform vec4 size; // size.x = world size of point sprites in world units [e.g. meters]
uniform float alpha; // global alpha multiplier
uniform float viewport_height;  // Required to convert world size to pixel size

const float MAX_PIXEL_SIZE = 100.0; // Limit max point size to avoid high fill rate
const float MIN_ALPHA_THRESHOLD = 0.04; // Minimum alpha threshold for stochastic point removal

// Simple hash function (point -> float [0,1])
float hash21(vec3 p)
{
  // add some randomness based on position
  p = fract(p * 0.3183099 + 0.1);
  p *= 17.0;
  return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

void main()
{
  gl_Position = worldviewproj_matrix * gl_Vertex;

  vec4 pos_rel_view = worldview_matrix * gl_Vertex;
  float depth = -pos_rel_view.z;

  if (depth < 1e-6)
  {
    // point is behind the camera, remove it
    gl_Position.w = -1.0; // outside frustum
    return;
  }

  float focal = projection_matrix[1][1];
  // Compute pixel size from world-size (size.x), focal, depth and viewport height
  // pixel_size = world_size * focal / depth * (viewport_height / 2)
  float pixelSize = size.x * focal / depth * (viewport_height * 0.5);
  gl_PointSize = clamp(pixelSize, 1.0, MAX_PIXEL_SIZE);

  // For very small points, reduce alpha to create a fading effect
  // alpha scales with pixel size squared (area)
  float sizeAlpha = clamp(pixelSize * pixelSize, 0.0, 1.0);
  float finalAlpha = gl_Color.a * alpha * sizeAlpha;

  // Stochastic point removal for very small alphas
  if (finalAlpha < MIN_ALPHA_THRESHOLD) {
    float h = hash21(gl_Vertex.xyz) * MIN_ALPHA_THRESHOLD;
    if (h > finalAlpha)
    {
      // remove point
      gl_Position.w = -1.0; // outside frustum
      return;
    }
    finalAlpha = MIN_ALPHA_THRESHOLD;
  }

  gl_FrontColor = vec4(gl_Color.rgb, finalAlpha);
}
