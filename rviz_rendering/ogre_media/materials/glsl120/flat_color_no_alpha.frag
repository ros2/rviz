#version 120

// Passes the fragment color

uniform vec4 highlight;


void main()
{
  vec3 col = gl_Color.xyz + gl_Color.xyz * highlight.xyz;
  gl_FragColor = vec4(col, gl_Color.a);
}
