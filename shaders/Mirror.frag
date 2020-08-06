#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec4 w_o = vec4(u_cam_pos, 0) - v_position;
  vec4 w_i = -w_o + (2 * dot(w_o, v_normal)) * v_normal;
  out_color = texture(u_texture_cubemap, vec3(w_i));
  out_color.a = 1;
}
