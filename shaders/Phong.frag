#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float k_a = 0.15;
  float k_d = 0.6;
  float k_s = 0.8;
  vec4 I_a = vec4(1, 1, 1, 1);
  float p = 30;
  vec4 norm_u_light_intensity = vec4(normalize(u_light_intensity), 0.0);
  float max_n_l = max(0.0, dot(v_normal, vec4(normalize(u_light_pos), 0.0)));
  vec4 h = (vec4(normalize(u_cam_pos), 0.0) + vec4(normalize(u_light_pos), 0.0)) / length(vec4(normalize(u_cam_pos), 0.0) + vec4(normalize(u_light_pos), 0.0));
  float max_n_h = max(0.0, dot(v_normal, h));
  out_color = u_color * ((k_a * I_a) + (k_d * norm_u_light_intensity * max_n_l) + (k_s * norm_u_light_intensity * pow(max_n_h, p)));
  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}