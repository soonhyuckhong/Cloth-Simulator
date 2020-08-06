#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float hF(vec2 uv) {
  // You may want to use this helper function...
 return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  vec3 t = vec3(normalize(v_tangent));
  vec3 n = vec3(v_normal);
  vec3 b = cross(n, t);
  mat3 tbn = mat3(t, b, n);
  
  float u = v_uv.x;
  float v = v_uv.y;
  float w = u_texture_2_size.x;
  float h = u_texture_2_size.y;

  float dU = (hF(vec2(u + 1 / w, v)) - hF(v_uv)) * u_height_scaling * u_normal_scaling;
  float dV = (hF(vec2(u, v + 1 / h)) - hF(v_uv)) * u_height_scaling * u_normal_scaling;

  vec3 n_o = vec3(-dU, -dV, 1);
  vec4 n_d = vec4(normalize(tbn * n_o), 0.0);

  // Blinn-Phong Shading
  float k_a = 0.15;
  float k_d = 0.6;
  float k_s = 0.8;
  vec4 I_a = vec4(1, 1, 1, 1);
  float p = 30;

  vec4 norm_u_light_intensity = vec4(normalize(u_light_intensity), 0.0);
  float max_n_l = max(0.0, dot(n_d, vec4(normalize(u_light_pos), 0.0)));
  vec4 H = (vec4(normalize(u_cam_pos), 0.0) + vec4(normalize(u_light_pos), 0.0)) / length(vec4(normalize(u_cam_pos), 0.0) + vec4(normalize(u_light_pos), 0.0));
  float max_n_H = max(0.0, dot(n_d, H));

  out_color = u_color * ((k_a * I_a) + (k_d * norm_u_light_intensity * max_n_l) + (k_s * norm_u_light_intensity * pow(max_n_H, p)));

  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}