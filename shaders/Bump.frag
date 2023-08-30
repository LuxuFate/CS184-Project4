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

float h(vec2 uv) {
  // You may want to use this helper function...
    return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
    vec3 b = cross(v_normal.xyz, v_tangent.xyz);
    mat3 tbn = mat3(v_tangent.xyz, b, v_normal.xyz);

    float u = v_uv.x;
    float v = v_uv.y;
    float w = u_texture_2_size.x;
    float ht = u_texture_2_size.y;
    
    float dU = (h(vec2(u+1.0/w, v)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
    float dV = (h(vec2(u, v+1.0/ht)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
    vec3 n0 = vec3(-dU, -dV, 1.0);
    vec3 nd = tbn * n0;

    float ka = 0.1;
    float kd = 0.8;
    float ks = 0.5;
    vec4 Ia = vec4(1.0, 1.0, 1.0, 1.0);
    int p = 100;
    
    vec3 d = u_light_pos - v_position.xyz;
    vec4 diffuse = kd * vec4((u_light_intensity * vec3(1, 1, 1) / (length(d)*length(d))) * max(0, dot(nd, normalize(d))), 1);

    vec4 r = vec4(d, 1.0);
    float r2 = length(r)*length(r);

    vec4 l = normalize(vec4(u_light_pos, 1.0) - v_position);
    vec4 v2 = normalize(vec4(u_cam_pos, 1.0) - v_position);
    vec4 ir = vec4(u_light_intensity, 1.0) / r2;

    vec4 h2 = normalize(v2 + l);
                   
    vec4 ambient = ka * Ia;
                   
    vec4 specular = ks * ir * pow(max(0, dot(vec4(nd, 1.0), h2)), p);

    out_color = ambient + diffuse + specular;
    
}
