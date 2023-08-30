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
  
  // (Placeholder code. You will want to replace it.)
    
//    float ka = 0.1;
//    vec4 kd = u_color; //wtf shouldnt this be float
//    float ks = 0.5;
//    vec4 Ia = vec4(1.0, 1.0, 1.0, 1.0);
//    int p = 100;
//    
//    
//    vec4 r = vec4(u_light_pos, 1.0) - v_position;
//    vec4 ir = vec4(u_light_intensity, 1.0) / (length(r) * (length(r)));
//    
//    vec4 l = normalize(r);
//    vec4 m = normalize(vec4(u_cam_pos, 1.0) - v_position);
//    vec4 h = normalize(m + l);
//    
//    vec4 ambient = ka * Ia;
//    
//    vec4 diffuse = kd * ir * max(0, dot(v_normal, l));
//    
//    vec4 specular = ks * ir * pow(max(0, dot(v_normal, h)), p);
    
        float ka = 0.1;
        float kd = 1.0;
        float ks = 0.5;
        vec4 Ia = vec4(1.0, 1.0, 1.0, 1.0);
        int p = 100;
    
        vec3 d = u_light_pos - v_position.xyz;
        vec4 diffuse = kd * vec4((u_light_intensity * vec3(1, 1, 1) / (length(d)*length(d))) * max(0, dot(v_normal.xyz, normalize(d))), 1);
    
        vec4 r = vec4(d, 1.0);
        float r2 = length(r)*length(r);
    
        vec4 l = normalize(vec4(u_light_pos, 1.0) - v_position);
        vec4 v = normalize(vec4(u_cam_pos, 1.0) - v_position);
        vec4 ir = vec4(u_light_intensity, 1.0) / r2;
    
        vec4 h = normalize(v + l);
                       
        vec4 ambient = ka * Ia;
                       
        vec4 specular = ks * ir * pow(max(0, dot(normalize(v_normal), h)), p);
    
    out_color = ambient + diffuse + specular;
    
}

