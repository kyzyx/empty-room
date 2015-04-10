#version 400
in vec2 v_coord;
uniform sampler2D rendered_image;
out vec2 f_texcoord;

// identity shader
void main(void) {
  gl_Position = vec4(v_coord, 0.0, 1.0);
  f_texcoord = (v_coord + 1.0) / 2.0;
}
