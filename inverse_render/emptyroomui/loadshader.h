#ifndef LOADSHADER_H
#define LOADSHADER_H
#include "GL/gl.h"
#include <string>
std::string readFile(const char *filePath);
GLuint LoadShader(const char *vertShaderSrc, const char *fragShaderSrc);
GLuint LoadShaderFromFiles(const char *vertex_path, const char *fragment_path);
#endif // LOADSHADER_H
