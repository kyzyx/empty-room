#ifndef LOADSHADER_H
#define LOADSHADER_H
#include "GL/gl.h"
#include <string>
std::string readFile(const char *filePath);
GLuint LoadShader(const char *vertShaderSrc, const char *fragShaderSrc, const char *geomShaderSrc=NULL);
GLuint LoadShaderFromFiles(const std::string& vertex_path, const std::string& fragment_path, const std::string& geometry_path="");
GLuint LoadShaderFromFiles(const char *vertex_path, const char *fragment_path, const char *geometry_path=NULL);
#endif // LOADSHADER_H
