#include "loadshader.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

std::string readFile(const char *filePath) {
    std::string content;
    std::ifstream fileStream(filePath, std::ios::in);

    if(!fileStream.is_open()) {
        std::cerr << "Could not read file " << filePath << ". File does not exist." << std::endl;
        return "";
    }

    std::string line = "";
    while(!fileStream.eof()) {
        std::getline(fileStream, line);
        content.append(line + "\n");
    }

    fileStream.close();
    return content;
}

GLuint LoadShaderFromFiles(const std::string& vertex_path, const std::string& fragment_path, const std::string& geometry_path) {
    return LoadShaderFromFiles(vertex_path.c_str(), fragment_path.c_str(), geometry_path.c_str());
}
GLuint LoadShader(const char *vertShaderSrc, const char *fragShaderSrc, const char *geomShaderSrc) {
    GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    GLuint geomShader;

    GLint result = GL_FALSE;
    int logLength;

    // Compile vertex shader
    std::cout << "Compiling vertex shader." << std::endl;
    glShaderSource(vertShader, 1, &vertShaderSrc, NULL);
    glCompileShader(vertShader);

    // Check vertex shader
    glGetShaderiv(vertShader, GL_COMPILE_STATUS, &result);
    glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &logLength);
    std::vector<char> vertShaderError((logLength > 1) ? logLength : 1);
    glGetShaderInfoLog(vertShader, logLength, NULL, &vertShaderError[0]);
    std::cout << &vertShaderError[0] << std::endl;

    // Compile fragment shader
    std::cout << "Compiling fragment shader." << std::endl;
    glShaderSource(fragShader, 1, &fragShaderSrc, NULL);
    glCompileShader(fragShader);

    // Check fragment shader
    glGetShaderiv(fragShader, GL_COMPILE_STATUS, &result);
    glGetShaderiv(fragShader, GL_INFO_LOG_LENGTH, &logLength);
    std::vector<char> fragShaderError((logLength > 1) ? logLength : 1);
    glGetShaderInfoLog(fragShader, logLength, NULL, &fragShaderError[0]);
    std::cout << &fragShaderError[0] << std::endl;

    bool useGeometry = geomShaderSrc && strlen(geomShaderSrc) > 0;
    if (useGeometry) {
        // Compile geometry shader
        geomShader = glCreateShader(GL_GEOMETRY_SHADER);
        std::cout << "Compiling geometry shader." << std::endl;
        glShaderSource(geomShader, 1, &geomShaderSrc, NULL);
        glCompileShader(geomShader);

        // Check geometry shader
        glGetShaderiv(geomShader, GL_COMPILE_STATUS, &result);
        glGetShaderiv(geomShader, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> geomShaderError((logLength > 1) ? logLength : 1);
        glGetShaderInfoLog(geomShader, logLength, NULL, &geomShaderError[0]);
        std::cout << &geomShaderError[0] << std::endl;
    }

    std::cout << "Linking program" << std::endl;
    GLuint program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
    if (useGeometry) glAttachShader(program, geomShader);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &result);
    if (!result) {
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> programError( (logLength > 1) ? logLength : 1 );
        glGetProgramInfoLog(program, logLength, NULL, &programError[0]);
        std::cout << &programError[0] << std::endl;
    }
    glValidateProgram(program);
    glGetProgramiv(program, GL_VALIDATE_STATUS, &result);
    if (!result) {
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> programError( (logLength > 1) ? logLength : 1 );
        glGetProgramInfoLog(program, logLength, NULL, &programError[0]);
        std::cout << &programError[0] << std::endl;
    }
    glDeleteShader(vertShader);
    glDeleteShader(fragShader);
    if (useGeometry) glDeleteShader(geomShader);

    return program;
}


GLuint LoadShaderFromFiles(const char *vertex_path, const char *fragment_path, const char *geometry_path) {
    // Read shaders
    std::string vertShaderStr = readFile(vertex_path);
    std::string fragShaderStr = readFile(fragment_path);
    std::string geomShaderStr;
    bool useGeometry = strlen(geometry_path) > 0;
    if (useGeometry) geomShaderStr = readFile(geometry_path);
    const char *vertShaderSrc = vertShaderStr.c_str();
    const char *fragShaderSrc = fragShaderStr.c_str();
    return LoadShader(vertShaderSrc, fragShaderSrc, useGeometry?geomShaderStr.c_str():NULL);
}
