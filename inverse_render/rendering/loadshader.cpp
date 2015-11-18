#include "loadshader.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

const char* VertexFileShaderProgram::DEFAULTFRAGFILE = "default.f.glsl";

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

void FileShaderProgram::compile()
{
    std::string vertShaderStr = readFile(vfile.c_str());
    std::string fragShaderStr = readFile(ffile.c_str());
    const char *vertShaderSrc = vertShaderStr.c_str();
    const char *fragShaderSrc = fragShaderStr.c_str();
    compileFromSource(vertShaderSrc, fragShaderSrc);
}

void ShaderProgram::compileFromSource(const char* vertShaderSrc, const char* fragShaderSrc)
{
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);

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
}

void ShaderProgram::prelink()
{
    program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
}

void ShaderProgram::link()
{
    GLint result = GL_FALSE;
    int logLength;
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
}

void ShaderProgram::cleanup()
{
    glDeleteShader(vertShader);
    glDeleteShader(fragShader);
}

void GeometryShader::docompile() {
    GLint result = GL_FALSE;
    int logLength;
    // Compile geometry shader
    geomShader = glCreateShader(GL_GEOMETRY_SHADER);
    std::string geomShaderStr = readFile(gfile.c_str());
    const char *geomShaderSrc = geomShaderStr.c_str();
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

void GeometryShader::doprelink() {
    glAttachShader(program, geomShader);
}

void GeometryShader::docleanup() {
    glDeleteShader(geomShader);
}

TransformFeedbackShader::TransformFeedbackShader(ShaderProgram* component, std::vector<std::string> names)
    : ShaderProgramDecorator(component)
{
    numvaryings = names.size();
    varyings = new GLchar*[numvaryings];
    for (int i = 0; i < numvaryings; i++) {
        varyings[i] = new GLchar[names[i].length()+1];
        strcpy(varyings[i], names[i].c_str());
    }
}

TransformFeedbackShader::~TransformFeedbackShader() {
    for (int i = 0; i < numvaryings; i++) {
        delete [] varyings[i];
    }
    delete varyings;
}

void TransformFeedbackShader::doprelink() {
    glTransformFeedbackVaryings(program, numvaryings, (const GLchar**) varyings, GL_INTERLEAVED_ATTRIBS);
}
