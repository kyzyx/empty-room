#include <GL/glew.h>
#include <iostream>
#include <fstream>
#include "hdrglwidget.h"

using namespace std;

/*
char floatToColor(float f, float lo, float hi, int mapping) {
    int r = 0;
    if (mapping == TMO_LINEAR) {
        r = (int) (255*(f-lo)/(hi-lo));
    } else if (mapping == TMO_LOG) {
        r = (int) (255*(log(f) - log(lo))/(log(hi) - log(lo)));
    } else if (mapping == TMO_GAMMA22) {
        r = (int) (255*pow((f-lo)/(hi-lo), 2.2));
    }
    if (r > 255) return 255;
    if (r < 0) return 0;
    return (char) r;
}*/


HDRGlWidget::HDRGlWidget(QWidget *parent) :
    QGLWidget(parent), mapping(TMO_LINEAR), mini(0), maxi(1)
{
}

HDRGlWidget::~HDRGlWidget()
{
    glDeleteBuffers(1, &vbo_fbo_vertices);
    glDeleteBuffers(1, &fbo);
}

void HDRGlWidget::setMapping(int v) {
    if (v != mapping) {
        mapping = v;
        updateGL();
    }
}

void HDRGlWidget::setScale(int lo, int hi) {
    mini = LINTOLOG(lo);
    maxi = LINTOLOG(hi);
    updateGL();
}

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


GLuint LoadShader(const char *vertex_path, const char *fragment_path) {
    GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);

    // Read shaders
    std::string vertShaderStr = readFile(vertex_path);
    std::string fragShaderStr = readFile(fragment_path);
    const char *vertShaderSrc = vertShaderStr.c_str();
    const char *fragShaderSrc = fragShaderStr.c_str();

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

    std::cout << "Linking program" << std::endl;
    GLuint program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
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

    return program;
}

void HDRGlWidget::initializeGL() {
    int w = 640;
    int h = 480;

    // Initialize fbo
    glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);

    glGenTextures(1, &fbo_tex);
    glBindTexture(GL_TEXTURE_2D, fbo_tex);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, w, h, 0, GL_RGB, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenRenderbuffers(1, &fbo_z);
    glBindRenderbuffer(GL_RENDERBUFFER, fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_tex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, fbo_z);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        qDebug((const char*)gluErrorString(status) );
        return;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Initialize texture quad vertices
    GLfloat fbo_vertices[] = {
        -1, -1,
         1, -1,
        -1,  1,
         1,  1,
    };
    glGenBuffers(1, &vbo_fbo_vertices);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_fbo_vertices);
    glBufferData(GL_ARRAY_BUFFER, sizeof(fbo_vertices), fbo_vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    const char* shadername[NUM_TMOS] = {
        "hdr.f.glsl",
        "hdr_log.f.glsl",
        "hdr_gamma.f.glsl",
    };

    GLchar* uniforms[NUM_UNIFORMS] = {
        "rendered_image",
        "hdr_bounds",
    };
    GLchar* attribute_name = "v_coord";
    for (int i = 0; i < NUM_TMOS; ++i) {
        progs[i].progid = LoadShader("hdr.v.glsl", shadername[i]);
        progs[i].v_coord = glGetAttribLocation(progs[i].progid, attribute_name);
        if (progs[i].v_coord == -1) {
            qDebug("Error binding attribute %s in shader %s", attribute_name, shadername[i]);
            return;
        }
        for (int j = 0; j < NUM_UNIFORMS; ++j) {
            progs[i].uniform_ids[j] = glGetUniformLocation(progs[i].progid, uniforms[j]);
            if (progs[i].uniform_ids[j] == -1) {
                qDebug("Error binding uniform %s in shader %s", uniforms[j], shadername[i]);
                return;
            }
        }
    }

    _dosetup();
}

void HDRGlWidget::paintGL() {
    glEnable(GL_TEXTURE_2D);
    glUseProgram(0);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    _dorender();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);

    HDRShaderProgram& p = progs[mapping];
    glUseProgram(p.progid);
    glBindTexture(GL_TEXTURE_2D, fbo_tex);
    glUniform1i(p.uniform_ids[UNIFORM_FBO_TEXTURE], 0);
    glUniform3f(p.uniform_ids[UNIFORM_HDR_BOUNDS], mini, maxi, 2.2);
    glEnableVertexAttribArray(p.v_coord);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_fbo_vertices);
    glVertexAttribPointer(
      p.v_coord,  // attribute
      2,                  // number of elements per vertex, here (x,y)
      GL_FLOAT,           // the type of each element
      GL_FALSE,           // take our values as-is
      0,                  // no extra data between each position
      0                   // offset of first element
    );
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray(p.v_coord);
}

void HDRGlWidget::resizeGL(int width, int height) {
    glBindTexture(GL_TEXTURE_2D, fbo_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    updateGL();
    _doresize(width, height);
}
