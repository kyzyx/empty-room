#ifndef _OPENGL_COMPAT_H
#define _OPENGL_COMPAT_H
#ifdef __APPLE__
    #include <OpenGL/gl3.h>
    #include <OpenGL/glu.h>
    #include <OpenGL/gl3ext.h>
    #include <GLUT/glut.h>
    }
#else
#define _USE_GLEW 1
#ifdef _WIN32
    #include <windows.h>
#endif
    #include <GL/glew.h>
    #include <GL/glut.h>
#endif
bool openglInit();
#endif
