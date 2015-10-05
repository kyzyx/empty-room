#include "opengl_compat.h"

bool openglInit() {
#ifdef _USE_GLEW
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) return false;
#endif
    return true;
}
