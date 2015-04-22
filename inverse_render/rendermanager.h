#ifndef _RENDERMANAGER_H
#define _RENDERMANAGER_H

#include "meshmanager.h"
#include "imagemanager.h"
#include "roommodel.h"

enum {
    VIEW_GEOMETRY,
    VIEW_AVERAGE,
    VIEW_SINGLEIMAGE,
    VIEW_LABELS,
};

enum {
    UNIFORM_COLOR=0,
    UNIFORM_ANGLES=1,
    UNIFORM_AUX=2,
    NUM_UNIFORM_TEXTURES=3,
    UNIFORM_PROJECTION_MATRIX=3,
    UNIFORM_MODELVIEW_MATRIX,
    UNIFORM_AUXDATA,
    NUM_UNIFORMS,
};

enum {
    SHADERFLAGS_USES_COLOR_UNIFORM=1,
    SHADERFLAGS_USES_ANGLES_UNIFORM=2,
    SHADERFLAGS_USES_AUX_UNIFORM=4,
    SHADERFLAGS_USES_ALL_UNIFORMS=7,
    SHADERFLAGS_USE_FLAT_FRAG_SHADER=32,
};

class ShaderType {
    public:
        ShaderType(
                const std::string& name,
                const std::string& description,
                int flags=0)
            : n(name), desc(description), f(flags)
        { ; }

        void init();

        int getFlags() const { return f; }
        GLuint getProgID() const { return progid; }
        GLuint projectionUniform() const { return uniforms[UNIFORM_PROJECTION_MATRIX]; }
        GLuint modelviewUniform() const { return uniforms[UNIFORM_MODELVIEW_MATRIX]; }
    private:
        // Descriptive Variables
        std::string n;
        std::string desc;
        int f;
        // Opengl Variables
        GLuint progid;
        std::vector<GLuint> uniforms;
};

class RenderManager {
public:
    RenderManager() : mmgr(NULL), room(NULL), numroomtriangles(0), samples_initialized(false), shaders_initialized(false) { initShaderTypes(); }
    RenderManager(MeshManager* meshmanager);
    ~RenderManager();

    void setMeshManager(MeshManager* meshmanager);
    void setRoomModel(roommodel::RoomModel* model);

    void setupMeshColors();
    void setupMeshGeometry();
    void setupRoomGeometry(roommodel::RoomModel* model);

    void updateMeshAuxiliaryData();

    void readFromRender(const CameraParams* cam, float*& image, int rendermode, bool preallocated=false);
    void renderMesh(int rendermode);
    void renderRoom();

    void lookThroughCamera(const CameraParams* cam);

    MeshManager* getMeshManager() { return mmgr; }
protected:
    void init();
    void initShaderTypes();
    void resizeReadFromRenderBuffer(int width, int height);

    MeshManager* mmgr;

    // Data for rendering room model from parametric form
    roommodel::RoomModel* room;
    R3Vector trans;
    int numroomtriangles;

    // OpenGL object ids
    //   Mesh geometry info
    GLuint vbo, ibo;
    GLuint vaoid;
    //   Generated room geometry info
    GLuint roomvbo, roomcbo;
    //   Shader data ids
    GLuint sampletex[NUM_UNIFORM_TEXTURES];
    GLuint auxvbo;
    //   Shader info
    std::vector<ShaderType> shaders;
    //   Read-from-render textures
    GLuint rfr_fbo, rfr_tex, rfr_fbo_z;

    bool samples_initialized;
    bool shaders_initialized;
};

#endif
