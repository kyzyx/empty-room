#ifndef _RENDERMANAGER_H
#define _RENDERMANAGER_H

#include "meshmanager.h"
#include "imagemanager.h"
#include "roommodel.h"

enum {
    VIEW_GEOMETRY,
    VIEW_AVERAGE,
    VIEW_IMAGEID,
    VIEW_LABEL,
    VIEW_LIGHTID,
    NUM_VIEWOPTIONS
};

static const int NUM_UNIFORM_TEXTURES = 3;
enum {
    UNIFORM_SAMPLE_COLORS=0,
    UNIFORM_SAMPLE_ANGLES=1,
    UNIFORM_SAMPLE_AUX=2,
    UNIFORM_MODELVIEW=3,
    UNIFORM_PROJECTION=4,
    NUM_UNIFORMS=5,
};

class RenderManager {
public:
    RenderManager() : mmgr(NULL), room(NULL), numroomtriangles(0), samples_initialized(false), shaders_initialized(false) {;}
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
    GLuint progids[NUM_VIEWOPTIONS];
    GLuint uniformids[NUM_VIEWOPTIONS][NUM_UNIFORMS];
    //   Read-from-render textures
    GLuint rfr_fbo, rfr_tex, rfr_fbo_z;

    bool samples_initialized;
    bool shaders_initialized;
};

#endif
