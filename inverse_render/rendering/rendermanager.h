#ifndef _RENDERMANAGER_H
#define _RENDERMANAGER_H

#include "datamanager/meshmanager.h"
#include "datamanager/imagemanager.h"
#include "roommodel/roommodel.h"

enum {
    VIEW_DEFAULT,
    VIEW_NORMALS,
    VIEW_AVERAGE,
    VIEW_SINGLEIMAGE,
    VIEW_VARIANCE,
    VIEW_LABELS,
    VIEW_LABELOVERLAY,
    VIEW_THRESHOLD,
    VIEW_SELECTOVERLAY,
    PRECALCULATE,
    SELECT_UNION,
    SELECT_DIFF,
    NUM_VIEW_TYPES,
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
    SHADERFLAGS_USEU_COLOR=1,
    SHADERFLAGS_USEU_ANGLES=2,
    SHADERFLAGS_USEU_AUX=4,
    SHADERFLAGS_USEU_ALL=7,
    SHADERFLAGS_USESH_FLAT_FRAG=32,
    SHADERFLAGS_PASS=64,
    SHADERFLAGS_VERTEX_COMPUTE=128,
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
        std::string getDescription() const { return desc; }
        GLuint getProgID() const { return progid; }
        GLuint projectionUniform() const { return uniforms[UNIFORM_PROJECTION_MATRIX]; }
        GLuint modelviewUniform() const { return uniforms[UNIFORM_MODELVIEW_MATRIX]; }
        GLuint getUniform(int i) const { return uniforms[i]; }
    private:
        // Descriptive Variables
        std::string n;
        std::string desc;
        int f;
        // Opengl Variables
        GLuint progid;
        std::vector<GLuint> uniforms;
};

const int FLOAT_SIG_BITS = 23;
const int FLOAT_EXP_MASK = 1 + (1 << FLOAT_SIG_BITS);
inline int ftoi(float f) {
    return (*reinterpret_cast<int*>(&f)) - FLOAT_EXP_MASK;
}
inline float itof(int i) {
    int tmp = i + FLOAT_EXP_MASK;
    return (*reinterpret_cast<float*>(&tmp));
}

class RenderManager {
public:
    RenderManager() : mmgr(NULL), room(NULL), numroomtriangles(0), samples_initialized(false), shaders_initialized(false), precalculated(-1) { initShaderTypes(); }
    RenderManager(MeshManager* meshmanager);
    ~RenderManager();

    void setMeshManager(MeshManager* meshmanager);
    void setRoomModel(roommodel::RoomModel* model);

    void setupMeshColors();
    void setupMeshGeometry();
    void setupRoomGeometry(roommodel::RoomModel* model);

    void updateMeshAuxiliaryData();
    void precalculateAverageSamples();
    void precalculateSingleImage(int n);
    void precalculateVariance();
    bool hasPrecalculatedColors() const { return precalculated >= 0; }

    void readFromRender(const CameraParams* cam, float*& image, int rendermode, bool preallocated=false);
    void renderMesh(int rendermode);
    void renderRoom();
    void saveMatricesForSelection() {
        for (int i = 0; i < 16; i++) {
            selmodelview[i] = modelview[i];
            selprojection[i] = projection[i];
        }
    }

    void lookThroughCamera(const CameraParams* cam);

    void getSelectedVertices(std::vector<int>& vertices);
    void selectVertices(int x, int y, int w, int h, int r, int op=SELECT_UNION);
    void clearSelectedVertices();
    void selectVertices(std::vector<int>& vertices);
    void selectComponent(int x, int y, int w, int h);

    MeshManager* getMeshManager() { return mmgr; }

    void createLabelImage(const CameraParams* cam, void* image);
    const ShaderType& getShader(int n) const { return shaders[n]; }
    int getNumShaderTypes() const { return shaders.size(); }
    roommodel::RoomModel* getRoomModel() { return room; }

    void setShaderAuxInt(int aux, int i=0) { auxint[i] = aux; }
protected:
    void init();
    void initShaderTypes();
    void resizeReadFromRenderBuffer(int width, int height);

    MeshManager* mmgr;

    // Data for rendering room model from parametric form
    roommodel::RoomModel* room;
    R3Vector trans;
    int numroomtriangles;

    GLint auxint[3];

    // OpenGL object ids
    //   Mesh geometry info
    GLuint vbo, ibo;
    GLuint vaoid;
    //   Generated room geometry info
    GLuint roomvbo, roomcbo;
    //   Shader data ids
    GLuint sampletex[NUM_UNIFORM_TEXTURES];
    GLuint auxvbo, precalcvbo;
    //   Shader info
    std::vector<ShaderType> shaders;
    //   Read-from-render textures
    GLuint rfr_fbo, rfr_tex, rfr_fbo_z;
    //   GPU selection data
    GLuint selectvbo[2];
    int selectvbooutput;
    GLfloat selmodelview[16];
    GLfloat selprojection[16];

    // Buffer holding per-vertex selection data
    GLuint* selectbuf;

    // Last modelview and projection matrices
    GLfloat modelview[16];
    GLfloat projection[16];

    bool samples_initialized;
    bool shaders_initialized;
    int precalculated;
};

#endif
