#include "opengl_compat.h"
#include "rendermanager.h"
#include "roommodel/geometrygenerator.h"
#include "loadshader.h"
#include "R3Graphics/R3Graphics.h"
#include <glm/glm.hpp>

const GLchar* uniformnames[NUM_UNIFORMS] = {
    "colors",
    "angles",
    "aux",
    "projectionmatrix",
    "modelviewmatrix",
    "auxdata",
};

std::string flatfragshadername = "flat.f.glsl";

void ShaderType::init() {
    ShaderProgram* prog;
    if (f&SHADERFLAGS_USESH_FLAT_FRAG) {
        prog = new GeometryShader(
                   new FileShaderProgram(n+".v.glsl", flatfragshadername),
                   n+".g.glsl"
               );
    } else if (f&SHADERFLAGS_VERTEX_COMPUTE) {
        std::vector<std::string> varyings;
        varyings.push_back("status");
        prog = new TransformFeedbackShader(
                   new VertexFileShaderProgram(n+".v.glsl"),
                   varyings
               );
    } else {
        prog = new VertexFileShaderProgram(n+".v.glsl");
    }
    prog->init();
    progid = prog->getProgId();
    delete prog;
    glUseProgram(progid);
    for (int j = 0; j < NUM_UNIFORM_TEXTURES; ++j) {
        if (f&(1<<j)) {
            GLuint u = glGetUniformLocation(progid,uniformnames[j]);
            glUniform1i(u, j+2);
            if (u == -1) {
                fprintf(stderr, "Error binding uniform %s in %s shader", uniformnames[j], n.c_str());
            }
            uniforms.push_back(u);
        } else {
            uniforms.push_back(-1);
        }
    }
    for (int j = NUM_UNIFORM_TEXTURES; j < NUM_UNIFORMS; ++j) {
        GLuint u = glGetUniformLocation(progid,uniformnames[j]);
        uniforms.push_back(u);
    }
    glUseProgram(0);
}

void RenderManager::initShaderTypes() {
    shaders.push_back(ShaderType("default", "Grayscale Lit"));
    shaders.push_back(ShaderType("normals", "Normals"));
    shaders.push_back(ShaderType("avg",
        "Weighted Average of Samples",
        SHADERFLAGS_USEU_COLOR|SHADERFLAGS_USEU_AUX));
    shaders.push_back(ShaderType("singleimage",
        "Reprojection of Single Image",
        SHADERFLAGS_USEU_COLOR|SHADERFLAGS_USEU_AUX));
    shaders.push_back(ShaderType("variance",
        "Surface Appearance Variance",
        SHADERFLAGS_USEU_COLOR|SHADERFLAGS_USEU_AUX));
    shaders.push_back(ShaderType("labels",
        "Per-Vertex Light IDs, Visibility, and Auxiliary Labels",
        SHADERFLAGS_USEU_AUX|SHADERFLAGS_USESH_FLAT_FRAG));
    shaders.push_back(ShaderType("overlay",
        "Overlay Labels",
        SHADERFLAGS_PASS|SHADERFLAGS_USESH_FLAT_FRAG));
    shaders.push_back(ShaderType("threshold",
        "Highlight pixels within a threshold",
        //SHADERFLAGS_USEU_COLOR|SHADERFLAGS_USEU_AUX|SHADERFLAGS_PASS|SHADERFLAGS_USESH_FLAT_FRAG));
        SHADERFLAGS_PASS|SHADERFLAGS_USESH_FLAT_FRAG));
    shaders.push_back(ShaderType("selectoverlay",
        "Selected vertices",
        SHADERFLAGS_PASS|SHADERFLAGS_USESH_FLAT_FRAG));
    shaders.push_back(ShaderType("precalculated",
        "Precalculated Colors",
        SHADERFLAGS_PASS));
    shaders.push_back(ShaderType("selectunion",
        "GPU Vertex Selection - Add to selection",
        SHADERFLAGS_USEU_AUX|SHADERFLAGS_PASS|SHADERFLAGS_VERTEX_COMPUTE));
    shaders.push_back(ShaderType("selectdiff",
        "GPU Vertex Selection - Remove from selection",
        SHADERFLAGS_USEU_AUX|SHADERFLAGS_PASS|SHADERFLAGS_VERTEX_COMPUTE));
}

RenderManager::RenderManager(MeshManager* meshmanager) {
    room = NULL;
    numroomtriangles = 0;
    samples_initialized = false;
    shaders_initialized = false;
    initShaderTypes();
    precalculated = -1;
    setMeshManager(meshmanager);
    selectbuf = NULL;
}

RenderManager::~RenderManager() {
    if (mmgr) {
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ibo);
        glDeleteBuffers(1, &roomvbo);
        glDeleteBuffers(1, &roomcbo);
        if (samples_initialized) glDeleteTextures(3, sampletex);
    }
    if (selectbuf) delete selectbuf;
}

void RenderManager::init() {
    openglInit();

    glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);

    // Initialize read-from-render info
    int ww = 640;
    int hh = 480;
    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &rfr_tex);
    glBindTexture(GL_TEXTURE_2D, rfr_tex);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, ww, hh, 0, GL_RGBA, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenRenderbuffers(1, &rfr_fbo_z);
    glBindRenderbuffer(GL_RENDERBUFFER, rfr_fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, ww, hh);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenFramebuffers(1, &rfr_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, rfr_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rfr_tex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rfr_fbo_z);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Initialize shaders
    for (int i = 0; i < shaders.size(); ++i) {
        shaders[i].init();
    }
    shaders_initialized = true;
}

void RenderManager::setMeshManager(MeshManager* meshmanager) {
    mmgr = meshmanager;
    if (!shaders_initialized) init();
    setupMeshGeometry();
}

void RenderManager::setRoomModel(roommodel::RoomModel* model) {
    room = model;
    setupRoomGeometry(model);
}

void RenderManager::setupMeshGeometry() {
    int varraysize = 2*3*mmgr->NVertices();
    float* vertices = new float[varraysize];
    glGenVertexArrays(1,&vaoid);
    glBindVertexArray(vaoid);
    // Initialize vertex positions and normals
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    int v = 0;
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        R3Point p = mmgr->VertexPosition(i);
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
        R3Vector n = mmgr->VertexNormal(i);
        vertices[v++] = n[0];
        vertices[v++] = n[1];
        vertices[v++] = n[2];
    }
    glBufferData(GL_ARRAY_BUFFER,
            varraysize*sizeof(float),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
    // Initialize triangle indices
    int iarraysize = 3*mmgr->NFaces();
    unsigned int* indices = new unsigned int[iarraysize];
    v = 0;
    for (int i = 0; i < mmgr->NFaces(); ++i) {
        indices[v++] = mmgr->VertexOnFace(i,0);
        indices[v++] = mmgr->VertexOnFace(i,1);
        indices[v++] = mmgr->VertexOnFace(i,2);
    }
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 iarraysize * sizeof(unsigned int),
                 indices, GL_STATIC_DRAW);
    delete [] indices;

    // Set up vertex attributes
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT, GL_FALSE, 6*sizeof(float), 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));

    glGenBuffers(1, &auxvbo);
    glBindBuffer(GL_ARRAY_BUFFER, auxvbo);
    glBufferData(GL_ARRAY_BUFFER,
            3*mmgr->NVertices()*sizeof(int),
            0, GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribIPointer(2, 3, GL_INT, 0, 0);
    glGenBuffers(1, &precalcvbo);
    glBindBuffer(GL_ARRAY_BUFFER, precalcvbo);
    glBufferData(GL_ARRAY_BUFFER,
            3*mmgr->NVertices()*sizeof(float),
            0, GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glBindVertexArray(0);

    // Set up vertex selection buffers
    glGenBuffers(2, selectvbo);
    for (int i = 0; i < 2; i++) {
        glBindBuffer(GL_ARRAY_BUFFER, selectvbo[i]);
        glBufferData(GL_ARRAY_BUFFER,
                sizeof(GLuint)*mmgr->NVertices(),
                0, GL_DYNAMIC_READ);
    }
    selectvbooutput = 0;
    selectbuf = new GLuint[mmgr->NVertices()];

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void RenderManager::setupMeshColors() {
    if (!shaders_initialized) init();
    // Sample texture layout:
    // For each vertex: Take max of 32 samples
    // Each sample is 3 "pixels"
        // Color
        // Incident angle
        // Confidence,solid angle,label/id
    static const int rowsize = 8192;
    static const int maxsamples = 32;
    int vertsperrow = rowsize/maxsamples;
    int nrows = (mmgr->NVertices()+vertsperrow-1)/vertsperrow;
    if (!samples_initialized) {
        glGenTextures(NUM_UNIFORM_TEXTURES, sampletex);
        for (int i = 0; i < NUM_UNIFORM_TEXTURES; ++i) {
            glActiveTexture(GL_TEXTURE0+i+2);
            glBindTexture(GL_TEXTURE_2D, sampletex[i]);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, rowsize, nrows, 0, GL_RGB, GL_FLOAT, 0);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        samples_initialized = true;
    }
    float* sampledata[NUM_UNIFORM_TEXTURES];
    for (int i = 0; i < NUM_UNIFORM_TEXTURES; ++i) {
        sampledata[i] = new float[rowsize*nrows*3];
        memset(sampledata[i], 0, sizeof(float)*rowsize*nrows*3);
    }
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        float* s[NUM_UNIFORM_TEXTURES];
        for (int j = 0; j < 3; ++j)  s[j] = sampledata[j] + i*maxsamples*3;
        int nsamplestoload = std::min(maxsamples, mmgr->getVertexSampleCount(i));
        std::vector<std::pair<float,int> > confidences;
        for (int j = 0; j < mmgr->getVertexSampleCount(i); ++j) {
            confidences.push_back(std::make_pair(mmgr->getSample(i,j).confidence, j));
        }
        std::sort(confidences.begin(), confidences.end(), std::greater<std::pair<float,int> >());
        for (int j = 0; j < nsamplestoload; ++j) {
            Sample sample = mmgr->getSample(i, confidences[j].second);
            s[0][3*j+0] = sample.r;
            s[0][3*j+1] = sample.g;
            s[0][3*j+2] = sample.b;

            s[1][3*j+0] = sample.x;
            s[1][3*j+1] = sample.y;
            s[1][3*j+2] = sample.z;

            s[2][3*j+0] = sample.confidence;
            s[2][3*j+1] = sample.dA;
            const uint16_t* tmp = (const uint16_t*)&sample.id;
            uint32_t l = sample.label;
            l = (l << 16) | *tmp;
            s[2][3*j+2] = glm::uintBitsToFloat(l);
        }
    }
    for (int i = 0; i < NUM_UNIFORM_TEXTURES; ++i) {
        glActiveTexture(GL_TEXTURE0+i+2);
        glBindTexture(GL_TEXTURE_2D, sampletex[i]);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, rowsize, nrows, GL_RGB, GL_FLOAT, sampledata[i]);
        glBindTexture(GL_TEXTURE_2D, 0);
        delete [] sampledata[i];
    }
    updateMeshAuxiliaryData();
}

void RenderManager::updateMeshAuxiliaryData() {
    glBindVertexArray(vaoid);
    // Initialize vertex positions and normals
    glBindBuffer(GL_ARRAY_BUFFER, auxvbo);
    int nch = 3;
    int* vertices = new int[nch*mmgr->NVertices()];
    int v = 0;
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        vertices[v++] = mmgr->getVertexSampleCount(i);
        for (int j = 1; j < nch; ++j) {
            vertices[v++] = mmgr->getLabel(i, j-1);
        }
    }
    glBufferData(GL_ARRAY_BUFFER,
            nch*mmgr->NVertices()*sizeof(int),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void RenderManager::precalculateSingleImage(int n) {
    glBindVertexArray(vaoid);
    glBindBuffer(GL_ARRAY_BUFFER, precalcvbo);
    float* colors = new float[3*mmgr->NVertices()];
    memset(colors, 0, sizeof(float)*3*mmgr->NVertices());
    float* curr = colors;
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        for (int j = 0; j < mmgr->getVertexSampleCount(i); ++j) {
            Sample sample = mmgr->getSample(i, j);
            if (sample.id == n) {
                curr[0] = sample.r;
                curr[1] = sample.g;
                curr[2] = sample.b;
            }
        }
        curr += 3;
    }
    glBufferData(GL_ARRAY_BUFFER,
            3*mmgr->NVertices()*sizeof(float),
            colors, GL_STATIC_DRAW);
    delete [] colors;
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    precalculated = VIEW_SINGLEIMAGE;
}

void RenderManager::precalculateAverageSamples() {
    glBindVertexArray(vaoid);
    glBindBuffer(GL_ARRAY_BUFFER, precalcvbo);
    float* colors = new float[3*mmgr->NVertices()];
    memset(colors, 0, sizeof(float)*3*mmgr->NVertices());
    float* curr = colors;
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        Material m = mmgr->getMedianVertexColor(i);
        for (int z = 0; z < 3; z++) *(curr++) = m(z);
    }
    glBufferData(GL_ARRAY_BUFFER,
            3*mmgr->NVertices()*sizeof(float),
            colors, GL_STATIC_DRAW);
    delete [] colors;
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    precalculated = VIEW_AVERAGE;
}

void RenderManager::precalculateVariance() {
    glBindVertexArray(vaoid);
    glBindBuffer(GL_ARRAY_BUFFER, precalcvbo);
    float* colors = new float[3*mmgr->NVertices()];
    memset(colors, 0, sizeof(float)*3*mmgr->NVertices());

    float* curr = colors;
    float avg[3];
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        double totalweight = 0;
        for (int z = 0; z < 3; z++) avg[z] = 0;
        for (int j = 0; j < mmgr->getVertexSampleCount(i); ++j) {
            Sample sample = mmgr->getSample(i, j);
            float w = std::abs(sample.confidence*sample.dA);
            avg[0] += sample.r*w;
            avg[1] += sample.g*w;
            avg[2] += sample.b*w;
            totalweight += w;
        }
        if (totalweight > 0) {
            for (int z = 0; z < 3; ++z) avg[z] /= totalweight;
        }
        double dev = 0;
        for (int j = 0; j < mmgr->getVertexSampleCount(i); ++j) {
            Sample sample = mmgr->getSample(i, j);
            float w = std::abs(sample.confidence*sample.dA);
            dev += (avg[0] - sample.r)*(avg[0] - sample.r)*w/((1+sample.r)*(1+sample.r));
            dev += (avg[1] - sample.g)*(avg[1] - sample.g)*w/((1+sample.g)*(1+sample.g));
            dev += (avg[2] - sample.b)*(avg[2] - sample.b)*w/((1+sample.b)*(1+sample.g));
        }
        if (totalweight > 0) {
            dev /= totalweight;
            for (int z = 0; z < 3; ++z) curr[z] = dev;
        }
        curr += 3;
    }

    glBufferData(GL_ARRAY_BUFFER,
            3*mmgr->NVertices()*sizeof(float),
            colors, GL_STATIC_DRAW);
    delete [] colors;
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    precalculated = VIEW_VARIANCE;
}

void RenderManager::setupRoomGeometry(roommodel::RoomModel* model) {
    if (!shaders_initialized) init();
    roommodel::GeometryGenerator gg(model);
    gg.generate();
    std::vector<double> triangles;
    gg.getTriangleGeometry(triangles);
    numroomtriangles = triangles.size()/6;
    float* vertices = new float[numroomtriangles*6];
    float* colors = new float[numroomtriangles*3];

    R4Matrix m = model->globaltransform;
    m = m.Inverse();
    R4Matrix reup = R4identity_matrix;
    reup.XRotate(M_PI/2);
    reup.YRotate(M_PI);
    reup.ZRotate(-M_PI/2);
    for (int i = 0; i < numroomtriangles; ++i) {
        R3Point p(triangles[6*i], triangles[6*i+1], triangles[6*i+2]);
        R3Vector n(triangles[6*i+3], triangles[6*i+4], triangles[6*i+5]);
        p = m*reup*p;
        n = m*reup*n;
        vertices[6*i] = p.X();
        vertices[6*i+1] = p.Y();
        vertices[6*i+2] = p.Z();
        vertices[6*i+3] = n.X();
        vertices[6*i+4] = n.Y();
        vertices[6*i+5] = n.Z();
    }
    glGenBuffers(1, &roomvbo);
    glBindBuffer(GL_ARRAY_BUFFER, roomvbo);
    glBufferData(GL_ARRAY_BUFFER,
            numroomtriangles*6*sizeof(float),
            vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    delete [] vertices;

    glGenBuffers(1, &roomcbo);
    glBindBuffer(GL_ARRAY_BUFFER, roomcbo);
    triangles.clear();
    gg.getTriangleVertexColors(triangles);
    for (int i = 0; i < triangles.size(); ++i) {
        colors[i] = triangles[i];
    }
    glBufferData(GL_ARRAY_BUFFER,
            numroomtriangles*3*sizeof(float),
            colors, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    delete [] colors;
}

void RenderManager::renderMesh(int rendermode) {
    if (!mmgr) return;
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    if (precalculated == rendermode) {
        rendermode = PRECALCULATE;
    }
    glUseProgram(shaders[rendermode].getProgID());
    if (samples_initialized) {
        for (int i = 0; i < NUM_UNIFORM_TEXTURES; ++i) {
            if (shaders[rendermode].getFlags()&(1<<i)) {
                glActiveTexture(GL_TEXTURE0+i+2);
                glBindTexture(GL_TEXTURE_2D, sampletex[i]);
            }
        }
    }
    glUniform3iv(shaders[rendermode].getUniform(UNIFORM_AUXDATA), 1, auxint);

    glBindVertexArray(vaoid);
    glBindBuffer(GL_ARRAY_BUFFER, selectvbo[selectvbooutput]);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 1, GL_UNSIGNED_INT, GL_FALSE, 0, 0);
    if (shaders[rendermode].getFlags()&SHADERFLAGS_VERTEX_COMPUTE) {
        glUniformMatrix4fv(shaders[rendermode].projectionUniform(), 1, GL_FALSE, selprojection);
        glUniformMatrix4fv(shaders[rendermode].modelviewUniform(), 1, GL_FALSE, selmodelview);
        selectvbooutput = 1 - selectvbooutput;
        glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, selectvbo[selectvbooutput]);
        glEnable(GL_RASTERIZER_DISCARD);
        glBeginTransformFeedback(GL_POINTS);
            glDrawArrays(GL_POINTS, 0, mmgr->NVertices());
        glEndTransformFeedback();
        glFlush();
        glDisable(GL_RASTERIZER_DISCARD);
    } else {
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
        glGetFloatv(GL_PROJECTION_MATRIX, projection);
        glUniformMatrix4fv(shaders[rendermode].projectionUniform(), 1, GL_FALSE, projection);
        glUniformMatrix4fv(shaders[rendermode].modelviewUniform(), 1, GL_FALSE, modelview);
        glDrawElements(GL_TRIANGLES, mmgr->NFaces()*3, GL_UNSIGNED_INT, 0);
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void RenderManager::renderRoom() {
    static R3DirectionalLight l(R3Vector(0.3, 0.5, -1), RNRgb(1, 1, 1), 1, TRUE);
    static R3Brdf b(RNRgb(0.2,0.2,0.2), RNRgb(0.8,0.8,0.8),
            RNRgb(0,0,0), RNRgb(0,0,0), 0.2, 1, 1);
    if (!room) return;
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glDisable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glColor3f(1,1,1);
    b.Draw();
    l.Draw(0);

    glMatrixMode(GL_MODELVIEW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, roomcbo);
    glColorPointer(3, GL_FLOAT, 3*sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, roomvbo);
    glVertexPointer(3, GL_FLOAT, 6*sizeof(float), 0);
    glNormalPointer(GL_FLOAT, 6*sizeof(float), (void*)(3*sizeof(float)));
    glDrawArrays(GL_TRIANGLES, 0, numroomtriangles);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glPopMatrix();
}

void RenderManager::lookThroughCamera(const CameraParams* cam) {
    const R3Point& p = cam->pos;
    const R3Vector& towards = cam->towards;
    const R3Vector& up = cam->up;
    glViewport(0,0,cam->width,cam->height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(cam->fov, cam->width/(double) cam->height, 0.001, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    R3Point at = p + towards;
    gluLookAt(p[0], p[1], p[2], at[0], at[1], at[2], up[0], up[1], up[2]);
}

void RenderManager::resizeReadFromRenderBuffer(int width, int height) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, rfr_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, rfr_fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

void RenderManager::readFromRender(const CameraParams* cam, float*& image, int rendermode, bool preallocated) {
    int w = cam->width;
    int h = cam->height;
    if (!preallocated) image = new float[w*h*3];

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    resizeReadFromRenderBuffer(w,h);
    lookThroughCamera(cam);
    glBindFramebuffer(GL_FRAMEBUFFER, rfr_fbo);
    if (rendermode == VIEW_LABELS) {
        float f = itof(0);
        glClearColor(f,f,f,f);
    } else {
        glClearColor(0.,0.,0.,0.);
    }
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderMesh(rendermode);
    glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,(void*)image);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glPopAttrib();
}

void RenderManager::getSelectedVertices(std::vector<int>& vertices) {
    glBindBuffer(GL_ARRAY_BUFFER, selectvbo[selectvbooutput]);
    glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLuint)*mmgr->NVertices(), selectbuf);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    for (int i = 0; i < mmgr->NVertices(); i++) {
        if (selectbuf[i]) vertices.push_back(i);
    }
}

void RenderManager::selectVertices(int x, int y, int w, int h, int r, int op) {
    // Prep shader data
    setShaderAuxInt((x<<16)|(h-y), 0);
    setShaderAuxInt((w<<16)|h, 1);
    setShaderAuxInt(r, 2);
    // Borrow rfr buffer
    resizeReadFromRenderBuffer(w,h);
    // Use last called mvp matrices
    glMatrixMode(GL_MODELVIEW_MATRIX);
    glLoadMatrixf(selmodelview);
    glMatrixMode(GL_PROJECTION_MATRIX);
    glLoadMatrixf(selprojection);
    // Render
    glBindFramebuffer(GL_FRAMEBUFFER, rfr_fbo);
    glClearColor(0.,0.,0.,0.);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // FIXME: Depth test
    if (op == SELECT_UNION || op == SELECT_DIFF) renderMesh(op);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderManager::clearSelectedVertices() {
    for (int i = 0; i < 2; i++) {
        glBindBuffer(GL_ARRAY_BUFFER, selectvbo[i]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLuint)*mmgr->NVertices(), 0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
void RenderManager::selectVertices(std::vector<int>& vertices) {
    memset(selectbuf, 0, sizeof(GLuint)*mmgr->NVertices());
    for (int i = 0; i < vertices.size(); i++) {
        selectbuf[vertices[i]] = 1;
    }
    for (int i = 0; i < 2; i++) {
        glBindBuffer(GL_ARRAY_BUFFER, selectvbo[i]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLuint)*mmgr->NVertices(), selectbuf);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void RenderManager::createLabelImage(const CameraParams* cam, void* image) {
    int w = cam->width;
    int h = cam->height;
    char* ret = (char*) image;
    float* rendered = new float[w*h*3];
    readFromRender(cam, rendered, VIEW_LABELS, true);
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            int idx = i*w+j;
            ret[i*w+j] = (char) ftoi(rendered[3*idx+2]);
        }
    }
    delete [] rendered;
}
