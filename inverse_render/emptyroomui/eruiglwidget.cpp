#include <GL/glew.h>
#include "eruiglwidget.h"
#include "R3Graphics/R3Graphics.h"
#define MAX_LIGHTS 127
#define MAX_CAMERAS 100

ERUIGLWidget::ERUIGLWidget(QWidget *parent) :
    HDRQGlViewerWidget(parent), mmgr(NULL),
    hasColors(false), hasGeometry(false),
    selectedCamera(0), renderoptions(this)
{
}

void ERUIGLWidget::setMeshManager(MeshManager* manager) {
    mmgr = manager;
    setupMeshGeometry();
    helper.emitSuggestRange(0,1);
}

void ERUIGLWidget::setupMeshGeometry()
{
    makeCurrent();
    //int varraysize = 3*3*mmgr->NFaces();
    int varraysize = 2*3*mmgr->NVertices();
    float* vertices = new float[varraysize];
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    int v = 0;
    R3Box bbox = R3zero_box;
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        R3Point p = mmgr->VertexPosition(i);
        bbox.Union(p);
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
        R3Vector n = mmgr->VertexNormal(i);
        vertices[v++] = n[0];
        vertices[v++] = n[1];
        vertices[v++] = n[2];
    }
    setSceneRadius(bbox.DiagonalRadius());
    R3Point ctr = bbox.Centroid();
    setSceneCenter(qglviewer::Vec(ctr.X(), ctr.Y(), ctr.Z()));
    /*for (int i = 0; i < mmgr->NFaces(); ++i) {
        R3Point p = mmgr->VertexPosition(mmgr->VertexOnFace(i,0));
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
        p = mmgr->VertexPosition(mmgr->VertexOnFace(i,1));
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
        p = mmgr->VertexPosition(mmgr->VertexOnFace(i,2));
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
    }*/
    glBufferData(GL_ARRAY_BUFFER,
            varraysize*sizeof(float),
            vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    delete [] vertices;

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
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    delete [] indices;
    hasGeometry = true;
    updateGL();
}

void ERUIGLWidget::setupMeshColors()
{
    makeCurrent();
    static bool initialize = true;
    if (initialize) {
        glGenBuffers(1, &cbo);
    }
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    int varraysize = 2*3*mmgr->NVertices();
    float* vertices = new float[varraysize];
    memset(vertices, 0, sizeof(float)*varraysize);
    int v = 0;
    for (int i = 0; i < mmgr->NVertices(); ++i) {
        vertices[v++] = 0;
        vertices[v++] = mmgr->getLabel(i,1)/128.;
        vertices[v++] = 0;
        Material m = mmgr->getVertexColor(i);
        vertices[v++] = m.r*0.06;
        vertices[v++] = m.g*0.06;
        vertices[v++] = m.b*0.06;
    }
    for (int i = 0; i < mmgr->NFaces(); ++i) {
        // If any vertex has no samples, discard this face
        bool valid = true;
        // If all vertices are the same light vertices, this is a light face
        int light = 0;
        for (int j = 0; j < 3; ++j) {
            int n = mmgr->VertexOnFace(i, j);
            char l = mmgr->getLabel(n);
            if (mmgr->getVertexSampleCount(n) == 0) {
                valid = false;
            }
            if (light == 0 && l > 0) light = l;
            else if (light > 0 && l == 0) light = -1; // Half light
            else if (light > 0 && l != light) light = -2; // Differing lights, should never happen
        }
        if (!valid) {
            for (int j = 0; j < 3; ++j) {
                int n = mmgr->VertexOnFace(i,j);
                int ind = 6*n;
                vertices[ind+2] = 1;
            }
        }
        if (light > 0) {
            for (int j = 0; j < 3; ++j) {
                int n = mmgr->VertexOnFace(i,j);
                int ind = 6*n;
                vertices[ind+0] = light/(float) MAX_LIGHTS;
            }
        }
    }
    if (initialize) {
        glBufferData(GL_ARRAY_BUFFER, varraysize*sizeof(float), vertices, GL_DYNAMIC_DRAW);
        initialize = false;
    }
    else {
        glBufferSubData(GL_ARRAY_BUFFER, 0, varraysize*sizeof(float), vertices);
    }
    delete [] vertices;
    hasColors = true;
    updateGL();
}

void ERUIGLWidget::setupCameras(ImageManager* imgr) {
    int inc = 1;
    if (imgr->size() > MAX_CAMERAS) {
        inc = imgr->size()/MAX_CAMERAS + 1;
    }
    for (int i = 0; i < imgr->size(); i += inc) {
        cameras.push_back(*imgr->getCamera(i));
        camids.push_back(i);
    }
    updateGL();
}

void ERUIGLWidget::highlightCamera(int cameraindex) {
    std::vector<int>::iterator it = std::lower_bound(camids.begin(), camids.end(), cameraindex);
    if (it == camids.end()) return;
    int idx = *it;
    if (camids[idx] == cameraindex) {
        selectedCamera = idx;
    }
    updateGL();
}

void ERUIGLWidget::init()
{
  glewInit();
  restoreStateFromFile();
  setShortcut(EXIT_VIEWER, 0);
  setShortcut(DRAW_GRID, 0);
  setShortcut(DRAW_AXIS, 0);
  setShortcut(STEREO, 0);
  setShortcut(FULL_SCREEN, 0);
  setShortcut(ANIMATION, 0);
  setShortcut(SNAPSHOT_TO_CLIPBOARD, 0);
  //help();
}

QString ERUIGLWidget::helpString() const
{
  QString text("<h2>S i m p l e V i e w e r</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}
static R3DirectionalLight l(R3Vector(0.3, 0.3, -1), RNRgb(1, 1, 1), 1, TRUE);
static R3Brdf b(RNRgb(0.2,0.2,0.2), RNRgb(0.8,0.8,0.8),
                RNRgb(0,0,0), RNRgb(0,0,0), 0.2, 1, 1);
// Draws a spiral
void ERUIGLWidget::draw()
{
    if (!hasGeometry) {
        const float nbSteps = 200.0;
        glBegin(GL_QUAD_STRIP);
        for (int i=0; i<nbSteps; ++i) {
          const float ratio = i/nbSteps;
          const float angle = 21.0*ratio;
          const float c = cos(angle);
          const float s = sin(angle);
          const float r1 = 1.0 - 0.8f*ratio;
          const float r2 = 0.8f - 0.8f*ratio;
          const float alt = ratio - 0.5f;
          const float nor = 0.5f;
          const float up = sqrt(1.0-nor*nor);
          glColor3f(1.0-ratio, 0.2f , ratio);
          glNormal3f(nor*c, up, nor*s);
          glVertex3f(r1*c, alt, r1*s);
          glVertex3f(r2*c, alt+0.05f, r2*s);
        }
        glEnd();
    } else {
        glDepthMask(true);
        glClearColor(0.,0.,0.,0.);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (renderoptions.shouldRenderMesh() && hasGeometry) renderMesh();

        if (renderoptions.shouldRenderAnyCameras()) {
            glEnable(GL_LIGHTING);
            glColor3f(1,1,1);
            b.Draw();
            l.Draw(0);
            for (size_t i = 0; i < cameras.size(); ++i) {
                if (renderoptions.getCameraFormat(i == selectedCamera)) {
                    if (i == selectedCamera)
                        glColor3f(0,100,100);
                    else
                        glColor3f(0,100,0);
                    renderCamera(cameras[i]);
                }
            }
            glDisable(GL_LIGHTING);
        }
    }
}

void ERUIGLWidget::renderMesh() {
    bool light = true;
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glDisable(GL_LIGHT0);
    if (hasColors) {
        glDisable(GL_LIGHTING);
        glEnableClientState(GL_COLOR_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, cbo);
        glColorPointer(3, GL_FLOAT, 6*sizeof(float), (void*) (light?3*sizeof(float):0));
    } else {
        glEnable(GL_LIGHTING);
        glColor3f(1,1,1);
        b.Draw();
        l.Draw(0);
    }
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, 6*sizeof(float), 0);
    glNormalPointer(GL_FLOAT, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glDrawElements(GL_TRIANGLES, mmgr->NFaces()*3, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    if (hasColors) glDisableClientState(GL_COLOR_ARRAY);
}

void ERUIGLWidget::drawWithNames() {

    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (renderoptions.shouldRenderAnyCameras()) {
        float col[3];
        for (int i = 0; i < cameras.size(); ++i) {
            IndexToRGB(i, col);
            glPushName(i);
            glColor3fv(col);
            if (renderoptions.getCameraFormat(i == selectedCamera)) renderCamera(cameras[i], true);
            glPopName();
        }
    }
}

void ERUIGLWidget::postSelection(const QPoint &point) {
    if (selectedName() >= 0) emit cameraSelected(selectedName());
}

void ERUIGLWidget::lookThroughCamera(const CameraParams* cam) {
    makeCurrent();
    double camratio = camera()->aspectRatio();
    double ratio = cam->width/(double) cam->height;
    double vfov = cam->fov*M_PI/180.;
    if (ratio < camratio) {
        // vertical fov is limiting
        camera()->setFieldOfView(vfov);
    } else {
        double hfov = 2 * atan(tan(vfov/2) * ratio);
        camera()->setHorizontalFieldOfView(hfov);
    }

    camera()->setPosition(qglviewer::Vec(cam->pos[0], cam->pos[1], cam->pos[2]));
    camera()->setUpVector(qglviewer::Vec(cam->up[0], cam->up[1], cam->up[2]));
    camera()->setViewDirection(qglviewer::Vec(cam->towards[0], cam->towards[1], cam->towards[2]));
    updateGL();
}

#define GLEXPAND(x) (x)[0], (x)[1], (x)[2]
void ERUIGLWidget::renderCamera(const CameraParams &cam, bool id_only) {
    R3Point p = cam.pos;
    float f = 0.1;
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    if (renderoptions.getCameraFormat(true) == ERUIRenderOptions::CAMRENDER_AXES) {
        R3Point v2 = p + cam.towards*f;
        R3Point v3 = p + cam.up*f;
        glBegin(GL_LINES);
            glVertex3f(GLEXPAND(p));
            glVertex3f(GLEXPAND(v2));
            if (!id_only) glColor3f(100,0,0);
            glVertex3f(GLEXPAND(p));
            glVertex3f(GLEXPAND(v3));
        glEnd();
    }
    else if (renderoptions.getCameraFormat(true) == ERUIRenderOptions::CAMRENDER_FRUSTUM) {
        double w = cam.width/(2*cam.focal_length);
        double h = cam.height/(2*cam.focal_length);

        R3Point ul = p + cam.towards*f - cam.right*f*w + cam.up*f*h;
        R3Point ur = p + cam.towards*f + cam.right*f*w + cam.up*f*h;
        R3Point ll = p + cam.towards*f - cam.right*f*w - cam.up*f*h;
        R3Point lr = p + cam.towards*f + cam.right*f*w - cam.up*f*h;
        glDisable(GL_CULL_FACE);
        if (!id_only) glEnable(GL_LIGHTING);
        // glColor3f decided by caller!
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glBegin(GL_TRIANGLES);
                glVertex3f(GLEXPAND(p));
                glVertex3f(GLEXPAND(ul));
                glVertex3f(GLEXPAND(ur));
                glVertex3f(GLEXPAND(p));
                glVertex3f(GLEXPAND(ll));
                glVertex3f(GLEXPAND(lr));
                glVertex3f(GLEXPAND(p));
                glVertex3f(GLEXPAND(ur));
                glVertex3f(GLEXPAND(lr));
                glVertex3f(GLEXPAND(p));
                glVertex3f(GLEXPAND(ul));
                glVertex3f(GLEXPAND(ll));
            glEnd();
        if (!id_only) {
            glDisable(GL_LIGHTING);
            glColor3f(0,0,0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glBegin(GL_TRIANGLES);
                    glVertex3f(GLEXPAND(p));
                    glVertex3f(GLEXPAND(ul));
                    glVertex3f(GLEXPAND(ur));
                    glVertex3f(GLEXPAND(p));
                    glVertex3f(GLEXPAND(ll));
                    glVertex3f(GLEXPAND(lr));
                    glVertex3f(GLEXPAND(p));
                    glVertex3f(GLEXPAND(ur));
                    glVertex3f(GLEXPAND(lr));
                    glVertex3f(GLEXPAND(p));
                    glVertex3f(GLEXPAND(ul));
                    glVertex3f(GLEXPAND(ll));
                glEnd();
        }
    }
    glPopAttrib();
}
