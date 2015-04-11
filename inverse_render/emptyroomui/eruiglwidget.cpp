#include <GL/glew.h>
#include "eruiglwidget.h"
#include "R3Graphics/R3Graphics.h"
#define MAX_LIGHTS 127

ERUIGLWidget::ERUIGLWidget(QWidget *parent) :
    HDRQGlViewerWidget(parent), mmgr(NULL), hasColors(false), hasGeometry(false)
{
}

void ERUIGLWidget::setMeshManager(MeshManager* manager) {
    mmgr = manager;
    setupMeshGeometry();
}

void ERUIGLWidget::setupMeshGeometry()
{
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
}

void ERUIGLWidget::setupMeshColors()
{
    glGenBuffers(1, &cbo);
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
            int ind = 6*(3*i + j);
            vertices[ind+1] = mmgr->getLabel(n,1)/128.;
            vertices[ind+2] = 1;
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
    glBufferData(GL_ARRAY_BUFFER, varraysize*sizeof(float),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
    hasColors = true;
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

// Draws a spiral
void ERUIGLWidget::draw()
{
    static R3DirectionalLight l(R3Vector(0.3, 0.3, -1), RNRgb(1, 1, 1), 1, TRUE);
    static R3Brdf b(RNRgb(0.2,0.2,0.2), RNRgb(0.8,0.8,0.8),
                    RNRgb(0,0,0), RNRgb(0,0,0), 0.2, 1, 1);
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
        bool light = true;

        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glDisable(GL_LIGHT0);
        glDepthMask(true);
        glClearColor(0.,0.,0.,0.);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexPointer(3, GL_FLOAT, 6*sizeof(float), 0);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(
            2,                               // attribute
            3,                               // size
            GL_FLOAT,                        // type
            GL_FALSE,                        // normalized?
            6*sizeof(float),                 // stride
            (void*)(3*sizeof(float))         // array buffer offset
        );
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glDrawElements(GL_TRIANGLES, mmgr->NFaces()*3, GL_UNSIGNED_INT, 0);
        //glDrawArrays(GL_TRIANGLES, 0, mesh->NFaces()*3);
        glDisableClientState(GL_VERTEX_ARRAY);
        if (hasColors) glDisableClientState(GL_COLOR_ARRAY);
    }
}
