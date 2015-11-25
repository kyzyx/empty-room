#include "opengl_compat.h"
#include "eruiglwidget.h"
#include "R3Graphics/R3Graphics.h"
#include <QMouseEvent>
#include <QBitmap>

#define MAX_CAMERAS 100

ERUIGLWidget::ERUIGLWidget(QWidget *parent) :
    HDRQGlViewerWidget(parent),
    orientationroommodel(NULL),
    renderoptions(this),
    selectedCamera(0),
    vertexselectmode(SELECT_UNION),
    vertexbrushsize(10),
    grid(NULL), cursor(NULL), defaultcursor(NULL)
{
    renderoptions.setRenderManager(&rendermanager);

}

ERUIGLWidget::~ERUIGLWidget() {
    if (defaultcursor) delete defaultcursor;
    if (cursor) delete cursor;
}

void ERUIGLWidget::_dosetup() {
    defaultcursor = new QCursor(Qt::OpenHandCursor);
    setCursor(*defaultcursor);
}

void ERUIGLWidget::setMeshManager(MeshManager* manager) {
    makeCurrent();
    rendermanager.setMeshManager(manager);
    // Compute scene center
    R3Box bbox = R3zero_box;
    for (int i = 0; i < manager->NVertices(); ++i) {
        bbox.Union(manager->VertexPosition(i));
    }
    setSceneRadius(bbox.DiagonalRadius());
    R3Point ctr = bbox.Centroid();
    setSceneCenter(qglviewer::Vec(ctr.X(), ctr.Y(), ctr.Z()));

    update();
    helper.emitSuggestRange(0,1);
}

void ERUIGLWidget::setupMeshColors()
{
    makeCurrent();
    rendermanager.setupMeshColors();
    update();
}

void ERUIGLWidget::setRoomModel(roommodel::RoomModel* model) {
    makeCurrent();
    rendermanager.setRoomModel(model);
    update();
}

void ERUIGLWidget::updateMeshAuxiliaryData() {
    makeCurrent();
    rendermanager.updateMeshAuxiliaryData();
    update();
}

void ERUIGLWidget::setInteractionMode(int mode) {
    interactionmode = mode;
    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, NO_CLICK_ACTION);
    setMouseBinding(Qt::ControlModifier, Qt::MiddleButton, NO_CLICK_ACTION);
    setMouseBinding(Qt::ControlModifier, Qt::RightButton, NO_CLICK_ACTION);
    if (interactionmode == INTERACTIONMODE_SELECT) {
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION, true);
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, NO_MOUSE_ACTION);
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, CAMERA, NO_MOUSE_ACTION);
        setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, NO_MOUSE_ACTION);
        setMouseBinding(Qt::ControlModifier, Qt::LeftButton, CAMERA, ROTATE);
        setMouseBinding(Qt::ControlModifier, Qt::MiddleButton, CAMERA, ZOOM);
        setMouseBinding(Qt::ControlModifier, Qt::RightButton, CAMERA, TRANSLATE);
        updateCursor();
    } else {
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, CAMERA, ZOOM);
        setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, TRANSLATE);
        setCursor(*defaultcursor);
    }
}

void ERUIGLWidget::setupCameras(ImageManager* imgr) {
    cameras.clear();
    for (int i = 0; i < imgr->size(); i++) {
        cameras.push_back(*imgr->getCamera(i));
        camcolors.push_back(RNRgb(0,1,0));
    }
    int inc = 1;
    if (imgr->size() > MAX_CAMERAS) {
        inc = imgr->size()/MAX_CAMERAS + 1;
    }
    for (int i = 0; i < imgr->size(); i += inc) {
        camids.push_back(i);
    }
    update();
}

void ERUIGLWidget::setupCameras(std::vector<CameraParams>& cams) {
    cameras.clear();
    for (int i = 0; i < cams.size(); i++) {
        cameras.push_back(cams[i]);
        camcolors.push_back(RNRgb(0,1,0));
    }
    int inc = 1;
    if (cams.size() > MAX_CAMERAS) {
        inc = cams.size()/MAX_CAMERAS + 1;
    }
    for (int i = 0; i < cams.size(); i += inc) {
        camids.push_back(i);
    }
    update();
}


void ERUIGLWidget::highlightCamera(int cameraindex) {
    renderoptions.setCurrentCamera(cameraindex);
    update();
}

void ERUIGLWidget::init()
{
  openglInit();
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

void ERUIGLWidget::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton &&
        (e->modifiers() == Qt::ControlModifier) !=
         (interactionmode == INTERACTIONMODE_SELECT))
    {
            makeCurrent();
            if (interactionmode != INTERACTIONMODE_SELECT) renderoptions.showSelected();
            rendermanager.selectVertices(e->x(), e->y(), width(), height(), vertexbrushsize, vertexselectmode);
            update();
    } else {
      HDRQGlViewerWidget::mousePressEvent(e);
    }
}

void ERUIGLWidget::mouseMoveEvent(QMouseEvent* e) {
    if ((e->buttons() & Qt::LeftButton) == Qt::LeftButton &&
        (e->modifiers() == Qt::ControlModifier) !=
         (interactionmode == INTERACTIONMODE_SELECT))
    {
            makeCurrent();
            if (interactionmode != INTERACTIONMODE_SELECT) renderoptions.showSelected();
            rendermanager.selectVertices(e->x(), e->y(), width(), height(), vertexbrushsize, vertexselectmode);
            update();
    } else {
      HDRQGlViewerWidget::mouseMoveEvent(e);
    }
}
void DrawCircle(QBitmap* img, QBitmap* mask, int r) {
    img->fill(Qt::color0);
    mask->fill(Qt::color0);
    QPainter* iptr = new QPainter(img);
    QPainter* mptr = new QPainter(mask);
    iptr->setPen(Qt::color1);
    mptr->setPen(Qt::color1);
    int w = img->width();
    int h = img->height();
    int x0 = w/2;
    int y0 = h/2;
    int x = r;
    int y = 0;
    int d = 1 - r;
    while (y <= x) {
        iptr->drawPoint( x + x0,  y + y0); // Octant 1
        iptr->drawPoint( y + x0,  x + y0); // Octant 2
        iptr->drawPoint(-x + x0,  y + y0); // Octant 4
        iptr->drawPoint(-y + x0,  x + y0); // Octant 3
        iptr->drawPoint(-x + x0, -y + y0); // Octant 5
        iptr->drawPoint(-y + x0, -x + y0); // Octant 6
        iptr->drawPoint( x + x0, -y + y0); // Octant 8
        iptr->drawPoint( y + x0, -x + y0); // Octant 7
        mptr->drawPoint( x + x0,  y + y0); // Octant 1
        mptr->drawPoint( y + x0,  x + y0); // Octant 2
        mptr->drawPoint(-x + x0,  y + y0); // Octant 4
        mptr->drawPoint(-y + x0,  x + y0); // Octant 3
        mptr->drawPoint(-x + x0, -y + y0); // Octant 5
        mptr->drawPoint(-y + x0, -x + y0); // Octant 6
        mptr->drawPoint( x + x0, -y + y0); // Octant 8
        mptr->drawPoint( y + x0, -x + y0); // Octant 7
        y++;
        if (d <= 0) {
            d += 2*y+1;
        } else {
            x--;
            d += 2*(y-x)+1;
        }
    }
}

void ERUIGLWidget::updateCursor() {
    QCursor* oldcursor = cursor;
    if (vertexbrushsize > 32) {
        cursor = new QCursor(Qt::CrossCursor);
    } else {
        QBitmap* bitmap;
        QBitmap* maskmap;
        if (vertexbrushsize <= 16) {
            bitmap = new QBitmap(32,32);
            maskmap = new QBitmap(32,32);
        } else if (vertexbrushsize <= 32) {
            bitmap = new QBitmap(64,64);
            maskmap = new QBitmap(64,64);
        }

        DrawCircle(bitmap, maskmap, vertexbrushsize);

        cursor = new QCursor(*bitmap,*maskmap);
    }

    setCursor(*cursor);
    if (oldcursor) delete oldcursor;
}

#define GLEXPAND(x) (x)[0], (x)[1], (x)[2]
static R3DirectionalLight l(R3Vector(0.3, 0.5, -1), RNRgb(1, 1, 1), 1, TRUE);
static R3Brdf b(RNRgb(0.2,0.2,0.2), RNRgb(0.8,0.8,0.8),
                RNRgb(0,0,0), RNRgb(0,0,0));
void ERUIGLWidget::draw()
{
    glDepthMask(true);
    glClearColor(0.,0.,0.,0.);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (renderoptions.shouldRenderMesh()) {
        if (renderoptions.getMeshRenderFormat() == VIEW_SINGLEIMAGE) {
            rendermanager.setShaderAuxInt(0,0);
            rendermanager.setShaderAuxInt(renderoptions.getCurrentCamera(), 1);
        }
        rendermanager.renderMesh(renderoptions.getMeshRenderFormat());
    }
    if (renderoptions.shouldRenderRoom())
        rendermanager.renderRoom();

    if (renderoptions.shouldRenderCameraTrajectory()) {
        glDisable(GL_LIGHTING);
        glBegin(GL_LINE_STRIP);
        glColor4f(0,1,0,0.5);
        for (size_t i = 0; i < camids.size(); ++i) {
            glVertex3f(GLEXPAND(cameras[camids[i]].pos));
        }
        glEnd();
    }
    if (renderoptions.shouldRenderAnyCameras()) {
        glEnable(GL_LIGHTING);
        glColor3f(1,1,1);
        b.Draw();
        l.Draw(0);
        for (size_t i = 0; i < camids.size(); ++i) {
            if (renderoptions.getCameraFormat() && camids[i] != renderoptions.getCurrentCamera()) {
                RNRgb rgb = camcolors[camids[i]];
                glColor4f(rgb[0], rgb[1], rgb[2], 0.5);
                renderCamera(cameras[camids[i]]);
            }
        }
        if (renderoptions.getCurrentCamera() < cameras.size() && renderoptions.getCameraFormat(true)) {
            glColor4f(1,0,1,0.5);
            renderCamera(cameras[renderoptions.getCurrentCamera()]);
        }
        glDisable(GL_LIGHTING);
    }
    if (renderoptions.shouldRenderMesh()) {
        //glEnable(GL_BLEND);
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if (renderoptions.shouldOverlay(VIEW_THRESHOLD)) {
            rendermanager.setShaderAuxInt(renderoptions.getLowerThreshold(),0);
            rendermanager.setShaderAuxInt(renderoptions.getUpperThreshold(),1);
            rendermanager.renderMesh(VIEW_THRESHOLD);
        }
        if (renderoptions.shouldOverlay(VIEW_LABELOVERLAY)) {
            for (int i = 0; i < 3; ++i) {
                rendermanager.setShaderAuxInt(i==renderoptions.getOverlayLabelIndex()?1:0,i);
            }
            rendermanager.renderMesh(VIEW_LABELOVERLAY);
        }
        if (renderoptions.shouldOverlay(VIEW_SELECTOVERLAY)) {
            rendermanager.renderMesh(VIEW_SELECTOVERLAY);
        }
        //glDisable(GL_BLEND);
    }
    if (renderoptions.shouldRenderWfHistogram()) {
        glEnable(GL_LIGHTING);
        glColor3f(1,1,1);
        b.Draw();
        l.Draw(0);
        renderHistogram();
        glDisable(GL_LIGHTING);
    }
    rendermanager.saveMatricesForSelection();
}

void ERUIGLWidget::drawWithNames() {
    float col[3];
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (renderoptions.shouldRenderAnyCameras()) {
        if (renderoptions.getCameraFormat()) {
            for (int i = 0; i < camids.size(); ++i) {
                IndexToRGB(i, col);
                glPushName(i);
                glColor3fv(col);
                renderCamera(cameras[camids[i]], true);
                glPopName();
            }
        }
    }
}

void ERUIGLWidget::postSelection(const QPoint &point) {
    int id = selectedName();
    if (id >= 0 && id < camids.size())
        emit cameraSelected(camids[id]);
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
    update();
}

#include <limits>
void ERUIGLWidget::computeWallFindingHistogram(double res) {
    gres = res;
    R4Matrix m;
    if (rendermanager.getRoomModel()) {
        m = rendermanager.getRoomModel()->globaltransform;
    } else if (orientationroommodel) {
        m = orientationroommodel->globaltransform;
    } else {
        return;
    }
    float maxx = -std::numeric_limits<float>::max();
    float maxz = -std::numeric_limits<float>::max();

    for (int i = 0; i < rendermanager.getMeshManager()->size(); ++i) {
        R3Point p = m*rendermanager.getMeshManager()->VertexPosition(i);
        maxx = std::max((float) p.X(), maxx);
        maxz = std::max((float) p.Z(), maxz);
    }
    gw = maxx/res + 1;
    gh = maxz/res + 1;
    if (grid) delete [] grid;
    grid = new int[gw*gh];
    bzero(grid, gw*gh*sizeof(int));
    for (int i = 0; i < rendermanager.getMeshManager()->size(); ++i) {
        R3Point p = m*rendermanager.getMeshManager()->VertexPosition(i);
        int r = p.Z()/res;
        int c = p.X()/res;
        if (r < 0 || c < 0 || r >= gh || c >= gw) continue;
        ++grid[r*gw+c];
    }
    gmax = *std::max_element(grid, grid+gw*gh);
}

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

void ERUIGLWidget::renderHistogram() {
    if (!grid) return;
    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    R4Matrix m;
    double scale;
    if (rendermanager.getRoomModel()) {
        m = rendermanager.getRoomModel()->globaltransform;
        scale = rendermanager.getRoomModel()->height/gmax;
    } else if (orientationroommodel) {
        m = orientationroommodel->globaltransform;
        scale = orientationroommodel->height/gmax;
    }
    m = m.Inverse();

    glBegin(GL_QUADS);
    for (int i = 0; i < gh; ++i) {
        for (int j = 0; j < gw; ++j) {
            R3Point p[4];
            R3Point t[4];
            p[0] = R3Point(j*gres, 0, i*gres);
            p[1] = R3Point((j+1)*gres, 0, i*gres);
            p[2] = R3Point((j+1)*gres, 0, (i+1)*gres);
            p[3] = R3Point(j*gres, 0, (i+1)*gres);
            double h = scale*std::min(grid[i*gw+j], renderoptions.getWfThreshold());
            for (int k = 0; k < 4; ++k) {
                t[k] = p[k] + h*R3yaxis_vector;
                p[k] = m*p[k];
                t[k] = m*t[k];
            }
            R3Vector n[3];
            n[0] = R3xaxis_vector;
            n[1] = R3yaxis_vector;
            n[2] = R3zaxis_vector;
            for (int k = 0; k < 3; ++k) n[k] = m*n[k];

            glColor4f(0.5,0.6,0.6,0.5);
            glNormal3f(GLEXPAND(-n[1]));
            glVertex3f(GLEXPAND(p[0]));
            glVertex3f(GLEXPAND(p[1]));
            glVertex3f(GLEXPAND(p[2]));
            glVertex3f(GLEXPAND(p[3]));

            glNormal3f(GLEXPAND(-n[2]));
            glVertex3f(GLEXPAND(p[0]));
            glVertex3f(GLEXPAND(p[1]));
            glVertex3f(GLEXPAND(t[1]));
            glVertex3f(GLEXPAND(t[0]));

            glNormal3f(GLEXPAND(n[0]));
            glVertex3f(GLEXPAND(p[1]));
            glVertex3f(GLEXPAND(p[2]));
            glVertex3f(GLEXPAND(t[2]));
            glVertex3f(GLEXPAND(t[1]));

            glNormal3f(GLEXPAND(n[2]));
            glVertex3f(GLEXPAND(p[2]));
            glVertex3f(GLEXPAND(p[3]));
            glVertex3f(GLEXPAND(t[3]));
            glVertex3f(GLEXPAND(t[2]));

            glNormal3f(GLEXPAND(-n[0]));
            glVertex3f(GLEXPAND(p[3]));
            glVertex3f(GLEXPAND(p[0]));
            glVertex3f(GLEXPAND(t[0]));
            glVertex3f(GLEXPAND(t[3]));

            if (grid[i*gw+j] >= renderoptions.getWfThreshold()) {
                p[0] = R3Point(j*gres, h, i*gres);
                p[1] = R3Point((j+1)*gres, h, i*gres);
                p[2] = R3Point((j+1)*gres, h, (i+1)*gres);
                p[3] = R3Point(j*gres, h, (i+1)*gres);
                h = scale*grid[i*gw+j];
                for (int k = 0; k < 4; ++k) {
                    t[k] = p[k];
                    t[k][1] = h;
                    p[k] = m*p[k];
                    t[k] = m*t[k];
                }

                glColor4f(1,0,0,0.5);

                glNormal3f(GLEXPAND(-n[2]));
                glVertex3f(GLEXPAND(p[0]));
                glVertex3f(GLEXPAND(p[1]));
                glVertex3f(GLEXPAND(t[1]));
                glVertex3f(GLEXPAND(t[0]));

                glNormal3f(GLEXPAND(n[0]));
                glVertex3f(GLEXPAND(p[1]));
                glVertex3f(GLEXPAND(p[2]));
                glVertex3f(GLEXPAND(t[2]));
                glVertex3f(GLEXPAND(t[1]));

                glNormal3f(GLEXPAND(n[2]));
                glVertex3f(GLEXPAND(p[2]));
                glVertex3f(GLEXPAND(p[3]));
                glVertex3f(GLEXPAND(t[3]));
                glVertex3f(GLEXPAND(t[2]));

                glNormal3f(GLEXPAND(-n[0]));
                glVertex3f(GLEXPAND(p[3]));
                glVertex3f(GLEXPAND(p[0]));
                glVertex3f(GLEXPAND(t[0]));
                glVertex3f(GLEXPAND(t[3]));
            }
            glNormal3f(GLEXPAND(n[1]));
            glVertex3f(GLEXPAND(t[0]));
            glVertex3f(GLEXPAND(t[1]));
            glVertex3f(GLEXPAND(t[2]));
            glVertex3f(GLEXPAND(t[3]));
        }
    }
    glEnd();
    glDisable(GL_COLOR_MATERIAL);
}
