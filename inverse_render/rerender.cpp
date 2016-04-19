#include "rerender.h"
#include "roommodel/geometrygenerator.h"
#include "lighting/generateimage.h"
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "Simplify.h"

using namespace std;
using namespace Eigen;

const double EPSILON = 0.005;
const double CIRCLETOLERANCE = 1.2;

int getFaceLightID(MeshManager& m, int f) {
    int lightid = -1;
    for (int j = 0; j < 3; ++j) {
        int n = m.VertexOnFace(f,j);
        unsigned char l = m.getLabel(n); // FIXME! Negative light id
        if (l == 0) {
            lightid = -1;
            break;
        } else {
            if (lightid <= 0) lightid = l;
            else if (lightid != l) {
                lightid = -1;
                break;
            }
        }
    }
    return lightid;
}

void estimateLightShape(MeshManager& m, int id, vector<R3Point>& points, vector<int>& indices) {
    vector<int> vid2point(m.NVertices(), -1);
    R3Point mean(0,0,0);
    for (int i = 0; i < m.NVertices(); ++i) {
        if (m.getLabel(i) == id) {
            points.push_back(m.VertexPosition(i));
            mean += points.back();
            vid2point[i] = points.size()-1;
        }
    }
    mean /= points.size();

    // PCA to find main directions
    MatrixXd A(points.size(), 3);
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j < 3; ++j) A(i,j) = points[i][j] - mean[j];
    }
    MatrixXd X = (A.transpose()*A)/(points.size() - 1);
    JacobiSVD<MatrixXd> svd(X, ComputeThinU | ComputeThinV);

    // Check for non-flat source
    if (svd.singularValues()[2] > EPSILON) {
        cout << "Non-flat source, outputting triangles" << endl;
        // Add light triangles to simplification structure
        for (int i = 0; i < points.size(); ++i) {
            Simplify::Vertex v;
            v.p = points[i] - R3null_point;
            Simplify::vertices.push_back(v);
        }
        for (int i = 0; i < m.NFaces(); ++i) {
            int lightid = getFaceLightID(m,i);
            if (lightid != id) continue;
            Simplify::Triangle t;
            for (int j = 0; j < 3; ++j) {
                int n = m.VertexOnFace(i,j);
                t.v[j] = vid2point[n];
            }
            Simplify::triangles.push_back(t);
        }
        // Simplify
        Simplify::simplify_mesh(60);
        // Recopy results
        points.clear();
        for (int i = 0; i < Simplify::vertices.size(); ++i) {
            points.push_back(Simplify::vertices[i].p + R3null_point);
        }
        for (int i = 0; i < Simplify::triangles.size(); ++i) {
            for (int j = 0; j < 3; ++j) indices.push_back(Simplify::triangles[i].v[j]);
        }
    } else if (svd.singularValues()[1] > EPSILON) {
        // Project points onto plane and find extrema
        Vector3d s = X.diagonal().cwiseSqrt();
        for (int i = 0; i < 3; ++i) if (s[i] == 0) s[i] = 1;
        MatrixXd variance = MatrixXd::Ones(points.size(),1)*s.transpose();
        MatrixXd Y = A.cwiseQuotient(variance)*svd.matrixV();
        Vector3d maxcoefs = Y.cwiseAbs().colwise().maxCoeff();
        Vector3d mincoefs = Y.cwiseAbs().colwise().minCoeff();

        // Check if approximately circular
        bool circle = true;
        double xv = X.diagonal()[0];
        double yv = X.diagonal()[1];
        cout << "Radius ratio: " << xv << " " << yv << endl;
        if (xv/yv > CIRCLETOLERANCE || yv/xv > CIRCLETOLERANCE) {
            cout << "Not a circle - aspect ratio irregular" << endl;
            circle = false;
        } else {
            // Count how many points lie outside the circle
            double xdiam = (maxcoefs-mincoefs)[0];
            double ydiam = (maxcoefs-mincoefs)[1];
            double r = max(xdiam,ydiam)/2;
            int numout = 0;
            for (int i = 0; i < points.size(); i++) {
                double d = Y(i,0)*Y(i,0) + Y(i,1)*Y(i,1);
                if (d - r*r > r*0.05) numout++;
            }
            cout << "Number of points outside circle: " << numout << "/" << points.size() << endl;
            if (numout/(double) points.size() > (4-M_PI)/M_PI) {
                cout << "Not a circle - points outside of circle" << endl;
                circle = false;
            }
        }

        if (circle) {
            Vector3d xax = Vector3d(maxcoefs[0],0,0).transpose()*svd.matrixV().transpose();
            Vector3d yax = Vector3d(0,maxcoefs[1],0).transpose()*svd.matrixV().transpose();
            Vector3d zax = xax.cross(yax);
            zax /= zax.norm();
            double radius = xax.norm();
            points.clear();
            points.push_back(mean);
            points.push_back(R3Point(radius, radius, radius));
            points.push_back(R3Point(zax(0), zax(1), zax(2)));
            points.push_back(R3Point(xax(0), xax(1), xax(2)));
        } else {
            // Transform extreme points back to original coordinates
            MatrixXd corners(4,3);
            corners << maxcoefs[0], maxcoefs[1],0,
                       maxcoefs[0], mincoefs[1],0,
                       mincoefs[0], mincoefs[1],0,
                       mincoefs[0], maxcoefs[1],0;
            corners = corners*svd.matrixV().transpose();
            points.clear();
            for (int i = 0; i < 4; ++i) {
                R3Point p;
                for (int j = 0; j < 3; ++j) {
                    p[j] = corners(i,j)*s[j] + mean[j];
                }
                points.push_back(p);
            }
            indices.push_back(0);
            indices.push_back(1);
            indices.push_back(2);
            indices.push_back(0);
            indices.push_back(2);
            indices.push_back(3);
        }
    } else {
        cout << "Error: Line source not implemented!" << endl;
    }
}

void outputRadianceFile(string filename, WallFinder& wf, MeshManager& m, InverseRender& ir) {
    ofstream out(filename);
    // output wall material
    out << "void plastic wallmat" << endl << 0 << endl << 0 << endl << 5 << " ";
    out << ir.wallMaterial(0) << " " << ir.wallMaterial(1) << " " << ir.wallMaterial(2);
    out << " 0 0" << endl << endl;

    // Output light materials
    // RADIANCE light parameters are in radiance; for diffuse surfaces the radiance L
    // is related to the radiosity J by
    // J = pi*L
    for (int i = 0; i < ir.lightintensities.size(); ++i) {
        if (!(ir.lightintensities[i]->typeId() & LIGHTTYPE_AREA)) continue;
        out << "void light l" << (i+1) << endl << 0 << endl << 0 << endl << 3 << " ";
        out << ir.lightintensities[i]->coef(0)/M_PI << " "
            << ir.lightintensities[i]->coef(1)/M_PI << " "
            << ir.lightintensities[i]->coef(2)/M_PI << endl << endl;
        vector<int> indices;
        vector<R3Point> points;
        int id = i+1;
        estimateLightShape(m, id, points, indices);
        if (points.size() == 3 && indices.size() == 0) {
            out << "l" << id << " ring light" << id << "_"  << endl << "0 0 8" << endl;
            out << points[0][0] << " " << points[0][1] << " " << points[0][2] << " ";
            out << points[2][0] << " " << points[2][1] << " " << points[2][2] << " ";
            out << "0 " << points[1][0] << endl << endl;
        } else {
            if (points.size()  == 4 && indices.size() == 6) {
                out << "l" << id << " polygon light" << id << "_"  << endl << "0 0 12" << endl;
                for (int i = 0; i < points.size(); ++i) {
                    for (int j = 0; j < 3; ++j) out << points[i][j] << " ";
                    out << endl;
                }
                out << endl;
            } else {
                out << "l" << id << " polygon light" << id << endl << "0 0 " << 3*indices.size() << endl;
                for (int i = 0; i < indices.size(); ++i) {
                    for (int j = 0; j < 3; ++j) out << points[indices[i]][j] << " ";
                    out << endl;
                }
                out << endl;
            }
        }
    }

    double hi = wf.ceilplane;
    double lo = wf.floorplane;

    // output walls
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        Eigen::Vector3f p[] = {
            wf.getWallEndpoint(i,1,0),
            wf.getWallEndpoint(i,1,1),
            wf.getWallEndpoint(i,0,1),
            wf.getWallEndpoint(i,0,0)
        };
        out << "wallmat polygon wall" << i << endl << 0 << endl << 0 << endl << 12 << " ";
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 3; ++k) out << p[j][k] << " ";
        }
        out << endl << endl;
    }
    double xmin = m.getMesh()->BBox().XMin();
    double zmin = m.getMesh()->BBox().ZMin();
    double xmax = m.getMesh()->BBox().XMax();
    double zmax = m.getMesh()->BBox().ZMax();
    // Output floor and ceiling
    out << "wallmat polygon ceil" << endl << 0 << endl << 0 << endl << 12 << " ";
    out << xmin << " " << hi << " " << zmin << " " << xmin << " " << hi << " " << zmax << " ";
    out << xmax << " " << hi << " " << zmax << " " << xmax << " " << hi << " " << zmin << endl << endl;
    out << "wallmat polygon floor" << endl << 0 << endl << 0 << endl << 12 << " ";
    out << xmin << " " << lo << " " << zmin << " " << xmax << " " << lo << " " << zmin << " ";
    out << xmax << " " << lo << " " << zmax << " " << xmin << " " << lo << " " << zmax << endl << endl;
}
void outputPlyFile(string filename, WallFinder& wf, MeshManager& m, InverseRender& ir) {
    ofstream out(filename);
    int n = wf.wallsegments.size()*2+8;
    int f = (wf.wallsegments.size()+2)*2;
    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << n << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "element face " << f << endl;
    out << "property list uchar int vertex_indices" << endl;
    out << "end_header" << endl;
    char color[50];
    sprintf(color, "%f %f %f", ir.wallMaterial(0), ir.wallMaterial(1), ir.wallMaterial(2));

    R3Box b;
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        Eigen::Vector3f p[] = {
            wf.getWallEndpoint(i,0,1),
            wf.getWallEndpoint(i,0,0)
        };
        Eigen::Vector3f c = wf.getNormalizedWallEndpoint(i,0,0);
        R3Point pp(c(0),c(1),c(2));
        b.Union(pp);
        for (int j = 0; j < 2; ++j) {
            out << p[j](0) << " " << p[j](1) << " " << p[j](2) << " ";
            out << color << endl;
        }
    }
    for (int i = 0; i < 8; ++i) {
        Vector4f p(b.Coord(i&1,0),((i>>2)&1)?wf.floorplane:wf.ceilplane,b.Coord((i>>1)&1,2),1);
        p = wf.getNormalizationTransform().inverse()*p;
        out <<  p(0)/p(3) << " " << p(1)/p(3) << " " << p(2)/p(3) << " ";
        out << color << endl;
    }
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        if (i == wf.wallsegments.size() - 1) {
            out << "3 " << 2*i << " " << 0 << " " << 2*i+1 << endl;
            out << "3 " << 0 << " " << 1 << " " << 2*i+1 << endl;
        } else {
            out << "3 " << 2*i << " " << 2*i+2 << " " << 2*i+1 << endl;
            out << "3 " << 2*i+2 << " " << 2*i+3 << " " << 2*i+1 << endl;
        }
    }
    n -= 8;
    // Floor triangles
    out << "3 " << n << " " << n+2 << " " << n+1 << endl;
    out << "3 " << n+1 << " " << n+2 << " " << n+3 << endl;
    // Ceiling triangles
    out << "3 " << n+4 << " " << n+5 << " " << n+6 << endl;
    out << "3 " << n+5 << " " << n+7 << " " << n+6 << endl;

}

void outputMaterial(ofstream& out, roommodel::Material m, const string& name) {
    if (m.texture) {
        out << "Texture \"FloorTexture\" \"color\" \"imagemap\"" << endl;
        out << "\t\"string filename\" [\"" << m.texture->filename << "\"]" << endl;
        out << "\t\"string wrap\" [\"repeat\"]" << endl;
        out << "\t\"float scale\" [" << m.texture->scale << "]" << endl << endl;
    }
    out << "MakeNamedMaterial \"" << name << "\"" << endl;
    if (m.texture) {
        out << "\t\"texture Kd\" [\"" << name << "Texture\"]" << endl;
    } else {
        out << "\t\"color Kd\" [";
        out << m.diffuse.r << " " << m.diffuse.g << " " << m.diffuse.b;
        out << "]" << endl;
    }
    out << "\t\"float sigma\" [0.0]" << endl;
    out << "\t\"string type\" [\"matte\"]" << endl << endl;
}

void outputTriangles(ofstream& out, vector<double>& triangles, const R4Matrix& t) {
    out << "\"point P\" [" << endl;
    for (int i = 0; i < triangles.size(); i += 5) {
        R3Point p(triangles[i], triangles[i+1], triangles[i+2]);
        p = t*p;
        out << '\t' << p[0] << " " << p[1] << " " << p[2] << endl;
    }
    out << "]" << endl;
    out << "\"integer indices\" [";
    for (int i = 0; i < triangles.size()/5; i++) {
        if (i%3 == 0) out << endl << '\t';
        out << i;
        if (i%3 != 2) out << " ";
    }
    out << "]" << endl;
    out << "\"float uv\" [" << endl;
    for (int i = 3; i < triangles.size(); i += 5) {
        out << '\t' << triangles[i] << " " << triangles[i+1] << endl;
    }
    out << "]" << endl;
}

void outputPbrtFile(
        std::string filename,
        roommodel::RoomModel* room,
        MeshManager& mmgr,
        vector<Light*>& lights,
        const CameraParams* cam) {
    ofstream out(filename);

    // Basic rendering info
    out << "# Main Scene File" << endl;
    out << "Scale -1 1 1" << endl;
    out << "Renderer \"sampler\"" << endl << endl;
    out << "Sampler \"lowdiscrepancy\"" << endl;
    out << "\"integer pixelsamples\" [1024]" << endl << endl;
    out << "SurfaceIntegrator \"path\"" << endl << endl;
    out << "PixelFilter \"mitchell\"" << endl;
    out << "\"float B\" [0.333333343267441]" << endl;
    out << "\"float C\" [0.333333343267441]" << endl;
    out << "\"float xwidth\" [2.000000000000000]" << endl;
    out << "\"float ywidth\" [2.000000000000000]" << endl << endl;
    out << "Film \"image\"" << endl;
    out << "\"integer xresolution\" [" << cam->width << "]" << endl;
    out << "\"integer yresolution\" [" << cam->height << "]" << endl;
    out << "LookAt ";
    for (int i = 0; i < 3; ++i) out << cam->pos[i] << " ";
    for (int i = 0; i < 3; ++i) out << cam->pos[i] + cam->towards[i] << " ";
    for (int i = 0; i < 3; ++i) out << cam->up[i] << " ";
    out << endl << endl;
    out << "Camera \"perspective\"" << endl;
    out << "\"float fov\" [" << cam->fov << "]" << endl;
    out << "\"float shutteropen\" [0.000000000000000]" << endl;
    out << "\"float shutterclose\" [0.041666666666667]" << endl << endl;

    // Generate geometry
    roommodel::GeometryGenerator gg(room);
    gg.generate();

    // Compute denormalization
    R4Matrix m = room->globaltransform;
    m = m.Inverse();
    R4Matrix reup = R4identity_matrix;
    reup.XRotate(M_PI/2);
    reup.YRotate(M_PI);
    reup.ZRotate(-M_PI/2);

    out << "WorldBegin" << endl;

    // Output materials
    outputMaterial(out, room->wallMaterial, "WallMaterial");
    outputMaterial(out, room->floorMaterial, "FloorMaterial");
    outputMaterial(out, room->ceilingMaterial, "CeilingMaterial");
    outputMaterial(out, room->baseboardMaterial, "BaseboardMaterial");

    vector<double> triangles;
    // Output wall geometry
    rectanglesToTriangles(gg.wallRectangles, triangles, false, false, true);
    if (!triangles.empty()) {
        out << "AttributeBegin" << endl;
        out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
        out << "NamedMaterial \"WallMaterial\"" << endl;
        out << "Shape \"trianglemesh\"" << endl;
        outputTriangles(out, triangles, m*reup);
        triangles.clear();
        out << "\"string name\" [\"Walls\"]" << endl;
        out << "AttributeEnd" << endl;
    }

    // Output baseboard geometry
    rectanglesToTriangles(gg.baseboardRectangles, triangles, false, false, true);
    if (!triangles.empty()) {
        out << "AttributeBegin" << endl;
        out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
        out << "NamedMaterial \"BaseboardMaterial\"" << endl;
        out << "Shape \"trianglemesh\"" << endl;
        outputTriangles(out, triangles, m*reup);
        triangles.clear();
        out << "\"string name\" [\"Baseboard\"]" << endl;
        out << "AttributeEnd" << endl;
    }

    // Output ceiling plane
    rectanglesToTriangles(gg.ceilRectangles, triangles, false, false, true);
    if (!triangles.empty()) {
        out << "AttributeBegin" << endl;
        out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
        out << "NamedMaterial \"CeilingMaterial\"" << endl;
        out << "Shape \"trianglemesh\"" << endl;
        outputTriangles(out, triangles, m*reup);
        triangles.clear();
        out << "\"string name\" [\"Ceiling\"]" << endl;
        out << "AttributeEnd" << endl;
    }

    // Output floor plane
    rectanglesToTriangles(gg.floorRectangles, triangles, false, false, true);
    if (!triangles.empty()) {
        out << "AttributeBegin" << endl;
        out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
        out << "NamedMaterial \"FloorMaterial\"" << endl;
        out << "Shape \"trianglemesh\"" << endl;
        outputTriangles(out, triangles, m*reup);
        triangles.clear();
        out << "\"string name\" [\"Floor\"]" << endl;
        out << "AttributeEnd" << endl;
    }

    // Output RWOs
    for (int i = 0; i < room->walls.size(); i++) {
        for (int j = 0; j < room->walls[i].windows.size(); j++) {

            stringstream matstream;
            matstream << "rwo" << i << "_" << j;
            string matname = matstream.str();
            outputMaterial(out, room->walls[i].windows[j].material, matname);
            vector<roommodel::Rect> windowrect;
            windowrect.push_back(gg.getRectangleForWindow(&(room->walls[i].windows[j])));
            rectanglesToTriangles(windowrect, triangles, false, false, true);
            if (!triangles.empty()) {
                out << "AttributeBegin" << endl;
                out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
                out << "NamedMaterial \"" << matname << "\"" << endl;
                out << "Shape \"trianglemesh\"" << endl;
                outputTriangles(out, triangles, m*reup);
                triangles.clear();
                out << "\"string name\" [\"Floor\"]" << endl;
                out << "AttributeEnd" << endl;
            }
        }
    }

    // Output light sources
    for (int i = 0; i < lights.size(); ++i) {
        RGBLight* rgbl = static_cast<RGBLight*>(lights[i]);
        stringstream lightfilenamestream;
        lightfilenamestream << "light" << i << ".exr";
        string lightfilename = lightfilenamestream.str();
        // Generate lightfile
        if (lights[i]->typeId() & LIGHTTYPE_LINE) {
            generateImage(rgbl, lightfilename);
            LineLight* ll = static_cast<LineLight*>(rgbl->getLight(0));
            for (int j = 0; j < ll->getNumSubdivs(); j++) {
                Vector3d p = ll->getSubpoint(j);
                Vector3d v = ll->getVector();
                Vector3d u = ll->getPerpendicularVector();
                Vector3d t = u.cross(v);
                out << "AttributeBegin" << endl;
                out << "Translate ";
                for (int k = 0; k < 3; k++) out << p[k] << " ";
                out << endl;
                out << "ConcatTransform [";
                // PBRT goniometric: X is "up", Y is light vector
                for (int x = 0; x < 3; x++)
                    out << u[x] << " ";
                out << " 0 ";
                for (int x = 0; x < 3; x++)
                    out << v[x] << " ";
                out << " 0 ";
                for (int x = 0; x < 3; x++)
                    out << t[x] << " ";
                out << " 0 ";
                out << "0 0 0 1]" << endl;
                out << "LightSource \"goniometric\" \"string mapname\" [\"" << lightfilename << "\"]" << endl;
                out << "AttributeEnd" << endl;
            }
        } else if (lights[i]->typeId() & LIGHTTYPE_POINT) {
            generateImage(rgbl, lightfilename);
            PointLight* pl = static_cast<PointLight*>(rgbl->getLight(0));
            out << "AttributeBegin" << endl;
            out << "Translate ";
            for (int j = 0; j < 3; j++) out << pl->getPosition(j) << " ";
            out << endl;
            out << "LightSource \"goniometric\" \"string mapname\" [\"" << lightfilename << "\"]" << endl;
            out << "AttributeEnd" << endl;
        } else if (lights[i]->typeId() & LIGHTTYPE_SH) {
            generateImage(rgbl, lightfilename);
            out << "AttributeBegin" << endl;
            out << "LightSource \"infinite\" \"string mapname\" [\"" << lightfilename << "\"]" << endl;
            out << "AttributeEnd" << endl;
        } else if (lights[i]->typeId() & LIGHTTYPE_ENVMAP) {
            generateImage(rgbl, lightfilename);
            out << "AttributeBegin" << endl;
            out << "LightSource \"infinite\" \"string mapname\" [\"" << lightfilename << "\"]" << endl;
            out << "AttributeEnd" << endl;
        } else if (lights[i]->typeId() & LIGHTTYPE_AREA) {
            out << "AttributeBegin" << endl;
            vector<R3Point> pts;
            vector<int> indices;
            estimateLightShape(mmgr, i+1, pts, indices);
            if (pts.size() == 4 && indices.size() == 0) {
                if (pts.size() == 4) {
                    // Disk area light
                    out << "AreaLightSource \"diffuse\" \"rgb L\" [";
                    for (int j = 0; j < 3; ++j) out << lights[i]->coef(j)/M_PI << " ";
                    out << "]" << endl;
                    Matrix3d rot;
                    Vector3d axes[3];
                    axes[0] = Vector3d(pts[3][0], pts[3][1], pts[3][2]);
                    axes[2] = Vector3d(pts[2][0], pts[2][1], pts[2][2]);
                    axes[1] = axes[2].cross(axes[0]);
                    for (int j = 0; j < 3; ++j) rot.col(j) = axes[j];
                    rot = rot.inverse();
                    out << "Transform [";
                    for (int j = 0; j < 3; ++j) {
                        for (int k = 0; k < 3; ++k) out << rot(j,k) << " ";
                        out << pts[0][j] << " ";
                    }
                    out << "0 0 0 1]" << endl;
                    out << "Shape \"disk\" \"float radius\" [" << pts[1][0] << "]" << endl;
                } else if (pts.size() == 1) {
                    // Point light
                    double radius = 0.02;
                    out << "AreaLightSource \"diffuse\" \"rgb L\" [";
                    for (int j = 0; j < 3; ++j) out << lights[i]->coef(j)/(M_PI*radius*radius) << " ";
                    out << "]" << endl;
                    out << "Transform [1 0 0 " << pts[0][0] <<
                        " 0 1 0 " << pts[0][1] <<
                        " 0 0 1 " << pts[0][2] <<
                        " 0 0 0 1]" << endl;
                    out << "Shape \"sphere\" \"float radius\" [" << radius << "]" << endl;
                }
            } else {
                out << "AreaLightSource \"diffuse\" \"rgb L\" [";
                for (int j = 0; j < 3; ++j) out << lights[i]->coef(j)/M_PI << " ";
                out << "]" << endl;
                out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
                for (int j = 0; j < pts.size(); ++j) {
                    Vector4f q(pts[j][0],pts[j][1],pts[j][2],1);
                    /*
                       q = wf.getNormalizationTransform()*q;
                       q /= q(3);
                       R3Point tmp = b.ClosestPoint(R3Point(q(0),q(1),q(2)));
                       q = wf.getNormalizationTransform().inverse()*Vector4f(tmp[0], tmp[1], tmp[2], 1);
                       q /= q(3);*/
                    pts[j] = R3Point(q(0),q(1),q(2));
                }
                out << "Shape \"trianglemesh\"" << endl;
                out << "\"point P\" [" << endl;
                for (int j = 0; j < pts.size(); ++j) {
                    out << '\t';
                    for (int k = 0; k < 3; ++k) out << pts[j][k] << " ";
                    out << endl;
                }
                out << "]" << endl;
                out << "\"integer indices\" [" << endl;
                for (int j = 0; j < indices.size(); j+=3) {
                    out << '\t';
                    for (int k = 0; k < 3; ++k) out << indices[j+k] << " ";
                    out << endl;
                }
                out << "]" << endl;
            }
            out << "AttributeEnd" << endl;
        }
    }
    out << "WorldEnd" << endl;
}
