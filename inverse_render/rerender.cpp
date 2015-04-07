#include "rerender.h"
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "Simplify.h"

using namespace std;
using namespace Eigen;

const double EPSILON = 0.0001;
const double CIRCLETOLERANCE = 1.2;

int getFaceLightID(MeshManager& m, int f) {
    int lightid = -1;
    for (int j = 0; j < 3; ++j) {
        int n = m.VertexOnFace(f,j);
        char l = m.getLabel(n);
        if (l <= 0) {
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
        Vector3d maxcoefs = Y.colwise().maxCoeff();
        Vector3d mincoefs = Y.colwise().minCoeff();

        // Check if approximately circular
        bool circle = true;
        double xv = X.diagonal()[0];
        double yv = X.diagonal()[1];
        cout << "Radius ratio: " << xv << " " << yv << endl;
        if (xv/yv > CIRCLETOLERANCE || yv/xv > CIRCLETOLERANCE) {
            cout << "Not a circle - aspect ratio irregular" << endl;
            circle = false;
        } else {
            // Check if points fall in the circle
            double maxradius = Y.cwiseProduct(Y).rowwise().sum().maxCoeff();
            double xdiam = (maxcoefs-mincoefs)[0];
            double ydiam = (maxcoefs-mincoefs)[1];
            double r = max(xdiam,ydiam)/2;
            cout << "Max radius: " << maxradius << "/" << r*r << endl;
            if (maxradius - r*r > 0.64) {
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
    for (int i = 0; i < ir.lights.size(); ++i) {
        out << "void light l" << (i+1) << endl << 0 << endl << 0 << endl << 3 << " ";
        out << ir.lights[i](0)/M_PI << " " << ir.lights[i](1)/M_PI << " " << ir.lights[i](2)/M_PI << endl << endl;
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
void outputPbrtFile(string filename, WallFinder& wf, MeshManager& m, InverseRender& ir, Texture floortex, const CameraParams* cam, string floortexfilename) {
    ofstream out(filename);

    // Basic rendering info
    out << "# Main Scene File" << endl;
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
    out << "\"string filename\" [\"scene.exr\"]" << endl << endl;
    out << "LookAt ";
    for (int i = 0; i < 3; ++i) out << cam->pos[i] << " ";
    for (int i = 0; i < 3; ++i) out << cam->up[i] << " ";
    for (int i = 0; i < 3; ++i) out << cam->towards[i] << " ";
    out << endl << endl;
    out << "Camera \"perspective\"" << endl;
    out << "\"float fov\" [" << cam->fov << "]" << endl;
    out << "\"float shutteropen\" [0.000000000000000]" << endl;
    out << "\"float shutterclose\" [0.041666666666667]" << endl << endl;

    // Calculate room dimensions and aspect ratio
    R3Box b = R3null_box;
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        Eigen::Vector3f c = wf.getNormalizedWallEndpoint(i,0,0);
        R3Point p(c(0),c(1),c(2));
        b.Union(p);
    }
    double roomw = b.XLength();
    double roomd = b.ZLength();

    out << "WorldBegin" << endl;
    // Output wall material
    out << "MakeNamedMaterial \"Wall\"" << endl;
    out << "\t\"color Kd\" [";
    for (int i = 0; i < 3; ++i) out << ir.wallMaterial(i) << " ";
    out << "]" << endl;
    out << "\t\"float sigma\" [0.0]" << endl;
    out << "\t\"string type\" [\"matte\"]" << endl << endl;
    // Output floor material
    if (floortexfilename.empty()) {
        out << "MakeNamedMaterial \"FloorMaterial\"" << endl;
        out << "\t\"color Kd\" [";
        for (int i = 0; i < 3; ++i) out << ir.wallMaterial(i) << " ";
        out << "]" << endl;
    } else {
        // Output floor texture
        out << "Texture \"FloorTexture\" \"color\" \"imagemap\"" << endl;
        out << "\t\"string filename\" [\"" << floortexfilename << "\"]" << endl;
        out << "\t\"string wrap\" [\"repeat\"]" << endl;
        out << "\t\"float scale\" [" << floortex.scale/min(roomw, roomd) << "]" << endl << endl;
        out << "MakeNamedMaterial \"FloorMaterial\"" << endl;
        out << "\t\"texture Kd\" [\"FloorTexture\"]" << endl;
    }
    out << "\t\"float sigma\" [0.0]" << endl;
    out << "\t\"string type\" [\"matte\"]" << endl << endl;
    // Output room geometry
    out << "AttributeBegin" << endl;
    out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
    out << "NamedMaterial \"Wall\"" << endl;
    out << "Shape \"trianglemesh\"" << endl;
    out << "\"point P\" [" << endl;
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        Eigen::Vector3f p[] = {
            wf.getWallEndpoint(i,0,1),
            wf.getWallEndpoint(i,0,0)
        };
        for (int j = 0; j < 2; ++j) {
            out << '\t' << p[j](0) << " " << p[j](1) << " " << p[j](2) << endl;
        }
    }
    for (int i = 0; i < 8; ++i) {
        Vector4f p(b.Coord(i&1,0),((i>>2)&1)?wf.floorplane:wf.ceilplane,b.Coord((i>>1)&1,2),1);
        p = wf.getNormalizationTransform().inverse()*p;
        out <<  '\t' << p(0)/p(3) << " " << p(1)/p(3) << " " << p(2)/p(3) << endl;
    }
    out << "]" << endl;
    out << "\"integer indices\" [" << endl;
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        if (i == wf.wallsegments.size() - 1) {
            out << '\t' << 2*i << " " << 0 << " " << 2*i+1 << endl;
            out << '\t' << 0 << " " << 1 << " " << 2*i+1 << endl;
        } else {
            out << '\t' << 2*i << " " << 2*i+2 << " " << 2*i+1 << endl;
            out << '\t' << 2*i+2 << " " << 2*i+3 << " " << 2*i+1 << endl;
        }
    }
    int n = wf.wallsegments.size()*2;
    // Ceiling triangles
    out << '\t' << n+4 << " " << n+5 << " " << n+6 << endl;
    out << '\t' << n+5 << " " << n+7 << " " << n+6 << endl;
    out << "]" << endl;
    out << "\"bool generatetangents\" [\"false\"]" << endl;
    out << "\"string name\" [\"Room\"]" << endl;
    out << "AttributeEnd" << endl;

    // Output floor plane
    out << "AttributeBegin" << endl;
    out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
    out << "NamedMaterial \"FloorMaterial\"" << endl;
    out << "Shape \"trianglemesh\"" << endl;
    out << "\"point P\" [" << endl;
    for (int i = 0; i < 4; ++i) {
        Vector4f p(b.Coord(i&1,0),wf.floorplane,b.Coord((i>>1)&1,2),1);
        p = wf.getNormalizationTransform().inverse()*p;
        out <<  '\t' << p(0)/p(3) << " " << p(1)/p(3) << " " << p(2)/p(3) << endl;
    }
    out << "]" << endl;
    out << "\"integer indices\" [" << endl;
        out << '\t' << 0 << " " << 2 << " " << 1 << endl;
        out << '\t' << 1 << " " << 2 << " " << 3 << endl;
    out << "]" << endl;
    out << "\"float uv\" [" << endl;
    if (roomw < roomd) {
        double s = roomd/roomw;
        out << "\t0 0 1 0 0 " << s << " 1 " << s;
    } else {
        double s = roomw/roomd;
        out << "\t0 0 " << s << " 0 0 1 " << s << " 1";
    }
    out << "]" << endl;
    out << "\"bool generatetangents\" [\"false\"]" << endl;
    out << "\"string name\" [\"Floor\"]" << endl;
    out << "AttributeEnd" << endl;

    // Output light sources
    for (int i = 0; i < ir.lights.size(); ++i) {
        if (ir.lights[i].isEmpty()) continue;
        vector<R3Point> pts;
        vector<int> indices;
        estimateLightShape(m, i+1, pts, indices);
        out << "AttributeBegin" << endl;
        if (pts.size() == 4 && indices.size() == 0) {
            if (pts.size() == 4) {
                // Disk area light
                out << "AreaLightSource \"diffuse\" \"rgb L\" [";
                for (int j = 0; j < 3; ++j) out << ir.lights[i](j)/M_PI << " ";
                out << "]" << endl;
                Matrix3d rot;
                Vector3d axes[3];
                axes[0] = Vector3d(pts[2][0], pts[2][1], pts[2][2]);
                axes[2] = Vector3d(pts[3][0], pts[3][1], pts[3][2]);
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
                for (int j = 0; j < 3; ++j) out << ir.lights[i](j)/(M_PI*radius*radius) << " ";
                out << "]" << endl;
                out << "Transform [1 0 0 " << pts[0][0] <<
                                 " 0 1 0 " << pts[0][1] <<
                                 " 0 0 1 " << pts[0][2] <<
                                 " 0 0 0 1]" << endl;
                out << "Shape \"sphere\" \"float radius\" [" << radius << "]" << endl;
            }
        } else {
            out << "AreaLightSource \"diffuse\" \"rgb L\" [";
            for (int j = 0; j < 3; ++j) out << ir.lights[i](j)/M_PI << " ";
            out << "]" << endl;
            out << "Transform [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]" << endl;
            for (int j = 0; j < pts.size(); ++j) {
                Vector4f q(pts[j][0],pts[j][1],pts[j][2],1);
                q = wf.getNormalizationTransform()*q;
                q /= q(3);
                R3Point tmp = b.ClosestPoint(R3Point(q(0),q(1),q(2)));
                q = wf.getNormalizationTransform().inverse()*Vector4f(tmp[0], tmp[1], tmp[2], 1);
                q /= q(3);
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
    out << "WorldEnd" << endl;
}
