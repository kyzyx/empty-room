#include "rerender.h"
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

const double EPSILON = 0.0001;
const double CIRCLETOLERANCE = 1.2;

void estimateLightShape(ofstream& out, Mesh& m, int id) {
    vector<R3Point> points;
    R3Point mean(0,0,0);
    for (int i = 0; i < m.getMesh()->NVertices(); ++i) {
        R3MeshVertex* v = m.getMesh()->Vertex(i);
        if (m.labels[m.getMesh()->VertexID(v)] == id) {
            points.push_back(m.getMesh()->VertexPosition(v));
            mean += points.back();
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
        // Output light triangles;
        for (int i = 0; i < m.getMesh()->NFaces(); ++i) {
            R3MeshFace* f = m.getMesh()->Face(i);
            int lightid = -1;
            // Check if face is a light
            for (int j = 0; j < 3; ++j) {
                int n = m.getMesh()->VertexID(m.getMesh()->VertexOnFace(f, j));
                if (m.labels[n] <= 0) {
                    lightid = -1;
                    break;
                } else {
                    if (lightid <= 0) lightid = m.labels[n];
                    else if (lightid != m.labels[n]) {
                        lightid = -1;
                        break;
                    }
                }
            }
            if (lightid != id) continue;
            out << "l" << lightid << " polygon light" << i << endl << "0 0 9" << endl;
            double eps = 0.0001;
            for (int j = 0; j < 3; ++j) {
                R3Point v = m.getMesh()->VertexPosition(m.getMesh()->VertexOnFace(f, j));
                v += m.getMesh()->FaceNormal(f)*eps;
                out << v[0] << " " << v[1] << " " << v[2] << " ";
            }
            out << endl << endl;
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
            Vector3d center(mean[0],mean[1],mean[2]);
            out << "l" << id << " ring light" << id << "_"  << endl << "0 0 8" << endl;
            out << mean[0] << " " << mean[1] << " " << mean[2] << " ";
            out << zax[0] << " " << zax[1] << " " << zax[2] << " ";
            out << "0 " << xax.norm() << endl << endl;
        } else {
            // Transform extreme points back to original coordinates
            MatrixXd corners(4,3);
            corners << maxcoefs[0], maxcoefs[1],0,
                       maxcoefs[0], mincoefs[1],0,
                       mincoefs[0], mincoefs[1],0,
                       mincoefs[0], maxcoefs[1],0;
            corners = corners*svd.matrixV().transpose();
            out << "l" << id << " polygon light" << id << "_"  << endl << "0 0 12" << endl;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) out << corners(i,j)*s[j] + mean[j] << " ";
            }
            out << endl << endl;
        }
    } else {
        cout << "Error: Line source not implemented!" << endl;
    }
}

void outputRadianceFile(string filename, WallFinder& wf, Mesh& m, InverseRender& ir) {
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
        estimateLightShape(out, m, i+1);
    }

    double hi = wf.ceilplane;
    double lo = wf.floorplane;

    // output walls
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        out << "wallmat polygon wall" << i << endl << 0 << endl << 0 << endl << 12 << " ";
        double x1 = wf.wallsegments[i].getCoords(wf.wallsegments[i].start).first;
        double z1 = wf.wallsegments[i].getCoords(wf.wallsegments[i].start).second;
        double x2 = wf.wallsegments[i].getCoords(wf.wallsegments[i].end).first;
        double z2 = wf.wallsegments[i].getCoords(wf.wallsegments[i].end).second;
        out << x1 << " " << lo << " " << z1 << " " << x1 << " " << hi << " " << z1 << " ";
        out << x2 << " " << hi << " " << z2 << " " << x2 << " " << lo << " " << z2 << endl << endl;
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
