#include "light.h"
#include "sh.h"
#include "cubemap.h"
#include <fstream>

using namespace std;

void Light::writeToStream(std::ostream& out, bool binary) {
    if (binary) {
        for (int i = 0; i < numParameters(); i++) {
            out.write((char*) &v[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < numParameters(); i++) {
            out << v[i] << " ";
        }
        out << endl;
    }
}
void Light::readFromStream(std::istream& in, bool binary) {
    v.resize(numParameters(), 0);
    if (binary) {
        for (int i = 0; i < numParameters(); i++) {
            in.read((char*) &v[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < numParameters(); i++) {
            in >> v[i];
        }
    }
}

double Light::lightContribution(double* it) const {
    double ret = 0;
    for (auto vit = v.begin(); vit != v.end(); ++vit, ++it) {
        ret += *vit * *it;
    }
    return ret;
}

double Light::lightContribution(vector<double>::const_iterator it) const {
    double ret = 0;
    for (auto vit = v.begin(); vit != v.end(); ++vit, ++it) {
        ret += *vit * *it;
    }
    return ret;
}

double Light::lightContribution(Light* l) const {
    return l->lightContribution(v.begin());
}

bool SHLight::addLightFF(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    int idx = 0;
    for (int band = 0; band < numbands; band++) {
        for (int m = -band; m <= band; m++) {
            v[idx++] += weight*SH(band, m, dx, dy, dz);
        }
    }
    return true;
}

bool CubemapLight::addLightFF(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    int envmapidx = getEnvmapCell(dx, dy, dz, res);
    v[envmapidx] += weight;
    return true;
}

void PointLight::writeToStream(std::ostream& out, bool binary) {
    if (binary) {
        for (int i = 0; i < 3; i++) {
            out.write((char*) &p[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < 3; i++) {
            out << p[i] << " ";
        }
        out << endl;
    }
    light->writeToStream(out, binary);
}
void PointLight::readFromStream(std::istream& in, bool binary) {
    if (binary) {
        for (int i = 0; i < 3; i++) {
            in.read((char*) &p[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < 3; i++) {
            in >> p[i];
        }
    }
    light->readFromStream(in, binary);
}

void PointLight::addIncident(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    dx = px-p[0];
    dy = py-p[1];
    dz = pz-p[2];
    /*double d2 = dx*dx + dy*dy + dz*dz;
    light->addLightFF(px, py, pz, dx, dy, dz, weight/d2);*/
    // NOTE: Must factor occlusion and squared distance into weight
    light->addLightFF(px, py, pz, dx, dy, dz, weight);
}

using namespace Eigen;

void LineLight::setPosition(int n, Vector3d pos) {
    p[n] = pos;
    v = p[1] - p[0];
    length = v.norm();
    v /= length;

    perp = abs(v[0])>abs(v[1])?Vector3d(0,1,0).cross(v):Vector3d(1,0,0).cross(v);
}

void LineLight::writeToStream(std::ostream& out, bool binary) {
    if (binary) {
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < 3; i++) {
                double d = p[j][i];
                out.write((char*) &d, sizeof(double));
            }
        }
    } else {
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < 3; i++) {
                out << p[j][i] << " ";
            }
        }
        out << endl;
    }
    light->writeToStream(out, binary);
}
void LineLight::readFromStream(std::istream& in, bool binary) {
    if (binary) {
        for (int i = 0; i < 2; i++) {
            double x,y,z;
            in.read((char*) &x, sizeof(double));
            in.read((char*) &y, sizeof(double));
            in.read((char*) &z, sizeof(double));
            setPosition(i, x, y, z);
        }
    } else {
        for (int i = 0; i < 2; i++) {
            double x,y,z;
            in >> x >> y >> z;
            setPosition(i, x, y, z);
        }
    }
    light->readFromStream(in, binary);
}

void LineLight::addIncident(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    Vector3d P(px, py, pz);
    Vector3d d = P - p[0];
    double t = d.dot(v);
    if (t < 0) {
    } else if (t > length) {
        d = P - p[1];
    } else {
        d -= t*v;
        /*
        d /= d.norm();
        double theta = atan2(d.dot(perp), d.dot(perp.cross(v)));
        light->addLightFF(px, py, pz, cos(theta), sin(theta), 0, weight);
        */
    }
    // NOTE: Must factor occlusion and mean squared distance into weight
    light->addLightFF(px, py, pz, d[0], d[1], d[2], weight);
}

void LineLight::computeFromPoints(vector<Vector3d> pts) {
    cout << pts.size() << endl;
    Vector3d mean(0,0,0);
    for (int i = 0; i < pts.size(); i++) {
        mean += pts[i];
    }
    mean /= pts.size();
    MatrixXd A(pts.size(), 3);
    for (int i = 0; i < pts.size(); ++i) {
        for (int j = 0; j < 3; ++j) A(i,j) = pts[i][j] - mean[j];
    }
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    MatrixXd Y(pts.size(), 3);
    Y = A*svd.matrixV();
    Vector3d maxcoefs = Y.colwise().maxCoeff();
    Vector3d mincoefs = Y.colwise().minCoeff();
    Vector3d xax = svd.matrixV().col(0);
    Vector3d yax = svd.matrixV().col(1);
    Vector3d yadj = yax*0.5*(maxcoefs[1] + mincoefs[1]);
    setPosition(0, mean+xax*maxcoefs[0]+yadj);
    setPosition(1, mean+xax*mincoefs[0]+yadj);
}

Light* NewLightFromLightType(int type) {
    switch (type) {
        case LIGHTTYPE_SH: return new SHLight;
        case LIGHTTYPE_ENVMAP: return new CubemapLight;
        case LIGHTTYPE_AREA: return new AreaLight;
        case (LIGHTTYPE_SH | LIGHTTYPE_POINT): return new PointLight(new SHLight);
        case (LIGHTTYPE_ENVMAP | LIGHTTYPE_POINT): return new PointLight(new CubemapLight);
        case (LIGHTTYPE_SH | LIGHTTYPE_LINE): return new LineLight(new SHLight);
        case (LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE): return new LineLight(new CubemapLight);
        default: return new Light();
    }
}

inline void writeint(ofstream& out, int v, bool binary) {
    if (binary) out.write((char*)&v, sizeof(int));
    else out << v << " ";
}
void writeLightsToFile(string filename, vector<vector<Light*> >& lights, bool binary) {
    ofstream out(filename, binary?(ofstream::binary):(ofstream::out));
    writeint(out, lights[0].size(), binary);
    if (!binary) out << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < lights[i].size(); j++) {
            if (lights[i][j]) {
                writeint(out, lights[i][j]->typeId(), binary);
                lights[i][j]->writeToStream(out, binary);
                if (!binary) out << endl;
            } else {
                writeint(out, -1, binary);
            }
        }
    }
}
void readLightsFromFile(string filename, vector<vector<Light*> >& lights, bool binary) {
    ifstream in(filename, binary?(ifstream::binary):(ifstream::out));
    int nlights;
    if (binary) in.read((char*)&nlights, sizeof(int));
    else in >> nlights;
    lights.clear();
    lights.resize(3);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < nlights; j++) {
            int lighttype;
            if (binary) in.read((char*)&lighttype, sizeof(int));
            else in >> lighttype;
            Light* l = NewLightFromLightType(lighttype);
            if (l) l->readFromStream(in, binary);
            lights[i].push_back(l);
        }
    }
}
