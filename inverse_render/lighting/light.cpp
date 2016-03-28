#include "light.h"
#include "sh.h"
#include "cubemap.h"

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

void SHLight::addLightFF(
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
}

void CubemapLight::addLightFF(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    int envmapidx = getEnvmapCell(dx, dy, dz, res);
    v[envmapidx] += weight;
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

void PointLight::addLightFF(
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

void LineLight::addLightFF(
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

Light* NewLightFromLightType(int type) {
    switch (type) {
        case LIGHTTYPE_SH: return new SHLight;
        case LIGHTTYPE_ENVMAP: return new CubemapLight;
        case LIGHTTYPE_AREA: return new AreaLight;
        case (LIGHTTYPE_SH | LIGHTTYPE_POINT): return new PointLight(new SHLight);
        case (LIGHTTYPE_ENVMAP | LIGHTTYPE_POINT): return new PointLight(new CubemapLight);
        case (LIGHTTYPE_SH | LIGHTTYPE_LINE): return new LineLight(new SHLight);
        case (LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE): return new LineLight(new CubemapLight);
        default: return NULL;
    }
}
