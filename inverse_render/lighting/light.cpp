#include "light.h"
#include "sh.h"
#include "cubemap.h"
#include <fstream>

using namespace std;

void Light::writeToStream(std::ostream& out, bool binary) {
    if (binary) {
        for (int i = 0; i < v.size(); i++) {
            out.write((char*) &v[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < v.size(); i++) {
            out << v[i] << " ";
        }
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
    vec = p[1] - p[0];
    length = vec.norm();
    vec /= length;

    perp = abs(vec[0])>abs(vec[1])?Vector3d(0,1,0).cross(vec):Vector3d(1,0,0).cross(vec);
}

void LineLight::setPerpendicularVector(Vector3d perpendicular) {
    perp = vec.cross(perpendicular.cross(vec));
}

void LineLight::writeToStream(std::ostream& out, bool binary) {
    if (binary) {
        char sym = symmetric?1:0;
        out.write(&sym, sizeof(char));
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < 3; i++) {
                double d = p[j][i];
                out.write((char*) &d, sizeof(double));
            }
        }
    } else {
        out << (symmetric?1:0) << " ";
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < 3; i++) {
                out << p[j][i] << " ";
            }
        }
        out << endl;
    }
    if (symmetric) {
        for (int i = 0; i < v.size(); i++) {
            int idx = i>=v.size()/2?(v.size()-1-i):i;
            if (binary) out.write((char*) &v[idx], sizeof(double));
            else out << v[idx] << " ";
        }
    } else {
        Light::writeToStream(out, binary);
    }
}
void LineLight::readFromStream(std::istream& in, bool binary) {
    if (binary) {
        char sym;
        in.read(&sym, sizeof(char));
        if (sym) setSymmetric();
        for (int i = 0; i < 2; i++) {
            double x,y,z;
            in.read((char*) &x, sizeof(double));
            in.read((char*) &y, sizeof(double));
            in.read((char*) &z, sizeof(double));
            setPosition(i, x, y, z);
        }
    } else {
        int sym;
        in >> sym;
        if (sym) setSymmetric();
        for (int i = 0; i < 2; i++) {
            double x,y,z;
            in >> x >> y >> z;
            setPosition(i, x, y, z);
        }
    }
    Light::readFromStream(in, binary);
}

void LineLight::addIncident(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    Vector3d P(px, py, pz);
    Vector3d d = P - p[0];
    double t = d.dot(vec);
    if (t < 0) {
    } else if (t > length) {
        //d = P - p[1];
    } else {
        /*
        d /= d.norm();
        double theta = atan2(d.dot(perp), d.dot(perp.cross(v)));
        light->addLightFF(px, py, pz, cos(theta), sin(theta), 0, weight);
        */
    }
    d -= t*vec;
    d /= d.norm();
    double theta = acos(d.dot(perp));
    if (!symmetric && (d.cross(perp)).dot(vec) < 0) theta = 2*M_PI - theta;
    int idx = numcells*theta/(2*M_PI);
    v[idx] += weight;
    // NOTE: Must factor occlusion and mean squared distance into weight
    //light->addLightFF(px, py, pz, d[0], d[1], d[2], weight);
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

RGBLight::RGBLight(Light* light) {
    l[0] = light;
    for (int i = 1; i < 3; i++) {
        Light* nl = NewLightFromLightType(light->typeId());
        if (light->typeId() & LIGHTTYPE_POINT) {
            PointLight* pl = (PointLight*) nl;
            PointLight* ol = (PointLight*) light;
            pl->setPosition(ol->getPosition(0), ol->getPosition(1), ol->getPosition(2));
        } else if (light->typeId() & LIGHTTYPE_LINE) {
            LineLight* ll = (LineLight*) nl;
            LineLight* ol = (LineLight*) light;
            ll->setPosition(0, ol->getPosition(0));
            ll->setPosition(1, ol->getPosition(1));
            ll->setSymmetric(ol->isSymmetric());
        }
        l[i] = nl;
    }
}
void IRGBLight::writeToStream(std::ostream& out, bool binary) {
    if (binary) {
        for (int i = 0; i < 3; i++) {
            out.write((char*) &v[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < 3; i++) {
            out << v[i] << " ";
        }
    }
    l->writeToStream(out, binary);
}
void IRGBLight::readFromStream(std::istream& in, bool binary) {
    v.resize(3, 0);
    if (binary) {
        for (int i = 0; i < 3; i++) {
            in.read((char*) &v[i], sizeof(double));
        }
    } else {
        for (int i = 0; i < 3; i++) {
            in >> v[i];
        }
    }
    l->readFromStream(in, binary);
}

RGBLight* IRGBLight::toRGBLight() {
    RGBLight* ret = new RGBLight(l);
    for (int i = 0; i < l->numParameters(); i++) {
        for (int j = 0; j < 3; j++) {
            ret->getLight(j)->coef(i) = v[j]*l->getCoef(i);
        }
    }
    return ret;
}

Light* NewLightFromLightType(int type) {
    if (type & LIGHTTYPE_RGB) {
        Light* l = NewLightFromLightType(type - LIGHTTYPE_RGB);
        return new RGBLight(l);
    } else if (type & LIGHTTYPE_IRGB) {
        Light* l = NewLightFromLightType(type - LIGHTTYPE_IRGB);
        return new IRGBLight(l);
    } else {
        switch (type) {
            case LIGHTTYPE_SH: return new SHLight;
            case LIGHTTYPE_ENVMAP: return new CubemapLight;
            case LIGHTTYPE_AREA: return new AreaLight;
            case (LIGHTTYPE_SH | LIGHTTYPE_POINT): return new PointLight(new SHLight);
            case (LIGHTTYPE_ENVMAP | LIGHTTYPE_POINT): return new PointLight(new CubemapLight);
            case LIGHTTYPE_LINE:
            case (LIGHTTYPE_SH | LIGHTTYPE_LINE):
            case (LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE):
                                                       return new LineLight();
            default: return new Light();
        }
    }
}

inline void writeint(ofstream& out, int v, bool binary) {
    if (binary) out.write((char*)&v, sizeof(int));
    else out << v << " ";
}
void writeLightsToFile(string filename, vector<Light*>& lights, bool binary) {
    ofstream out(filename, binary?(ofstream::binary):(ofstream::out));
    writeint(out, lights.size(), binary);
    if (!binary) out << endl;
    for (int i = 0; i < lights.size(); i++) {
        if (lights[i]) {
            writeint(out, lights[i]->typeId(), binary);
            lights[i]->writeToStream(out, binary);
            if (!binary) out << endl;
        } else {
            writeint(out, -1, binary);
        }
    }
}
void readLightsFromFile(string filename, vector<Light*>& lights, bool binary) {
    ifstream in(filename, binary?(ifstream::binary):(ifstream::out));
    int nlights;
    if (binary) in.read((char*)&nlights, sizeof(int));
    else in >> nlights;
    lights.clear();
    for (int i = 0; i < nlights; i++) {
        int lighttype;
        if (binary) in.read((char*)&lighttype, sizeof(int));
        else in >> lighttype;
        Light* l = NewLightFromLightType(lighttype);
        if (l) l->readFromStream(in, binary);
        lights.push_back(l);
    }
}
