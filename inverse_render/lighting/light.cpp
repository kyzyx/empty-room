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

void SHEnvironmentLight::addLightFF(
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

void CubemapEnvironmentLight::addLightFF(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    int envmapidx = getEnvmapCell(dx, dy, dz, res);
    v[envmapidx] += weight;
}

Light* NewLightFromLightType(int type) {
    switch (type) {
        case LIGHTTYPE_SH: return new SHEnvironmentLight;
        case LIGHTTYPE_ENVMAP: return new CubemapEnvironmentLight;
        case LIGHTTYPE_AREA: return new AreaLight;
        default: return NULL;
    }
}
