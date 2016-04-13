#include "datamanager/imageio.h"
#include "lighting/generateimage.h"
#include "lighting/sh.h"
#include "lighting/cubemap.h"

double sampleSH(double theta, double phi, SHLight* light) {
    double ret = 0;
    int l = 0;
    int m = 0;
    for (int i = 0; i < light->numParameters(); i++) {
        if (m > l) {
            l++;
            m = -l;
        }
        ret += light->getCoef(i)*SH(l, m, theta, phi);
        m++;
    }
    return ret;
}
double sampleCubemap(double theta, double phi, CubemapLight* light) {
    double s = sin(theta);
    int n = getEnvmapCell(s*cos(phi), s*sin(phi), cos(theta), light->getCubemapResolution());
    return light->getCoef(n);
}
double sampleLine(double theta, LineLight* light) {
    //if (theta < 0) theta += 2*M_PI;
    theta += M_PI;
    if (light->isSymmetric() && theta > M_PI) theta = 2*M_PI - theta;
    int n = light->numParameters()*theta/(2*M_PI);
    return light->getCoef(n)*light->getLength()/light->getNumSubdivs();
}
bool negative = false;
void generateImage(RGBLight* l, std::string filename, int res) {
    float* image = new float[res*res*2*3];
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res*2; j++) {
            for (int ch = 0; ch < 3; ch++) {
                Light* light = l->getLight(ch);
                double phi = M_PI*(j-res)/res;
                double theta = M_PI*i/res;
                if (light->typeId() & LIGHTTYPE_POINT) {
                    // pbrt uses y-up for goniometric diagrams (as opposed to z-up)
                    double p = atan2(cos(theta), sin(theta)*cos(phi));
                    double t = acos(sin(theta)*sin(phi));
                    phi = p;
                    theta = t;
                    light = static_cast<PointLight*>(light)->getLight();
                }
                double v;
                if (l->typeId() & LIGHTTYPE_SH) {
                    v = sampleSH(theta, -phi, static_cast<SHLight*>(light));
                } else if (l->typeId() & LIGHTTYPE_ENVMAP) {
                    v = sampleCubemap(theta, -phi, static_cast<CubemapLight*>(light));
                } else if (l->typeId() & LIGHTTYPE_LINE) {
                    v = sampleLine(phi, static_cast<LineLight*>(light));
                    v *= sin(theta);
                }
                if (!negative) v = std::max(v, 0.);
                image[3*(i*res*2+j) + ch] = v;
            }
        }
    }
    ImageIO::writeExrImage(filename, image, res*2, res, 3);
}
