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

double CatmullRomSplineInterp(double y0, double y1, double y2, double y3, double x) {
    const double alpha = 0.5;
    double t0 = 0;
    double t1 = pow(sqrt(1+(y1-y0)*(y1-y0)), alpha) + t0;
    double t2 = pow(sqrt(1+(y2-y1)*(y2-y1)), alpha) + t1;
    double t3 = pow(sqrt(1+(y3-y2)*(y3-y2)), alpha) + t2;
    double t = x*t2 + (1-x)*t1;
    double A1 = (t1-t)/(t1-t0)*y0 + (t-t0)/(t1-t0)*y1;
    double A2 = (t2-t)/(t2-t1)*y1 + (t-t1)/(t2-t1)*y2;
    double A3 = (t3-t)/(t3-t2)*y2 + (t-t2)/(t3-t2)*y3;
    double B1 = (t2-t)/(t2-t0)*A1 + (t-t0)/(t2-t0)*A2;
    double B2 = (t3-t)/(t3-t1)*A2 + (t-t1)/(t3-t1)*A3;
    double C  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2;
    A1 = (t1-t)/(t1-t0)*0 + (t-t0)/(t1-t0)*1;
    A2 = (t2-t)/(t2-t1)*1 + (t-t1)/(t2-t1)*2;
    A3 = (t3-t)/(t3-t2)*2 + (t-t2)/(t3-t2)*3;
    B1 = (t2-t)/(t2-t0)*A1 + (t-t0)/(t2-t0)*A2;
    B2 = (t3-t)/(t3-t1)*A2 + (t-t1)/(t3-t1)*A3;
    double Cx  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2;
    return C;
}

double sampleLine(double theta, LineLight* light) {
    //if (theta < 0) theta += 2*M_PI;
    theta += M_PI;
    if (light->isSymmetric() && theta > M_PI) theta = 2*M_PI - theta;
    int n = light->numParameters()*theta/(2*M_PI);
    double x = light->numParameters()*theta/(2*M_PI) - n;
    double y[4];
    for (int i = 0; i < 4; i++) {
        int idx = (n + i - 1 + light->numParameters())%light->numParameters();
        y[i] = light->getCoef(idx);
    }
    //return light->getCoef(n)*light->getLength()/light->getNumSubdivs();
    return CatmullRomSplineInterp(y[0], y[1], y[2], y[3], x)*light->getLength()/light->getNumSubdivs();
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
