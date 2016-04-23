#ifndef _LIGHT_H
#define _LIGHT_H

#include <iterator>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

enum LightType {
    LIGHTTYPE_NULL=0,
    LIGHTTYPE_SH=2,
    LIGHTTYPE_ENVMAP=1,
    LIGHTTYPE_AREA=3,
    LIGHTTYPE_POINT=16,
    LIGHTTYPE_LINE=32,
    LIGHTTYPE_RGB=64,
    LIGHTTYPE_IRGB=128,
};

class Light {
    public:
        Light() : reglambda(0) {;}
        virtual int numParameters() const { return 0; }
        virtual int typeId() const { return LIGHTTYPE_NULL; }
        double operator[](int n) const { return getCoef(n); }
        virtual double& coef(int n) { return v[n]; }
        virtual double getCoef(int n) const { return v[n]; }
        virtual void setRegularization(double d) { reglambda = d; }
        virtual double getRegularization() const { return reglambda; }

        virtual bool addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1) { return false; }

        virtual void writeToStream(std::ostream& out, bool binary=false);
        virtual void readFromStream(std::istream& in, bool binary=false);

    protected:
        std::vector<double> v;
        double reglambda;
};

class RGBLight : public Light {
    public:
        RGBLight(Light* light);
        Light* getLight(int n) { return l[n]; }
        virtual int numParameters() const { return l[0]->numParameters()*3; }
        virtual int typeId() const { return l[0]->typeId() | LIGHTTYPE_RGB; }
        virtual double& coef(int n) { return l[n/l[0]->numParameters()]->coef(n%l[0]->numParameters()); }
        virtual double getCoef(int n) const { return l[n/l[0]->numParameters()]->coef(n%l[0]->numParameters()); }
        virtual void setRegularization(double d) {
            Light::setRegularization(d);
            for (int i = 0; i < 3; i++) l[i]->setRegularization(d);
        }


        virtual void writeToStream(std::ostream& out, bool binary=false) {
            for (int i = 0; i < 3; i++) l[i]->writeToStream(out, binary);
        }
        virtual void readFromStream(std::istream& in, bool binary=false) {
            for (int i = 0; i < 3; i++) l[i]->readFromStream(in, binary);
        }
    protected:
        Light* l[3];
};

class IRGBLight : public Light {
    public:
        IRGBLight(Light* light) : l(light) {
            v.resize(3,0);
        }
        virtual int numParameters() const { return l->numParameters()+3; }
        virtual int typeId() const { return l->typeId() | LIGHTTYPE_IRGB; }
        virtual double& coef(int n) {
            if (n < 3) return v[n];
            else return l->coef(n-3);
        }
        virtual double getCoef(int n) const {
            if (n < 3) return v[n];
            else return l->coef(n-3);
        }

        Light* getLight() { return l; }
        virtual void setRegularization(double d) {
            Light::setRegularization(d);
            l->setRegularization(d);
        }
        virtual double getRegularization() const { return reglambda; }

        RGBLight* toRGBLight();

        virtual void writeToStream(std::ostream& out, bool binary=false);
        virtual void readFromStream(std::istream& in, bool binary=false);
    protected:
        Light* l;
};

class AreaLight : public Light {
    public:
        AreaLight() { v.resize(numParameters(), 0); }
        virtual int numParameters() const { return 1; }
        virtual int typeId() const { return LIGHTTYPE_AREA; }

        virtual bool addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1)
        {
            v[0] += weight;
            return true;
        }

    protected:
};

class SHLight : public Light {
    public:
        SHLight() : numbands(5) { v.resize(numParameters(), 0); }
        virtual int numParameters() const { return numbands*numbands; }
        virtual int typeId() const { return LIGHTTYPE_SH; }

        virtual bool addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

    protected:
        int numbands;
};

class CubemapLight : public Light {
    public:
        CubemapLight() : res(5) { v.resize(numParameters(), 0); }
        virtual int numParameters() const { return res*res*6; }
        virtual int typeId() const { return LIGHTTYPE_ENVMAP; }

        int getCubemapResolution() const { return res; }

        virtual bool addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

    protected:
        int res;
};

class PointLight : public Light {
    public:
        PointLight(Light* l)
            : light(l)
        {
            setPosition(0,0,0);
        }
        PointLight(double x, double y, double z, Light* l)
            : light(l)
        {
            setPosition(x,y,z);
        }
        void setPosition(double x, double y, double z) {
            p[0] = x;
            p[1] = y;
            p[2] = z;
        }
        double getPosition(int n) {
            return p[n];
        }
        Light* getLight() { return light; }
        virtual int numParameters() const { return light->numParameters(); }
        virtual int typeId() const { return LIGHTTYPE_POINT | light->typeId(); }
        virtual double& coef(int n) { return light->coef(n); }
        virtual double getCoef(int n) const { return light->getCoef(n); }

        virtual void addIncident(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

        virtual void writeToStream(std::ostream& out, bool binary=false);
        virtual void readFromStream(std::istream& in, bool binary=false);
    protected:
        double p[3];
        Light* light;
};

class LineLight : public Light {
    public:
        LineLight()
            : numcells(32), symmetric(false), numsubdivs(30)
        {
            setPosition(0,0,0,0);
            setPosition(1,0,0,0);
            v.resize(numParameters());
        }
        LineLight(LineLight* ll)
            : numcells(32), symmetric(false), numsubdivs(30)
        {
            setPosition(0, ll->getPosition(0));
            setPosition(1, ll->getPosition(1));
            setSymmetric(ll->isSymmetric());
            v.resize(numParameters());
        }
        LineLight(double x1, double y1, double z1,
                  double x2, double y2, double z2)
            : numcells(32), symmetric(false), numsubdivs(30)
        {
            setPosition(0,x1,y1,z1);
            setPosition(1,x2,y2,z2);
            v.resize(numParameters());
        }
        LineLight(Eigen::Vector3d p1, Eigen::Vector3d p2)
            : numcells(32), symmetric(false), numsubdivs(30)
        {
            setPosition(0,p1);
            setPosition(1,p2);
            v.resize(numParameters());
        }
        virtual int numParameters() const { return numcells; }
        virtual int typeId() const { return LIGHTTYPE_LINE; }
        void setPosition(int n, double x, double y, double z) {
            setPosition(n, Eigen::Vector3d(x,y,z));
        }
        void setPosition(int n, Eigen::Vector3d pos);
        void setSymmetric(bool sym=true) { symmetric = sym; }
        bool isSymmetric() const { return symmetric; }
        void setPerpendicularVector(Eigen::Vector3d perpendicular);
        Eigen::Vector3d getPerpendicularVector() const { return perp; }
        void computeFromPoints(std::vector<Eigen::Vector3d> pts);
        double getPosition(int n, int c) const {
            return p[n][c];
        }
        double getVector(int n) const {
            return vec[n];
        }
        Eigen::Vector3d getVector() const {
            return vec;
        }
        double getLength() const { return length; }
        Eigen::Vector3d getPosition(int n) const {
            return p[n];
        }
        int getNumSubdivs() const { return numsubdivs; }
        Eigen::Vector3d getSubpoint(int n) const {
            double dx = length/numsubdivs;
            return p[0] + vec*dx*(n + 0.5);
        }

        virtual void addIncident(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

        virtual void writeToStream(std::ostream& out, bool binary=false);
        virtual void readFromStream(std::istream& in, bool binary=false);
    protected:
        Eigen::Vector3d p[2];
        Eigen::Vector3d vec;
        Eigen::Vector3d perp;
        bool symmetric;
        double length;
        int numsubdivs;
        int numcells;
};

Light* NewLightFromLightType(int type);
void writeLightsToFile(std::string filename, std::vector<Light*>& lights, bool binary=false);
void readLightsFromFile(std::string filename, std::vector<Light*>& lights, bool binary=false);

template <typename T>
void lightContribution(Light* l, T* ret, const Light* const arr, T const* v=NULL) {
    if (l->typeId() & LIGHTTYPE_RGB) {
        RGBLight* rgbl = static_cast<RGBLight*>(l);
        int np = rgbl->getLight(0)->numParameters();
        for (int ch = 0; ch < 3; ch++) {
            lightContribution(rgbl->getLight(ch), ret+ch, arr, v?v+np*ch:NULL);
        }
    } else if (l->typeId() & LIGHTTYPE_IRGB) {
        for (int i = 3; i < l->numParameters(); i++) {
            for (int ch = 0; ch < 3; ch++) {
                if (v) ret[ch] += v[i]*v[ch]*T(arr->getCoef(i-3));
                else   ret[ch] += T(l->getCoef(ch)*l->getCoef(i)*arr->getCoef(i-3));
            }
        }
    } else {
        for (int i = 0; i < l->numParameters(); i++) {
            if (v) ret[0] += v[i]*T(arr->getCoef(i));
            else   ret[0] += T(l->getCoef(i)*arr->getCoef(i));
        }
    }
}
#endif
