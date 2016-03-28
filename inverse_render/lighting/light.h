#ifndef _LIGHT_H
#define _LIGHT_H

#include <iterator>
#include <iostream>
#include <vector>

enum LightType {
    LIGHTTYPE_NULL=0,
    LIGHTTYPE_SH=2,
    LIGHTTYPE_ENVMAP=1,
    LIGHTTYPE_AREA=3,
    NUMLIGHTTYPES,
};

class Light {
    public:
        Light() : reglambda(0) {;}
        virtual int numParameters() const { return 0; }
        virtual int typeId() const = 0;
        virtual double& coef(int n) { return v[n]; }
        virtual void setRegularization(double d) { reglambda = d; }
        virtual double getRegularization() const { return reglambda; }

        virtual double lightContribution(double* it) const;
        virtual double lightContribution(std::vector<double>::const_iterator it) const;
        virtual double lightContribution(Light* l) const;
        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1) {;}

        virtual void writeToStream(std::ostream& out, bool binary=false);
        virtual void readFromStream(std::istream& in, bool binary=false);

    protected:
        std::vector<double> v;
        double reglambda;
};

class AreaLight : public Light {
    public:
        AreaLight() { v.resize(numParameters(), 0); }
        virtual int numParameters() const { return 1; }
        virtual int typeId() const { return LIGHTTYPE_AREA; }

        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1)
        {
            v[0] += weight;
        }

    protected:
};

class SHEnvironmentLight : public Light {
    public:
        SHEnvironmentLight() : numbands(5) { v.resize(numParameters(), 0); }
        virtual int numParameters() const { return numbands*numbands; }
        virtual int typeId() const { return LIGHTTYPE_SH; }

        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

    protected:
        int numbands;
};

class CubemapEnvironmentLight : public Light {
    public:
        CubemapEnvironmentLight() : res(5) { v.resize(numParameters(), 0); }
        virtual int numParameters() const { return res*res*6; }
        virtual int typeId() const { return LIGHTTYPE_ENVMAP; }

        int getCubemapResolution() const { return res; }

        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

    protected:
        int res;
};

Light* NewLightFromLightType(int type);
#endif
