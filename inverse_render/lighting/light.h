#ifndef _LIGHT_H
#define _LIGHT_H

#include <iterator>
#include <iostream>
#include <vector>

namespace ceres {
    class Problem;
}

enum LightType {
    LIGHTTYPE_NULL=0,
    LIGHTTYPE_SH=1,
    LIGHTTYPE_ENVMAP=2,
    LIGHTTYPE_AREA=3,
    NUMLIGHTTYPES,
};

class Light {
    public:
        virtual int numParameters() const { return 0; }
        virtual int typeId() const = 0;
        virtual double& coef(int n) { return v[n]; }

        virtual double lightContribution(double* it) const;
        virtual double lightContribution(std::vector<double>::const_iterator it) const;
        virtual double lightContribution(Light* l) const;
        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1) {;}

        virtual void addCeres(ceres::Problem* problem) {;}
        virtual void writeToStream(std::ostream& out, bool binary=false) {;}
        virtual void readFromStream(std::istream& in, bool binary=false) {;}

    protected:
        std::vector<double> v;
};

class AreaLight : public Light {
    public:
        virtual int numParameters() const { return 1; }
        virtual int typeId() const { return LIGHTTYPE_AREA; }

        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1)
        {
            v[0] += weight;
        }

        //virtual void addCeres(ceres::Problem* problem);
        //virtual void writeToStream(std::ostream& out);
        //virtual void readFromStream(std::istream& in);
    protected:
};

class SHEnvironmentLight : public Light {
    public:
        virtual int numParameters() const { return numbands*numbands; }
        virtual int typeId() const { return LIGHTTYPE_SH; }

        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

        //virtual void addCeres(ceres::Problem* problem) {;}
        //virtual void writeToStream(std::ostream& out);
        //virtual void readFromStream(std::istream& in);
    protected:
        int numbands;
};

class CubemapEnvironmentLight : public Light {
    public:
        virtual int numParameters() const { return res*res*6; }
        virtual int typeId() const { return LIGHTTYPE_ENVMAP; }

        virtual void addLightFF(
                double px, double py, double pz,
                double dx, double dy, double dz,
                double weight=1);

        //virtual void addCeres(ceres::Problem* problem) {;}
        //virtual void writeToStream(std::ostream& out);
        //virtual void readFromStream(std::istream& in);
    protected:
        int res;
};

Light* NewLightFromLightType(int type);
#endif
