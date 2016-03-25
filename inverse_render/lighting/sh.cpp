#include "sh.h"
#include <cmath>

// -----------------------------------------------------
#pragma region Polynomial forms of SH functions
const double RP2 = 1./(2*sqrt(M_PI));

typedef double (*SHPfn)(double, double, double);
static const SHPfn pf[] = {
// Band 0
    [](double x, double y, double z) -> double { return RP2; },

// Band 1
    [](double x, double y, double z) -> double { return RP2*-sqrt(3)*y; },
    [](double x, double y, double z) -> double { return RP2* sqrt(3)*z; },
    [](double x, double y, double z) -> double { return RP2*-sqrt(3)*x; },

// Band 2
    [](double x, double y, double z) -> double { return RP2* sqrt(15)*y*x; },
    [](double x, double y, double z) -> double { return RP2*-sqrt(15)*y*z; },
    [](double x, double y, double z) -> double { return RP2* sqrt(5) *0.5*(3*z*z-1); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(15)*x*z; },
    [](double x, double y, double z) -> double { return RP2* sqrt(15)*0.5*(x*x - y*y); },

// Band 3
    [](double x, double y, double z) -> double { return RP2*-sqrt(70) *0.25*y*(3*x*x-y*y); },
    [](double x, double y, double z) -> double { return RP2* sqrt(105)*x*y*z; },
    [](double x, double y, double z) -> double { return RP2*-sqrt(42) *0.25*y*(5*z*z-1); },
    [](double x, double y, double z) -> double { return RP2* sqrt(7)  *0.5*z*(5*z*z-3); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(42) *0.25*x*(5*z*z-1); },
    [](double x, double y, double z) -> double { return RP2* sqrt(105)*0.5*z*(x*x-y*y); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(70) *0.25*x*(x*x-3*y*y); },

// Band 4
    [](double x, double y, double z) -> double { return RP2* sqrt(35)*1.5*y*x*(x*x-y*y); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(70)*0.75*y*z*(3*x*x-y*y); },
    [](double x, double y, double z) -> double { return RP2* sqrt(5) *1.5*y*x*(7*z*z-1); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(10)*0.75*y*z*(7*z*z-3); },
    [](double x, double y, double z) -> double { return RP2          *0.375*(35*z*z*z*z - 30*z*z + 3); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(10)*0.75*x*z*(7*z*z-3); },
    [](double x, double y, double z) -> double { return RP2* sqrt(5) *0.75*(x*x-y*y)*(7*z*z-1); },
    [](double x, double y, double z) -> double { return RP2*-sqrt(70)*0.75*x*z*(x*x-3*y*y); },
    [](double x, double y, double z) -> double { return RP2* sqrt(35)*0.375*(x*x*x*x - 6*x*x*y*y + y*y*y*y); },
};
#pragma endregion
// -----------------------------------------------------

// ----------------------------------------------
#pragma region Direct calculation of SH functions
long long fac[] = {
1LL,
1LL,
2LL,
6LL,
24LL,
120LL,
720LL,
5040LL,
40320LL,
362880LL,
3628800LL,
39916800LL,
479001600LL,
6227020800LL,
87178291200LL,
1307674368000LL,
20922789888000LL,
355687428096000LL,
6402373705728000LL,
121645100408832000LL,
2432902008176640000LL,
};
long long factorial(int n) {
    if (n < 21) return fac[n];
    else return 0;
}
// Ref http://www.research.scea.com/gdc2003/spherical-harmonic-lighting.pdf
double K(int l, int m) {
    return sqrt(((2.0*l+1.0)*factorial(l-m)) / (4.0*M_PI*factorial(l+m)));
}
double P(int l,int m,double x) {
    double pmm = 1.0;
    if(m>0) {
        double somx2 = sqrt((1.0-x)*(1.0+x));
        double fact = 1.0;
        for(int i=1; i<=m; i++) {
            pmm *= (-fact) * somx2;
            fact += 2.0;
        }
    }
    if(l==m) return pmm;
    double pmmp1 = x * (2.0*m+1.0) * pmm;
    if(l==m+1) return pmmp1;
    double pll = 0.0;
    for(int ll=m+2; ll<=l; ++ll) {
        pll = ( (2.0*ll-1.0)*x*pmmp1-(ll+m-1.0)*pmm ) / (ll-m);
        pmm = pmmp1;
        pmmp1 = pll;
    }
    return pll;
}
double DSH(int l, int m, double theta, double phi) {
     const double sqrt2 = sqrt(2.0);
     if(m==0) return K(l,0)*P(l,m,cos(theta));
     else if(m>0) return sqrt2*K(l,m)*cos(m*phi)*P(l,m,cos(theta));
     else return sqrt2*K(l,-m)*sin(-m*phi)*P(l,-m,cos(theta));
}
#pragma endregion
// ----------------------------------------------

double SH(int band, int m, double x, double y, double z) {
    int idx = band*band + m + band;
    return pf[idx](x,y,z);
    // double r = sqrt(x*x + y*y + z*z);
    // return SH(band, m, acos(z/r), atan2(y,x));
}

double SH(int band, int m, double theta, double phi) {
    double s = sin(theta);
    return SH(band, m, s*cos(phi), s*sin(phi), cos(theta));
    //return DSH(band, m, theta, phi);
}
