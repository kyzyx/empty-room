#include "rectify.h"
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;


Matrix3d getRectificationMatrix(vector<Vector2d>& a, vector<Vector2d>& b) {
    assert(a.size() == b.size());
    assert(a.size() >= 4);
    /*
     * | A B C |
     * | D E F | b = a
     * | G H I |
     *
     * Axb + Byb + C
     * ------------- = xa
     * Gxb + Hyb + I
     *
     * Axb + Byb + C - Gxbxa - Hybxa - Ixa = 0
     * Similarly,
     * Dxb + Eyb + F - Gxbya - Hybya - Iya = 0
     *
     */
    Matrix3d ret;
    ret.setIdentity(3,3);

    MatrixXd m;
    m.setZero(2*b.size(),9);
    for (int i = 0; i < b.size(); ++i) {
        m(2*i, 0) = b[i](0);
        m(2*i, 1) = b[i](1);
        m(2*i, 2) = 1;
        m(2*i, 6) = -b[i](0)*a[i](0);
        m(2*i, 7) = -b[i](1)*a[i](0);
        m(2*i, 8) = -a[i](0);

        m(2*i+1, 3) = b[i](0);
        m(2*i+1, 4) = b[i](1);
        m(2*i+1, 5) = 1;
        m(2*i+1, 6) = -b[i](0)*a[i](1);
        m(2*i+1, 7) = -b[i](1)*a[i](1);
        m(2*i+1, 8) = -a[i](1);
    }
    VectorXd res = m.jacobiSvd(ComputeFullV).matrixV().col(8);
    for (int i = 0; i < 9; ++i) {
        ret(i/3,i%3) = res(i);
    }
    return ret;
}

void rectify(Matrix3d m, R2Image_Old::R2Image* image) {
    R2Image_Old::R2Image orig(*image);
    // Find bounds of image
    int w = image->Width();
    int h = image->Height();
    int xx[] = {0,w,w,0};
    int yy[] = {0,0,h,h};
    w = 0;
    h = 0;
    for (int i = 0; i < 4; ++i) {
        Vector3d src = m.inverse()*Vector3d(xx[i],yy[i],1);
        src /= src(2);
        if (src(0) > w) w = src(0);
        if (src(1) > h) h = src(1);
    }

    R2Image_Old::R2Image newimage(w,h);

    // Resample old image
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            Vector3d src = m*Vector3d(j,i,1);
            src /= src(2);
            double x = src(0);
            double y = src(1);
            double r = 0;
            int cy[] = {i,i,i+1,i-1};
            int cx[] = {j+1,j-1,j,j};
            for (int k = 0; k < 4; ++k) {
                Vector3d v = m*Vector3d(cx[k],cy[k],1);
                v /= v(2);
                double d = (v - src).norm();
                if (d > r) r = d;
            }
            R2Pixel p;
            //if (r < 1)
                p = orig.Sample(x,y,R2Image_Old::R2_IMAGE_BILINEAR_SAMPLING, 1, 0, true);
            //else
                //p = orig.Sample(x,y,R2Image_Old::R2_IMAGE_GAUSSIAN_SAMPLING, min(r,5.), 0, true);
            newimage.SetPixel(j, i, p);
        }
    }
    *image = newimage;
}
