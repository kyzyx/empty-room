#include "orientededgefilter.h"
#include "RNBasics/RNBasics.h"
#include "R3Shapes/R3Shapes.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace Eigen;
using namespace std;

const double EPSILON = 1e-5;
cv::Vec3f getPixel(const cv::Mat& img, double px, double py) {
    int x = (int)px;
    int y = (int)py;
    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REPLICATE);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REPLICATE);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REPLICATE);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REPLICATE);
    float a = px - (float)x;
    float c = py - (float)y;
    float b = ((img.at<cv::Vec3f>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3f>(y0, x1)[0] * a) * (1.f - c)
            + (img.at<cv::Vec3f>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3f>(y1, x1)[0] * a) * c);
    float g = ((img.at<cv::Vec3f>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3f>(y0, x1)[1] * a) * (1.f - c)
            + (img.at<cv::Vec3f>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3f>(y1, x1)[1] * a) * c);
    float r = ((img.at<cv::Vec3f>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3f>(y0, x1)[2] * a) * (1.f - c)
            + (img.at<cv::Vec3f>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3f>(y1, x1)[2] * a) * c);
    return cv::Vec3f(b, g, r);
}

void findVanishingPoints(const CameraParams& cam, R4Matrix normalization, vector<Eigen::Vector3d>& vps) {
    R3Vector a = normalization*cam.towards;
    R3Vector b = normalization*cam.up;
    R3Vector c = normalization*cam.right;
    R3Point p = normalization*cam.pos;
    R4Matrix t(
        c[0], b[0], -a[0], p[0],
        c[1], b[1], -a[1], p[1],
        c[2], b[2], -a[2], p[2],
        0, 0, 0, 1
    );
    t = t.Inverse();
    for (int i = 0; i < 3; ++i) {
        R3Vector a =  R3zero_vector;
        a[i] = 1;
        a = t*a;
        if (abs(a[2]) < EPSILON) {
            Vector3d vp(a[0],a[1],0);
            vps.push_back(vp);
        } else {
            Vector3d vp(-a[0]/a[2]*cam.focal_length + cam.width/2,
                        -a[1]/a[2]*cam.focal_length + cam.height/2,
                        1);
            vps.push_back(vp);
        }
    }
}

void orientedEdgeFilterVP(const char* image, float* edges, int w, int h, vector<Vector3d>& vps, int windowy, int windowx) {
    double** window[2];
    for (int k = 0; k < 2; ++k) {
        window[k] = new double*[windowx];
        for (int i = 0; i < windowx; ++i) window[k][i] = new double[windowy];
    }
    memset(edges, 0, w*h*3*sizeof(float));
    float* copy = new float[w*h*3];
    memcpy(copy, image, w*h*3*sizeof(float));
    Mat img(h, w, CV_32FC3, copy);
    cvtColor(img, img, CV_BGR2HSV);
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            for (int v = 0; v < vps.size(); ++v) {
                Vector3d vp = vps[v];
                float maxresponse = 0;
                // Determine window orientation
                Vector2d lk;
                Vector2d p(j,i);
                if (vp[2] == 0) {
                    lk = vp.head(2);
                } else {
                    lk = vp.head(2) - p;
                }
                if (lk.norm() == 0) {
                    maxresponse = numeric_limits<float>::infinity();
                } else {
                    lk /= lk.norm();
                    Vector2d lpk(lk(1), -lk(0));
                    // Get window pixels
                    for (int x = 0; x < windowx; ++x) {
                        for (int y = 0; y < windowy; ++y) {
                            double xx = x - windowx/2.;
                            double yy = y - windowy/2.;
                            Vector2d pp = p + xx*lpk + yy*lk;
                            Vec3f pix = getPixel(img, pp[0], pp[1]);
                            for (int k = 0; k < 2; ++k) {
                                window[k][x][y] = pix.val[2*k];
                            }
                        }
                    }
                    // Get aggregate filter responses
                    for (int k = 0; k < 2; ++k) {
                        float response = 0;
                        float presponse = 0;
                        for (int x = 1; x < windowx-1; ++x) {
                            for (int y = 1; y < windowy-1; ++y) {
                                response += abs(window[k][x+1][y] - window[k][x-1][y]);
                                presponse += abs(window[k][x][y+1] - window[k][x][y-1]);
                            }
                        }
                        if (response > 0) {
                            if (presponse == 0) maxresponse = numeric_limits<double>::infinity();
                            else maxresponse = max(maxresponse, response/presponse);
                        }
                    }
                }
                int idx = (h-i-1)*w+j;
                edges[3*idx+v] = maxresponse;
            }
        }
    }
    delete [] copy;
    delete [] window[0];
    delete [] window[1];
}
void createEdgeImage(const CameraParams* cam, const void* colorimage, void* image) {
    createEdgeImage(cam, R4identity_matrix, colorimage, image);
}
void createEdgeImage(const CameraParams* cam, R4Matrix normalization, const void* colorimage, void* image) {
    vector<Vector3d> vps;
    findVanishingPoints(*cam, normalization, vps);
    orientedEdgeFilterVP((const char*) colorimage, (float*) image, cam->width, cam->height, vps);
}
