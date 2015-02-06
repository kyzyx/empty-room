#include "colorhelper.h"

#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfHeader.h>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#include "rgbe.h"
#include <png.h>
#include <sys/stat.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#define OUTPUT_RADIANCE_CAMERAS

using namespace std;
using namespace pcl;
using namespace Imf;
using namespace Imath;

inline bool fileexists(const string& filename) {
    struct stat buffer;
    return (stat (filename.c_str(), &buffer) == 0);
}
string replaceExtension(const string& s, string newext) {
    size_t i = s.find_last_of(".");
    if (i == string::npos) return s + newext;
    else return s.substr(0,i+1) + newext;
}

bool ColorHelper::load(string cameraFile, int flags) {
    if (filenames.empty()) readCameraFile(cameraFile);
    depth.resize(filenames.size(), NULL);
    conf.resize(filenames.size(), NULL);
    data.resize(filenames.size(), NULL);
    labels.resize(filenames.size(), NULL);
    edges.resize(filenames.size(), NULL);
    for (int i = 0; i < filenames.size(); ++i) {
        if (!load(i, flags)) return false;
    }
    return true;
}
bool ColorHelper::load(int i, int flags) {
    int w = cameras[i]->width;
    int h = cameras[i]->height;
    if (flags & READ_CONFIDENCE) {
        conf[i] = (float*) readConfidenceFile(i);
        if (!conf[i]) {
            cerr << "Error reading confidence file!" << endl;
            return false;
        }
        flip((char*)conf[i], w, h, sizeof(float));
    }
    if (flags & READ_DEPTH) {
        if (fileexists(replaceExtension(filenames[i], "pcd"))) {
            depth[i] = (float*) readDepthMap(i);
            if (depth[i]) flip((char*)depth[i], w, h, sizeof(float));
        }
    }
    if (flags & READ_COLOR) {
        data[i] = (char*) readImage(i);
        if (!data[i]) {
            cerr << "Error reading image " << filenames[i] << endl;
            return false;
        }
        flip(data[i], w, h, 3*sizeof(float));
    }
    if (flags & READ_EDGES) {
        if (fileexists(replaceExtension(filenames[i], "edges.exr"))) {
            readExrImage(replaceExtension(filenames[i], "edges.exr"), edges[i], w, h, 1);
            if (!edges[i]) {
                cerr << "Error reading image " << filenames[i] << endl;
                return false;
            }
        }
    }
    return true;
}
bool endswith(const string& s, string e) {
    if (s.length() > e.length())
        return s.compare(s.length()-e.length(), e.length(), e) == 0;
    else
        return false;
}

void* ColorHelper::readImage(int idx) {
    string filename = filenames[idx];
    if (endswith(filename, ".png"))
        return readPngImage(idx);
    else if (endswith(filename, ".hdr") || endswith(filename, ".pic"))
        return readHdrImage(idx);
    else if (endswith(filename, ".exr"))
        return readExrImage(idx);
    else
        return NULL;
}

void* ColorHelper::readDepthMap(int idx) {
    string filename = replaceExtension(filenames[idx], "pcd");
    try {
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        if (io::loadPCDFile<PointXYZ>(filename, *cloud) == -1) {
            return NULL;
        }
        int w = cameras[idx]->width;
        int h = cameras[idx]->height;
        float* f = new float[w*h];
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                PointXYZ p = cloud->at(j,i);
                if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
                    f[(h-i-1)*w+j] = numeric_limits<float>::infinity();
                } else {
                    f[(h-i-1)*w+j] = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
                }
            }
        }
        return f;
    } catch(...) {
        return NULL;
    }
}

void* ColorHelper::readConfidenceFile(int idx) {
    try {
        string filename = replaceExtension(filenames[idx], "conf");
        int w = cameras[idx]->width;
        int h = cameras[idx]->height;
        float* confidences = new float[w*h];

        ifstream in(filename.c_str(), ifstream::binary);
        for (int i = 0; i < w*h; ++i) {
            in.read((char*) &confidences[i], sizeof(float));
        }
        return confidences;
    } catch (...) {
        return NULL;
    }
}

bool ColorHelper::writeExrImage(const string& filename,
        const float* image,
        int width,
        int height,
        int channels)
{
    Rgba* pixels = new Rgba[width*height];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int idx = j + (height-i-1)*width;
            int fidx = j + i*width;
            pixels[fidx].r = image[channels*idx];
            if (channels > 1) {
                pixels[fidx].g = image[channels*idx+1];
                pixels[fidx].b = image[channels*idx+2];
            } else {
                pixels[fidx].g = image[idx];
                pixels[fidx].b = image[idx];
            }
            if (channels > 3) {
                pixels[fidx].a = image[channels*idx+3];
            } else {
                pixels[fidx].a = 1;
            }
        }
    }
    RgbaOutputFile file(filename.c_str(), width, height, WRITE_RGBA);
    file.setFrameBuffer(pixels, 1, width);
    file.writePixels(height);
    delete pixels;
}

void* ColorHelper::readExrImage(int i) {
    float* image;
    int width, height;
    ColorHelper::readExrImage(filenames[i], image, width, height);
    return image;
}

bool ColorHelper::readExrImage(const string& filename, float*& image, int& width, int& height, int channels) {
    RgbaInputFile f(filename.c_str());
    Box2i dw = f.dataWindow();
    width = dw.max.x - dw.min.x + 1;
    height = dw.max.y - dw.min.y + 1;
    Array2D<Rgba> pixels;
    pixels.resizeErase(height, width);
    f.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * width, 1, width);
    f.readPixels(dw.min.y, dw.max.y);
    image = new float[channels*width*height];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int idx = width*(height-i-1) + j;
            image[channels*idx] = pixels[i][j].r;
            if (channels > 1) {
                image[channels*idx+1] = pixels[i][j].g;
                image[channels*idx+2] = pixels[i][j].b;
                if (channels > 3)
                    image[channels*idx+3] = pixels[i][j].a;
            }
        }
    }
    return true;
}

void* ColorHelper::readHdrImage(int idx) {
    int width, height;
    rgbe_header_info info;
    FILE* file = fopen(filenames[idx].c_str(), "rb");

    RGBE_ReadHeader(file, &width, &height, &info);
    float* image = new float[3*width*height];
    RGBE_ReadPixels_RLE(file, image, width, height);
    float expadj = info.exposure;
    for (int i = 0; i < width*height*3; ++i) {
        image[i] /= expadj;
    }
    // Flip y-coordinate (standard is 0,0 at top)
    float* row = new float[3*width];
    for (int i = 0; i < height/2; ++i) {
        memcpy(row, image+3*(i*width), 3*width*sizeof(float));
        memcpy(image+3*(i*width), image+3*((height-i-1)*width), 3*width*sizeof(float));
        memcpy(image+3*((height-i-1)*width), row, 3*width*sizeof(float));
    }
    delete [] row;
    fclose(file);
    return image;
}
// From http://blog.nobel-joergensen.com/2010/11/07/loading-a-png-as-texture-in-opengl-using-libpng/
void* ColorHelper::readPngImage(int idx)
{
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    FILE *fp;

    if ((fp = fopen(filenames[idx].c_str(), "rb")) == NULL)
        return NULL;

    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL) {
        fclose(fp);
        return NULL;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        fclose(fp);
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return NULL;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return NULL;
    }
    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, sig_read);
    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_STRIP_ALPHA | PNG_TRANSFORM_PACKING | PNG_TRANSFORM_EXPAND, NULL);
    png_uint_32 width, height;
    int bit_depth;
    png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type,
                 &interlace_type, NULL, NULL);

    unsigned int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    char* imagedata = new char[width*height*3];

    png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);

    for (int i = 0; i < height; i++) {
        memcpy(imagedata+(row_bytes * (height-1-i)), row_pointers[i], row_bytes);
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
    return imagedata;
}

bool ColorHelper::readCameraFile(const string& filename)
{
    // Format:
    //    FrameCount Width Height vfov [transformfilename]
    //    filename1 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z
    //    filename2 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z
    //    filename3 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z
    //    ...
    //    Note: angles in degrees
    try {
        ifstream in(filename.c_str());
        int frames, w, h;
        double hfov, vfov;
        string infoline;
        getline(in, infoline);
        size_t numparams = count(infoline.begin(), infoline.end(), ' ') + 1;
        stringstream infoin(infoline);
        if (numparams < 4) {
            cerr << "Error! Invalid camera file!" << endl;
            return false;
        }
        infoin >> frames >> w >> h >> vfov;
        double foc = h/(2*tan(vfov*M_PI/360));
        double a,b,c;
        string s;
        for (int i = 0; i < frames; ++i) {
            CameraParams* curr = new CameraParams;
            in >> s >> a >> b >> c;
            filenames.push_back(s);
#ifdef OUTPUT_RADIANCE_CAMERAS
            printf("view= pos%d rvu -vtv -vp %f %f %f", i, a, b, c);
#endif
            curr->pos.Reset(a,b,c);
            in >> a >> b >> c;
            curr->up.Reset(a,b,c);
            in >> a >> b >> c;
            curr->towards.Reset(a,b,c);
#ifdef OUTPUT_RADIANCE_CAMERAS
            printf(" -vu %f %f %f", curr->up[0], curr->up[1], curr->up[2]);
            printf(" -vd %f %f %f", curr->towards[0], curr->towards[1], curr->towards[2]);
            hfov = vfov*curr->width/curr->height;
            printf(" -vh %f -vv %f -vo 0 -va 0 -vs 0 -vl 0\n", hfov, vfov);
#endif
            curr->right = curr->towards%curr->up;
            curr->width = w;
            curr->height = h;
            curr->focal_length = foc;
            curr->fov = vfov;
            cameras.push_back(curr);
        }
        if (numparams > 4) {
            string camxform;
            infoin >> camxform;
            ifstream xformin(camxform.c_str());
            double m[16];
            for (int i = 0; i < 16; ++i) xformin >> m[i];
            depth2rgb = R4Matrix(m);
            transformAllCameras(depth2rgb);
        }
    } catch (...) {
        return false;
    }
    return true;
}

void ColorHelper::transformAllCameras(const R4Matrix& m) {
    for (int i = 0; i < cameras.size(); ++i) {
        CameraParams* cam = cameras[i];
        cam->pos = m*cam->pos;
        cam->up = m*cam->up;
        cam->towards = m*cam->towards;
        cam->right = m*cam->right;
    }
}

void ColorHelper::flip(char* a, int w, int h, size_t bytes) {
    char* tmp = new char[bytes];
    if (flip_x) {
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w/2; ++j) {
                memcpy(tmp, a+(i*w+j)*bytes, bytes);
                memcpy(a+(i*w+j)*bytes, a+(i*w+w-j-1)*bytes, bytes);
                memcpy(a+(i*w+w-j-1)*bytes, tmp, bytes);
            }
        }
    }
    delete tmp;
    tmp = new char[bytes*w];
    if (flip_y) {
        for (int i = 0; i < h/2; ++i) {
            memcpy(tmp, a+i*w*bytes, bytes*w);
            memcpy(a+i*w*bytes, a+(h-i-1)*w*bytes, bytes*w);
            memcpy(a+(h-i-1)*w*bytes, tmp, bytes*w);
        }
    }
    delete tmp;
}

void ColorHelper::writeEdgeImages() {
    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i])
            writeExrImage(replaceExtension(filenames[i], "edges.exr"),
                          edges[i], cameras[i]->width, cameras[i]->height, 3);
    }
}
