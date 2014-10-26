// Source file for image class



// Include files

#include "R2ImageOld.h"

#include <ctime>
#include <cstdio>
#include <cstring>
#include <cctype>

////////////////////////////////////////////////////////////////////////
// Constructors/Destructors
////////////////////////////////////////////////////////////////////////


using namespace R2Image_Old;

using namespace std;

const double R2Image::IMGGAMMA = 2.2;

R2Image::
    R2Image(void)
: pixels(NULL),
    npixels(0),
    width(0),
    height(0)
{
}



R2Image::
    R2Image(const char *filename)
: pixels(NULL),
    npixels(0),
    width(0),
    height(0)
{
    // Read image
    Read(filename);
}



R2Image::
    R2Image(int width, int height)
: pixels(NULL),
    npixels(width * height),
    width(width),
    height(height)
{
    // Allocate pixels
    pixels = new R2Pixel [ npixels ];
    assert(pixels);
}



R2Image::
    R2Image(int width, int height, const R2Pixel *p)
: pixels(NULL),
    npixels(width * height),
    width(width),
    height(height)
{
    // Allocate pixels
    pixels = new R2Pixel [ npixels ];
    assert(pixels);

    // Copy pixels
    for (int i = 0; i < npixels; i++)
        pixels[i] = p[i];
}



R2Image::
    R2Image(const R2Image& image)
: pixels(NULL),
    npixels(image.npixels),
    width(image.width),
    height(image.height)

{
    // Allocate pixels
    pixels = new R2Pixel [ npixels ];
    assert(pixels);

    // Copy pixels
    for (int i = 0; i < npixels; i++)
        pixels[i] = image.pixels[i];
}



R2Image::
~R2Image(void)
{
    // Free image pixels
    if (pixels) delete [] pixels;
}



R2Image& R2Image::
operator=(const R2Image& image)
{
    // Delete previous pixels
    if (pixels) { delete [] pixels; pixels = NULL; }

    // Reset width and height
    npixels = image.npixels;
    width = image.width;
    height = image.height;

    // Allocate new pixels
    pixels = new R2Pixel [ npixels ];
    assert(pixels);

    // Copy pixels
    for (int i = 0; i < npixels; i++)
        pixels[i] = image.pixels[i];

    // Return image
    return *this;
}



////////////////////////////////////////////////////////////////////////
// Image processing functions
// YOU IMPLEMENT THE FUNCTIONS IN THIS SECTION
////////////////////////////////////////////////////////////////////////

// Per-pixel Operations ////////////////////////////////////////////////

void R2Image::
Brighten(double factor)
{
    // Brighten the image by multiplying each pixel component by the factor,
    // then clamping the result to a valid range.

    Interpolate(R2Pixel(), factor);
}

void R2Image::
AddNoise(double factor)
{
    // Add noise to an image.  The amount of noise is given by the factor
    // in the range [0.0..1.0].  0.0 adds no noise.  1.0 adds a lot of noise.
    // TODO: Is this right?
    for (int i = 0; i < npixels; ++i) {
        for (int j = 0; j < 3; ++j) {
            double r = rand();
            //r = r/(MAX_RAND >> 1) - 1.;  // -1 < r < 1
            pixels[i][j] *= r+1;
        }
        pixels[i].Clamp();
    }
}

void R2Image::
ChangeContrast(double factor)
{
    // Change the contrast of an image by interpolating between the image
    // and a constant gray image with the average luminance.
    // Interpolation reduces constrast, extrapolation boosts constrast,
    // and negative factors generate inverted images.

    // Calculate average luminance
    double tl = 0;
    for (int i = 0; i < npixels; ++i) {
        tl += pixels[i].Luminance();
    }
    tl /= npixels;
    R2Pixel p(tl, tl, tl, 0);
    Interpolate(p, factor);
}

void R2Image::
ChangeSaturation(double factor)
{
    // Changes the saturation of an image by interpolating between the
    // image and a gray level version of the image.  Interpolation
    // decreases saturation, extrapolation increases it, negative factors
    // preserve luminance  but invert the hue of the input image.

    R2Image bw = *this;
    bw.BlackAndWhite();
    Interpolate(bw, factor);
}

void R2Image::
ApplyGamma(double exponent)
{
    // Apply a gamma correction with exponent to each pixel
    for (int i = 0; i < npixels; ++i) {
        pixels[i].Gamma(exponent);
    }
}

void R2Image::
BlackAndWhite(void)
{
    // Replace each pixel with its luminance value
    // Put this in each channel, so the result is grayscale
    for (int i = 0; i < npixels; ++i) {
        double l = pixels[i].Luminance();
        pixels[i].Reset(l, l, l, pixels[i].Alpha());
    }
}

void R2Image::
ExtractChannel(int channel)
{
    // Extracts a channel of an image (e.g., R2_IMAGE_RED_CHANNEL).
    // Leaves the specified channel intact,
    // and sets all the other ones to zero.
    for (int i = 0; i < npixels; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (j != channel) pixels[i][j] = 0;
        }
    }
}

// Linear filtering ////////////////////////////////////////////////
static double gaussian2(double x, double s) {
    return exp(-x/(2*s*s));
}

R2Pixel& R2Image::getKernelPixel(int i, int j, int x, int y, Filter& f) {
    int xx = x - f.xsize()/2 + j;
    int yy = y - f.ysize()/2 + i;
    // When we cross an edge, reflect
    if (xx < 0) {
        xx = -xx;
    }
    else if (xx >= width) {
        xx = 2*width - xx - 1;
    }
    if (yy < 0) {
        yy = -yy;
    }
    else if (yy >= height) {
        yy = 2*height - yy - 1;
    }
    return Pixel(xx, yy);
}

R2Pixel R2Image::applyFilterToPixel(int x, int y, Filter& k) {
    R2Pixel val(0,0,0, Pixel(x,y).Alpha());
    for (int i = 0; i < k.ysize(); ++i) {
        for (int j = 0; j < k.xsize(); ++j) {
            val += getKernelPixel(i, j, x, y, k)*k[i][j];
        }
    }
    return val;
}

void R2Image::
LinearFilter(Filter& k)
{
    // Apply the linear filter with kernel k
    ApplyGamma(IMGGAMMA);
    R2Image orig = *this;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Pixel(x, y) = orig.applyFilterToPixel(x, y, k);
            Pixel(x, y).Clamp();
        }
    }
    ApplyGamma(1/IMGGAMMA);
}

Filter& R2Image::getGaussian(Filter& f, double sigma) {
    int dim = ceil(3*sigma);
    f.reset(2*dim + 1);
    for (int i = 0; i <= dim; ++i) {
        for (int j = i; j <= dim; ++j) {
            double d = (dim - i)*(dim - i) + (dim - j)*(dim - j);
            double n = gaussian2(d, sigma);
            f[i][j] = n;
            f[i][2*dim - j] = n;
            f[2*dim - i][j] = n;
            f[2*dim - i][2*dim - j] = n;
            if (j != i) {
                f[j][i] = n;
                f[j][2*dim - i] = n;
                f[2*dim - j][i] = n;
                f[2*dim - j][2*dim - i] = n;
            }
        }
    }
    f.normalize();
    return f;
}

void R2Image::
Blur(double sigma)
{
    // Blur an image with a Gaussian filter with a given sigma.
    Filter k;
    LinearFilter(getGaussian(k, sigma));
}


void R2Image::
Sharpen()
{
    // Sharpen an image using a linear filter
    Filter k(3);
    for (int i = 0; i < k.size(); ++i) {
        for (int j = 0; j < k.size(); ++j) {
            k[i][j] = -1;
        }
    }
    k[1][1] = 9;

    LinearFilter(k);
}


void R2Image::
EdgeDetect(void)
{
    // Detect edges in an image.
    Filter k(3);
    for (int i = 0; i < k.size(); ++i) {
        for (int j = 0; j < k.size(); ++j) {
            k[i][j] = -1;
        }
    }
    k[1][1] = 8;

    LinearFilter(k);
}

void R2Image::
SobelEdgeDetect(void)
{
    // Detect edges in an image.
    Filter kx(3);
    Filter ky(3);
    for (int i = 0; i < 3; ++i) {
        kx[0][i] = -1;
        kx[1][i] = 0;
        kx[2][i] = 1;
        ky[i][0] = -1;
        ky[i][1] = 0;
        ky[i][2] = 1;
    }
    for (int i = 0; i < 3; ++i) {
        kx[i][1] *= 2;
        ky[1][i] *= 2;
    }
    R2Image ix = *this;
    ix.LinearFilter(kx);
    LinearFilter(ky);
    for (int i = 0; i < npixels; ++i) {
        for (int j = 0; j < 3; ++j) {
            pixels[i][j] =
                pixels[i][j]*pixels[i][j] + ix.pixels[i][j]*ix.pixels[i][j];
            pixels[i][j] = sqrt(pixels[i][j]);
        }
    }
}



void R2Image::
MotionBlur(int amount)
{
    // Perform horizontal motion blur

    // convolve in X direction with a linear ramp of amount non-zero pixels
    // the image should be strongest on the right hand side (see example)
    if (!(amount % 2)) ++amount;
    Filter f(amount, 1);
    for (int i = 0; i <= amount/2; ++i) {
        f[0][i] = 0;
    }
    for (int i = amount/2 + 1; i < f.xsize(); ++i) {
        f[0][i] = amount - i;
        //f[0][i] = 1;
    }
    f.normalize();

    LinearFilter(f);
}


// Non-Linear filtering ////////////////////////////////////////////////

void toCielab(R2Pixel& a) {
    double aa = 0.055;
    // Linearize
    for (int i = 0; i < 3; ++i) {
        if (a[i] < 0.04045) {
            a[i] /= 12.92;
        }
        else {
            a[i] = pow((a[i] + aa)/(aa+1), 2.4);
        }
    }

    // Convert to xyz
    double x = 0.412453*a[0] + 0.357580*a[1] + 0.180423*a[2];
    double y = 0.212671*a[0] + 0.715160*a[1] + 0.072169*a[2];
    double z = 0.019334*a[0] + 0.119193*a[1] + 0.950227*a[2];

    // Normalize
    x /= 0.950456;
    z /= 1.088754;
    a[0] = x;
    a[1] = y;
    a[2] = z;


    // Convert to CIELAB
    for (int i = 0; i < 3; ++i) {
        if (a[i] > (6*6*6)/(29*29*29)) {
            a[i] = pow(a[i], 1./3);
        }
        else {
            a[i] = 29*29*a[i]/(6*6*3) + 4./29;
        }
    }

    x = a[0];
    y = a[1];
    z = a[2];
    a[0] = 116*y - 16;
    a[1] = 500*(x - y);
    a[2] = 200*(y - z);
}

double diff2(R2Pixel& a, R2Pixel& b) {
    double tot = 0;
    for (int i = 0; i < 3; ++i) tot += (a[i] - b[i])*(a[i] - b[i]);
    return tot;
}

void swap(double a[], int i, int j) {
    double tmp = a[i];
    a[i] = a[j];
    a[j] = tmp;
}

double qsel(double a[], int l, int r, int k) {
    // Quick select implementation based on COS226
    while (1) {
        if (l >= r) {
            return a[l];
        }
        int pv = (rand()%(r-l)) + l;

        double pivot = a[pv];
        swap(a, pv, l);
        int lt = l;
        int gt = r;
        int i = l;
        while (i <= gt) {
            if (a[i] == pivot) ++i;
            else if (a[i] < pivot) swap(a, lt++, i++);
            else swap(a, i, gt--);
        }

        if (k <= gt && k >= lt) {
            return pivot;
        }
        else if (k < lt) {
            r = lt-1;
        }
        else {
            l = gt+1;
        }
    }
    return -1;
}

R2Pixel findMedian(R2Pixel** window, double arr[], int dim) {
    R2Pixel ret;
    for (int k = 0; k < 3; ++k) {
        int z = 0;
        for (int i = 0; i < dim; ++i) {
            for (int j = 0; j < dim; ++j) {
                if (window[i][j][3] >= 0) arr[z++] = window[i][j][k];
            }
        }
        ret[k] = qsel(arr, 0, z-1, z/2);
    }
    return ret;
}

R2Pixel R2Image::
MedGetPixel(int x, int y) {
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return R2Pixel(0,0,0,-1);
    }
    return Pixel(x,y);
}

void R2Image::
MedianFilter(double sigma)
{
    // Perform median filtering with a given width
    int s = sigma;
    int dim = s*2 + 1;
    R2Pixel** m;
    m = new R2Pixel*[dim];
    for (int i = 0; i < dim; ++i) m[i] = new R2Pixel[dim];
    double arr[dim*dim];
    R2Image orig = *this;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int i = -s; i <= s; ++i) {
                for (int j = -s; j <= s; ++j) {
                    m[i+s][j+s] = orig.MedGetPixel(x+j, y+i);
                }
            }
            Pixel(x,y) = findMedian(m, arr, dim);
        }
    }
    for (int i = 0; i < dim; ++i) delete [] m[i];
    delete [] m;
}

void R2Image::
BilateralFilter(double rangesigma, double domainsigma)
{
    // Perform bilateral filtering with a given range and domain widths.
    int maxwidth = ceil(3*domainsigma);
    double tot;
    R2Image cielab = *this;
    for (int i = 0; i < npixels; ++i) {
        toCielab(cielab.pixels[i]);
    }
    R2Pixel curr;
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            curr.Reset(0,0,0,Pixel(x,y).Alpha());
            tot = 0;
            for (int i = -maxwidth; i <= maxwidth; ++i) {
                if (x+i < 0) continue;
                if (x+i >= width) break;
                for (int j = -maxwidth; j <= maxwidth; ++j) {
                    if (y+j < 0) continue;
                    if (y+j >= height) break;
                    double c = gaussian2(i*i + j*j, domainsigma);
                    double d = diff2(cielab.Pixel(x,y), cielab.Pixel(x+i, y+j));
                    double s = gaussian2(d, rangesigma);
                    curr += Pixel(x+i, y+j)*c*s;
                    tot += c*s;
                }
            }
            Pixel(x,y) = curr/tot;
        }
    }
}


// Resampling operations  ////////////////////////////////////////////////

R2Pixel R2Image::
Sample(double x, double y, int method, int extent=1, int boundary=0, bool filtered=false) {
    if (boundary) {
        if (x < 0) x = 0;
        else if (x >= width) x = width-1;

        if (y < 0) y = 0;
        else if (y >= height) y = height-1;
    }
    else if (x < 0 || y < 0 || x >= width || y >= height) return R2null_pixel;

    switch (method) {
        case R2_IMAGE_POINT_SAMPLING:
            return SamplePoint(x, y, filtered);
        case R2_IMAGE_BILINEAR_SAMPLING:
            return SampleBilinear(x, y, filtered);
        case R2_IMAGE_GAUSSIAN_SAMPLING:
            return SampleGaussian(x, y, extent, filtered);
        default:
            fprintf(stderr, "Error: Unknown sampling method");
    }
    return R2Pixel();
}

R2Pixel R2Image::
SampleGaussian(double x, double y, int extent, bool filtered) {
    if (filtered && Pixel(x+0.5,y+0.5).Alpha() == 0) return R2null_pixel;
    if (extent < 1) extent = 1;
    int xlo = max(0., x - extent);
    int xhi = min((double) width, x + extent + 0.5);
    int ylo = max(0., y - extent);
    int yhi = min((double) height, y + extent + 0.5);

    double tot = 0;
    R2Pixel p;
    for (int i = xlo; i < xhi; ++i) {
        for (int j = ylo; j < yhi; ++j) {
            double f = (x-i)*(x-i) + (y-j)*(y-j);
            if (!filtered || Pixel(i,j).Alpha() > 0) {
                f = gaussian2(f, extent);
                p += f*Pixel(i, j);
                tot += f;
            }
        }
    }
    if (tot == 0) return R2null_pixel;
    p /= tot;
    return p;
}

R2Pixel R2Image::
SampleBilinear(double x, double y, bool filtered) {
    int xx = x;
    int yy = y;
    if (x >= width-1 || y >= height-1 || (filtered && Pixel(xx,yy).Alpha() == 0))
        return R2Pixel();

    R2Pixel px;
    R2Pixel p = (xx + 1 - x)*(yy + 1 - y)*Pixel(xx,yy);
    if (!filtered || Pixel(xx+1, yy+1).Alpha() > 0) px = Pixel(xx+1, yy+1);
    else px = Pixel(xx,yy);
    p += (x - xx)*(y - yy)*px;
    if (!filtered || Pixel(xx,   yy+1).Alpha() > 0) px = Pixel(xx,   yy+1);
    else px = Pixel(xx,yy);
    p += (xx + 1 - x)*(y - yy)*px;
    if (!filtered || Pixel(xx+1, yy  ).Alpha() > 0) px = Pixel(xx+1, yy);
    else px = Pixel(xx,yy);
    p += (x - xx)*(yy + 1 - y)*px;

    return p;
}

R2Pixel R2Image::
SamplePoint(double x, double y, bool filtered) {
    int xx = x + 0.5;
    int yy = y + 0.5;
    /*if (xx < 0) xx = 0;
    if (yy < 0) yy = 0;
    if (xx >= width)  xx = width-1;
    if (yy >= height) yy = height-1;*/
    if (!filtered || Pixel(xx,yy).Alpha() > 0) return Pixel(xx, yy);
    else return R2null_pixel;
}

void R2Image::
Scale(double sx, double sy, int sampling_method)
{
    // Scale an image in x by sx, and y by sy.

    R2Image src = *this;
    delete [] pixels;

    int w = ceil(max(1/sx, 1/sy));
    width = sx*width;
    height = sy*height;

    pixels = new R2Pixel[width*height];
    assert(pixels);

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            double u = i/sx;
            double v = j/sy;
            Pixel(i,j) = src.Sample(u, v, sampling_method, w);
        }
    }
}


void R2Image::
Rotate(double angle, int sampling_method)
{
    // Rotate an image by the given angle.
    static double PI = 3.1415926536f;
    R2Image src = *this;
    delete [] pixels;

    angle = fmod(angle, 2*PI);
    if (angle < 0) angle += 2*PI;

    // Calculate width and height
    int hh = ceil(abs(width*sin(angle)) + abs(height*cos(angle)));
    int ww = ceil(abs(width*cos(angle)) + abs(height*sin(angle)));

    double di = 0;
    double dj = 0;
    double a;
    if (angle < PI/2) {
        dj = height*sin(angle);
    }
    else if (angle < PI) {
        a = angle - PI/2;
        dj = width*sin(a) + height*cos(a);
        di = height*sin(a);
    }
    else if (angle < 3*PI/2) {
        a = angle - PI;
        dj = width*cos(a);
        di = width*sin(a) + height*cos(a);
    }
    else {
        a = angle - 3*PI/2;
        di = width*cos(a);
    }

    width = ww;
    height = hh;
    pixels = new R2Pixel[width*height];
    assert(pixels);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double u = (j-dj)*cos(-angle) - (i-di)*sin(-angle);
            double v = (i-di)*cos(-angle) + (j-dj)*sin(-angle);
            Pixel(j,i) = src.Sample(u, v, sampling_method);
        }
    }
}

// Dither operations ////////////////////////////////////////////////

void quantizePixel(R2Pixel& pixel, int nbits) {
    for (int j = 0; j < 4; ++j) {
        int level = pixel[j]*((1<<nbits) - 1) + 0.5;
        pixel[j] = ((double) level)/((1<<nbits)-1);
    }
    pixel.Clamp();
}

void R2Image::
Quantize (int nbits)
{
    // Quantizes an image with "nbits" bits per channel.
    for (int i = 0; i < npixels; ++i) {
        quantizePixel(pixels[i], nbits);
    }
}

void R2Image::
RandomDither(int nbits)
{
    // Converts and image to nbits per channel using random dither.
    srand(time(0));
    double noiselevel = 1./((1<<nbits) - 1);

    for (int i = 0; i < npixels; ++i) {
        for (int j = 0; j < 3; ++j) {
            double r = ((double) rand())/RAND_MAX - 0.5;
            pixels[i][j] += r * noiselevel;
        }
        quantizePixel(pixels[i], nbits);
    }
}



void R2Image::
OrderedDither(int nbits)
{
    // Converts an image to nbits per channel using ordered dither,
    // with a 4x4 Bayer's pattern matrix.
    static double bayer[4][4]  = {{15./16,  7./16, 13./16,  5./16},
                                  { 3./16, 11./16,  1./16,  9./16},
                                  {12./16,  4./16, 14./16,  6./16},
                                  { 0,      8./16,  2./16, 10./16}};

    double incr = 1./((1<<nbits) - 1);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            for (int k = 0; k < 4; ++k) {
                double r = bayer[i%4][j%4] - 0.5;
                Pixel(j,i)[k] += r*incr;
            }
            quantizePixel(Pixel(j,i), nbits);
        }
    }
}



void R2Image::
FloydSteinbergDither(int nbits)
{
    // Converts an image to nbits per channel using Floyd-Steinberg dither.
    // with error diffusion.
    double coef[] = {7./16, 3./16, 5./16, 1./16};
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            R2Pixel orig = Pixel(j,i);
            quantizePixel(Pixel(j,i), nbits);
            R2Pixel d = orig - Pixel(j,i);
            if (j != width - 1)                    Pixel(j+1,   i) += coef[0]*d;
            if (j != 0 && i != height - 1)         Pixel(j-1, i+1) += coef[1]*d;
            if (i != height - 1)                   Pixel(j,   i+1) += coef[2]*d;
            if (i != height - 1 && j != width - 1) Pixel(j+1, i+1) += coef[3]*d;
        }
    }
}



// Miscellaneous operations ////////////////////////////////////////////////

void R2Image::
CopyChannel(const R2Image& from_image, int from_channel, int to_channel)
{
    // Copies one channel of an image (e.g., R2_IMAGE_RED_CHANNEL).
    // to another channel

    for (int i = 0; i < npixels; ++i) {
        pixels[i][to_channel] = from_image.pixels[i][from_channel];
    }
}

void R2Image::
Composite(const R2Image& top, int operation)
{
    // Composite passed image on top of this one using operation OVER
    double ftop = 1;
    double fthis = 0;
    for (int i = 0; i < npixels; ++i) {
        if (operation == R2_IMAGE_OVER_COMPOSITION ||
                operation == R2_IMAGE_ATOP_COMPOSITION ||
                operation == R2_IMAGE_XOR_COMPOSITION) {
            fthis = 1 - top.pixels[i].Alpha();
        }

        if (operation == R2_IMAGE_OUT_COMPOSITION ||
                operation == R2_IMAGE_XOR_COMPOSITION) {
            ftop = 1 - pixels[i].Alpha();
        }
        else if (operation == R2_IMAGE_IN_COMPOSITION ||
                operation == R2_IMAGE_ATOP_COMPOSITION) {
            ftop = pixels[i].Alpha();
        }

        pixels[i] = ftop*top.pixels[i].Alpha()*top.pixels[i] +
            fthis*pixels[i].Alpha()*pixels[i];
    }
}

void R2Image::
Crop(int x, int y, int w, int h)
{
    // Extracts a sub image from the image,
    // at position (x, y), width w, and height h.
    if (x + w > width || y + h > height) {
        fprintf(stderr, "Error! Invalid arguments!\n");
        return;
    }
    R2Pixel* newpixels = new R2Pixel[w*h];
    int z = 0;

    for (int i = y; i < y+h; ++i) {
        for (int j = x; j < x+w; ++j) {
            newpixels[z++] = Pixel(j, i);
        }
    }

    delete pixels;
    pixels = newpixels;
    npixels = w*h;
    width = w;
    height = h;
}

void R2Image::
Add(const R2Image& image)
{
    // Add passed image pixel-by-pixel.
    if (image.height != height || image.width != width) {
        fprintf(stderr, "Error! Images aren't the same size!\n");
        return;
    }
    for (int i = 0; i < npixels; ++i) {
        pixels[i] += image.pixels[i];
        pixels[i].Clamp();
    }
}

void R2Image::
Subtract(const R2Image& image)
{
    // Subtract passed image from this image.
    if (image.height != height || image.width != width) {
        fprintf(stderr, "Error! Images aren't the same size!\n");
        return;
    }
    for (int i = 0; i < npixels; ++i) {
        pixels[i] -= image.pixels[i];
        pixels[i].Clamp();
    }
}

void R2Image::
Interpolate(const R2Image& image, double factor)
{
    // Interpolate between this image and the passed image by factor
    if (image.height != height || image.width != width) {
        fprintf(stderr, "Error! Interpolating images aren't the same size!\n");
        return;
    }
    for (int i = 0; i < npixels; ++i) {
        pixels[i] = pixels[i]*factor + image.pixels[i]*(1-factor);
        pixels[i].Clamp();
    }
}

void R2Image::
Interpolate(const R2Pixel& pixel, double factor)
{
    // Interpolate between this image and the passed image by factor
    for (int i = 0; i < npixels; ++i) {
        pixels[i] = pixels[i]*factor + pixel*(1-factor);
        pixels[i].Clamp();
    }
}

void R2Image::
Dab(double res) {
    if (res < 0) res = 0;
    if (res > 1) res = 1;
    srand(time(0));
    int nx = res*(width/6 - 1) + 1;
    int ny = res*(height/6 - 1) + 1;

    double ux = ((double) width)/nx;
    double uy = ((double) height)/ny;

    // Generate random central dab points
    int xcoords[ny][nx];
    int ycoords[ny][nx];
    int counts[ny][nx];
    int closest[height][width];
    for (int i = 0; i < ny; ++i) {
        for (int j = 0; j < nx; ++j) {
            xcoords[i][j] = j*ux + (rand()%((int)(ux*.9))) + 0.5;
            ycoords[i][j] = i*uy + (rand()%((int)(uy*.9))) + 0.5;
            counts[i][j] = 0;
        }
    }

    R2Pixel vals[ny][nx];
    // Average colors in all regions
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Calculate nearest point
            int by = -1;
            int bx = -1;
            int d2 = 999999999;
            int a1 = (y + uy/2)/uy-1;
            int b1 = (x + ux/2)/ux-1;
            if (a1 < 0) ++a1;
            if (b1 < 0) ++b1;
            for (int a = a1; a < a1 + 3 && a < ny; ++a) {
                for (int b = b1; b < b1 + 3 && b < nx; ++b) {
                    int dist = (y-ycoords[a][b])*(y-ycoords[a][b]) +
                               (x-xcoords[a][b])*(x-xcoords[a][b]);
                    if (dist < d2) {
                        by = a;
                        bx = b;
                        d2 = dist;
                    }
                }
            }
                if (by == -1) by = ny - 1;
            if (bx == -1) bx = nx - 1;

            // Increment average and store closest point
            vals[by][bx] += Pixel(x, y);
            ++counts[by][bx];
            closest[y][x] = by + bx*ny;
        }
    }
    for (int i = 0; i < ny; ++i) {
        for (int j = 0; j < nx; ++j) {
            vals[i][j] /= counts[i][j];
            for (int k = 0; k < 3; ++k) {
                vals[i][j][k] += ((rand()%11) - 5)*0.004;
            }
            vals[i][j][3] = 1.;
            vals[i][j].Clamp();
        }
    }

    // Write pixels
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int by = closest[i][j]%ny;
            int bx = closest[i][j]/ny;
            Pixel(j,i) = vals[by][bx];
        }
    }
}

void R2Image::
Mosaic(double r) {
    static double EPS = 4e-3;
    Dab(r);
    R2Image edges = *this;
    Brighten(1.1);
    edges.BlackAndWhite();
    edges.EdgeDetect();
    edges.ChangeSaturation(1000);
    for (int i = 0; i < npixels; ++i) {
        if (edges.pixels[i][0] <= EPS &&
                edges.pixels[i][1] <= EPS &&
                edges.pixels[i][2] <= EPS) {
            edges.pixels[i][3] = 0.;
        }
        else {
            for (int j = 0; j < 3; ++j) {
                edges.pixels[i][j] = 0;
            }
        }
    }
    Composite(edges, R2_IMAGE_OVER_COMPOSITION);
}

////////////////////////////////////////////////////////////////////////
// I/O Functions
////////////////////////////////////////////////////////////////////////

int R2Image::
Read(const char *filename)
{
    // Initialize everything
    if (pixels) { delete [] pixels; pixels = NULL; }
    npixels = width = height = 0;

    // Parse input filename extension
    char *input_extension;
    if (!(input_extension = (char*)strrchr(filename, '.'))) {
        fprintf(stderr, "Input file has no extension (e.g., .jpg).\n");
        return 0;
    }

    // Read file of appropriate type
    if (!strncmp(input_extension, ".bmp", 4)) return ReadBMP(filename);
    else if (!strncmp(input_extension, ".ppm", 4)) return ReadPPM(filename);
    else if (!strncmp(input_extension, ".jpg", 4)) return ReadJPEG(filename);
    else if (!strncmp(input_extension, ".jpeg", 5)) return ReadJPEG(filename);

    // Should never get here
    fprintf(stderr, "Unrecognized image file extension");
    return 0;
}



int R2Image::
Write(const char *filename) const
{
    // Parse input filename extension
    char *input_extension;
    if (!(input_extension = (char*)strrchr(filename, '.'))) {
        fprintf(stderr, "Input file has no extension (e.g., .jpg).\n");
        return 0;
    }

    // Write file of appropriate type
    if (!strncmp(input_extension, ".bmp", 4)) return WriteBMP(filename);
    else if (!strncmp(input_extension, ".ppm", 4)) return WritePPM(filename, 1);
    else if (!strncmp(input_extension, ".jpg", 5)) return WriteJPEG(filename);
    else if (!strncmp(input_extension, ".jpeg", 5)) return WriteJPEG(filename);

    // Should never get here
    fprintf(stderr, "Unrecognized image file extension");
    return 0;
}



////////////////////////////////////////////////////////////////////////
// BMP I/O
////////////////////////////////////////////////////////////////////////

#if !defined(_WIN32)

typedef struct tagBITMAPFILEHEADER {
    unsigned short int bfType;
    unsigned int bfSize;
    unsigned short int bfReserved1;
    unsigned short int bfReserved2;
    unsigned int bfOffBits;
} BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER {
    unsigned int biSize;
    int biWidth;
    int biHeight;
    unsigned short int biPlanes;
    unsigned short int biBitCount;
    unsigned int biCompression;
    unsigned int biSizeImage;
    int biXPelsPerMeter;
    int biYPelsPerMeter;
    unsigned int biClrUsed;
    unsigned int biClrImportant;
} BITMAPINFOHEADER;

typedef struct tagRGBTRIPLE {
    unsigned char rgbtBlue;
    unsigned char rgbtGreen;
    unsigned char rgbtRed;
} RGBTRIPLE;

typedef struct tagRGBQUAD {
    unsigned char rgbBlue;
    unsigned char rgbGreen;
    unsigned char rgbRed;
    unsigned char rgbReserved;
} RGBQUAD;

#endif

#define BI_RGB        0L
#define BI_RLE8       1L
#define BI_RLE4       2L
#define BI_BITFIELDS  3L

#define BMP_BF_TYPE 0x4D42 /* word BM */
#define BMP_BF_OFF_BITS 54 /* 14 for file header + 40 for info header (not sizeof(), but packed size) */
#define BMP_BI_SIZE 40 /* packed size of info header */


static unsigned short int WordReadLE(FILE *fp)
{
    // Read a unsigned short int from a file in little endian format
    unsigned short int lsb, msb;
    lsb = getc(fp);
    msb = getc(fp);
    return (msb << 8) | lsb;
}



static void WordWriteLE(unsigned short int x, FILE *fp)
{
    // Write a unsigned short int to a file in little endian format
    unsigned char lsb = (unsigned char) (x & 0x00FF); putc(lsb, fp);
    unsigned char msb = (unsigned char) (x >> 8); putc(msb, fp);
}



static unsigned int DWordReadLE(FILE *fp)
{
    // Read a unsigned int word from a file in little endian format
    unsigned int b1 = getc(fp);
    unsigned int b2 = getc(fp);
    unsigned int b3 = getc(fp);
    unsigned int b4 = getc(fp);
    return (b4 << 24) | (b3 << 16) | (b2 << 8) | b1;
}



static void DWordWriteLE(unsigned int x, FILE *fp)
{
    // Write a unsigned int to a file in little endian format
    unsigned char b1 = (x & 0x000000FF); putc(b1, fp);
    unsigned char b2 = ((x >> 8) & 0x000000FF); putc(b2, fp);
    unsigned char b3 = ((x >> 16) & 0x000000FF); putc(b3, fp);
    unsigned char b4 = ((x >> 24) & 0x000000FF); putc(b4, fp);
}



static int LongReadLE(FILE *fp)
{
    // Read a int word from a file in little endian format
    int b1 = getc(fp);
    int b2 = getc(fp);
    int b3 = getc(fp);
    int b4 = getc(fp);
    return (b4 << 24) | (b3 << 16) | (b2 << 8) | b1;
}



static void LongWriteLE(int x, FILE *fp)
{
    // Write a int to a file in little endian format
    char b1 = (x & 0x000000FF); putc(b1, fp);
    char b2 = ((x >> 8) & 0x000000FF); putc(b2, fp);
    char b3 = ((x >> 16) & 0x000000FF); putc(b3, fp);
    char b4 = ((x >> 24) & 0x000000FF); putc(b4, fp);
}



int R2Image::
ReadBMP(const char *filename)
{
    // Open file
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Unable to open image file: %s\n", filename);
        return 0;
    }

    /* Read file header */
    BITMAPFILEHEADER bmfh;
    bmfh.bfType = WordReadLE(fp);
    bmfh.bfSize = DWordReadLE(fp);
    bmfh.bfReserved1 = WordReadLE(fp);
    bmfh.bfReserved2 = WordReadLE(fp);
    bmfh.bfOffBits = DWordReadLE(fp);

    /* Check file header */
    assert(bmfh.bfType == BMP_BF_TYPE);
    /* ignore bmfh.bfSize */
    /* ignore bmfh.bfReserved1 */
    /* ignore bmfh.bfReserved2 */
    assert(bmfh.bfOffBits == BMP_BF_OFF_BITS);

    /* Read info header */
    BITMAPINFOHEADER bmih;
    bmih.biSize = DWordReadLE(fp);
    bmih.biWidth = LongReadLE(fp);
    bmih.biHeight = LongReadLE(fp);
    bmih.biPlanes = WordReadLE(fp);
    bmih.biBitCount = WordReadLE(fp);
    bmih.biCompression = DWordReadLE(fp);
    bmih.biSizeImage = DWordReadLE(fp);
    bmih.biXPelsPerMeter = LongReadLE(fp);
    bmih.biYPelsPerMeter = LongReadLE(fp);
    bmih.biClrUsed = DWordReadLE(fp);
    bmih.biClrImportant = DWordReadLE(fp);

    // Check info header
    assert(bmih.biSize == BMP_BI_SIZE);
    assert(bmih.biWidth > 0);
    assert(bmih.biHeight > 0);
    assert(bmih.biPlanes == 1);
    assert(bmih.biBitCount == 24);  /* RGB */
    assert(bmih.biCompression == BI_RGB);   /* RGB */
    int lineLength = bmih.biWidth * 3;  /* RGB */
    if ((lineLength % 4) != 0) lineLength = (lineLength / 4 + 1) * 4;
    assert(bmih.biSizeImage == (unsigned int) lineLength * (unsigned int) bmih.biHeight);

    // Assign width, height, and number of pixels
    width = bmih.biWidth;
    height = bmih.biHeight;
    npixels = width * height;

    // Allocate unsigned char buffer for reading pixels
    int rowsize = 3 * width;
    if ((rowsize % 4) != 0) rowsize = (rowsize / 4 + 1) * 4;
    int nbytes = bmih.biSizeImage;
    unsigned char *buffer = new unsigned char [nbytes];
    if (!buffer) {
        fprintf(stderr, "Unable to allocate temporary memory for BMP file");
        fclose(fp);
        return 0;
    }

    // Read buffer
    fseek(fp, (long) bmfh.bfOffBits, SEEK_SET);
    if (fread(buffer, 1, bmih.biSizeImage, fp) != bmih.biSizeImage) {
        fprintf(stderr, "Error while reading BMP file %s", filename);
        return 0;
    }

    // Close file
    fclose(fp);

    // Allocate pixels for image
    pixels = new R2Pixel [ width * height ];
    if (!pixels) {
        fprintf(stderr, "Unable to allocate memory for BMP file");
        fclose(fp);
        return 0;
    }

    // Assign pixels
    for (int j = 0; j < height; j++) {
        unsigned char *p = &buffer[j * rowsize];
        for (int i = 0; i < width; i++) {
            double b = (double) *(p++) / 255;
            double g = (double) *(p++) / 255;
            double r = (double) *(p++) / 255;
            R2Pixel pixel(r, g, b, 1);
            SetPixel(i, j, pixel);
        }
    }

    // Free unsigned char buffer for reading pixels
    delete [] buffer;

    // Return success
    return 1;
}



int R2Image::
WriteBMP(const char *filename) const
{
    // Open file
    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        fprintf(stderr, "Unable to open image file: %s\n", filename);
        return 0;
    }

    // Compute number of bytes in row
    int rowsize = 3 * width;
    if ((rowsize % 4) != 0) rowsize = (rowsize / 4 + 1) * 4;

    // Write file header
    BITMAPFILEHEADER bmfh;
    bmfh.bfType = BMP_BF_TYPE;
    bmfh.bfSize = BMP_BF_OFF_BITS + rowsize * height;
    bmfh.bfReserved1 = 0;
    bmfh.bfReserved2 = 0;
    bmfh.bfOffBits = BMP_BF_OFF_BITS;
    WordWriteLE(bmfh.bfType, fp);
    DWordWriteLE(bmfh.bfSize, fp);
    WordWriteLE(bmfh.bfReserved1, fp);
    WordWriteLE(bmfh.bfReserved2, fp);
    DWordWriteLE(bmfh.bfOffBits, fp);

    // Write info header
    BITMAPINFOHEADER bmih;
    bmih.biSize = BMP_BI_SIZE;
    bmih.biWidth = width;
    bmih.biHeight = height;
    bmih.biPlanes = 1;
    bmih.biBitCount = 24;       /* RGB */
    bmih.biCompression = BI_RGB;    /* RGB */
    bmih.biSizeImage = rowsize * (unsigned int) bmih.biHeight;  /* RGB */
    bmih.biXPelsPerMeter = 2925;
    bmih.biYPelsPerMeter = 2925;
    bmih.biClrUsed = 0;
    bmih.biClrImportant = 0;
    DWordWriteLE(bmih.biSize, fp);
    LongWriteLE(bmih.biWidth, fp);
    LongWriteLE(bmih.biHeight, fp);
    WordWriteLE(bmih.biPlanes, fp);
    WordWriteLE(bmih.biBitCount, fp);
    DWordWriteLE(bmih.biCompression, fp);
    DWordWriteLE(bmih.biSizeImage, fp);
    LongWriteLE(bmih.biXPelsPerMeter, fp);
    LongWriteLE(bmih.biYPelsPerMeter, fp);
    DWordWriteLE(bmih.biClrUsed, fp);
    DWordWriteLE(bmih.biClrImportant, fp);

    // Write image, swapping blue and red in each pixel
    int pad = rowsize - width * 3;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            const R2Pixel& pixel = (*this)[i][j];
            double r = 255.0 * pixel.Red();
            double g = 255.0 * pixel.Green();
            double b = 255.0 * pixel.Blue();
            if (r >= 255) r = 255;
            if (g >= 255) g = 255;
            if (b >= 255) b = 255;
            fputc((unsigned char) b, fp);
            fputc((unsigned char) g, fp);
            fputc((unsigned char) r, fp);
        }

        // Pad row
        for (int i = 0; i < pad; i++) fputc(0, fp);
    }

    // Close file
    fclose(fp);

    // Return success
    return 1;
}



////////////////////////////////////////////////////////////////////////
// PPM I/O
////////////////////////////////////////////////////////////////////////

int R2Image::
ReadPPM(const char *filename)
{
    // Open file
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Unable to open image file: %s\n", filename);
        return 0;
    }

    // Read PPM file magic identifier
    char buffer[128];
    if (!fgets(buffer, 128, fp)) {
        fprintf(stderr, "Unable to read magic id in PPM file");
        fclose(fp);
        return 0;
    }

    // skip comments
    int c = getc(fp);
    while (c == '#') {
        while (c != '\n') c = getc(fp);
        c = getc(fp);
    }
    ungetc(c, fp);

    // Read width and height
    if (fscanf(fp, "%d%d", &width, &height) != 2) {
        fprintf(stderr, "Unable to read width and height in PPM file");
        fclose(fp);
        return 0;
    }

    npixels = width * height;

    // Read max value
    double max_value;
    if (fscanf(fp, "%lf", &max_value) != 1) {
        fprintf(stderr, "Unable to read max_value in PPM file");
        fclose(fp);
        return 0;
    }

    // Allocate image pixels
    pixels = new R2Pixel [ width * height ];
    if (!pixels) {
        fprintf(stderr, "Unable to allocate memory for PPM file");
        fclose(fp);
        return 0;
    }

    // Check if raw or ascii file
    if (!strcmp(buffer, "P6\n")) {
        // Read up to one character of whitespace (\n) after max_value
        int c = getc(fp);
        if (!isspace(c)) putc(c, fp);

        // Read raw image data
        // First ppm pixel is top-left, so read in opposite scan-line order
        for (int j = height-1; j >= 0; j--) {
            for (int i = 0; i < width; i++) {
                double r = (double) getc(fp) / max_value;
                double g = (double) getc(fp) / max_value;
                double b = (double) getc(fp) / max_value;
                R2Pixel pixel(r, g, b, 1);
                SetPixel(i, j, pixel);
            }
        }
    }
    else {
        // Read asci image data
        // First ppm pixel is top-left, so read in opposite scan-line order
        for (int j = height-1; j >= 0; j--) {
            for (int i = 0; i < width; i++) {
                // Read pixel values
                int red, green, blue;
                if (fscanf(fp, "%d%d%d", &red, &green, &blue) != 3) {
                    fprintf(stderr, "Unable to read data at (%d,%d) in PPM file", i, j);
                    fclose(fp);
                    return 0;
                }

                // Assign pixel values
                double r = (double) red / max_value;
                double g = (double) green / max_value;
                double b = (double) blue / max_value;
                R2Pixel pixel(r, g, b, 1);
                SetPixel(i, j, pixel);
            }
        }
    }

    // Close file
    fclose(fp);

    // Return success
    return 1;
}



int R2Image::
WritePPM(const char *filename, int ascii) const
{
    // Check type
    if (ascii) {
        // Open file
        FILE *fp = fopen(filename, "w");
        if (!fp) {
            fprintf(stderr, "Unable to open image file: %s\n", filename);
            return 0;
        }

        // Print PPM image file
        // First ppm pixel is top-left, so write in opposite scan-line order
        fprintf(fp, "P3\n");
        fprintf(fp, "%d %d\n", width, height);
        fprintf(fp, "255\n");
        for (int j = height-1; j >= 0 ; j--) {
            for (int i = 0; i < width; i++) {
                const R2Pixel& p = (*this)[i][j];
                int r = (int) (255 * p.Red());
                int g = (int) (255 * p.Green());
                int b = (int) (255 * p.Blue());
                fprintf(fp, "%-3d %-3d %-3d  ", r, g, b);
                if (((i+1) % 4) == 0) fprintf(fp, "\n");
            }
            if ((width % 4) != 0) fprintf(fp, "\n");
        }
        fprintf(fp, "\n");

        // Close file
        fclose(fp);
    }
    else {
        // Open file
        FILE *fp = fopen(filename, "wb");
        if (!fp) {
            fprintf(stderr, "Unable to open image file: %s\n", filename);
            return 0;
        }

        // Print PPM image file
        // First ppm pixel is top-left, so write in opposite scan-line order
        fprintf(fp, "P6\n");
        fprintf(fp, "%d %d\n", width, height);
        fprintf(fp, "255\n");
        for (int j = height-1; j >= 0 ; j--) {
            for (int i = 0; i < width; i++) {
                const R2Pixel& p = (*this)[i][j];
                int r = (int) (255 * p.Red());
                int g = (int) (255 * p.Green());
                int b = (int) (255 * p.Blue());
                fprintf(fp, "%c%c%c", r, g, b);
            }
        }

        // Close file
        fclose(fp);
    }

    // Return success
    return 1;
}



////////////////////////////////////////////////////////////////////////
// JPEG I/O
////////////////////////////////////////////////////////////////////////


extern "C" {
#   define XMD_H // Otherwise, a conflict with INT32
#   undef FAR // Otherwise, a conflict with windows.h
#   include "jpeg/jpeglib.h"
};



int R2Image::
ReadJPEG(const char *filename)
{
    // Open file
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Unable to open image file: %s\n", filename);
        return 0;
    }

    // Initialize decompression info
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, fp);
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);

    // Remember image attributes
    width = cinfo.output_width;
    height = cinfo.output_height;
    npixels = width * height;
    int ncomponents = cinfo.output_components;

    // Allocate pixels for image
    pixels = new R2Pixel [ npixels ];
    if (!pixels) {
        fprintf(stderr, "Unable to allocate memory for BMP file");
        fclose(fp);
        return 0;
    }

    // Allocate unsigned char buffer for reading image
    int rowsize = ncomponents * width;
    if ((rowsize % 4) != 0) rowsize = (rowsize / 4 + 1) * 4;
    int nbytes = rowsize * height;
    unsigned char *buffer = new unsigned char [nbytes];
    if (!buffer) {
        fprintf(stderr, "Unable to allocate temporary memory for JPEG file");
        fclose(fp);
        return 0;
    }

    // Read scan lines
    // First jpeg pixel is top-left, so read pixels in opposite scan-line order
    while (cinfo.output_scanline < cinfo.output_height) {
        int scanline = cinfo.output_height - cinfo.output_scanline - 1;
        unsigned char *row_pointer = &buffer[scanline * rowsize];
        jpeg_read_scanlines(&cinfo, &row_pointer, 1);
    }

    // Free everything
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    // Close file
    fclose(fp);

    // Assign pixels
    for (int j = 0; j < height; j++) {
        unsigned char *p = &buffer[j * rowsize];
        for (int i = 0; i < width; i++) {
            double r, g, b, a;
            if (ncomponents == 1) {
                r = g = b = (double) *(p++) / 255;
                a = 1;
            }
            else if (ncomponents == 1) {
                r = g = b = (double) *(p++) / 255;
                a = 1;
                p++;
            }
            else if (ncomponents == 3) {
                r = (double) *(p++) / 255;
                g = (double) *(p++) / 255;
                b = (double) *(p++) / 255;
                a = 1;
            }
            else if (ncomponents == 4) {
                r = (double) *(p++) / 255;
                g = (double) *(p++) / 255;
                b = (double) *(p++) / 255;
                a = (double) *(p++) / 255;
            }
            else {
                fprintf(stderr, "Unrecognized number of components in jpeg image: %d\n", ncomponents);
                return 0;
            }
            R2Pixel pixel(r, g, b, a);
            SetPixel(i, j, pixel);
        }
    }

    // Free unsigned char buffer for reading pixels
    delete [] buffer;

    // Return success
    return 1;
}




int R2Image::
WriteJPEG(const char *filename) const
{
    // Open file
    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        fprintf(stderr, "Unable to open image file: %s\n", filename);
        return 0;
    }

    // Initialize compression info
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);
    cinfo.image_width = width; 	/* image width and height, in pixels */
    cinfo.image_height = height;
    cinfo.input_components = 3;		/* # of color components per pixel */
    cinfo.in_color_space = JCS_RGB; 	/* colorspace of input image */
    cinfo.dct_method = JDCT_ISLOW;
    jpeg_set_defaults(&cinfo);
    cinfo.optimize_coding = TRUE;
    jpeg_set_quality(&cinfo, 75, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    // Allocate unsigned char buffer for reading image
    int rowsize = 3 * width;
    if ((rowsize % 4) != 0) rowsize = (rowsize / 4 + 1) * 4;
    int nbytes = rowsize * height;
    unsigned char *buffer = new unsigned char [nbytes];
    if (!buffer) {
        fprintf(stderr, "Unable to allocate temporary memory for JPEG file");
        fclose(fp);
        return 0;
    }

    // Fill buffer with pixels
    for (int j = 0; j < height; j++) {
        unsigned char *p = &buffer[j * rowsize];
        for (int i = 0; i < width; i++) {
            const R2Pixel& pixel = (*this)[i][j];
            int r = (int) (255 * pixel.Red());
            int g = (int) (255 * pixel.Green());
            int b = (int) (255 * pixel.Blue());
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            *(p++) = r;
            *(p++) = g;
            *(p++) = b;
        }
    }



    // Output scan lines
    // First jpeg pixel is top-left, so write in opposite scan-line order
    while (cinfo.next_scanline < cinfo.image_height) {
        int scanline = cinfo.image_height - cinfo.next_scanline - 1;
        unsigned char *row_pointer = &buffer[scanline * rowsize];
        jpeg_write_scanlines(&cinfo, &row_pointer, 1);
    }

    // Free everything
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    // Close file
    fclose(fp);

    // Free unsigned char buffer for reading pixels
    delete [] buffer;

    // Return number of bytes written
    return 1;
}

