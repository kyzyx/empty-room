// Include file for the pixel class
#ifndef R2_PIXEL_INCLUDED
#define R2_PIXEL_INCLUDED
#include <assert.h>
#include <cmath>


// Class definition

class R2Pixel {
 public:
  // Constructor functions
  R2Pixel(void);
  R2Pixel(const R2Pixel& pixel);
  R2Pixel(RNScalar r, RNScalar g, RNScalar b, RNScalar a=1.0);
  R2Pixel(const RNScalar rgba[4]);

  // Coord access functions/operators
  RNScalar Red(void) const;
  RNScalar Green(void) const;
  RNScalar Blue(void) const;
  RNScalar Alpha(void) const;
  RNScalar R(void) const;
  RNScalar G(void) const;
  RNScalar B(void) const;
  RNScalar A(void) const;
  RNScalar operator[](int i) const;
  RNScalar& operator[](int i);
  RNScalar Coord(int i) const;
  RNScalar *Coords(void);

  // Property functions/operators
  RNScalar Luminance(void) const;
  bool IsBlack(void) const;
  bool IsWhite(void) const;
  bool operator==(const R2Pixel& pixel) const;
  bool operator!=(const R2Pixel& pixel) const;

  // Conversion to YIQ
  RNScalar Y(void) const;
  RNScalar I(void) const;
  RNScalar Q(void) const;

  // Manipulation functions/operations
  void SetRed(RNScalar red);
  void SetGreen(RNScalar green);
  void SetBlue(RNScalar blue);
  void SetAlpha(RNScalar alpha);
  void SetYIQ(RNScalar Y, RNScalar I, RNScalar Q);
  void Reset(RNScalar red, RNScalar green, RNScalar blue, RNScalar alpha=1.0);
  void Clamp(RNScalar maximum_value = 1.0);
  void Gamma(RNScalar exponent);

  // Assignment operators
  R2Pixel& operator=(const R2Pixel& pixel);
  R2Pixel& operator+=(const R2Pixel& pixel);
  R2Pixel& operator-=(const R2Pixel& pixel);
  R2Pixel& operator*=(const R2Pixel& pixel);
  R2Pixel& operator*=(RNScalar scale);
  R2Pixel& operator/=(RNScalar scale);

  // Arithmetic operations
  friend R2Pixel operator+(const R2Pixel& pixel1, const R2Pixel& pixel2);
  friend R2Pixel operator-(const R2Pixel& pixel1, const R2Pixel& pixel2);
  friend R2Pixel operator*(const R2Pixel& pixel1, const R2Pixel& pixel2);
  friend R2Pixel operator*(const R2Pixel& pixel, RNScalar scale);
  friend R2Pixel operator*(RNScalar scale, const R2Pixel& pixel);
  friend R2Pixel operator/(const R2Pixel& pixel, RNScalar scale);

 private:
  RNScalar c[4];
};



// Public variables

extern R2Pixel R2null_pixel;
extern R2Pixel R2black_pixel;
extern R2Pixel R2red_pixel;
extern R2Pixel R2green_pixel;
extern R2Pixel R2blue_pixel;
extern R2Pixel R2yellow_pixel;
extern R2Pixel R2cyan_pixel;
extern R2Pixel R2magenta_pixel;
extern R2Pixel R2white_pixel;
extern R2Pixel R2null_rgb;
extern R2Pixel R2black_rgb;
extern R2Pixel R2red_rgb;
extern R2Pixel R2green_rgb;
extern R2Pixel R2blue_rgb;
extern R2Pixel R2yellow_rgb;
extern R2Pixel R2cyan_rgb;
extern R2Pixel R2magenta_rgb;
extern R2Pixel R2white_rgb;



// Inline functions

inline RNScalar R2Pixel::
Red(void) const
{
  // Return red component of pixel
  return(c[0]);
}
inline RNScalar R2Pixel::
R(void) const
{
  // Return red component of pixel
  return(c[1]);
}



inline RNScalar R2Pixel::
Green(void) const
{
  // Return green component of pixel
  return(c[1]);
}
inline RNScalar R2Pixel::
G(void) const
{
  // Return green component of pixel
  return(c[1]);
}



inline RNScalar R2Pixel::
Blue(void) const
{
  // Return blue component of pixel
  return(c[2]);
}
inline RNScalar R2Pixel::
B(void) const
{
  // Return blue component of pixel
  return(c[2]);
}



inline RNScalar R2Pixel::
Alpha(void) const
{
  // Return alpha component of pixel
  return(c[3]);
}
inline RNScalar R2Pixel::
A(void) const
{
  // Return alpha component of pixel
  return(c[3]);
}



inline RNScalar *R2Pixel::
Coords(void)
{
  // Return pixel array
  return c;
}



inline RNScalar R2Pixel::
Coord(int i) const
{
  assert((i>=0)&&(i<=3));
  return(c[i]);
}



inline RNScalar R2Pixel::
operator[](int i) const
{
  return Coord(i);
}



inline RNScalar& R2Pixel::
operator[] (int i)
{
  assert((i>=0)&&(i<=3));
  return(c[i]);
}



inline RNScalar R2Pixel::
Y(void) const
{
  // Return Y component of YIQ color model
  return 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2];
}



inline RNScalar R2Pixel::
I(void) const
{
  // Return I component of YIQ color model
  return 0.596 * c[0] - 0.274 * c[1] - 0.322 * c[2];
}



inline RNScalar R2Pixel::
Q(void) const
{
  // Return Q component of YIQ color model
  return 0.212 * c[0] - 0.523 * c[1] + 0.311 * c[2];
}



inline bool R2Pixel::
IsBlack (void) const
{
  // Return whether color is black
  return ((c[0] == 0.0) && (c[1] == 0.0) && (c[2] == 0.0));
}



inline bool R2Pixel::
IsWhite (void) const
{
  // Return whether color is white
  return ((c[0] == 1.0) && (c[1] == 1.0) && (c[2] == 1.0));
}



inline RNScalar R2Pixel::
Luminance(void) const
{
  // Return luminance
  return Y();
}



inline void R2Pixel::
SetRed(RNScalar red)
{
  // Set red component
  c[0] = red;
}



inline void R2Pixel::
SetGreen(RNScalar green)
{
  // Set green component
  c[1] = green;
}



inline void R2Pixel::
SetBlue(RNScalar blue)
{
  // Set blue component
  c[2] = blue;
}



inline void R2Pixel::
SetAlpha(RNScalar alpha)
{
  // Set alpha component
  c[3] = alpha;
}



inline void R2Pixel::
SetYIQ(RNScalar Y, RNScalar I, RNScalar Q)
{
  // Set all components
  c[0] = Y + 0.956*I + 0.621*Q;
  c[1] = Y - 0.272*I - 0.647*Q;
  c[2] = Y - 1.105*I + 1.702*Q;
  c[3] = 1;
}



inline void R2Pixel::
Reset (RNScalar red, RNScalar green, RNScalar blue, RNScalar alpha)
{
  // Set all components
  c[0] = red;
  c[1] = green;
  c[2] = blue;
  c[3] = alpha;
}



inline void R2Pixel::
Clamp(RNScalar maximum_value)
{
  // Set all components
  if (c[0] > maximum_value) c[0] = maximum_value;
  if (c[1] > maximum_value) c[1] = maximum_value;
  if (c[2] > maximum_value) c[2] = maximum_value;
  if (c[3] > maximum_value) c[3] = maximum_value;

  for( int i=0; i<4; i++ )
	  if ( c[i] < 0.0 )
		  c[i] = 0.0;
}



inline R2Pixel
operator+(const R2Pixel& pixel1, const R2Pixel& pixel2)
{
  // Add rgb components of two pixels
  RNScalar r = pixel1.c[0] + pixel2.c[0];
  RNScalar g = pixel1.c[1] + pixel2.c[1];
  RNScalar b = pixel1.c[2] + pixel2.c[2];
  RNScalar a = pixel1.c[3];
  return R2Pixel(r, g, b, a);
}



inline R2Pixel
operator-(const R2Pixel& pixel1, const R2Pixel& pixel2)
{
  // Subtract rgb components of two pixels
  RNScalar r = pixel1.c[0] - pixel2.c[0];
  RNScalar g = pixel1.c[1] - pixel2.c[1];
  RNScalar b = pixel1.c[2] - pixel2.c[2];
  RNScalar a = pixel1.c[3];
  return R2Pixel(r, g, b, a);
}



inline R2Pixel
operator*(const R2Pixel& pixel1, const R2Pixel& pixel2)
{
  // Multiply rgb components of two pixels
  RNScalar r = pixel1.c[0] * pixel2.c[0];
  RNScalar g = pixel1.c[1] * pixel2.c[1];
  RNScalar b = pixel1.c[2] * pixel2.c[2];
  RNScalar a = pixel1.c[3];
  return R2Pixel(r, g, b, a);
}



inline R2Pixel
operator*(const R2Pixel& pixel, RNScalar scale)
{
  // Scale rgb components by scalar
  RNScalar r = pixel.c[0] * scale;
  RNScalar g = pixel.c[1] * scale;
  RNScalar b = pixel.c[2] * scale;
  RNScalar a = pixel.c[3];
  return R2Pixel(r, g, b, a);
}



inline R2Pixel
operator*(RNScalar scale, const R2Pixel& pixel)
{
  return pixel * scale;
}



inline R2Pixel
operator/(const R2Pixel& pixel, RNScalar scale)
{
  // Divide rgb components by scalar
  assert(scale != 0);
  RNScalar r = pixel.c[0] / scale;
  RNScalar g = pixel.c[1] / scale;
  RNScalar b = pixel.c[2] / scale;
  RNScalar a = pixel.c[3];
  return R2Pixel(r, g, b, a);
}



// Public functions

inline R2Pixel::
R2Pixel(void)
{
  // Initialize components to zero
  c[0] = 0;
  c[1] = 0;
  c[2] = 0;
  c[3] = 0;
}



inline R2Pixel::
R2Pixel(const R2Pixel& pixel)
{
  // Copy components
  c[0] = pixel.c[0];
  c[1] = pixel.c[1];
  c[2] = pixel.c[2];
  c[3] = pixel.c[3];
}



inline R2Pixel::
R2Pixel(RNScalar red, RNScalar green, RNScalar blue, RNScalar alpha)
{
  // Set components
  c[0] = red;
  c[1] = green;
  c[2] = blue;
  c[3] = alpha;
}

inline R2Pixel::
R2Pixel(const RNScalar rgba[4])
{
  // Set components
  c[0] = rgba[0];
  c[1] = rgba[1];
  c[2] = rgba[2];
  c[3] = rgba[3];
}



inline bool R2Pixel::
operator==(const R2Pixel& pixel) const
{
  // Return whether pixel is equal
  return ((c[0] == pixel.c[0]) && (c[1] == pixel.c[1]) && (c[2] == pixel.c[2]) && (c[3] == pixel.c[3]));
}



inline bool R2Pixel::
operator!=(const R2Pixel& pixel) const
{
  // Return whether pixel is not equal
  return ((c[0] != pixel.c[0]) || (c[1] != pixel.c[1]) || (c[2] != pixel.c[2]) || (c[3] != pixel.c[3]));
}



inline R2Pixel& R2Pixel::
operator=(const R2Pixel& pixel)
{
  // Copy components
  c[0] = pixel.c[0];
  c[1] = pixel.c[1];
  c[2] = pixel.c[2];
  c[3] = pixel.c[3];
  return *this;
}



inline R2Pixel& R2Pixel::
operator+=(const R2Pixel& pixel)
{
  c[0] += pixel.c[0];
  c[1] += pixel.c[1];
  c[2] += pixel.c[2];
  return *this;
}



inline R2Pixel& R2Pixel::
operator-=(const R2Pixel& pixel)
{
  c[0] -= pixel.c[0];
  c[1] -= pixel.c[1];
  c[2] -= pixel.c[2];
  return *this;
}



inline R2Pixel& R2Pixel::
operator*=(const R2Pixel& pixel)
{
  c[0] *= pixel.c[0];
  c[1] *= pixel.c[1];
  c[2] *= pixel.c[2];
  return *this;
}



inline R2Pixel& R2Pixel::
operator*=(RNScalar a)
{
  c[0] *= a;
  c[1] *= a;
  c[2] *= a;
  return *this;
}



inline R2Pixel& R2Pixel::
operator/=(RNScalar a)
{
  //  assert(!zero(a));
  c[0] /= a;
  c[1] /= a;
  c[2] /= a;
  return *this;
}


inline void R2Pixel::
Gamma(RNScalar exponent)
{
    c[0] = pow(c[0], exponent);
    c[1] = pow(c[1], exponent);
    c[2] = pow(c[2], exponent);
    Clamp();
}

typedef R2Pixel RNRgb;

#endif
