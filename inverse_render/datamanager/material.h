#ifndef _MATERIAL_H
#define _MATERIAL_H

class Texture {
    public:
        Texture() : texture(0), size(0), scale(1) {}
        Texture(float* tex, int texsize, float texscale) : texture(tex), size(texsize), scale(texscale) {}

        Texture& operator=(const Texture& t) {
            texture = t.texture;
            size = t.size;
            scale = t.scale;
            return *this;
        }

        bool isEmpty() const { return size > 0; }

        float* texture;
        int size;
        float scale;
};

class Material {
    public:
        Material(float red, float green, float blue, Texture tex)
            : r(red), g(green), b(blue), t(tex) {;}
        Material(float red, float green, float blue)
            : r(red), g(green), b(blue), t() {;}
        Material()
            : r(0), g(0), b(0), t() {;}
        const float& operator[](int n) const {
            if (n == 0) return r;
            else if (n == 1) return g;
            else return b;
        }
        float& operator()(int n) {
            if (n == 0) return r;
            else if (n == 1) return g;
            else return b;
        }
        bool isEmpty() {
            return r == 0 && g == 0 && b == 0;
        }
        Material operator+(const Material& m) {
            return Material(r + m.r, g + m.g, b + m.b, !t.isEmpty()?t:m.t);
        }
        Material& operator+=(const Material& m) {
            r += m.r;
            g += m.g;
            b += m.b;
            if (t.isEmpty()) t = m.t;
            return *this;
        }
        Material operator-(const Material& m) {
            return Material(r - m.r, g - m.g, b - m.b, !t.isEmpty()?t:m.t);
        }
        Material& operator-=(const Material& m) {
            r -= m.r;
            g -= m.g;
            b -= m.b;
            if (t.isEmpty()) t = m.t;
            return *this;
        }
        Material operator*(const Material& m) {
            return Material(r*m.r, g*m.g, b*m.b, !t.isEmpty()?t:m.t);
        }
        Material operator*(double f) {
            return Material(f*r, f*g, f*b, t);
        }
        Material& operator*=(double f) {
            r *= f;
            g *= f;
            b *= f;
            return *this;
        }
        Material operator/(const Material& m) {
            return Material(r/m.r, g/m.g, b/m.b, !t.isEmpty()?t:m.t);
        }
        Material operator/(double f) {
            return Material(r/f, g/f, b/f, t);
        }
        Material& operator/=(double f) {
            r /= f;
            g /= f;
            b /= f;
            return *this;
        }
        float r;
        float g;
        float b;
        Texture t;
};
#endif
