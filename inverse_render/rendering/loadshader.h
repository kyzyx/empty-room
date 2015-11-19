#ifndef _LOADSHADER_H
#define _LOADSHADER_H
#include "opengl_compat.h"
#include <string>
#include <vector>

class ShaderProgram {
    public:
        ShaderProgram() {;}
        GLuint getProgId() const { return program; }

        virtual void init() { compile(); prelink(); link(); cleanup(); }
        virtual void compile() = 0;
        virtual void prelink();
        virtual void link();
        virtual void cleanup();
    protected:

        void compileFromSource(const char* vertShaderSrc, const char* fragShaderSrc);
        GLuint program;
        GLuint fragShader, vertShader;
};

class FileShaderProgram : public ShaderProgram {
    public:
        FileShaderProgram(const std::string& vert_path, const std::string& frag_path)
            : vfile(vert_path), ffile(frag_path) {;}

        virtual void compile() override;

    protected:
        std::string vfile, ffile;
};

class VertexFileShaderProgram : public FileShaderProgram {
    public:
        VertexFileShaderProgram(const std::string& vert_path)
            : FileShaderProgram(vert_path, DEFAULTFRAGFILE) {;}
    private:
        static const char* DEFAULTFRAGFILE;
};

class SourceShaderProgram : public ShaderProgram {
    public:
        SourceShaderProgram(const char* vert_src, const char* frag_src)
            : vsrc(vert_src), fsrc(frag_src) {;}

        virtual void compile() override { compileFromSource(vsrc, fsrc); }

    protected:
        const char* vsrc;
        const char* fsrc;
};

class ShaderProgramDecorator : public ShaderProgram {
    public:
        ShaderProgramDecorator(ShaderProgram* component) : prog(component) {;}

        virtual void compile() override { prog->compile(); docompile(); }
        virtual void prelink() override {
            prog->prelink();
            program = prog->getProgId();
            doprelink();
        }
        virtual void cleanup() override { prog->cleanup(); docleanup(); }

    protected:
        virtual void docompile() {;}
        virtual void doprelink() {;}
        virtual void docleanup() {;}
    private:
        ShaderProgram* prog;
};

class GeometryShader : public ShaderProgramDecorator {
    public:
        GeometryShader(ShaderProgram* component, const std::string& geom_path)
            : ShaderProgramDecorator(component), gfile(geom_path) {;}
    protected:
        virtual void docompile() override;
        virtual void doprelink() override;
        virtual void docleanup() override;
    private:
        GLuint geomShader;
        std::string gfile;
};

class TransformFeedbackShader : public ShaderProgramDecorator {
    public:
        TransformFeedbackShader(ShaderProgram* component, std::vector<std::string> names);
        ~TransformFeedbackShader();
    protected:
        virtual void doprelink() override;

        GLchar** varyings;
        int numvaryings;
};

#endif // _LOADSHADER_H
