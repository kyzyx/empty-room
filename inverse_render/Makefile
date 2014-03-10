CC=g++
CPPFLAGS=-Wall -I. -O3
LDFLAGS=

PKG_LIBS= \
  R3Graphics/libR3Graphics.a \
  R3Shapes/libR3Shapes.a \
  R2Shapes/libR2Shapes.a \
  RNBasics/libRNBasics.a \
  jpeg/libjpeg.a 
  #fglut/libfglut.a \

all: $(PKG_LIBS)

#
# OpenGL Libraries
#
OPENGL_LIBS=-lGLU -lGL


R3Graphics/libR3Graphics.a: 
	    cd R3Graphics; make

R3Shapes/libR3Shapes.a: 
	    cd R3Shapes; make

R2Shapes/libR2Shapes.a: 
	    cd R2Shapes; make

RNBasics/libRNBasics.a: 
	    cd RNBasics; make

fglut/libfglut.a: 
	    cd fglut; make

jpeg/libjpeg.a: 
	    cd jpeg; make
