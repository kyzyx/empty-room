// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <vector>
#include <string>

namespace roommodel {

// Basic reflectance structures
class Color {
public:
	Color() : r(0), g(0), b(0) { ; }
	Color(double red, double green, double blue)
		: r(red), g(green), b(blue)
	{
		;
	}
	double r, g, b;
};
class Texture {
public:
	float* texture;
	double scale;
	int width, height;
	std::string filename;
};
class Material {
public:
	Material() {
		diffuse.r = 0;
		diffuse.g = 0;
		diffuse.b = 0;
		texture = NULL;
	}
	Material(double r, double g, double b) {
		diffuse.r = r;
		diffuse.g = g;
		diffuse.b = b;
		texture = NULL;
	}
	Color diffuse;    // Or more complex reflectance model
	Texture* texture;
};

// ----------------------------------
// Architectural features definitions
// ----------------------------------
class RectangleWallObject;

class Wall {
public:
	// Axis aligned walls
	double length;
	int normal;    // Normal towards positive or negative axis
	std::vector<RectangleWallObject> windows;
};
class RectangleWallObject {
public:
	// Axis aligned rectangular architectural features on walls,
	// most importantly doors and windows.
	Wall* wall;

	// Position
	double width;
	double height;
	double horizontalposition; // offset from Wall->start of the start of RWO
	double verticalposition;

	// Geometry detail
	double recessed;      // How far recessed the doorframe is (e.g. closets)
	double trimWidth;    // Frame dimensions
	double trimDepth;

	/* More specific things to add:
	* - Doorknobs
	*      Knob position (2 vars), knob material, knob shape
	* - Multi-RWO structures, e.g. double doors, adjacent windows with frame
	* - Detail geometry, i.e. bump map for doors
	* - Nested RWO, e.g. door with a window inside
	*/
	Material frameMaterial;
	Material material;
};

// -------------------
// Emitter definitions
// -------------------
class Light {
public:
	Light() {
		position = FVector(0, 0, 0);
		direction = FVector(0, 0, 0);
		dropoff = 0;
		cutoff = 360;
	}
	Light(FVector pos, Color i) {
		position = pos;
		direction = FVector(0, 0, 0);
		intensity = i;
		dropoff = 0;
		cutoff = 360;
	}
	Color intensity;
	FVector position;
	FVector direction;
	double dropoff;
	double cutoff;

	virtual std::string getType() const { return "point";  }
};

class LineLight : public Light {
public:
	FVector endpoint;
	virtual std::string getType() const { return "line";  }
};
class RoomWindow : public Light {
public:
	RectangleWallObject* rwo;
	Texture* texture;
	Color directionalintensity;
	virtual std::string getType() const { return "window"; }
};

// -----------------
// Master room model
// -----------------
class ROOMVISUALIZER_API RoomModel
{
public:
	// -------------------------------------------
	// Floor plan
	//    Assumes axis-aligned walls with floor at y=0
	// Floor plan geometry
	std::vector<Wall> walls;
	UPROPERTY(VisibleAnywhere, BlueprintAssignable, Category = "RoomGeometry")
	float height;
	// Floor plan reflectance
	Material wallMaterial;
	Material floorMaterial;
	Material ceilingMaterial;

    // WallFinder Helpers
    FMatrix globaltransform;
    double originalFloor;

	// -------------------------------------------
	// Baseboard
	//    Assumes blockish baseboard
	//    Could be supplemented by a 1D heightmap
	UPROPERTY(VisibleAnywhere, BlueprintAssignable, Category = "RoomGeometry")
	float baseboardHeight;
	UPROPERTY(VisibleAnywhere, BlueprintAssignable, Category = "RoomGeometry")
	float baseboardDepth;
	Material baseboardMaterial;
	// -------------------------------------------
	// Lighting
	std::vector<Light*> lights;
	RoomModel(double l, double w, double h) {
        globaltransform = IdentityMatrix;
        originalFloor = 0;
		Wall wall;
		wall.length = l;
		wall.normal = -1;
		walls.push_back(wall);
		wall.length = w;
		wall.normal = -1;
		walls.push_back(wall);
		wall.length = l;
		wall.normal = 1;
		walls.push_back(wall);
		wall.length = w;
		wall.normal = 1;
		walls.push_back(wall);
		height = h;
	}
	RoomModel() {
        globaltransform = IdentityMatrix;
        originalFloor = 0;
    }

	RoomModel(const std::string& filename);
};

bool load(RoomModel& r, const std::string& filename);
bool save(RoomModel& r, const std::string& filename);
}
