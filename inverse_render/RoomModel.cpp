#include "roommodel.h"
#include "rapidjson/rapidjson.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h" // for stringify JSON
#include "rapidjson/filestream.h"   // wrapper of C stream for prettywriter as output
#include "imageio.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace rapidjson;
using namespace ImageIO;


namespace roommodel {
#define REQ_OBJECT(d,s) if (!d.HasMember(s) || !d[s].IsObject()) return false
#define REQ_ARR(d,s) if (!d.HasMember(s) || !d[s].IsArray()) return false
#define REQ_NUM(d,s) if (!d.HasMember(s) || !d[s].IsNumber()) return false
#define REQ_SET_NUM(d,s,n) if (!d.HasMember(s) || !d[s].IsNumber()) return false; else n = d[s].GetDouble()
#define REQ_SET_INT(d,s,n) if (!d.HasMember(s) || !d[s].IsNumber()) return false; else n = d[s].GetInt()
#define REQ_STRING(d,s) if (!d.HasMember(s) || !d[s].IsString()) return false
#define REQ_SET_STRING(d,s,n) if (!d.HasMember(s) || !d[s].IsString()) return false; else n = d[s].GetString()

#define OPT_OBJECT(d,s) if (d.HasMember(s) && d[s].IsObject())
#define OPT_ARR(d,s) if (d.HasMember(s) && d[s].IsArray())
#define OPT_NUM(d,s) if (d.HasMember(s) && d[s].IsNumber())
#define OPT_SET_NUM(d,s,n) if (d.HasMember(s) && d[s].IsNumber()) n = d[s].GetDouble()
#define OPT_SET_INT(d,s,n) if (d.HasMember(s) && d[s].IsNumber()) n = d[s].GetInt()
#define OPT_STRING(d,s) if (d.HasMember(s) && d[s].IsString())
#define OPT_SET_STRING(d,s,n) if (d.HasMember(s) && d[s].IsString()) n = d[s].GetString()

bool parseColor(const Value& v, Color& c) {
	REQ_SET_NUM(v, "r", c.r);
	REQ_SET_NUM(v, "g", c.g);
	REQ_SET_NUM(v, "b", c.b);
	return true;
}
bool parseTexture(const Value& v, Texture& tex) {
	REQ_SET_NUM(v, "scale", tex.scale);
	REQ_SET_STRING(v, "texturefilename", tex.filename);
	readExrImage(tex.filename, &tex.texture, tex.width, tex.height);
	return true;
}
bool parseMaterial(const Value& v, Material& mat) {
	OPT_OBJECT(v, "diffuse") if (!parseColor(v["diffuse"], mat.diffuse)) return false;
	OPT_OBJECT(v, "texture") {
		mat.texture = new Texture();
		if (!parseTexture(v["texture"], *(mat.texture))) {
			delete mat.texture;
			mat.texture = NULL;
			return false;
		}
	}
	return true;
}
bool parseRWO(const Value& v, RectangleWallObject& rwo) {
	REQ_SET_NUM(v, "width", rwo.width);
	REQ_SET_NUM(v, "height", rwo.height);
	REQ_SET_NUM(v, "horizontalposition", rwo.horizontalposition);
	REQ_SET_NUM(v, "verticalposition", rwo.verticalposition);
	OPT_SET_NUM(v, "recessed", rwo.recessed);
	OPT_SET_NUM(v, "trimWidth", rwo.trimWidth);
	OPT_SET_NUM(v, "trimDepth", rwo.trimDepth);
	OPT_OBJECT(v, "frameMaterial") if (!parseMaterial(v["frameMaterial"], rwo.frameMaterial)) return false;
	OPT_OBJECT(v, "material") if (!parseMaterial(v["material"], rwo.material)) return false;
	return true;
}

bool parseWall(const Value& v, Wall& wall) {
	REQ_SET_NUM(v, "length", wall.length);
	REQ_SET_NUM(v, "normal", wall.normal);
	if (wall.normal > 0) wall.normal = 1;
	else wall.normal = -1;

	OPT_ARR(v, "windows") {
		for (size_t i = 0; i < v["windows"].Size(); ++i) {
			RectangleWallObject newrwo;
			if (!parseRWO(v["windows"][i], newrwo)) return false;
			newrwo.wall = &wall;
			wall.windows.push_back(newrwo);
		}
	}
	return true;
}

bool parseLight(const Value& v, Light*& l, vector<Wall>& walls) {
	string s;
	REQ_SET_STRING(v, "type", s);
	if (s == "window") {
		RoomWindow* rw = new RoomWindow();
		double w,i;
		REQ_SET_INT(v, "wall", w);
		REQ_SET_INT(v, "windowindex", i);
		rw->rwo = &(walls[w].windows[i]);
		OPT_OBJECT(v, "directionalintensity") if (!parseColor(v["directionalintensity"], rw->directionalintensity)) return false;
		rw->texture = NULL;
		// How to reference windows?
		OPT_OBJECT(v, "texture") {
			rw->texture = new Texture();
			if (!parseTexture(v["texture"], *(rw->texture))) {
				delete rw->texture;
				rw->texture = NULL;
				return false;
			}
		}
		l = rw;
	}
	else if (s == "line") {
		LineLight* ll = new LineLight();
		double x, y, z;
		REQ_SET_NUM(v, "ex", x);
		REQ_SET_NUM(v, "ey", y);
		REQ_SET_NUM(v, "ez", z);
		ll->endpoint = FVector(x, y, z);
		l = ll;
	}
	else {
		l = new Light();
	}

	REQ_OBJECT(v, "intensity");
	if (!parseColor(v["intensity"], l->intensity)) return false;
	double x, y, z;
	REQ_SET_NUM(v, "px", x);
	REQ_SET_NUM(v, "py", y);
	REQ_SET_NUM(v, "pz", z);
	l->position = FVector(x, y, z);
	x = y = z = 0;
	OPT_SET_NUM(v, "dx", x);
	OPT_SET_NUM(v, "dy", y);
	OPT_SET_NUM(v, "dz", z);
	l->direction = FVector(x, y, z);
	OPT_SET_NUM(v, "dropoff", l->dropoff);
	OPT_SET_NUM(v, "cutoff", l->cutoff);
	return true;
}

#define SERIALIZE_STRING(w,s,v) w.String(s); w.String(v)
#define SERIALIZE_DOUBLE(w,s,v) w.String(s); w.Double(v)
#define SERIALIZE_INT(w,s,v) w.String(s); w.Int(v)
template <typename Writer>
void serializeColor(const Color& c, Writer& writer) {
	writer.StartObject();
	SERIALIZE_DOUBLE(writer, "r", c.r);
	SERIALIZE_DOUBLE(writer, "g", c.g);
	SERIALIZE_DOUBLE(writer, "b", c.b);
	writer.EndObject();
}
template <typename Writer>
void serializeTexture(const Texture& tex, Writer& writer) {
	writer.StartObject();
	SERIALIZE_DOUBLE(writer, "scale", tex.scale);
	SERIALIZE_STRING(writer, "texturefilename", tex.filename.c_str());
	writer.EndObject();
}
template <typename Writer>
void serializeMaterial(const Material& mat, Writer& writer) {
	writer.StartObject();
	if (mat.texture) {
		writer.String("texture");
		serializeTexture(*(mat.texture), writer);
	}
	else {
		writer.String("diffuse");
		serializeColor(mat.diffuse, writer);
	}
	writer.EndObject();
}
template <typename Writer>
void serializeRWO(const RectangleWallObject& rwo, Writer& writer) {
	writer.StartObject();
	SERIALIZE_DOUBLE(writer, "width", rwo.width);
	SERIALIZE_DOUBLE(writer, "height", rwo.height);
	SERIALIZE_DOUBLE(writer, "horizontalposition", rwo.horizontalposition);
	SERIALIZE_DOUBLE(writer, "verticalposition", rwo.verticalposition);
	SERIALIZE_DOUBLE(writer, "recessed", rwo.recessed);
	SERIALIZE_DOUBLE(writer, "trimWidth", rwo.trimWidth);
	SERIALIZE_DOUBLE(writer, "trimDepth", rwo.trimDepth);
	writer.String("frameMaterial");
	serializeMaterial(rwo.frameMaterial, writer);
	writer.String("material");
	serializeMaterial(rwo.material, writer);
	writer.EndObject();
}
template <typename Writer>
void serializeWall(const Wall& wall, Writer& writer) {
	writer.StartObject();
	SERIALIZE_DOUBLE(writer, "length", wall.length);
	SERIALIZE_INT(writer, "normal", wall.normal);
	writer.String("windows");
	writer.StartArray();
	for (int i = 0; i < wall.windows.size(); ++i) {
		serializeRWO(wall.windows[i], writer);
	}
	writer.EndArray();
	writer.EndObject();
}

template <typename Writer>
void serializeLight(Light*& l, vector<Wall>& walls, Writer& writer) {
	writer.StartObject();
	SERIALIZE_STRING(writer, "type", l->getType().c_str());
	if (l->getType() == "window") {
		RoomWindow* rw = (RoomWindow*) l;
		for (int w = 0; w < walls.size(); ++w) {
			for (int i = 0; i < walls[w].windows.size(); ++i) {
				if (rw->rwo == &(walls[w].windows[i])) {
					SERIALIZE_INT(writer, "wall", w);
					SERIALIZE_INT(writer, "windowindex", i);
				}
			}
		}
		writer.String("directionalintensity");
		serializeColor(rw->directionalintensity, writer);
		if (rw->texture) {
			writer.String("texture");
			serializeTexture(*(rw->texture), writer);
		}
	}
	else if (l->getType() == "line") {
		LineLight* ll = new LineLight();
		SERIALIZE_DOUBLE(writer, "ex", ll->endpoint.X());
		SERIALIZE_DOUBLE(writer, "ey", ll->endpoint.Y());
		SERIALIZE_DOUBLE(writer, "ez", ll->endpoint.Z());
	}

	writer.String("intensity");
	serializeColor(l->intensity, writer);
	SERIALIZE_DOUBLE(writer, "px", l->position.X());
	SERIALIZE_DOUBLE(writer, "py", l->position.Y());
	SERIALIZE_DOUBLE(writer, "pz", l->position.Z());
	SERIALIZE_DOUBLE(writer, "dx", l->direction.X());
	SERIALIZE_DOUBLE(writer, "dy", l->direction.Y());
	SERIALIZE_DOUBLE(writer, "dz", l->direction.Z());
	SERIALIZE_DOUBLE(writer, "dropoff", l->dropoff);
	SERIALIZE_DOUBLE(writer, "cutoff", l->cutoff);

	writer.EndObject();
}

#include <cstdio>

bool load(RoomModel& r, const string& filename) {
	Document d;
	FILE* fin = fopen(filename.c_str(), "r");
	FileStream in(fin);
	d.ParseStream(in);

	// Parse walls
	REQ_ARR(d, "walls");
	for (size_t i = 0; i < d["walls"].Size(); ++i) {
		Wall newwall;
		if (!parseWall(d["walls"][i], newwall)) return false;
		r.walls.push_back(newwall);
	}
	// Parse height
	REQ_SET_NUM(d, "height", r.height);

	// Parse floor plan reflectances
	REQ_OBJECT(d, "wallMaterial");
	if (!parseMaterial(d["wallMaterial"], r.wallMaterial)) return false;
	REQ_OBJECT(d, "floorMaterial");
	if (!parseMaterial(d["floorMaterial"], r.floorMaterial)) return false;
	REQ_OBJECT(d, "ceilingMaterial");
	if (!parseMaterial(d["ceilingMaterial"], r.ceilingMaterial)) return false;

	OPT_SET_NUM(d, "baseboardHeight", r.baseboardHeight);
	OPT_SET_NUM(d, "baseboardDepth", r.baseboardDepth);
	OPT_OBJECT(d, "baseboardMaterial") if (!parseMaterial(d["baseboardMaterial"], r.baseboardMaterial)) return false;

	// Parse lights
	REQ_ARR(d, "lights");
	for (size_t i = 0; i < d["lights"].Size(); ++i) {
		Light* l;
		if (!parseLight(d["lights"][i], l, r.walls)) return false;
		r.lights.push_back(l);
	}
	fclose(fin);
	return true;
}

bool save(RoomModel& r, const string& filename) {
	FILE* out = fopen(filename.c_str(), "w");
	FileStream stream(out);
	PrettyWriter<FileStream> writer(stream);
	writer.StartObject();
	writer.String("walls");
	writer.StartArray();
	for (int i = 0; i < r.walls.size(); ++i) {
		serializeWall(r.walls[i], writer);
	}
	writer.EndArray();
	SERIALIZE_DOUBLE(writer, "height", r.height);
	SERIALIZE_DOUBLE(writer, "baseboardHeight", r.baseboardHeight);
	SERIALIZE_DOUBLE(writer, "baseboardDepth", r.baseboardDepth);
	writer.String("wallMaterial");
	serializeMaterial(r.wallMaterial, writer);
	writer.String("floorMaterial");
	serializeMaterial(r.floorMaterial, writer);
	writer.String("ceilingMaterial");
	serializeMaterial(r.ceilingMaterial, writer);
	writer.String("baseboardMaterial");
	serializeMaterial(r.baseboardMaterial, writer);
	writer.String("lights");
	writer.StartArray();
	for (int i = 0; i < r.lights.size(); ++i) {
		serializeLight(r.lights[i], r.walls, writer);
	}
	writer.EndArray();
	writer.EndObject();
	fclose(out);
	return true;
}

RoomModel::RoomModel(const string& filename)
{
	load(*this, filename);
}
}
