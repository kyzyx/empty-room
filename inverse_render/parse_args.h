#ifndef _PARSE_ARGS_H
#define _PARSE_ARGS_H
#include <string>

extern bool all_cameras;
extern bool all_project;
extern bool project_debug;
extern bool show_frustrum;
extern bool wallinput;
extern bool read_eq, write_eq;
extern int project;
extern int camera;
extern std::string outfile, infile, camfile, walloutfile, wallfile, sampleoutfile, samplefile, radfile;
extern int wallthreshold;
extern int numsamples;
extern int hemicuberesolution;
extern double discardthreshold;
extern bool output_reprojection;
extern bool output_wall;
extern bool input;
extern bool display;
extern bool prune;
extern bool ccw;
extern double anglethreshold;
extern double resolution;
extern double minlength;
extern bool do_wallfinding;
extern bool do_reprojection;
extern bool do_sampling;
extern double hdr_threshold;
extern double displayscale;

bool parseargs(int argc, char** argv);
#endif
