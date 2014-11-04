#ifndef _PARSE_ARGS_H
#define _PARSE_ARGS_H
#include <string>

extern bool all_cameras;
extern bool all_project;
extern bool project_debug;
extern bool show_frustrum;
extern bool use_confidence_files;
extern bool wallinput;
extern bool read_eq, write_eq, write_matlab;
extern int project;
extern int camera;
extern std::string outfile, infile, camfile, walloutfile, wallfile, sampleoutfile, samplefile, radfile, matlabsamplefile;
extern int wallthreshold;
extern int numsamples;
extern int hemicuberesolution;
extern int minlightsize;
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
extern bool do_texture;
extern double hdr_threshold;
extern double displayscale;
extern bool image_flip_x, image_flip_y;

bool parseargs(int argc, char** argv);
#endif
