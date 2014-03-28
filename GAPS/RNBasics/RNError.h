/* Include file for GAPS error utility */



/* Initialization functions */

int RNInitError();
void RNStopError();



/* Error file functions */

void RNSetErrorFile(FILE *fp);
void RNSetErrorLevel(int level);



/* Error reporting functions */

void RNAbort(const char *fmt, ...);
void RNFail(const char *fmt, ...);
void RNWarning(const char *fmt, ...);






