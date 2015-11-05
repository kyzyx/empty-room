#!/usr/bin/env python
import sys, subprocess, tempfile, shutil, multiprocessing

convert_binary = '/home/zyxlyr/Code/empty-room/inverse_render/build/convertimage'

def run_cmd(cmd):
    print ' '.join(cmd)
    subprocess.call(cmd)

if len(sys.argv) < 3:
    print "Usage: batchconvert camfile.cam format [convert_args]"
else:
    n = 0
    pool = multiprocessing.Pool(None)
    cmds = []
    fmt = sys.argv[2]

    for line in open(sys.argv[1]).readlines()[1:]:
        toks = line.split()
        outfile = toks[0][0:toks[0].rfind('.')] + "." + fmt
        cmd = [convert_binary, toks[0], outfile] + sys.argv[3:]
        cmds.append(cmd)
        n += 1

    pool.map(run_cmd, cmds)
