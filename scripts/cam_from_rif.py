#!/usr/bin/env python
import sys

if len(sys.argv) != 3:
    print "Usage: cam_from_rif.py radfile.rif output.cam"
else:
    views = [l[6:] for l in open(sys.argv[1], 'r').readlines() if l[0:5] == "view="]
    out = open(sys.argv[2],'w')
    filepattern = sys.argv[1][:-4] + "_%s.hdr"

    f = open(filepattern%views[0].split(' ',1)[0], 'r')
    l = f.readline()
    while '+X' not in l:
        l = f.readline()
    width = int(l.split()[3])
    height = int(l.split()[1])
    f.close()

    hfov = float(views[0][views[0].find("-vh "):].split(' ',2)[1])
    out.write("%d %d %d %f\n"%(len(views),width,height,hfov/2))
    for v in views:
        out.write(filepattern%v.split(' ',1)[0])
        out.write(" ")
        pos = ' '.join(v[v.find("-vp "):].split()[1:4])
        up = ' '.join(v[v.find("-vu "):].split()[1:4])
        towards = ' '.join(v[v.find("-vd "):].split()[1:4])
        out.write(pos)
        out.write(" ")
        out.write(up)
        out.write(" ")
        out.write(towards)
        out.write("\n")
