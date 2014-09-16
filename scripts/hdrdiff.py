#!/usr/bin/env python
import subprocess,sys,tempfile

if len(sys.argv) < 3:
    print "Usage: hdrdiff.py im1.hdr im2.hdr"
else:
    tf = tempfile.NamedTemporaryFile(suffix='.hdr')
    subprocess.call(['composite', sys.argv[1], sys.argv[2], '-compose', 'minus_src', tf.name])
    subprocess.call(['pfsv', tf.name])
    tf.close()
