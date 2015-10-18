#!/usr/bin/env python2

import subprocess
import sys

if len(sys.argv) != 2:
    print("Usage: send_real_data.py cram_log.owl")
else:
    subprocess.call(["rosrun", "owl_memory_converter", "converter/build/install/converter/bin/converter", sys.argv[1]])