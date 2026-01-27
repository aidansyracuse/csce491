# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Part of the University of South Carolina CSCE491 course materials. Used by
# instructors for test case generators. Do not redistribute.

import sys
import os

###############################################################################

# This block is setup code that loads the utility library for this assignment.
# You shouldn't mess with it unless you know what you are doing.

# this is the directory where our code is (main.py)
code_dir = os.path.split(os.path.abspath(sys.argv[0]))[0]

# this will be ./.. - the project directory
parent_dir = os.path.split(code_dir)[0]

# the python utils live in ../utils/python_utils
python_utils_dir = os.path.join(parent_dir, "utils", "python_utils")

# append this to Python's import path and import it
sys.path.append(python_utils_dir)
from waves import Waves

###############################################################################

# Read in all the data from standard input and parse it.
w = Waves()
w.loadText(sys.stdin.read())

# Display some information about it. We can use sys.stderr.write() to send a
# log message to standard error without it interfering with the data we
# send to standard out for grade.sh to interpret.
sys.stderr.write("read a waves file with {} signals and {} samples\n".format(len(w.signals()), w.samples()))
sys.stderr.write("input has these signals:\n")
for s in w.signals():
    sys.stderr.write("\t* {} ({} bits)\n".format(s, w.sizes[s]))

