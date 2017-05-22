Introduction
------------

This package contains a collection of geometry filters-- programs that
are for manipulating polgonal representations for 3D graphical objects.

This source code is undergoing revision, and I expect that a new
version of the code, together with many more filters, will be
available shortly.

Support Requirements
--------------------

These geometry filters have been developed on a Silicon Graphics
workstation using the native C compiler.  The code may very well run
unmodified on other platforms but this has not yet been verified.

You will need some Inventor or VRML viewer to look at the models that
are created using these filters.  The programs "ivview" and "SceneViewer"
are such programs that are shipped with SGI workstations.

Compiling
---------

The command "make all" should compile and link all of the geometry filters.

Documentation
-------------

Manual pages have not yet been prepared for the filters.  However, most
programs will print out a simple usage line when invoked with the
option "--", such as "platoply --".

Below is the documentation as it exists now.  The most important file
to test out the code is EXAMPLES.txt.

ALL_FILTERS.txt -  one-line descriptions of all the geometry filters
PLY_FILES.txt   -  details about the PLY geometry file format
README.txt      -  the file you are reading
cube.ply        -  a cube in the PLY file format
flipply.c       -  heavily commented read_file() and write_file() routines
                   to illustrate the support routines for file I/O


(c) 1998 Greg Turk

