/*

Create a sphere.

Greg Turk

-----------------------------------------------------------------------

Copyright (c) 1998 Georgia Institute of Technology.  All rights reserved.   
  
Permission to use, copy, modify and distribute this software and its   
documentation for any purpose is hereby granted without fee, provided   
that the above copyright notice and this permission notice appear in   
all copies of this software and that you do not sell the software.   
  
THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,   
EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY   
WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.   

*/

#include <stdio.h>
#include <math.h>
#include <strings.h>
#include <ply.h>


/* user's vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z;            /* position */
  float nx,ny,nz;         /* surface normal */
  float s,t;              /* texture coordinates */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
} Face;

char *elem_names[] = { /* list of the kinds of elements in the user's object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
  {"nx", Float32, Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
  {"ny", Float32, Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
  {"nz", Float32, Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
  {"s", Float32, Float32, offsetof(Vertex,s), 0, 0, 0, 0},
  {"t", Float32, Float32, offsetof(Vertex,t), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


/*** the PLY object ***/

static int nverts,nfaces;
static Vertex *vlist;
static Face *flist;
static int nelems = 2;
static int texture_coords = 0;

void make_sphere(int, int, float, int);


/******************************************************************************
Main program.
******************************************************************************/

main(int argc, char *argv[])
{
  int i,j;
  char *s;
  char *progname;
  int naround = 20;
  int ndown = 20;
  float radius = 1;
  int triangle_flag = 0;

  progname = argv[0];

  while (--argc > 0 && (*++argv)[0]=='-') {
    for (s = argv[0]+1; *s; s++)
      switch (*s) {
        case 'n':
          naround = atoi (*++argv);
          ndown = atoi (*++argv);
          argc -= 2;
          break;
        case 'r':
          radius = atof (*++argv);
          argc -= 1;
          break;
        case 'c':
          texture_coords = 1 - texture_coords;
          break;
        case 't':
          triangle_flag = 1 - triangle_flag;
          break;
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  make_sphere (naround, ndown, radius, triangle_flag);
  write_file();
}


/******************************************************************************
Print out usage information.
******************************************************************************/

usage(char *progname)
{
  fprintf (stderr, "usage: %s [flags] >out.ply\n", progname);
  fprintf (stderr, "         -n num_around num_down { default: 20 20 }\n");
  fprintf (stderr, "         -r radius   { default is 1 }\n");
  fprintf (stderr, "         -c          { write texture coordinates }\n");
  fprintf (stderr, "         -t          { output tris (default is quads) }\n");
}


/******************************************************************************
Create a sphere.

Entry:
  naround - number of vertices around
  ndown   - number of vertices down
  radius  - radius of sphere
  tflag   - write triangles?
******************************************************************************/

void make_sphere(int naround, int ndown, float radius, int tflag)
{
  int i,j;
  int i2,j2;
  float a,b,c;
  float theta,theta2;
  float c1,s1;
  float s,t;
  float r,z;
  int index;

  naround++;
  ndown++;

  nverts = 0;
  nfaces = 0;

  /* allocate spaces for vertices and faces */

  vlist = (Vertex *) malloc (sizeof (Vertex) * naround * ndown);

  if (tflag)
    flist = (Face *) malloc (sizeof (Face) * 2 * (naround-1) * (ndown-1));
  else
    flist = (Face *) malloc (sizeof (Face) * (naround-1) * (ndown-1));

  /* make vertices and faces simultaneously */

  for (i = 0; i < naround; i++) {

    s = i / (float) (naround-1);
    theta = 2 * 3.1415926535 * s;
    c1 = cos (theta);
    s1 = sin (theta);

    for (j = 0; j < ndown; j++) {

      t = j / (float) (ndown-1);
      theta2 = 3.1415926535 * (t - 0.5);
      r = cos (theta2);
      z = sin (theta2);

      /* vertex position and normal */

      vlist[nverts].x = r * c1 * radius;
      vlist[nverts].y = r * s1 * radius;
      vlist[nverts].z = z * radius;
      vlist[nverts].nx = r * c1;
      vlist[nverts].ny = r * s1;
      vlist[nverts].nz = z;

      /* texture coordinates */

      vlist[nverts].s = s;
      vlist[nverts].t = t;

      nverts++;

      /* maybe make a face or two */

      if (i < naround-1 && j < ndown-1)
        if (tflag) {
          /* make two triangles */

          flist[nfaces].nverts = 3;
          flist[nfaces].verts = (int *) malloc (sizeof (int) * 3);
          index = j + i * ndown;
          flist[nfaces].verts[0] = index;
          flist[nfaces].verts[1] = index + 1;
          flist[nfaces].verts[2] = index + ndown + 1;
          nfaces++;

          flist[nfaces].nverts = 3;
          flist[nfaces].verts = (int *) malloc (sizeof (int) * 3);
          index = j + i * ndown;
          flist[nfaces].verts[0] = index;
          flist[nfaces].verts[1] = index + ndown + 1;
          flist[nfaces].verts[2] = index + ndown;
          nfaces++;
        }
        else {
          /* make one quadrilateral */

          flist[nfaces].nverts = 4;
          flist[nfaces].verts = (int *) malloc (sizeof (int) * 4);
          index = j + i * ndown;
          flist[nfaces].verts[0] = index;
          flist[nfaces].verts[1] = index + 1;
          flist[nfaces].verts[2] = index + ndown + 1;
          flist[nfaces].verts[3] = index + ndown;
          nfaces++;
        }

    }
  }

  /* force the last row of vertices to coincide with the first row */

  for (j = 0; j < ndown; j++) {
    index = ndown * (naround - 1) + j;
    vlist[j].x = vlist[index].x;
    vlist[j].y = vlist[index].y;
    vlist[j].z = vlist[index].z;
  }
}


/******************************************************************************
Write out the PLY file to standard out.
******************************************************************************/

write_file()
{
  int i;
  PlyFile *ply;
  int num_elem_types;

  /*** Write out the transformed PLY object ***/

  ply = write_ply (stdout, nelems, elem_names, PLY_ASCII);

  /* describe what properties go into the vertex elements */

  describe_element_ply (ply, "vertex", nverts);
  describe_property_ply (ply, &vert_props[0]);
  describe_property_ply (ply, &vert_props[1]);
  describe_property_ply (ply, &vert_props[2]);
  describe_property_ply (ply, &vert_props[3]);
  describe_property_ply (ply, &vert_props[4]);
  describe_property_ply (ply, &vert_props[5]);
  if (texture_coords) {
    describe_property_ply (ply, &vert_props[6]);
    describe_property_ply (ply, &vert_props[7]);
  }

  describe_element_ply (ply, "face", nfaces);
  describe_property_ply (ply, &face_props[0]);

  append_comment_ply (ply, "created by sphereply");

  header_complete_ply (ply);

  /* set up and write the vertex elements */
  put_element_setup_ply (ply, "vertex");
  for (i = 0; i < nverts; i++)
    put_element_ply (ply, (void *) &vlist[i]);

  /* set up and write the face elements */
  put_element_setup_ply (ply, "face");
  for (i = 0; i < nfaces; i++)
    put_element_ply (ply, (void *) &flist[i]);

  close_ply (ply);
  free_ply (ply);
}

