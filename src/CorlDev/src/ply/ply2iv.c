/*

Convert a PLY file to an Inventor file.

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


/* vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z;
  float r,g,b;
  float nx,ny,nz;
  void *other_props;       /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
} Face;

char *elem_names[] = { /* list of the elements in the object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
  {"r", Float32, Float32, offsetof(Vertex,r), 0, 0, 0, 0},
  {"g", Float32, Float32, offsetof(Vertex,g), 0, 0, 0, 0},
  {"b", Float32, Float32, offsetof(Vertex,b), 0, 0, 0, 0},
  {"nx", Float32, Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
  {"ny", Float32, Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
  {"nz", Float32, Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


/*** the PLY object ***/

static int nverts,nfaces;
static Vertex **vlist;
static Face **flist;

static PlyOtherProp *vert_other,*face_other;

static int per_vertex_color = 0;
static int has_normals = 0;


/******************************************************************************
Main program.
******************************************************************************/

main(int argc, char *argv[])
{
  int i,j;
  char *s;
  char *progname;

  progname = argv[0];

  while (--argc > 0 && (*++argv)[0]=='-') {
    for (s = argv[0]+1; *s; s++)
      switch (*s) {
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  read_file();
  write_inventor();
}


/******************************************************************************
Print out usage information.
******************************************************************************/

usage(char *progname)
{
  fprintf (stderr, "usage: %s [flags] <in.ply >out.iv\n", progname);
}


/******************************************************************************
Read in the PLY file from standard in.
******************************************************************************/

read_file()
{
  int i,j;
  int elem_count;
  char *elem_name;
  PlyFile *in_ply;

  /*** Read in the original PLY object ***/

  in_ply  = read_ply (stdin);

  for (i = 0; i < in_ply->num_elem_types; i++) {

    /* prepare to read the i'th list of elements */
    elem_name = setup_element_read_ply (in_ply, i, &elem_count);

    if (equal_strings ("vertex", elem_name)) {

      /* create a vertex list to hold all the vertices */
      vlist = (Vertex **) malloc (sizeof (Vertex *) * elem_count);
      nverts = elem_count;

      /* set up for getting vertex elements */

      setup_property_ply (in_ply, &vert_props[0]);
      setup_property_ply (in_ply, &vert_props[1]);
      setup_property_ply (in_ply, &vert_props[2]);

      for (j = 0; j < in_ply->elems[i]->nprops; j++) {
	PlyProperty *prop;
	prop = in_ply->elems[i]->props[j];
	if (equal_strings ("r", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[3]);
	  per_vertex_color = 1;
	}
	if (equal_strings ("g", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[4]);
	  per_vertex_color = 1;
	}
	if (equal_strings ("b", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[5]);
	  per_vertex_color = 1;
	}
	if (equal_strings ("nx", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[6]);
	  has_normals = 1;
	}
	if (equal_strings ("ny", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[7]);
	  has_normals = 1;
	}
	if (equal_strings ("nz", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[8]);
	  has_normals = 1;
	}
      }

      vert_other = get_other_properties_ply (in_ply, 
					     offsetof(Vertex,other_props));

      /* grab all the vertex elements */
      for (j = 0; j < elem_count; j++) {
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
	vlist[j]->r = 1;
	vlist[j]->g = 1;
	vlist[j]->b = 1;
        get_element_ply (in_ply, (void *) vlist[j]);
      }
    }
    else if (equal_strings ("face", elem_name)) {

      /* create a list to hold all the face elements */
      flist = (Face **) malloc (sizeof (Face *) * elem_count);
      nfaces = elem_count;

      /* set up for getting face elements */

      setup_property_ply (in_ply, &face_props[0]);
      face_other = get_other_properties_ply (in_ply, 
					     offsetof(Face,other_props));

      /* grab all the face elements */
      for (j = 0; j < elem_count; j++) {
        flist[j] = (Face *) malloc (sizeof (Face));
        get_element_ply (in_ply, (void *) flist[j]);
      }
    }
    else
      get_other_element_ply (in_ply);
  }

  close_ply (in_ply);
  free_ply (in_ply);
}


/******************************************************************************
Write out an Inventor file.
******************************************************************************/

write_inventor()
{
  int i,j,k;

  printf ("#Inventor V2.1 ascii\n");
  printf ("\n");

  printf ("Separator {\n");
  printf ("\n");

  /* write out the coordinates */

  printf ("Coordinate3 {\n");
  printf ("  point [\n");

  for (i = 0; i < nverts; i++)
    printf ("    %g %g %g,\n", vlist[i]->x, vlist[i]->y, vlist[i]->z);

  printf ("  ]\n");
  printf ("}\n");
  printf ("\n");

  /* if we have them, write surface normals */

  if (has_normals) {
    printf ("Normal {\n");
    printf ("  vector [\n");

    for (i = 0; i < nverts; i++)
      printf ("    %g %g %g,\n", vlist[i]->nx, vlist[i]->ny, vlist[i]->nz);

    printf ("  ]\n");
    printf ("}\n");
    printf ("\n");
  }

  /* write out the vertex colors */

  printf ("Material {\n");
  printf ("  diffuseColor [\n");
  if (per_vertex_color) {
    for (i = 0; i < nverts; i++)
      printf ("    %g %g %g,\n", vlist[i]->r, vlist[i]->g, vlist[i]->b);
  }
  else
    printf ("    1 1 1\n");
  printf ("  ]\n");
  printf ("}\n");
  printf ("\n");

  printf ("\n");
  printf ("MaterialBinding { value PER_VERTEX_INDEXED }\n");
  printf ("\n");

  /* write the faces */

  printf ("IndexedFaceSet {\n");
  printf ("  coordIndex [\n");

  for (i = 0; i < nfaces; i++) {
    printf ("   ");
    for (j = flist[i]->nverts - 1; j >= 0; j--)
      printf (" %d,", flist[i]->verts[j]);
    printf (" -1,\n");
  }

  printf ("  ]\n");

  printf ("  materialIndex [\n");

  for (i = 0; i < nfaces; i++) {
    printf ("   ");
    for (j = flist[i]->nverts - 1; j >= 0; j--)
      if (per_vertex_color) {
	printf (" %d,", flist[i]->verts[j]);
      }
      else
	printf (" 0,");
    printf (" -1,\n");
  }

  printf ("  ]\n");
  printf ("}\n");
  printf ("\n");

  /* end separator */

  printf ("}\n");
  printf ("\n");
}

