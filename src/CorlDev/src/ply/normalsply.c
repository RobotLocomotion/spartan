/*

Compute vertex normals.

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
  float x,y,z;
  float nx,ny,nz;
  void *other_props;       /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
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
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


/*** the PLY object ***/

static PlyFile *in_ply;
static int nverts,nfaces;
static Vertex **vlist;
static Face **flist;
static PlyOtherProp *vert_other,*face_other;

static int flip_sign = 0;       /* flip the sign of the normals? */


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
        case 'f':
          flip_sign = 1;
          break;
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  read_file();
  compute_normals();
  write_file();
}


/******************************************************************************
Print out usage information.
******************************************************************************/

usage(char *progname)
{
  fprintf (stderr, "usage: %s [flags] <in.ply >out.ply\n", progname);
  fprintf (stderr, "       -f (flip sign of normals)\n");
}


/******************************************************************************
Compute normals at the vertices.
******************************************************************************/

compute_normals()
{
  int i,j;
  Face *face;
  Vertex *vert;
  int *verts;
  float x,y,z;
  float x0,y0,z0;
  float x1,y1,z1;
  float len;
  float recip;

  /* zero out all normal information at vertices */

  for (i = 0; i < nverts; i++) {
    vlist[i]->nx = 0;
    vlist[i]->ny = 0;
    vlist[i]->nz = 0;
  }

  /* find normal of each face and add it to each vertex adjacent to the face */

  for (i = 0; i < nfaces; i++) {

    face = flist[i];
    verts = face->verts;

    /* determine vectors parallel to two edges of face */

    x0 = vlist[verts[face->nverts-1]]->x - vlist[verts[0]]->x;
    y0 = vlist[verts[face->nverts-1]]->y - vlist[verts[0]]->y;
    z0 = vlist[verts[face->nverts-1]]->z - vlist[verts[0]]->z;

    x1 = vlist[verts[1]]->x - vlist[verts[0]]->x;
    y1 = vlist[verts[1]]->y - vlist[verts[0]]->y;
    z1 = vlist[verts[1]]->z - vlist[verts[0]]->z;

    /* find cross-product between these vectors */
    x = y0 * z1 - z0 * y1;
    y = z0 * x1 - x0 * z1;
    z = x0 * y1 - y0 * x1;

    /* normalize this vector */
    len = x*x + y*y + z*z;
    if (len == 0) {
      x = y = z = 0;
    }
    else {
      recip = 1 / sqrt (len);
      x *= recip;
      y *= recip;
      z *= recip;
    }

    /* add this normal to each vertex that is adjacent to face */
    for (j = 0; j < face->nverts; j++) {
      vlist[verts[j]]->nx += x;
      vlist[verts[j]]->ny += y;
      vlist[verts[j]]->nz += z;
    }
  }

  /* normalize all the normals at the vertices */

  for (i = 0; i < nverts; i++) {
    vert = vlist[i];
    len = vert->nx * vert->nx + vert->ny * vert->ny + vert->nz * vert->nz;
    if (len == 0) {
      vert->nx = 0;
      vert->ny = 0;
      vert->nz = 0;
    }
    else {
      if (flip_sign)
        recip = -1 / sqrt (len);
      else
        recip = 1 / sqrt (len);
      vert->nx *= recip;
      vert->ny *= recip;
      vert->nz *= recip;
    }
  }
}


/******************************************************************************
Read in the PLY file from standard in.
******************************************************************************/

read_file()
{
  int i,j;
  int elem_count;
  char *elem_name;

  /*** Read in the original PLY object ***/

  in_ply = read_ply (stdin);

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
	if (equal_strings ("nx", prop->name))
	  setup_property_ply (in_ply, &vert_props[3]);
	if (equal_strings ("ny", prop->name))
	  setup_property_ply (in_ply, &vert_props[4]);
	if (equal_strings ("nz", prop->name))
	  setup_property_ply (in_ply, &vert_props[5]);
      }

      vert_other = get_other_properties_ply (in_ply, 
					     offsetof(Vertex,other_props));

      /* grab all the vertex elements */
      for (j = 0; j < elem_count; j++) {
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
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
}


/******************************************************************************
Write out the PLY file to standard out.
******************************************************************************/

write_file()
{
  int i;
  PlyFile *ply;
  char **elist;
  int num_elem_types;

  /*** Write out the transformed PLY object ***/

  elist = get_element_list_ply (in_ply, &num_elem_types);
  ply = write_ply (stdout, num_elem_types, elist, in_ply->file_type);

  /* describe what properties go into the vertex elements */

  describe_element_ply (ply, "vertex", nverts);
  describe_property_ply (ply, &vert_props[0]);
  describe_property_ply (ply, &vert_props[1]);
  describe_property_ply (ply, &vert_props[2]);
  describe_property_ply (ply, &vert_props[3]);
  describe_property_ply (ply, &vert_props[4]);
  describe_property_ply (ply, &vert_props[5]);
  describe_other_properties_ply (ply, vert_other, offsetof(Vertex,other_props));

  describe_element_ply (ply, "face", nfaces);
  describe_property_ply (ply, &face_props[0]);
  describe_other_properties_ply (ply, face_other, offsetof(Face,other_props));

  describe_other_elements_ply (ply, in_ply->other_elems);

  copy_comments_ply (ply, in_ply);
  append_comment_ply (ply, "modified by normalsply");
  copy_obj_info_ply (ply, in_ply);

  header_complete_ply (ply);

  /* set up and write the vertex elements */
  put_element_setup_ply (ply, "vertex");
  for (i = 0; i < nverts; i++)
    put_element_ply (ply, (void *) vlist[i]);

  /* set up and write the face elements */
  put_element_setup_ply (ply, "face");
  for (i = 0; i < nfaces; i++)
    put_element_ply (ply, (void *) flist[i]);

  put_other_elements_ply (ply);

  close_ply (ply);
  free_ply (ply);
}

