/*

Scale and translate a PLY object so that it fits within a given bounding box.

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
#include <ply.h>


/* user's vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z;
  void *other_props;       /* other properties */
} Vertex;

char *elem_names[] = { /* list of the kinds of elements in the user's object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
};


/*** the PLY object ***/

PlyFile *in_ply;
int nverts;
Vertex **vlist;
PlyOtherProp *vert_other;

static float xcenter = 0;
static float ycenter = 0;
static float zcenter = 0;

static float box_size = 2;

static int print_only = 0;  /* print out info, but don't transform */
static int use_mass = 0;    /* use center of mass instead of geometric center */


/******************************************************************************
Transform a PLY file.
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
        case 'm':
          use_mass = 1 - use_mass;
          break;
        case 'p':
          print_only = 1 - print_only;
          break;
        case 'b':
          box_size = atof (*++argv);
          argc -= 1;
          break;
        case 'c':
          xcenter = atof (*++argv);
          ycenter = atof (*++argv);
          zcenter = atof (*++argv);
          argc -= 3;
          break;
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  read_file();

  transform();

  if (!print_only)
    write_file();
}


/******************************************************************************
Print out usage information.
******************************************************************************/

usage(char *progname)
{
  fprintf (stderr, "usage: %s [flags] <in.ply >out.ply\n", progname);
  fprintf (stderr, "       -b box_size (default = 2)\n");
  fprintf (stderr, "       -c xcenter ycenter zcenter (default is origin)\n");
  fprintf (stderr, "       -m (use mass center instead of geometric center)\n");
  fprintf (stderr, "       -p (only print information)\n");
}


/******************************************************************************
Transform the PLY object.
******************************************************************************/

transform()
{
  int i;
  Vertex *vert;
  float xmin,ymin,zmin;
  float xmax,ymax,zmax;
  float xdiff,ydiff,zdiff;
  float diff;
  float scale;
  float xt,yt,zt;
  float xc,yc,zc;
  float xsum,ysum,zsum;

  if (nverts == 0)
    return;

  vert = vlist[0];

  xmin = vert->x;
  ymin = vert->y;
  zmin = vert->z;

  xmax = vert->x;
  ymax = vert->y;
  zmax = vert->z;

  xsum = ysum = zsum = 0;

#define MIN(a,b) ((a < b) ? (a) : (b))
#define MAX(a,b) ((a > b) ? (a) : (b))

  /* find minimum and maximum extent of vertices */

  for (i = 0; i < nverts; i++) {

    vert = vlist[i];

    xmin = MIN (vert->x, xmin);
    ymin = MIN (vert->y, ymin);
    zmin = MIN (vert->z, zmin);

    xmax = MAX (vert->x, xmax);
    ymax = MAX (vert->y, ymax);
    zmax = MAX (vert->z, zmax);

    xsum += vert->x;
    ysum += vert->y;
    zsum += vert->z;
  }

  xdiff = xmax - xmin;
  ydiff = ymax - ymin;
  zdiff = zmax - zmin;

  diff = MAX (xdiff, MAX (ydiff, zdiff));
  scale = box_size / diff;

  if (print_only) {
    fprintf (stderr, "\n");
    fprintf (stderr, "xmin xmax: %g %g\n", xmin, xmax);
    fprintf (stderr, "ymin ymax: %g %g\n", ymin, ymax);
    fprintf (stderr, "zmin zmax: %g %g\n", zmin, zmax);
    fprintf (stderr, "geometric center = %g %g %g\n",
             0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
    fprintf (stderr, "center of mass   = %g %g %g\n",
             xsum / nverts, ysum / nverts, zsum / nverts);
    fprintf (stderr, "maximum side length = %g\n", diff);
    fprintf (stderr, "\n");
    return;
  }

  /* compute center of object, either geometric or mass center */

  if (use_mass) {
    xc = xsum / nverts;
    yc = ysum / nverts;
    zc = zsum / nverts;
  }
  else {
    xc = 0.5 * (xmin + xmax);
    yc = 0.5 * (ymin + ymax);
    zc = 0.5 * (zmin + zmax);
  }

  xt = xcenter - scale * xc;
  yt = ycenter - scale * yc;
  zt = zcenter - scale * zc;

  /* scale and translate vertices to fit into bounding box, */
  /* and also center the object */

  for (i = 0; i < nverts; i++) {
    vert = vlist[i];
    vert->x = xt + vert->x * scale;
    vert->y = yt + vert->y * scale;
    vert->z = zt + vert->z * scale;
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
      vert_other = get_other_properties_ply (in_ply, 
					     offsetof(Vertex,other_props));

      /* grab all the vertex elements */
      for (j = 0; j < elem_count; j++) {
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
        get_element_ply (in_ply, (void *) vlist[j]);
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
  describe_other_properties_ply (ply, vert_other, offsetof(Vertex,other_props));

  describe_other_elements_ply (ply, in_ply->other_elems);

  copy_comments_ply (ply, in_ply);
  append_comment_ply (ply, "modified by boundply");
  copy_obj_info_ply (ply, in_ply);

  header_complete_ply (ply);

  /* set up and write the vertex elements */
  put_element_setup_ply (ply, "vertex");
  for (i = 0; i < nverts; i++)
    put_element_ply (ply, (void *) vlist[i]);

  put_other_elements_ply (ply);

  close_ply (ply);
  free_ply (ply);
}

