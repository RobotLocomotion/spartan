/*

Apply a 3-D transformation to an object from a PLY file.

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

static PlyProperty vert_props[] = {
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
};


/*** the PLY object ***/

int nverts;
Vertex **vlist;

PlyFile *in_ply;  /* input PLY file */

PlyOtherProp *vert_other;

/*** transformation records ***/

#define  TRANSLATE  1
#define  SCALE      2
#define  ROTATE     3

typedef struct Transform {
  int type;      /* scale, translate or rotate */
  float x,y,z;
  float angle;
} Transform;


static Transform *transforms = NULL;
static int num_trans = 0;
static int max_trans = 1;

typedef float gtMatrix[4][4];
typedef float gtVector[3];

void get_rotation(float angle, float ax, float ay, float az, gtMatrix mat);



/******************************************************************************
Create a new transformation record.
******************************************************************************/

Transform *new_transform()
{
  /* allocate space for transformations, if necessary */

  if (transforms == NULL) {
    transforms = (Transform *) malloc (sizeof (Transform) * max_trans);
  }
  else if (num_trans >= max_trans) {
    max_trans += 1;
    transforms = (Transform *)
                 realloc (transforms, sizeof (Transform) * max_trans);
  }

  num_trans++;
  return (&transforms[num_trans-1]);
}


/******************************************************************************
Main routine.
******************************************************************************/

main(int argc, char *argv[])
{
  int i,j;
  char *s;
  char *progname;
  Transform *trans;

  progname = argv[0];

  while (--argc > 0 && (*++argv)[0]=='-') {
    for (s = argv[0]+1; *s; s++)
      switch (*s) {
        case 's':
          trans = new_transform();
          trans->type = SCALE;
          trans->x = atof (*++argv);
          trans->y = atof (*++argv);
          trans->z = atof (*++argv);
          argc -= 3;
          break;
        case 't':
          trans = new_transform();
          trans->type = TRANSLATE;
          trans->x = atof (*++argv);
          trans->y = atof (*++argv);
          trans->z = atof (*++argv);
          argc -= 3;
          break;
        case 'r':
          trans = new_transform();
          trans->type = ROTATE;
          trans->angle = atof (*++argv);
          trans->x = atof (*++argv);
          trans->y = atof (*++argv);
          trans->z = atof (*++argv);
          argc -= 4;
          break;
        default:
          fprintf (stderr, "usage: %s [flags] <in.ply >out.ply\n", progname);
          fprintf (stderr, "       -t xtrans ytrans ztrans\n");
          fprintf (stderr, "       -s xscale yscale zscale\n");
          fprintf (stderr, "       -r angle (in degrees) xaxis yaxis zaxis\n");
          fprintf (stderr, "\n");
          fprintf (stderr,
            "Transformations are applied in the order in which they appear\n");
          fprintf (stderr,
            "in the command line.  There is no limit on the number of\n");
          fprintf (stderr, "transformations that may be used.\n");
          exit (-1);
          break;
      }
  }

  read_file();
  transform();
  write_file();
}


/******************************************************************************
Transform the PLY object.
******************************************************************************/

transform()
{
  int i,k;
  Vertex *vert;
  float x,y,z;
  float xx,yy,zz;
  float angle;
  gtMatrix rmat;

  for (k = 0; k < num_trans; k++) {

    x = transforms[k].x;
    y = transforms[k].y;
    z = transforms[k].z;
    angle = transforms[k].angle;

    switch (transforms[k].type) {
      case TRANSLATE:
        for (i = 0; i < nverts; i++) {
          vert = vlist[i];
          vert->x += x;
          vert->y += y;
          vert->z += z;
        }
        break;
      case SCALE:
        for (i = 0; i < nverts; i++) {
          vert = vlist[i];
          vert->x *= x;
          vert->y *= y;
          vert->z *= z;
        }
        break;
      case ROTATE:
        get_rotation (angle, x, y, z, rmat);
        for (i = 0; i < nverts; i++) {
          vert = vlist[i];
          xx = vert->x;
          yy = vert->y;
          zz = vert->z;
          vert->x = xx * rmat[0][0] + yy * rmat[1][0] + zz * rmat[2][0];
          vert->y = xx * rmat[0][1] + yy * rmat[1][1] + zz * rmat[2][1];
          vert->z = xx * rmat[0][2] + yy * rmat[1][2] + zz * rmat[2][2];
        }
        break;
    }
  }
}


/******************************************************************************
Copy a matrix.

Entry:
  dest   - destination matrix
  source - source matrix to copy
******************************************************************************/

static void copy_matrix(gtMatrix dest, gtMatrix source)
{
  int i,j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      dest[i][j] = source[i][j];
}


/******************************************************************************
Set a matrix to the identity matrix.
******************************************************************************/

static void identity_matrix(gtMatrix mat)
{
  int i,j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      mat[i][j] = (i == j);
}


/******************************************************************************
Transpose a matrix.
******************************************************************************/

static void transpose_matrix(gtMatrix m)
{
  int i,j;
  gtMatrix m2;

  /* copy */
  copy_matrix (m2, m);

  /* transpose */
  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      m[i][j] = m2[j][i];
}


/******************************************************************************
Multiply two matrices.

Entry:
  m1,m2 - matrices to multipy

Exit:
  dest - result of m1 * m2
******************************************************************************/

static void mult_matrix(gtMatrix dest, gtMatrix m1, gtMatrix m2)
{
  int i,j,k;
  gtMatrix m;

  /* perform multiplication */

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) {
      m[i][j] = 0;
      for (k = 0; k < 4; k++)
	m[i][j] += m1[k][j] * m2[i][k];
    }

  /* copy the answer */

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      dest[i][j] = m[i][j];
}


/******************************************************************************
Normalize a vector.
******************************************************************************/

void normalize_vector(gtVector v)
{
  float len,recip;

  len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];

  if (len == 0) {
    fprintf (stderr, "normalize_vector: zero length vector\n");
    return;
  }

  recip = 1.0 / sqrt (len);
  v[0] *= recip;
  v[1] *= recip;
  v[2] *= recip;
}


/******************************************************************************
Form the cross-product of two vectors.

Entry:
  a,b - vectors to form cross-product of

Exit:
  c - a cross b
******************************************************************************/

void cross(gtVector c, gtVector a, gtVector b)
{
  gtVector vtemp;

  /* perform cross-product */
  vtemp[0] = a[1]*b[2] - a[2]*b[1];
  vtemp[1] = a[2]*b[0] - a[0]*b[2];
  vtemp[2] = a[0]*b[1] - a[1]*b[0];

  /* copy the result */
  c[0] = vtemp[0];
  c[1] = vtemp[1];
  c[2] = vtemp[2];
}


/******************************************************************************
Determine the matrix that rotates around an arbitrary axis.

Entry:
  angle    - number of degrees to rotate around a given axis
  ax,ay,az - the given axis

Exit:
  mat - the rotation matrix
******************************************************************************/

void get_rotation(float angle, float ax, float ay, float az, gtMatrix mat)
{
  gtMatrix r1,r2,r3;
  float theta;
  gtVector n;
  gtVector a,b,c;

  a[0] = ax;
  a[1] = ay;
  a[2] = az;
  normalize_vector(a);

  /* create vector not parallel to "a" */

  ax = fabs(a[0]);
  ay = fabs(a[1]);
  az = fabs(a[2]);

  n[0] = n[1] = n[2] = 0;

  if (ax > ay) {
    if (ay > az)
      n[2] = 1;    /* z is smallest */
    else
      n[1] = 1;    /* y is smallest */
  }
  else {
    if (ax > az)
      n[2] = 1;    /* z is smallest */
    else
      n[0] = 1;    /* x is smallest */
  }

  /* create "b" orthogonal to "a" */

  cross (b, a, n);
  normalize_vector(b);

  /* create "c" orthogonal to "a" and "b" */

  cross (c, a, b);

  /* make matrix that rotates a,b,c into x,y,z axes */

  identity_matrix (r1);
  r1[0][0] = a[0];
  r1[1][0] = a[1];
  r1[2][0] = a[2];
  r1[0][1] = b[0];
  r1[1][1] = b[1];
  r1[2][1] = b[2];
  r1[0][2] = c[0];
  r1[1][2] = c[1];
  r1[2][2] = c[2];

  /* make matrix for rotation by theta degrees around x-axis */

  theta = angle * 3.1415926535 / 180.0;
  identity_matrix (r2);
  r2[1][1] =  cos(theta);
  r2[2][2] =  cos(theta);
  r2[1][2] =  sin(theta);
  r2[2][1] = -sin(theta);

  /* make matrix that is inverse of r1 */
  copy_matrix (r3, r1);
  transpose_matrix (r3);

  /* compose these matrices for final matrix mat = r3 * r2 * r1 */
  mult_matrix (mat, r3, r2);
  mult_matrix (mat, mat, r1);
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
  append_comment_ply (ply, "modified by xformply");
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

