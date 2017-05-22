/*

Convert from Wavefront OBJ format to PLY format.

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
#include <stdlib.h>
#include <math.h>
#include <strings.h>
#include <ply.h>


/* user's vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z,w;          /* position */
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
  {"w", Float32, Float32, offsetof(Vertex,w), 0, 0, 0, 0},
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

static int nverts = 0;
static int max_verts = 0;
static Vertex *vlist;

static int nfaces = 0;
static int max_faces = 0;
static Face *flist;

static int nelems = 2;

static int ncomments = 0;
static int max_comments = 0;
static char **comments = NULL;

static int texture_coords = 0;
static int has_normals = 0;
static int has_w = 0;

/* for file reading */
static char **words;
static int max_words = 0;
static int num_words = 0;
#define BIG_STRING 4096
static char str[BIG_STRING];
static char str_orig[BIG_STRING];
static int flip_vertex_order = 1;


/******************************************************************************
Main program.
******************************************************************************/

main(int argc, char *argv[])
{
  int i,j;
  char *s;
  char *progname;
  int num_major = 20;
  int num_minor = 20;
  float r_major = 1;
  float r_minor = 0.5;

  progname = argv[0];

  while (--argc > 0 && (*++argv)[0]=='-') {
    for (s = argv[0]+1; *s; s++)
      switch (*s) {
        case 'f':
          flip_vertex_order = 1 - flip_vertex_order;
          break;
#if 0
        case 't':
          texture_coords = 1 - texture_coords;
          break;
#endif
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  read_obj();
  write_file();
}


/******************************************************************************
Print out usage information.
******************************************************************************/

usage(char *progname)
{
  fprintf (stderr, "usage: %s [flags] <in.obj >out.ply\n", progname);
  fprintf (stderr, "       -f { flip vertex order in polygons }\n");
}


/******************************************************************************
Create a new vertex.

Entry:
  x,y,z,w - 3D positions, maybe with homogeneous component
******************************************************************************/

make_vertex(float x, float y, float z, float w)
{
  Vertex *v;

  /* see if we need to allocate space for vertices */

  if (max_verts == 0) {
    max_verts = 200;
    vlist = (Vertex *) malloc (sizeof (Vertex) * max_verts);
  }
  else if (nverts == max_verts) {
    max_verts *= 2;
    vlist = (Vertex *) realloc (vlist, sizeof (Vertex) * max_verts);
  }

  v = &vlist[nverts++];
  v->x = x;
  v->y = y;
  v->z = z;
  v->w = w;
}


/******************************************************************************
Break up a word that may have slash-separated numbers into one or more
numbers.

Entry:
  word - word to break up

Exit:
  vindex - first number (vertex index)
  tindex - second number (texture index)
  nindex - third number (normal vector index)
******************************************************************************/

get_indices(char *word, int *vindex, int *tindex, int *nindex)
{
  char *null = " ";
  char *ptr;
  char *tp;
  char *np;

  /* by default, the texture and normal pointers are set to the null string */

  tp = null;
  np = null;

  /* replace slashes with null characters and cause tp and np to point */
  /* to character immediately following the first or second slash */

  for (ptr = word; *ptr != '\0'; ptr++) {

    if (*ptr == '/') {

      if (tp == null)
        tp = ptr + 1;
      else
        np = ptr + 1;

      *ptr = '\0';
    }
  }

  *vindex = atoi (word);
  *tindex = atoi (tp);
  *nindex = atoi (np);
}


/******************************************************************************
Create a new face.

Entry:
  words  - list of words describing the vertex
  nwords - number of words in list
******************************************************************************/

make_face(char **words, int nwords)
{
  static int warning = 0;
  int i,ii;
  Face *f;
  int vindex;
  int nindex;
  int tindex;

  /* see if we need to allocate space for vertices */

  if (max_faces == 0) {
    max_faces = 200;
    flist = (Face *) malloc (sizeof (Face) * max_faces);
  }
  else if (nfaces == max_faces) {
    max_faces *= 2;
    flist = (Face *) realloc (flist, sizeof (Face) * max_faces);
  }

  f = &flist[nfaces++];
  f->nverts = nwords;
  f->verts = (int *) malloc (sizeof (int) * nwords);

  for (i = 0; i < nwords; i++) {

    get_indices (words[i], &vindex, &tindex, &nindex);

#if 0
printf ("vtn: %d %d %d\n", vindex, tindex, nindex);
#endif

    /* maybe flip vertex order */

    if (flip_vertex_order)
      ii = nwords - i - 1;
    else
      ii = i;

    /* store the vertex index */

    if (vindex > 0)       /* indices are from one, not zero */
      f->verts[ii] = vindex - 1;
    else if (vindex < 0)  /* negative indices mean count backwards */
      f->verts[ii] = nverts + vindex;
    else {
      fprintf (stderr, "Zero indices not allowed: '%s'\n", str_orig);
      exit (-1);
    }

    if ((tindex != 0 || nindex != 0) && warning == 0) {
      fprintf (stderr, "\n");
      fprintf (stderr, "Warning: textures and normals currently ignored.\n");
      fprintf (stderr, "\n");
      warning = 1;
    }

  }
}


/******************************************************************************
Save a new comment.

Entry:
  comment - comment to tuck away
******************************************************************************/

make_comment(char *comment)
{
  /* see if we need to allocate space for comments */

  if (max_comments == 0) {
    max_comments = 10;
    comments = (char **) malloc (sizeof (char *) * max_comments);
  }
  if (ncomments == max_comments) {
    max_comments += 10;
    comments = (char **) realloc (comments, sizeof (char *) * max_comments);
  }

  comments[ncomments] = strdup (comment);
  ncomments++;
}


/******************************************************************************
Get a text line and see if it is a line of comments.

Entry:
  fp - file to read from

Exit:
  returns a pointer to comments or NULL if not a comment line or -1 if EOF
******************************************************************************/

char *fetch_line(FILE *fp)
{
  int i,j;
  char *ptr;
  char *ptr2;
  char *result;
  char *comment_ptr;

  /* read in a line */
  result = fgets (str, BIG_STRING, fp);

  /* return NULL if we're at the end-of-file */
  if (result == NULL)
    return ((char *) -1);

  /* convert line-feed and tabs into spaces */
  /* (this guarentees that there will be a space before the */
  /*  null character at the end of the string) */

  str[BIG_STRING-2] = ' ';
  str[BIG_STRING-1] = '\0';

  for (ptr = str; *ptr != '\0'; ptr++) {
    if (*ptr == '\t') {
      *ptr = ' ';
    }
    else if (*ptr == '\n') {
      *ptr = ' ';
      break;
    }
  }

  /* copy the line */
  for (ptr = str, ptr2 = str_orig; *ptr != '\0'; ptr++, ptr2++)
    *ptr2 = *ptr;
  *ptr2 = '\0';

  /* look to see if this is a comment line (first non-space is '#') */

  for (ptr = str; *ptr != '\0'; ptr++) {
    if (*ptr == '#') {
      ptr++;
      while (*ptr == ' ')
        ptr++;
      return (ptr);
    }
    else if (*ptr != ' ') {
      break;
    }
  }

  /* if we get here, we've got a non-comment line */

  /* strip off trailing comments */

  while (*ptr != '\0') {
    if (*ptr == '#') {
      *ptr++ = ' ';
      *ptr = '\0';
      break;
    }
    ptr++;
  }

  return (NULL);
}


/******************************************************************************
Break up the last read line into words.

Exit:
  returns the number of words in the line
******************************************************************************/

int fetch_words()
{
  char *ptr;

  /* allocate room for words if necessary */
  if (max_words == 0) {
    max_words = 20;
    words = (char **) malloc (sizeof (char *) * max_words);
  }

  /* find the words in the line */

  ptr = str;
  num_words = 0;

  while (*ptr != '\0') {

    /* jump over leading spaces */
    while (*ptr == ' ')
      ptr++;

    /* break if we reach the end */
    if (*ptr == '\0')
      break;

    /* allocate more room for words if necessary */
    if (num_words >= max_words) {
      max_words += 10;
      words = (char **) realloc (words, sizeof (char *) * max_words);
    }

    /* save pointer to beginning of word */
    words[num_words++] = ptr;

    /* jump over non-spaces */
    while (*ptr != ' ')
      ptr++;

    /* place a null character here to mark the end of the word */
    *ptr++ = '\0';
  }

  /* return the number of words */
  return (num_words);
}


/******************************************************************************
Read in a Wavefront OBJ file.
******************************************************************************/

read_obj()
{
  int i,j,k;
  FILE *fp;
  int nwords;
  char *comment_ptr;
  char *first_word;
  float x,y,z,w;

  /* read from standard input */
  fp = stdin;

  while (1) {

    comment_ptr = fetch_line (fp);

    if (comment_ptr == (char *) -1)  /* end-of-file */
      break;

    /* did we get a comment? */
    if (comment_ptr) {
      make_comment (comment_ptr);
      continue;
    }

    /* if we get here, the line was not a comment */

    nwords = fetch_words();

    /* skip empty lines */
    if (nwords == 0)
      continue;

    first_word = words[0];

    if (equal_strings (first_word, "v")) {
      if (nwords < 4) {
	fprintf (stderr, "Too few coordinates: '%s'", str_orig);
	exit (-1);
      }
      x = atof (words[1]);
      y = atof (words[2]);
      z = atof (words[3]);
      if (nwords == 5) {
        w = atof (words[3]);
	has_w = 1;
      }
      else
        w = 1.0;
      make_vertex (x, y, z, w);
    }
    else if (equal_strings (first_word, "vn")) {
    }
    else if (equal_strings (first_word, "vt")) {
    }
    else if (equal_strings (first_word, "f")) {
      make_face (&words[1], nwords-1);
    }
    else {
      fprintf (stderr, "Do not recognize: '%s'\n", str_orig);
    }

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
  if (has_normals) {
    describe_property_ply (ply, &vert_props[3]);
    describe_property_ply (ply, &vert_props[4]);
    describe_property_ply (ply, &vert_props[5]);
  }
  if (texture_coords) {
    describe_property_ply (ply, &vert_props[6]);
    describe_property_ply (ply, &vert_props[7]);
  }

  describe_element_ply (ply, "face", nfaces);
  describe_property_ply (ply, &face_props[0]);

  for (i = 0; i < ncomments; i++)
    append_comment_ply (ply, comments[i]);

  append_comment_ply (ply, "converted from OBJ by obj2ply");

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

