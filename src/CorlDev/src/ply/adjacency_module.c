/*

Routines for creating pointers between faces and vertices.

This file does not stand alone, but rather should be included in code that
defines its own versions of Vertex and Face.  This way, different programs
can make use of these routines even if their Vertex and Face definitions
vary from one another.

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


/* the different types of vertices */

#define MANIFOLD_VERTEX  1
#define BOUNDARY_VERTEX  2
#define SPECIAL_VERTEX   3


/******************************************************************************
Find out if there is another face that shares an edge with a given face.

Entry:
  f1    - face that we're looking to share with
  v1,v2 - two vertices of f1 that define edge

Exit:
  return the matching face, or NULL if there is no such face
******************************************************************************/

Face *find_common_edge(Face *f1, Vertex *v1, Vertex *v2)
{
  int i,j,k;
  Face *f2;
  Face *adjacent = NULL;

  /* look through all faces of the first vertex */

  for (i = 0; i < v1->nfaces; i++) {
    f2 = v1->faces[i];
    if (f2 == f1)
      continue;
    /* examine the vertices of the face for a match with the second vertex */
    for (j = 0; j < f2->nverts; j++) {

      /* look for a match */
      if (f2->verts[j] == v2) {

#if 0
	/* watch out for triple edges */

        if (adjacent != NULL) {

	  fprintf (stderr, "model has triple edges\n");

	  fprintf (stderr, "face 1: ");
	  for (k = 0; k < f1->nverts; k++)
	    fprintf (stderr, "%d ", f1->iverts[k]);
	  fprintf (stderr, "\nface 2: ");
	  for (k = 0; k < f2->nverts; k++)
	    fprintf (stderr, "%d ", f2->iverts[k]);
	  fprintf (stderr, "\nface 3: ");
	  for (k = 0; k < adjacent->nverts; k++)
	    fprintf (stderr, "%d ", adjacent->iverts[k]);
	  fprintf (stderr, "\n");

	}

	/* if we've got a match, remember this face */
        adjacent = f2;
#endif

#if 1
	/* if we've got a match, return this face */
        return (f2);
#endif

      }
    }
  }

  return (adjacent);
}


/******************************************************************************
Create pointers from each face to all of its adjacent faces.
******************************************************************************/

face_to_face_ptrs()
{
  int i,j,k;
  Face *f;
  Vertex *v1,*v2;

  /* make pointers from faces to adjacent faces */

  for (i = 0; i < nfaces; i++) {
    f = flist[i];
    f->faces = (Face **) malloc (sizeof (Face *) * f->nverts);
    for (j = 0; j < f->nverts; j++) {
      v1 = f->verts[j];
      v2 = f->verts[(j+1) % f->nverts];
      f->faces[j] = find_common_edge (f, v1, v2);
    }
  }
}


/******************************************************************************
Create pointers from vertices to faces.
******************************************************************************/

vertex_to_face_ptrs()
{
  int i,j,k;
  Face *f;
  Vertex *v;

  /* zero the count of number of pointers to faces */

  for (i = 0; i < nverts; i++)
    vlist[i]->nfaces = 0;

  /* first just count all the face pointers needed for each vertex */

  for (i = 0; i < nfaces; i++) {
    f = flist[i];
    for (j = 0; j < f->nverts; j++)
      f->verts[j]->nfaces++;
  }

  /* allocate memory for face pointers of vertices */

  for (i = 0; i < nverts; i++) {
    vlist[i]->faces = (Face **)
		      malloc (sizeof (Face *) * vlist[i]->nfaces);
    vlist[i]->nfaces = 0;
  }

  /* now actually create the face pointers */

  for (i = 0; i < nfaces; i++) {
    f = flist[i];
    for (j = 0; j < f->nverts; j++) {
      v = f->verts[j];
      v->faces[v->nfaces] = f;
      v->nfaces++;
    }
  }
}


/******************************************************************************
Order the pointers to faces that are around a given vertex.

Entry:
  v - vertex whose face list is to be ordered
******************************************************************************/

order_vertex_to_face_ptrs(Vertex *v)
{
  int i,j,k;
  Face *f;
  Face *fnext;
  int nf;
  int vindex;
  int boundary;
  int count;

  nf = v->nfaces;
  f = v->faces[0];

  /* go backwards (clockwise) around faces that surround a vertex */
  /* to find out if we reach a boundary */

  boundary = 0;

  for (i = 1; i <= nf; i++) {

    /* find reference to v in f */
    vindex = -1;
    for (j = 0; j < f->nverts; j++)
      if (f->verts[j] == v) {
	vindex = j;
	break;
      }

    /* error check */
    if (vindex == -1) {
      fprintf (stderr, "can't find vertex #1\n");
      exit (-1);
    }

    /* corresponding face is the previous one around v */
    fnext = f->faces[vindex];

    /* see if we've reached a boundary, and if so then place the */
    /* current face in the first position of the vertice's face list */

    if (fnext == NULL) {
      /* find reference to f in v */
      for (j = 0; j < v->nfaces; j++)
        if (v->faces[j] == f) {
	  v->faces[j] = v->faces[0];
	  v->faces[0] = f;
	  break;
	}
      boundary = 1;
      break;
    }

    f = fnext;
  }

  /* now walk around the faces in the forward direction and place */
  /* them in order */

  f = v->faces[0];
  count = 0;

  for (i = 1; i < nf; i++) {

    /* find reference to vertex in f */
    vindex = -1;
    for (j = 0; j < f->nverts; j++)
      if (f->verts[(j+1) % f->nverts] == v) {
	vindex = j;
	break;
      }

    /* error check */
    if (vindex == -1) {
      fprintf (stderr, "can't find vertex #2\n");
      exit (-1);
    }

    /* corresponding face is next one around v */
    fnext = f->faces[vindex];

    /* break out of loop if we've reached a boundary */
    count = i;
    if (fnext == NULL) {
      break;
    }

    /* swap the next face into its proper place in the face list */
    for (j = 0; j < v->nfaces; j++)
      if (v->faces[j] == fnext) {
	v->faces[j] = v->faces[i];
	v->faces[i] = fnext;
	break;
      }

    f = fnext;
  }

  /* categorize the vertex as manafold (normal), on the boundary, */
  /* or neither (special) */

  if (boundary == 0)
    v->type = MANIFOLD_VERTEX;
  else if (count+1 == nf)
    v->type = BOUNDARY_VERTEX;
  else
    v->type = SPECIAL_VERTEX;
}


/******************************************************************************
Find the index to a given vertex in the list of vertices of a given face.

Entry:
  f - face whose vertex list is to be searched
  v - vertex to return reference to

Exit:
  returns index in face's list, or -1 if vertex not found
******************************************************************************/

int face_to_vertex_ref(Face *f, Vertex *v)
{
  int j;
  int vindex = -1;

  for (j = 0; j < f->nverts; j++)
    if (f->verts[j] == v) {
      vindex = j;
      break;
    }

  return (vindex);
}


/******************************************************************************
Create the pointers from a given vertex to other vertices.

Entry:
  v - vertex whose adjacent vertex list is to be created
******************************************************************************/

vertex_to_vertex_ptrs(Vertex *v)
{
  int i,j;
  Face *f1;
  Vertex *v1,*v2;
  int vindex;
  int there1,there2;

  if (v->type == MANIFOLD_VERTEX || v->type == BOUNDARY_VERTEX) {

    /* boundary case has one more vertex than surrounding faces */

    if (v->type == MANIFOLD_VERTEX) {
      v->verts = (Vertex **) malloc (sizeof (Vertex *) * v->nfaces);
      v->nverts = v->nfaces;
    }
    else {
      v->verts = (Vertex **) malloc (sizeof (Vertex *) * (v->nfaces + 1));
      v->nverts = v->nfaces + 1;
    }

    for (i = 0; i < v->nfaces; i++) {

      /* look for reference to v in f1 */

      f1 = v->faces[i];

      vindex = face_to_vertex_ref (f1, v);
      vindex = (vindex + f1->nverts - 1) % f1->nverts;

      /* error check */
      if (vindex == -1) {
	fprintf (stderr, "can't find reference to vertex\n");
	exit (-1);
      }

      /* the vertex we want is the one just before v */
      v->verts[i] = f1->verts[vindex];
    }

    /* add final vertex if we're on a boundary */

    if (v->type == BOUNDARY_VERTEX) {
      f1 = v->faces[0];
      vindex = face_to_vertex_ref (f1, v);
      vindex = (vindex + 1) % f1->nverts;
      v->verts[v->nfaces] = f1->verts[vindex];
    }

  }
  else {  /* non-manifold, non-boundary case */

    /* we may have up to twice the number of adjacent vertices as faces */
    v->verts = (Vertex **) malloc (sizeof (Vertex *) * v->nfaces * 2);
    v->nverts = 0;

    for (i = 0; i < v->nfaces; i++) {

      f1 = v->faces[i];

      /* look for reference to v in f1 */
      vindex = face_to_vertex_ref (f1, v);

      /* error check */
      if (vindex == -1) {
	fprintf (stderr, "can't find reference to vertex\n");
	exit (-1);
      }

      /* look at the two vertices just before and after v */
      v1 = f1->verts[(vindex + 1) % f1->nverts];
      v2 = f1->verts[(vindex + f1->nverts - 1) % f1->nverts];

      /* are either of these not in the adjacent vertex list already? */
      there1 = 0;
      there2 = 0;
      for (j = 0; j < v->nverts; j++) {
	if (v1 == v->verts[j])
	  there1 = 1;
	if (v2 == v->verts[j])
	  there2 = 1;
      }

      /* add vertices that aren't already in the list of adjacent vertices */
      if (!there1)
	v->verts[v->nverts++] = v1;
      if (!there2)
	v->verts[v->nverts++] = v2;
    }
  }
}


/******************************************************************************
Create various face and vertex pointers.
******************************************************************************/

create_pointers()
{
  int i;

  /* create pointers from vertices to faces */
  vertex_to_face_ptrs();

  /* make pointers from faces to adjacent faces */
  face_to_face_ptrs();

  /* order the pointers from vertices to faces */
  for (i = 0; i < nverts; i++)
    order_vertex_to_face_ptrs(vlist[i]);

  /* create the pointers from vertices to vertices */
  for (i = 0; i < nverts; i++)
    vertex_to_vertex_ptrs (vlist[i]);
}

