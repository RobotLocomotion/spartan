/*

Convert any PLY file to either a binary or an ascii PLY file, depending on
which of WRITE_ASCII or WRITE_BINARY is defined during compilation.

This program does NOT use the other_element routines because they require
storing the entire model.  Here, each individual element is saved right
away and then the memory is used again for the next element.  This means
that running out of memory is not a problem.

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


/******************************************************************************
Convert binary PLY files to ascii.
******************************************************************************/

main(int argc, char *argv[])
{
  int i,j,k;
  PlyFile *in_ply;
  PlyFile *out_ply;
  int nelems;
  char **elist;
  int file_type;
  float version;
  int nprops;
  int num_elems;
  PlyProperty **plist;
  PlyProperty **plist_copy;
  PlyProperty *prop;
  char *elem_name;
  char *data;
  int offset;
  int *offset_list;
  int **lists;
  int *list_count;
  char **list_ptr;
  int verbose_flag = 0;

#ifndef WRITE_ASCII
#ifndef WRITE_BINARY

  fprintf (stderr, "'%s' compiled incorrectly.\n", argv[0]);
  fprintf (stderr,
           "Must have WRITE_ASCII or WRITE_BINARY defined during compile.\n");
  exit (-1);

#endif
#endif

  /* maybe print out help message */

#ifdef WRITE_ASCII
  if (argc > 2 || (argc == 2 && !equal_strings (argv[1], "-p"))) {
    fprintf (stderr, "usage: %s [flags] <infile >outfile\n", argv[0]);
    fprintf (stderr, "          -p (print element labels)\n");
    exit (0);
  }
#endif

#ifdef WRITE_BINARY
  if (argc > 1) {
    fprintf (stderr, "usage: %s <infile >outfile\n", argv[0]);
    exit (0);
  }
#endif

  if (argc == 2 && equal_strings (argv[1], "-p"))
    verbose_flag = 1;

  /* open the input and output files */

  in_ply  = read_ply (stdin);
  elist = get_element_list_ply (in_ply, &nelems);

#ifdef WRITE_ASCII
  out_ply = write_ply (stdout, nelems, elist, PLY_ASCII);
#endif

#ifdef WRITE_BINARY
  out_ply = write_ply (stdout, nelems, elist, PLY_BINARY_BE);
#endif

  /* allocate space for various lists that keep track of the elements */

  plist_copy = (PlyProperty **) malloc (sizeof (PlyProperty *) * nelems);
  offset_list = (int *) malloc (sizeof (int) * nelems);
  lists = (int **) malloc (sizeof (int *) * nelems);
  list_count = (int *) malloc (sizeof (int));

  /* go through each kind of element that we learned is in the file */
  /* and come up with a list that has offsets for all properties */

  for (i = 0; i < nelems; i++) {

    /* get the description of the element */
    elem_name = elist[i];
    plist = get_element_description_ply(in_ply, elem_name, &num_elems, &nprops);

    /* make room for a list of the lists in an element, so that */
    /* we can later easily free up the space created by malloc'ed lists */

    list_count[i] = 0;
    lists[i] = (int *) malloc (sizeof (int) * nprops);

    /* set up pointers into data */

    offset = 0;
    for (j = 0; j < nprops; j++) {
      plist[j]->offset = offset;
      offset += 8;
      if (plist[j]->is_list) {
        plist[j]->count_offset = offset;
        lists[i][list_count[i]] = offset - 8;
        list_count[i]++;
        offset += 8;
      }
    }

    offset_list[i] = offset;

    /* copy the property list */

    plist_copy[i] = (PlyProperty *) malloc (sizeof (PlyProperty) * nprops);
    prop = plist_copy[i];

    for (j = 0; j < nprops; j++) {
      prop->name = plist[j]->name;
      prop->external_type = plist[j]->external_type;
      prop->internal_type = plist[j]->external_type;
      prop->offset = plist[j]->offset;
      prop->is_list = plist[j]->is_list;
      prop->count_external = plist[j]->count_external;
      prop->count_internal = plist[j]->count_external;
      prop->count_offset = plist[j]->count_offset;
      prop++;
    }

    element_layout_ply (out_ply, elem_name, num_elems, nprops, plist_copy[i]);
  }

  /* copy the comments and obj_info */

  copy_comments_ply (out_ply, in_ply);
  copy_obj_info_ply (out_ply, in_ply);

  /* finish the header for the output file */
  header_complete_ply (out_ply);

  /* copy all the element information */

  for (i = 0; i < nelems; i++) {

    /* get the description of the element */
    elem_name = elist[i];
    plist = get_element_description_ply(in_ply, elem_name, &num_elems, &nprops);

    /* allocate space for an element */
    data = (char *) malloc (8 * offset_list[i]);

    /* set up for getting elements */
    get_element_setup_ply (in_ply, elem_name, nprops, plist_copy[i]);
    put_element_setup_ply (out_ply, elem_name);

    /* possibly print out name of element */
    if (verbose_flag)
      fprintf (out_ply->fp, "%s:\n", elem_name);

    /* copy all the elements */

    if (list_count[i]) {
      /* need to free the lists */
      for (j = 0; j < num_elems; j++) {
        get_element_ply (in_ply, (void *) data);
        put_element_ply (out_ply, (void *) data);
        for (k = 0; k < list_count[i]; k++) {
          list_ptr = (char **) (data + lists[i][k]);
          free (*list_ptr);
        }
      }
    }
    else {
      /* no lists */
      for (j = 0; j < num_elems; j++) {
        get_element_ply (in_ply, (void *) data);
        put_element_ply (out_ply, (void *) data);
      }
    }

  }

  /* close the PLY files */

  close_ply (in_ply);
  close_ply (out_ply);
}

