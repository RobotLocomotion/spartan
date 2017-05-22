/*

Print out the header of a PLY file.

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

void read_header(FILE *fp);

/******************************************************************************
Main program.
******************************************************************************/

main(int argc, char *argv[])
{
  int i,j;
  char *s;
  char *progname;
  char filename[80];
  FILE *fp;

  progname = argv[0];

  while (--argc > 0 && (*++argv)[0]=='-') {
    for (s = argv[0]+1; *s; s++)
      switch (*s) {
	/*
        case 'f':
          fract = atof (*++argv);
          argc -= 1;
          break;
	*/
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  /* read from named file or from stdin */

  if (argc >= 1) {

    strcpy (filename, *argv);
    if (strlen (filename) < 4 ||
        strcmp (filename + strlen (filename) - 4, ".ply") != 0)
        strcat (filename, ".ply");

    fp = fopen (filename, "r");

    if (fp == NULL) {
      fprintf (stderr, "Cannot open file '%s'\n", filename);
      exit (-1);
    }

    read_header (fp);
  }
  else
    read_header (stdin);
}


/******************************************************************************
Print out usage information.
******************************************************************************/

usage(char *progname)
{
  fprintf (stderr, "usage: %s <in.ply\n", progname);
  fprintf (stderr, "  or\n");
  fprintf (stderr, "usage: %s in.[ply]\n", progname);
}


/******************************************************************************
Get a line from a text file.

Entry:
  fp - file to read from

Exit:
  returns string of characters from the file
******************************************************************************/

char *get_line(FILE *fp)
{
  static char str[200];
  char *ptr;

  fgets (str, 200, fp);
  str[200] = '\0';

  /* replace new-line with null character */
  for (ptr = str; *ptr != '\0'; ptr++)
    if (*ptr == '\n') {
      *ptr = '\0';
      break;
    }

  return (str);
}


/******************************************************************************
Read the header from a PLY file and print it out.

Entry:
  fp - file pointer of the file to read from
******************************************************************************/

void read_header(FILE *fp)
{
  char *str;
  int count = 0;

  str = get_line (fp);

  /* make sure this is a PLY file */
  if (strcmp(str, "ply") != 0) {
    fprintf (stderr, "This is not a PLY file.\n");
    exit (-1);
  }

  printf ("%s\n", str);

  while (1) {

    str = get_line (fp);
    printf ("%s\n", str);
    count++;

    if (count > 100) {
      fprintf (stderr, "Header seems to be too long, so we're quitting.\n");
      break;
    }

    if (strcmp(str, "end_header") == 0)
      break;
  }

}

