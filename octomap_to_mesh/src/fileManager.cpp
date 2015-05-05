/*
 * fileManager.cpp
 *
 * Code generation for function 'fileManager'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/fileManager.h"
#include "octomap_to_mesh/OctomapToMesh_data.h"
#include <stdio.h>

/* Function Declarations */
static FILE * d_fileManager(int8_T varargin_1);
static int8_T filedata();
static real_T rt_roundd_snf(real_T u);

/* Function Definitions */
static FILE * d_fileManager(int8_T varargin_1)
{
  FILE * f;
  int8_T fileid;
  fileid = varargin_1;
  if (varargin_1 < 0) {
    fileid = -1;
  }

  if (fileid >= 3) {
    f = eml_openfiles[fileid - 3];
  } else if (fileid == 0) {
    f = stdin;
  } else if (fileid == 1) {
    f = stdout;
  } else if (fileid == 2) {
    f = stderr;
  } else {
    f = NULL;
  }

  return f;
}

static int8_T filedata()
{
  int8_T f;
  int8_T k;
  boolean_T exitg1;
  f = 0;
  k = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (k < 21)) {
    if (eml_openfiles[k - 1] == (FILE *)NULL) {
      f = k;
      exitg1 = TRUE;
    } else {
      k++;
    }
  }

  return f;
}

static real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

void b_fileManager(real_T varargin_1, FILE * *f, boolean_T *a)
{
  int8_T fileid;
  fileid = (int8_T)rt_roundd_snf(varargin_1);
  if ((fileid > 20) || (fileid < 0)) {
    fileid = -1;
  }

  if (fileid >= 3) {
    fileid -= 2;
    *f = eml_openfiles[fileid - 1];
    *a = eml_autoflush[fileid - 1];
  } else if (fileid == 0) {
    *f = stdin;
    *a = TRUE;
  } else if (fileid == 1) {
    *f = stdout;
    *a = TRUE;
  } else if (fileid == 2) {
    *f = stderr;
    *a = TRUE;
  } else {
    *f = NULL;
    *a = TRUE;
  }
}

int32_T c_fileManager(real_T varargin_1)
{
  int32_T f;
  int8_T fileid;
  FILE * filestar;
  int32_T cst;
  f = -1;
  fileid = (int8_T)rt_roundd_snf(varargin_1);
  if ((fileid > 20) || (fileid < 0)) {
    fileid = -1;
  }

  filestar = d_fileManager(fileid);
  if ((filestar == (FILE *)NULL) || (fileid < 3)) {
  } else {
    cst = fclose(filestar);
    if (cst == 0) {
      f = 0;
      fileid -= 2;
      eml_openfiles[fileid - 1] = NULL;
      eml_autoflush[fileid - 1] = TRUE;
    }
  }

  return f;
}

int8_T fileManager(const char_T * varargin_1_data, const int32_T
                   varargin_1_size[2])
{
  int8_T f;
  int8_T j;
  int32_T loop_ub;
  int32_T i15;
  char_T tmp_data[80];
  char_T cv0[3];
  static const char_T cv1[3] = { 'w', 'b', '\x00' };

  FILE * filestar;
  f = -1;
  j = filedata();
  if (j < 1) {
  } else {
    loop_ub = varargin_1_size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      tmp_data[i15] = varargin_1_data[varargin_1_size[0] * i15];
    }

    tmp_data[varargin_1_size[1]] = '\x00';
    for (i15 = 0; i15 < 3; i15++) {
      cv0[i15] = cv1[i15];
    }

    filestar = fopen(&tmp_data[0], cv0);
    if (filestar != (FILE *)NULL) {
      eml_openfiles[j - 1] = filestar;
      eml_autoflush[j - 1] = TRUE;
      i15 = j + 2;
      if (i15 > 127) {
        i15 = 127;
      }

      f = (int8_T)i15;
    }
  }

  return f;
}

/* End of code generation (fileManager.cpp) */
