/*
 * fprintf.cpp
 *
 * Code generation for function 'fprintf'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/fprintf.h"
#include "octomap_to_mesh/fileManager.h"
#include <stdio.h>

/* Function Definitions */
void b_fprintf(real_T fileID)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[11];
  int32_T i16;
  static const char_T b_cfmt[11] = { 's', 'o', 'l', 'i', 'd', ' ', 'v', 'c', 'g',
    '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i16 = 0; i16 < 11; i16++) {
      cfmt[i16] = b_cfmt[i16];
    }

    fprintf(filestar, cfmt);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

void c_fprintf(real_T fileID, real_T varargin_1, real_T varargin_2, real_T
               varargin_3)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[28];
  int32_T i17;
  static const char_T b_cfmt[28] = { ' ', ' ', 'f', 'a', 'c', 'e', 't', ' ', 'n',
    'o', 'r', 'm', 'a', 'l', ' ', ' ', '%', 'f', ' ', ' ', '%', 'f', ' ', ' ',
    '%', 'f', '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i17 = 0; i17 < 28; i17++) {
      cfmt[i17] = b_cfmt[i17];
    }

    fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

void d_fprintf(real_T fileID)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[16];
  int32_T i18;
  static const char_T b_cfmt[16] = { ' ', ' ', ' ', ' ', 'o', 'u', 't', 'e', 'r',
    ' ', 'l', 'o', 'o', 'p', '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i18 = 0; i18 < 16; i18++) {
      cfmt[i18] = b_cfmt[i18];
    }

    fprintf(filestar, cfmt);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

void e_fprintf(real_T fileID, real_T varargin_1, real_T varargin_2, real_T
               varargin_3)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[27];
  int32_T i19;
  static const char_T b_cfmt[27] = { ' ', ' ', ' ', ' ', ' ', ' ', 'v', 'e', 'r',
    't', 'e', 'x', ' ', ' ', ' ', '%', 'f', ' ', ' ', '%', 'f', ' ', ' ', '%',
    'f', '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i19 = 0; i19 < 27; i19++) {
      cfmt[i19] = b_cfmt[i19];
    }

    fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

void f_fprintf(real_T fileID)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[13];
  int32_T i20;
  static const char_T b_cfmt[13] = { ' ', ' ', ' ', ' ', 'e', 'n', 'd', 'l', 'o',
    'o', 'p', '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i20 = 0; i20 < 13; i20++) {
      cfmt[i20] = b_cfmt[i20];
    }

    fprintf(filestar, cfmt);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

void g_fprintf(real_T fileID)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[12];
  int32_T i21;
  static const char_T b_cfmt[12] = { ' ', ' ', 'e', 'n', 'd', 'f', 'a', 'c', 'e',
    't', '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i21 = 0; i21 < 12; i21++) {
      cfmt[i21] = b_cfmt[i21];
    }

    fprintf(filestar, cfmt);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

void h_fprintf(real_T fileID)
{
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  char_T cfmt[14];
  int32_T i22;
  static const char_T b_cfmt[14] = { 'e', 'n', 'd', 's', 'o', 'l', 'i', 'd', ' ',
    'v', 'c', 'g', '\x0a', '\x00' };

  b_NULL = NULL;
  b_fileManager(fileID, &filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    for (i22 = 0; i22 < 14; i22++) {
      cfmt[i22] = b_cfmt[i22];
    }

    fprintf(filestar, cfmt);
    if (autoflush) {
      fflush(filestar);
    }
  }
}

/* End of code generation (fprintf.cpp) */
