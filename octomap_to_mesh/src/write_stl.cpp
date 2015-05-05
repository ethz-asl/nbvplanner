/*
 * write_stl.cpp
 *
 * Code generation for function 'write_stl'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/write_stl.h"
#include "octomap_to_mesh/fprintf.h"
#include "octomap_to_mesh/fclose.h"
#include "octomap_to_mesh/fopen.h"
#include <stdio.h>

/* Function Definitions */
void write_stl(const emxArray_real_T *vertices, const emxArray_real_T *faces,
               const char_T * fname_stl_data, const int32_T fname_stl_size[2])
{
  real_T fileID;
  int32_T i;
  int32_T b_faces;
  int32_T c_faces;
  real_T a[3];
  int32_T j;
  real_T b[3];
  int32_T d_faces;
  fileID = b_fopen(fname_stl_data, fname_stl_size);
  b_fprintf(fileID);
  for (i = 0; i < faces->size[0]; i++) {
    b_faces = (int32_T)faces->data[i + faces->size[0]];
    c_faces = (int32_T)faces->data[i];
    for (j = 0; j < 3; j++) {
      a[j] = vertices->data[(b_faces + vertices->size[0] * j) - 1] -
        vertices->data[(c_faces + vertices->size[0] * j) - 1];
    }

    b_faces = (int32_T)faces->data[i + (faces->size[0] << 1)];
    c_faces = (int32_T)faces->data[i];
    for (j = 0; j < 3; j++) {
      b[j] = vertices->data[(b_faces + vertices->size[0] * j) - 1] -
        vertices->data[(c_faces + vertices->size[0] * j) - 1];
    }

    c_fprintf(fileID, a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0]
              * b[1] - a[1] * b[0]);
    d_fprintf(fileID);
    for (j = 0; j < 3; j++) {
      b_faces = (int32_T)faces->data[i + faces->size[0] * j];
      c_faces = (int32_T)faces->data[i + faces->size[0] * j];
      d_faces = (int32_T)faces->data[i + faces->size[0] * j];
      e_fprintf(fileID, vertices->data[b_faces - 1], vertices->data[(c_faces +
                 vertices->size[0]) - 1], vertices->data[(d_faces +
                 (vertices->size[0] << 1)) - 1]);
    }

    f_fprintf(fileID);
    g_fprintf(fileID);
  }

  h_fprintf(fileID);
  b_fclose(fileID);
}

/* End of code generation (write_stl.cpp) */
