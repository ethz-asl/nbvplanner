/*
 * OctomapToMesh.cpp
 *
 * Code generation for function 'OctomapToMesh'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/sum.h"
#include "octomap_to_mesh/ind2sub.h"
#include "octomap_to_mesh/OctomapToMesh_emxutil.h"
#include "octomap_to_mesh/write_stl.h"
#include "octomap_to_mesh/retrieve_meshformat.h"
#include "octomap_to_mesh/circshift.h"
#include "octomap_to_mesh/cat.h"
#include <octomap_to_mesh/OctomapToMesh_struct.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <assert.h>
#include <fstream>


//extern std::string pkgPath;

/* Function Definitions */
void OctomapToMesh(OctomapToMesh_T * data, const char_T *fname_ply_data, struct_T *MeshedOctomap) {

  std::string pkgPath = ros::package::getPath("octomap_to_mesh");
  
  real_T  *octomap_gridX_data = data->octomap_gridX_data;
  int32_T *octomap_gridX_size = data->octomap_gridX_size;
  real_T  *octomap_gridY_data = data->octomap_gridY_data;
  int32_T *octomap_gridY_size = data->octomap_gridY_size;
  real_T  *octomap_gridZ_data = data->octomap_gridZ_data;
  int32_T *octomap_gridZ_size = data->octomap_gridZ_size;
  emxArray_real_T *octomap_voxels_map = data->octomap_voxels_map;
  
  int32_T fname_ply_size[2];
  fname_ply_size[0] = 1;
  fname_ply_size[1] = (int32_T)strlen(fname_ply_data);
  
  assert(octomap_gridX_size);
  assert(octomap_gridX_size[0] == 1);
  assert(octomap_gridY_size);
  assert(octomap_gridY_size[0] == 1);
  assert(octomap_gridZ_size);
  assert(octomap_gridZ_size[0] == 1);
  int32_T NN = octomap_gridX_size[1];
  int32_T KK = octomap_gridY_size[1];
  int32_T MM = octomap_gridZ_size[1];
  assert(fname_ply_size);
  assert(fname_ply_size[0] == 1);
  emxArray_boolean_T *b_octomap_voxels_map;
  int32_T i0;
  int32_T loop_ub;
  emxArray_boolean_T *x;
  emxArray_int32_T *ii;
  int32_T nx;
  int32_T idx;
  int32_T facetcountthisvoxel;
  boolean_T exitg2;
  boolean_T guard2 = FALSE;
  emxArray_int32_T *b_ii;
  emxArray_real_T *c_ii;
  real_T octomap_gridX[3];
  emxArray_real_T *objectX;
  emxArray_real_T *objectY;
  emxArray_real_T *objectZ;
  real_T facetcount;
  real_T mtmp;
  int32_T i1;
  emxArray_boolean_T *c_octomap_voxels_map;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T tmp_data[NN+1];
  real_T b_octomap_gridX_data[NN];
  emxArray_boolean_T *d_octomap_voxels_map;
  int32_T b_tmp_data[KK+1];
  real_T b_octomap_gridY_data[KK];
  emxArray_boolean_T *e_octomap_voxels_map;
  int32_T c_tmp_data[MM+1];
  real_T b_octomap_gridZ_data[MM];
  int32_T d_tmp_data[NN];
  int32_T e_tmp_data[NN-1];
  real_T octomap_gridXsteps_data[NN-1];
  real_T octomap_gridXlower_data[NN];
  real_T octomap_gridXupper_data[NN];
  int32_T f_tmp_data[KK];
  int32_T g_tmp_data[KK-1];
  real_T octomap_gridYsteps_data[KK-1];
  real_T octomap_gridYlower_data[KK];
  real_T A_data[NN];
  real_T octomap_gridYupper_data[KK];
  int32_T h_tmp_data[MM];
  int32_T i_tmp_data[MM-1];
  real_T octomap_gridZsteps_data[MM-1];
  real_T octomap_gridZlower_data[MM];
  real_T octomap_gridZupper_data[MM];
  int8_T iv0[3];
  emxArray_real_T *octomap_voxels_map_shifted;
  emxArray_real_T *octomap_voxels_map_withborder;
  int32_T tmp_size[3];
  emxArray_real_T *r0;
  int32_T b_tmp_size[3];
  emxArray_real_T *r1;
  emxArray_real_T *r2;
  emxArray_real_T *b_octomap_voxels_map_shifted;
  int32_T c_tmp_size[3];
  emxArray_real_T *r3;
  int32_T d_tmp_size[3];
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *c_octomap_voxels_map_shifted;
  int32_T e_tmp_size[2];
  int32_T f_tmp_size[2];
  emxArray_real_T *r6;
  int32_T g_tmp_size[2];
  int32_T h_tmp_size[2];
  emxArray_real_T *r7;
  emxArray_real_T *r8;
  emxArray_real_T *d_octomap_voxels_map_shifted;
  boolean_T exitg1;
  boolean_T guard1 = FALSE;
  emxArray_int32_T *d_ii;
  emxArray_real_T *edgevoxelindices;
  emxArray_real_T *e_ii;
  emxArray_real_T *f_ii;
  emxArray_real_T *e_octomap_voxels_map_shifted;
  emxArray_boolean_T *neighbourlist;
  emxArray_real_T *meshXYZ;
  real_T facetcountsofar;
  real_T subZ;
  boolean_T b_neighbourlist[6];
  real_T y;
  real_T facetCOtemp_data[108];
  emxArray_real_T *normalsXYZ;
  emxArray_real_T *vertices;
  b_emxInit_boolean_T(&b_octomap_voxels_map, 3);

  /*  */
  /*    Syntax:  */
  /*    MeshedOctomap = OctomapToMesh(octomap_voxels_map,octomap_gridX,octomap_gridY,octomap_gridZ,fname_stl)  */
  /*  */
  /*    Inputs: */
  /*    octomap_voxels_map  : octomap NxMxK matrix with "1s" when the voxel is */
  /*                          occupued */
  /*    octomap_gridX       : x components of the octomap  */
  /*    octomap_gridY       : y components of the octomap */
  /*    octomap_gridZ       : z components of the octomap */
  /*    fname_ply           : save *.PLY of the mesh */
  /*  */
  /*    Outputs: */
  /*    MeshedOctomap       : structure with elements .faces, .vertices of the  */
  /*                          reconstructed mesh */
  /*  */
  /*    Authors: */
  /*    Kostas Alexis (konstantinos.alexis@mavt.ethz.ch) */
  /*  */
  i0 = b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
    b_octomap_voxels_map->size[2];
  b_octomap_voxels_map->size[0] = octomap_voxels_map->size[0];
  b_octomap_voxels_map->size[1] = octomap_voxels_map->size[1];
  b_octomap_voxels_map->size[2] = octomap_voxels_map->size[2];
  emxEnsureCapacity((emxArray__common *)b_octomap_voxels_map, i0, (int32_T)
                    sizeof(boolean_T));
  loop_ub = octomap_voxels_map->size[0] * octomap_voxels_map->size[1] *
    octomap_voxels_map->size[2];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_octomap_voxels_map->data[i0] = (octomap_voxels_map->data[i0] != 0.0);
  }

  b_emxInit_boolean_T(&x, 3);

  /*    Remove outer unused areas from the octomap voxel grid */
  i0 = x->size[0] * x->size[1] * x->size[2];
  x->size[0] = b_octomap_voxels_map->size[0];
  x->size[1] = b_octomap_voxels_map->size[1];
  x->size[2] = b_octomap_voxels_map->size[2];
  emxEnsureCapacity((emxArray__common *)x, i0, (int32_T)sizeof(boolean_T));
  loop_ub = b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
    b_octomap_voxels_map->size[2];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (b_octomap_voxels_map->data[i0] == 1);
  }

  emxInit_int32_T(&ii, 1);
  nx = x->size[0] * x->size[1] * x->size[2];
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = nx;
  emxEnsureCapacity((emxArray__common *)ii, i0, (int32_T)sizeof(int32_T));
  facetcountthisvoxel = 1;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && (facetcountthisvoxel <= nx)) {
    guard2 = FALSE;
    if (x->data[facetcountthisvoxel - 1]) {
      idx++;
      ii->data[idx - 1] = facetcountthisvoxel;
      if (idx >= nx) {
        exitg2 = TRUE;
      } else {
        guard2 = TRUE;
      }
    } else {
      guard2 = TRUE;
    }

    if (guard2 == TRUE) {
      facetcountthisvoxel++;
    }
  }

  if (nx == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity((emxArray__common *)ii, i0, (int32_T)sizeof(int32_T));
    }
  } else {
    if (1 > idx) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    emxInit_int32_T(&b_ii, 1);
    i0 = b_ii->size[0];
    b_ii->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)b_ii, i0, (int32_T)sizeof(int32_T));
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_ii->data[i0] = ii->data[i0];
    }

    i0 = ii->size[0];
    ii->size[0] = b_ii->size[0];
    emxEnsureCapacity((emxArray__common *)ii, i0, (int32_T)sizeof(int32_T));
    loop_ub = b_ii->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      ii->data[i0] = b_ii->data[i0];
    }

    emxFree_int32_T(&b_ii);
  }

  c_emxInit_real_T(&c_ii, 1);
  octomap_gridX[0] = (int8_T)octomap_gridX_size[1];
  octomap_gridX[1] = (int8_T)octomap_gridY_size[1];
  octomap_gridX[2] = (int8_T)octomap_gridZ_size[1];
  i0 = c_ii->size[0];
  c_ii->size[0] = ii->size[0];
  emxEnsureCapacity((emxArray__common *)c_ii, i0, (int32_T)sizeof(real_T));
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_ii->data[i0] = ii->data[i0];
  }

  c_emxInit_real_T(&objectX, 1);
  c_emxInit_real_T(&objectY, 1);
  c_emxInit_real_T(&objectZ, 1);
  ind2sub(octomap_gridX, c_ii, objectX, objectY, objectZ);
  emxFree_real_T(&c_ii);
  if (objectX->data[0] != objectX->data[objectX->size[0] - 1]) {
    facetcount = objectX->data[0];
    if ((objectX->size[0] > 1) && (1 < objectX->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectX->size[0];
           facetcountthisvoxel++) {
        if (objectX->data[facetcountthisvoxel] < facetcount) {
          facetcount = objectX->data[facetcountthisvoxel];
        }
      }
    }

    mtmp = objectX->data[0];
    if ((objectX->size[0] > 1) && (1 < objectX->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectX->size[0];
           facetcountthisvoxel++) {
        if (objectX->data[facetcountthisvoxel] > mtmp) {
          mtmp = objectX->data[facetcountthisvoxel];
        }
      }
    }

    if (facetcount > mtmp) {
      i0 = 0;
      i1 = 0;
    } else {
      i0 = (int32_T)facetcount - 1;
      i1 = (int32_T)mtmp;
    }

    b_emxInit_boolean_T(&c_octomap_voxels_map, 3);
    facetcountthisvoxel = b_octomap_voxels_map->size[1];
    nx = b_octomap_voxels_map->size[2];
    i2 = c_octomap_voxels_map->size[0] * c_octomap_voxels_map->size[1] *
      c_octomap_voxels_map->size[2];
    c_octomap_voxels_map->size[0] = i1 - i0;
    c_octomap_voxels_map->size[1] = facetcountthisvoxel;
    c_octomap_voxels_map->size[2] = nx;
    emxEnsureCapacity((emxArray__common *)c_octomap_voxels_map, i2, (int32_T)
                      sizeof(boolean_T));
    for (i2 = 0; i2 < nx; i2++) {
      for (i3 = 0; i3 < facetcountthisvoxel; i3++) {
        loop_ub = i1 - i0;
        for (i4 = 0; i4 < loop_ub; i4++) {
          c_octomap_voxels_map->data[(i4 + c_octomap_voxels_map->size[0] * i3) +
            c_octomap_voxels_map->size[0] * c_octomap_voxels_map->size[1] * i2] =
            b_octomap_voxels_map->data[((i0 + i4) + b_octomap_voxels_map->size[0]
            * i3) + b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1]
            * i2];
        }
      }
    }

    i0 = b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
      b_octomap_voxels_map->size[2];
    b_octomap_voxels_map->size[0] = c_octomap_voxels_map->size[0];
    b_octomap_voxels_map->size[1] = c_octomap_voxels_map->size[1];
    b_octomap_voxels_map->size[2] = c_octomap_voxels_map->size[2];
    emxEnsureCapacity((emxArray__common *)b_octomap_voxels_map, i0, (int32_T)
                      sizeof(boolean_T));
    loop_ub = c_octomap_voxels_map->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      nx = c_octomap_voxels_map->size[1];
      for (i1 = 0; i1 < nx; i1++) {
        facetcountthisvoxel = c_octomap_voxels_map->size[0];
        for (i2 = 0; i2 < facetcountthisvoxel; i2++) {
          b_octomap_voxels_map->data[(i2 + b_octomap_voxels_map->size[0] * i1) +
            b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] * i0] =
            c_octomap_voxels_map->data[(i2 + c_octomap_voxels_map->size[0] * i1)
            + c_octomap_voxels_map->size[0] * c_octomap_voxels_map->size[1] * i0];
        }
      }
    }

    emxFree_boolean_T(&c_octomap_voxels_map);
    facetcount = objectX->data[0];
    if ((objectX->size[0] > 1) && (1 < objectX->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectX->size[0];
           facetcountthisvoxel++) {
        if (objectX->data[facetcountthisvoxel] < facetcount) {
          facetcount = objectX->data[facetcountthisvoxel];
        }
      }
    }

    mtmp = objectX->data[0];
    if ((objectX->size[0] > 1) && (1 < objectX->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectX->size[0];
           facetcountthisvoxel++) {
        if (objectX->data[facetcountthisvoxel] > mtmp) {
          mtmp = objectX->data[facetcountthisvoxel];
        }
      }
    }

    if (facetcount > mtmp) {
      i0 = 1;
      i1 = 0;
    } else {
      i0 = (int32_T)facetcount;
      i1 = (int32_T)mtmp;
    }

    loop_ub = i1 - i0;
    for (i2 = 0; i2 <= loop_ub; i2++) {
      tmp_data[i2] = i0 + i2;
    }

    facetcountthisvoxel = (i1 - i0) + 1;
    loop_ub = (i1 - i0) + 1;
    for (i0 = 0; i0 < loop_ub; i0++) {
      i1 = 0;
      while (i1 <= 0) {
        b_octomap_gridX_data[i0] = octomap_gridX_data[tmp_data[i0] - 1];
        i1 = 1;
      }
    }

    octomap_gridX_size[0] = 1;
    octomap_gridX_size[1] = facetcountthisvoxel;
    for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
      octomap_gridX_data[i0] = b_octomap_gridX_data[i0];
    }
  }

  emxFree_real_T(&objectX);
  if (objectY->data[0] != objectY->data[objectY->size[0] - 1]) {
    facetcount = objectY->data[0];
    if ((objectY->size[0] > 1) && (1 < objectY->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectY->size[0];
           facetcountthisvoxel++) {
        if (objectY->data[facetcountthisvoxel] < facetcount) {
          facetcount = objectY->data[facetcountthisvoxel];
        }
      }
    }

    mtmp = objectY->data[0];
    if ((objectY->size[0] > 1) && (1 < objectY->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectY->size[0];
           facetcountthisvoxel++) {
        if (objectY->data[facetcountthisvoxel] > mtmp) {
          mtmp = objectY->data[facetcountthisvoxel];
        }
      }
    }

    if (facetcount > mtmp) {
      i0 = 0;
      i1 = 0;
    } else {
      i0 = (int32_T)facetcount - 1;
      i1 = (int32_T)mtmp;
    }

    b_emxInit_boolean_T(&d_octomap_voxels_map, 3);
    facetcountthisvoxel = b_octomap_voxels_map->size[0];
    nx = b_octomap_voxels_map->size[2];
    i2 = d_octomap_voxels_map->size[0] * d_octomap_voxels_map->size[1] *
      d_octomap_voxels_map->size[2];
    d_octomap_voxels_map->size[0] = facetcountthisvoxel;
    d_octomap_voxels_map->size[1] = i1 - i0;
    d_octomap_voxels_map->size[2] = nx;
    emxEnsureCapacity((emxArray__common *)d_octomap_voxels_map, i2, (int32_T)
                      sizeof(boolean_T));
    for (i2 = 0; i2 < nx; i2++) {
      loop_ub = i1 - i0;
      for (i3 = 0; i3 < loop_ub; i3++) {
        for (i4 = 0; i4 < facetcountthisvoxel; i4++) {
          d_octomap_voxels_map->data[(i4 + d_octomap_voxels_map->size[0] * i3) +
            d_octomap_voxels_map->size[0] * d_octomap_voxels_map->size[1] * i2] =
            b_octomap_voxels_map->data[(i4 + b_octomap_voxels_map->size[0] * (i0
            + i3)) + b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size
            [1] * i2];
        }
      }
    }

    i0 = b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
      b_octomap_voxels_map->size[2];
    b_octomap_voxels_map->size[0] = d_octomap_voxels_map->size[0];
    b_octomap_voxels_map->size[1] = d_octomap_voxels_map->size[1];
    b_octomap_voxels_map->size[2] = d_octomap_voxels_map->size[2];
    emxEnsureCapacity((emxArray__common *)b_octomap_voxels_map, i0, (int32_T)
                      sizeof(boolean_T));
    loop_ub = d_octomap_voxels_map->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      nx = d_octomap_voxels_map->size[1];
      for (i1 = 0; i1 < nx; i1++) {
        facetcountthisvoxel = d_octomap_voxels_map->size[0];
        for (i2 = 0; i2 < facetcountthisvoxel; i2++) {
          b_octomap_voxels_map->data[(i2 + b_octomap_voxels_map->size[0] * i1) +
            b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] * i0] =
            d_octomap_voxels_map->data[(i2 + d_octomap_voxels_map->size[0] * i1)
            + d_octomap_voxels_map->size[0] * d_octomap_voxels_map->size[1] * i0];
        }
      }
    }

    emxFree_boolean_T(&d_octomap_voxels_map);
    facetcount = objectY->data[0];
    if ((objectY->size[0] > 1) && (1 < objectY->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectY->size[0];
           facetcountthisvoxel++) {
        if (objectY->data[facetcountthisvoxel] < facetcount) {
          facetcount = objectY->data[facetcountthisvoxel];
        }
      }
    }

    mtmp = objectY->data[0];
    if ((objectY->size[0] > 1) && (1 < objectY->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectY->size[0];
           facetcountthisvoxel++) {
        if (objectY->data[facetcountthisvoxel] > mtmp) {
          mtmp = objectY->data[facetcountthisvoxel];
        }
      }
    }

    if (facetcount > mtmp) {
      i0 = 1;
      i1 = 0;
    } else {
      i0 = (int32_T)facetcount;
      i1 = (int32_T)mtmp;
    }

    loop_ub = i1 - i0;
    for (i2 = 0; i2 <= loop_ub; i2++) {
      b_tmp_data[i2] = i0 + i2;
    }

    facetcountthisvoxel = (i1 - i0) + 1;
    loop_ub = (i1 - i0) + 1;
    for (i0 = 0; i0 < loop_ub; i0++) {
      i1 = 0;
      while (i1 <= 0) {
        b_octomap_gridY_data[i0] = octomap_gridY_data[b_tmp_data[i0] - 1];
        i1 = 1;
      }
    }

    octomap_gridY_size[0] = 1;
    octomap_gridY_size[1] = facetcountthisvoxel;
    for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
      octomap_gridY_data[i0] = b_octomap_gridY_data[i0];
    }
  }

  emxFree_real_T(&objectY);
  if (objectZ->data[0] != objectZ->data[objectZ->size[0] - 1]) {
    facetcount = objectZ->data[0];
    if ((objectZ->size[0] > 1) && (1 < objectZ->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectZ->size[0];
           facetcountthisvoxel++) {
        if (objectZ->data[facetcountthisvoxel] < facetcount) {
          facetcount = objectZ->data[facetcountthisvoxel];
        }
      }
    }

    mtmp = objectZ->data[0];
    if ((objectZ->size[0] > 1) && (1 < objectZ->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectZ->size[0];
           facetcountthisvoxel++) {
        if (objectZ->data[facetcountthisvoxel] > mtmp) {
          mtmp = objectZ->data[facetcountthisvoxel];
        }
      }
    }

    if (facetcount > mtmp) {
      i0 = 0;
      i1 = 0;
    } else {
      i0 = (int32_T)facetcount - 1;
      i1 = (int32_T)mtmp;
    }

    b_emxInit_boolean_T(&e_octomap_voxels_map, 3);
    facetcountthisvoxel = b_octomap_voxels_map->size[0];
    nx = b_octomap_voxels_map->size[1];
    i2 = e_octomap_voxels_map->size[0] * e_octomap_voxels_map->size[1] *
      e_octomap_voxels_map->size[2];
    e_octomap_voxels_map->size[0] = facetcountthisvoxel;
    e_octomap_voxels_map->size[1] = nx;
    e_octomap_voxels_map->size[2] = i1 - i0;
    emxEnsureCapacity((emxArray__common *)e_octomap_voxels_map, i2, (int32_T)
                      sizeof(boolean_T));
    loop_ub = i1 - i0;
    for (i1 = 0; i1 < loop_ub; i1++) {
      for (i2 = 0; i2 < nx; i2++) {
        for (i3 = 0; i3 < facetcountthisvoxel; i3++) {
          e_octomap_voxels_map->data[(i3 + e_octomap_voxels_map->size[0] * i2) +
            e_octomap_voxels_map->size[0] * e_octomap_voxels_map->size[1] * i1] =
            b_octomap_voxels_map->data[(i3 + b_octomap_voxels_map->size[0] * i2)
            + b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
            (i0 + i1)];
        }
      }
    }

    i0 = b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
      b_octomap_voxels_map->size[2];
    b_octomap_voxels_map->size[0] = e_octomap_voxels_map->size[0];
    b_octomap_voxels_map->size[1] = e_octomap_voxels_map->size[1];
    b_octomap_voxels_map->size[2] = e_octomap_voxels_map->size[2];
    emxEnsureCapacity((emxArray__common *)b_octomap_voxels_map, i0, (int32_T)
                      sizeof(boolean_T));
    loop_ub = e_octomap_voxels_map->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      nx = e_octomap_voxels_map->size[1];
      for (i1 = 0; i1 < nx; i1++) {
        facetcountthisvoxel = e_octomap_voxels_map->size[0];
        for (i2 = 0; i2 < facetcountthisvoxel; i2++) {
          b_octomap_voxels_map->data[(i2 + b_octomap_voxels_map->size[0] * i1) +
            b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] * i0] =
            e_octomap_voxels_map->data[(i2 + e_octomap_voxels_map->size[0] * i1)
            + e_octomap_voxels_map->size[0] * e_octomap_voxels_map->size[1] * i0];
        }
      }
    }

    emxFree_boolean_T(&e_octomap_voxels_map);
    facetcount = objectZ->data[0];
    if ((objectZ->size[0] > 1) && (1 < objectZ->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectZ->size[0];
           facetcountthisvoxel++) {
        if (objectZ->data[facetcountthisvoxel] < facetcount) {
          facetcount = objectZ->data[facetcountthisvoxel];
        }
      }
    }

    mtmp = objectZ->data[0];
    if ((objectZ->size[0] > 1) && (1 < objectZ->size[0])) {
      for (facetcountthisvoxel = 1; facetcountthisvoxel + 1 <= objectZ->size[0];
           facetcountthisvoxel++) {
        if (objectZ->data[facetcountthisvoxel] > mtmp) {
          mtmp = objectZ->data[facetcountthisvoxel];
        }
      }
    }

    if (facetcount > mtmp) {
      i0 = 1;
      i1 = 0;
    } else {
      i0 = (int32_T)facetcount;
      i1 = (int32_T)mtmp;
    }

    loop_ub = i1 - i0;
    for (i2 = 0; i2 <= loop_ub; i2++) {
      c_tmp_data[i2] = i0 + i2;
    }

    facetcountthisvoxel = (i1 - i0) + 1;
    loop_ub = (i1 - i0) + 1;
    for (i0 = 0; i0 < loop_ub; i0++) {
      i1 = 0;
      while (i1 <= 0) {
        b_octomap_gridZ_data[i0] = octomap_gridZ_data[c_tmp_data[i0] - 1];
        i1 = 1;
      }
    }

    octomap_gridZ_size[0] = 1;
    octomap_gridZ_size[1] = facetcountthisvoxel;
    for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
      octomap_gridZ_data[i0] = b_octomap_gridZ_data[i0];
    }
  }

  emxFree_real_T(&objectZ);

  /*    compute the lower and upper limits of each voxel */
  if (2 > octomap_gridX_size[1]) {
    i0 = 1;
    i1 = 0;
  } else {
    i0 = 2;
    i1 = octomap_gridX_size[1];
  }

  if (1 > octomap_gridX_size[1] - 1) {
    loop_ub = -1;
  } else {
    loop_ub = octomap_gridX_size[1] - 2;
  }

  facetcountthisvoxel = (i1 - i0) + 1;
  nx = i1 - i0;
  for (i1 = 0; i1 <= nx; i1++) {
    d_tmp_data[i1] = i0 + i1;
  }

  for (i0 = 0; i0 <= loop_ub; i0++) {
    e_tmp_data[i0] = 1 + i0;
  }

  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    octomap_gridXsteps_data[i0] = octomap_gridX_data[d_tmp_data[i0] - 1] -
      octomap_gridX_data[e_tmp_data[i0] - 1];
  }

  b_octomap_gridX_data[0] = octomap_gridXsteps_data[0];
  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    b_octomap_gridX_data[i0 + 1] = octomap_gridXsteps_data[i0];
  }

  loop_ub = octomap_gridX_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_gridXlower_data[i0] = octomap_gridX_data[octomap_gridX_size[0] * i0]
      - b_octomap_gridX_data[i0] / 2.0;
  }

  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    octomap_gridXupper_data[i0] = octomap_gridXsteps_data[i0];
  }

  octomap_gridXupper_data[facetcountthisvoxel] =
    octomap_gridXsteps_data[facetcountthisvoxel - 1];
  loop_ub = octomap_gridX_size[0] * octomap_gridX_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_gridXupper_data[i0] = octomap_gridX_data[i0] +
      octomap_gridXupper_data[i0] / 2.0;
  }

  if (2 > octomap_gridY_size[1]) {
    i0 = 1;
    i1 = 0;
  } else {
    i0 = 2;
    i1 = octomap_gridY_size[1];
  }

  if (1 > octomap_gridY_size[1] - 1) {
    loop_ub = -1;
  } else {
    loop_ub = octomap_gridY_size[1] - 2;
  }

  facetcountthisvoxel = (i1 - i0) + 1;
  nx = i1 - i0;
  for (i1 = 0; i1 <= nx; i1++) {
    f_tmp_data[i1] = i0 + i1;
  }

  for (i0 = 0; i0 <= loop_ub; i0++) {
    g_tmp_data[i0] = 1 + i0;
  }

  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    octomap_gridYsteps_data[i0] = octomap_gridY_data[f_tmp_data[i0] - 1] -
      octomap_gridY_data[g_tmp_data[i0] - 1];
  }

  b_octomap_gridX_data[0] = octomap_gridYsteps_data[0];
  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    b_octomap_gridX_data[i0 + 1] = octomap_gridYsteps_data[i0];
  }

  loop_ub = octomap_gridY_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_gridYlower_data[i0] = octomap_gridY_data[octomap_gridY_size[0] * i0]
      - b_octomap_gridX_data[i0] / 2.0;
  }

  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    A_data[i0] = octomap_gridYsteps_data[i0];
  }

  A_data[facetcountthisvoxel] = octomap_gridYsteps_data[facetcountthisvoxel - 1];
  loop_ub = octomap_gridY_size[0] * octomap_gridY_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_gridYupper_data[i0] = octomap_gridY_data[i0] + A_data[i0] / 2.0;
  }

  if (2 > octomap_gridZ_size[1]) {
    i0 = 1;
    i1 = 0;
  } else {
    i0 = 2;
    i1 = octomap_gridZ_size[1];
  }

  if (1 > octomap_gridZ_size[1] - 1) {
    loop_ub = -1;
  } else {
    loop_ub = octomap_gridZ_size[1] - 2;
  }

  facetcountthisvoxel = (i1 - i0) + 1;
  nx = i1 - i0;
  for (i1 = 0; i1 <= nx; i1++) {
    h_tmp_data[i1] = i0 + i1;
  }

  for (i0 = 0; i0 <= loop_ub; i0++) {
    i_tmp_data[i0] = 1 + i0;
  }

  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    octomap_gridZsteps_data[i0] = octomap_gridZ_data[h_tmp_data[i0] - 1] -
      octomap_gridZ_data[i_tmp_data[i0] - 1];
  }

  b_octomap_gridX_data[0] = octomap_gridZsteps_data[0];
  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    b_octomap_gridX_data[i0 + 1] = octomap_gridZsteps_data[i0];
  }

  loop_ub = octomap_gridZ_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_gridZlower_data[i0] = octomap_gridZ_data[octomap_gridZ_size[0] * i0]
      - b_octomap_gridX_data[i0] / 2.0;
  }

  for (i0 = 0; i0 < facetcountthisvoxel; i0++) {
    A_data[i0] = octomap_gridZsteps_data[i0];
  }

  A_data[facetcountthisvoxel] = octomap_gridZsteps_data[facetcountthisvoxel - 1];
  loop_ub = octomap_gridZ_size[0] * octomap_gridZ_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_gridZupper_data[i0] = octomap_gridZ_data[i0] + A_data[i0] / 2.0;
  }

  /*    check dims of the grid */
  /*    for each voxel, identify whethers its 6 neighbors are within the */
  /*    object. If any neighbour is outside the object, draw facets between the */
  /*    voxel and that neighbor.  */
  for (i0 = 0; i0 < 3; i0++) {
    iv0[i0] = (int8_T)b_octomap_voxels_map->size[i0];
  }

  emxInit_real_T(&octomap_voxels_map_shifted, 3);
  i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size[1]
    * octomap_voxels_map_shifted->size[2];
  octomap_voxels_map_shifted->size[0] = iv0[0];
  emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0, (int32_T)
                    sizeof(real_T));
  i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size[1]
    * octomap_voxels_map_shifted->size[2];
  octomap_voxels_map_shifted->size[1] = iv0[1];
  emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0, (int32_T)
                    sizeof(real_T));
  i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size[1]
    * octomap_voxels_map_shifted->size[2];
  octomap_voxels_map_shifted->size[2] = iv0[2];
  emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0, (int32_T)
                    sizeof(real_T));
  loop_ub = iv0[0] * iv0[1] * iv0[2];
  for (i0 = 0; i0 < loop_ub; i0++) {
    octomap_voxels_map_shifted->data[i0] = 0.0;
  }

  emxInit_real_T(&octomap_voxels_map_withborder, 3);
  if (octomap_gridX_size[1] > 2) {
    tmp_size[0] = 1;
    tmp_size[1] = octomap_gridY_size[1];
    tmp_size[2] = octomap_gridZ_size[1];
    emxInit_real_T(&r0, 3);
    cat(tmp_size, b_octomap_voxels_map, r0);
    i0 = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    octomap_voxels_map_withborder->size[0] = r0->size[0];
    octomap_voxels_map_withborder->size[1] = r0->size[1];
    octomap_voxels_map_withborder->size[2] = r0->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_withborder, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = r0->size[0] * r0->size[1] * r0->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_withborder->data[i0] = r0->data[i0];
    }

    /* Add border */
    b_tmp_size[0] = 1;
    b_tmp_size[1] = octomap_gridY_size[1];
    b_tmp_size[2] = octomap_gridZ_size[1];
    emxInit_real_T(&r1, 3);
    i0 = r1->size[0] * r1->size[1] * r1->size[2];
    r1->size[0] = iv0[0];
    emxEnsureCapacity((emxArray__common *)r1, i0, (int32_T)sizeof(real_T));
    i0 = r1->size[0] * r1->size[1] * r1->size[2];
    r1->size[1] = iv0[1];
    emxEnsureCapacity((emxArray__common *)r1, i0, (int32_T)sizeof(real_T));
    i0 = r1->size[0] * r1->size[1] * r1->size[2];
    r1->size[2] = iv0[2];
    emxEnsureCapacity((emxArray__common *)r1, i0, (int32_T)sizeof(real_T));
    loop_ub = iv0[0] * iv0[1] * iv0[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r1->data[i0] = 0.0;
    }

    b_cat(b_tmp_size, r1, r0);
    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    octomap_voxels_map_shifted->size[0] = r0->size[0];
    octomap_voxels_map_shifted->size[1] = r0->size[1];
    octomap_voxels_map_shifted->size[2] = r0->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = r0->size[0] * r0->size[1] * r0->size[2];
    emxFree_real_T(&r1);
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_shifted->data[i0] = r0->data[i0];
    }

    /* Add border */
    i0 = r0->size[0] * r0->size[1] * r0->size[2];
    r0->size[0] = octomap_voxels_map_withborder->size[0];
    r0->size[1] = octomap_voxels_map_withborder->size[1];
    r0->size[2] = octomap_voxels_map_withborder->size[2];
    emxEnsureCapacity((emxArray__common *)r0, i0, (int32_T)sizeof(real_T));
    loop_ub = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r0->data[i0] = octomap_voxels_map_withborder->data[i0];
    }

    emxInit_real_T(&r2, 3);
    circshift(r0);
    i0 = r2->size[0] * r2->size[1] * r2->size[2];
    r2->size[0] = octomap_voxels_map_withborder->size[0];
    r2->size[1] = octomap_voxels_map_withborder->size[1];
    r2->size[2] = octomap_voxels_map_withborder->size[2];
    emxEnsureCapacity((emxArray__common *)r2, i0, (int32_T)sizeof(real_T));
    loop_ub = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r2->data[i0] = octomap_voxels_map_withborder->data[i0];
    }

    b_circshift(r2);
    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    nx = octomap_voxels_map_shifted->size[0];
    idx = octomap_voxels_map_shifted->size[1];
    facetcountthisvoxel = octomap_voxels_map_shifted->size[2];
    loop_ub = nx * idx * facetcountthisvoxel;
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_shifted->data[i0] = (octomap_voxels_map_shifted->
        data[i0] + r0->data[i0]) + r2->data[i0];
    }

    emxFree_real_T(&r0);
    emxFree_real_T(&r2);
    if (2 > octomap_voxels_map_shifted->size[0] - 1) {
      i0 = 0;
      i1 = 1;
    } else {
      i0 = 1;
      i1 = octomap_voxels_map_shifted->size[0];
    }

    emxInit_real_T(&b_octomap_voxels_map_shifted, 3);
    nx = octomap_voxels_map_shifted->size[1];
    idx = octomap_voxels_map_shifted->size[2];
    i2 = b_octomap_voxels_map_shifted->size[0] *
      b_octomap_voxels_map_shifted->size[1] * b_octomap_voxels_map_shifted->
      size[2];
    b_octomap_voxels_map_shifted->size[0] = (i1 - i0) - 1;
    b_octomap_voxels_map_shifted->size[1] = nx;
    b_octomap_voxels_map_shifted->size[2] = idx;
    emxEnsureCapacity((emxArray__common *)b_octomap_voxels_map_shifted, i2,
                      (int32_T)sizeof(real_T));
    for (i2 = 0; i2 < idx; i2++) {
      for (i3 = 0; i3 < nx; i3++) {
        loop_ub = i1 - i0;
        for (i4 = 0; i4 <= loop_ub - 2; i4++) {
          b_octomap_voxels_map_shifted->data[(i4 +
            b_octomap_voxels_map_shifted->size[0] * i3) +
            b_octomap_voxels_map_shifted->size[0] *
            b_octomap_voxels_map_shifted->size[1] * i2] =
            octomap_voxels_map_shifted->data[((i0 + i4) +
            octomap_voxels_map_shifted->size[0] * i3) +
            octomap_voxels_map_shifted->size[0] *
            octomap_voxels_map_shifted->size[1] * i2];
        }
      }
    }

    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    octomap_voxels_map_shifted->size[0] = b_octomap_voxels_map_shifted->size[0];
    octomap_voxels_map_shifted->size[1] = b_octomap_voxels_map_shifted->size[1];
    octomap_voxels_map_shifted->size[2] = b_octomap_voxels_map_shifted->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = b_octomap_voxels_map_shifted->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      nx = b_octomap_voxels_map_shifted->size[1];
      for (i1 = 0; i1 < nx; i1++) {
        facetcountthisvoxel = b_octomap_voxels_map_shifted->size[0];
        for (i2 = 0; i2 < facetcountthisvoxel; i2++) {
          octomap_voxels_map_shifted->data[(i2 +
            octomap_voxels_map_shifted->size[0] * i1) +
            octomap_voxels_map_shifted->size[0] *
            octomap_voxels_map_shifted->size[1] * i0] =
            b_octomap_voxels_map_shifted->data[(i2 +
            b_octomap_voxels_map_shifted->size[0] * i1) +
            b_octomap_voxels_map_shifted->size[0] *
            b_octomap_voxels_map_shifted->size[1] * i0];
        }
      }
    }

    emxFree_real_T(&b_octomap_voxels_map_shifted);

    /* Remove border */
  }

  if (octomap_gridY_size[1] > 2) {
    c_tmp_size[0] = octomap_gridX_size[1];
    c_tmp_size[1] = 1;
    c_tmp_size[2] = octomap_gridZ_size[1];
    emxInit_real_T(&r3, 3);
    c_cat(c_tmp_size, b_octomap_voxels_map, r3);
    i0 = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    octomap_voxels_map_withborder->size[0] = r3->size[0];
    octomap_voxels_map_withborder->size[1] = r3->size[1];
    octomap_voxels_map_withborder->size[2] = r3->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_withborder, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = r3->size[0] * r3->size[1] * r3->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_withborder->data[i0] = r3->data[i0];
    }

    /* Add border */
    d_tmp_size[0] = octomap_gridX_size[1];
    d_tmp_size[1] = 1;
    d_tmp_size[2] = octomap_gridZ_size[1];
    emxInit_real_T(&r4, 3);
    d_cat(d_tmp_size, octomap_voxels_map_shifted, r4);
    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    octomap_voxels_map_shifted->size[0] = r4->size[0];
    octomap_voxels_map_shifted->size[1] = r4->size[1];
    octomap_voxels_map_shifted->size[2] = r4->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = r4->size[0] * r4->size[1] * r4->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_shifted->data[i0] = r4->data[i0];
    }

    emxFree_real_T(&r4);

    /* Add border */
    i0 = r3->size[0] * r3->size[1] * r3->size[2];
    r3->size[0] = octomap_voxels_map_withborder->size[0];
    r3->size[1] = octomap_voxels_map_withborder->size[1];
    r3->size[2] = octomap_voxels_map_withborder->size[2];
    emxEnsureCapacity((emxArray__common *)r3, i0, (int32_T)sizeof(real_T));
    loop_ub = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r3->data[i0] = octomap_voxels_map_withborder->data[i0];
    }

    emxInit_real_T(&r5, 3);
    c_circshift(r3);
    i0 = r5->size[0] * r5->size[1] * r5->size[2];
    r5->size[0] = octomap_voxels_map_withborder->size[0];
    r5->size[1] = octomap_voxels_map_withborder->size[1];
    r5->size[2] = octomap_voxels_map_withborder->size[2];
    emxEnsureCapacity((emxArray__common *)r5, i0, (int32_T)sizeof(real_T));
    loop_ub = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r5->data[i0] = octomap_voxels_map_withborder->data[i0];
    }

    d_circshift(r5);
    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    nx = octomap_voxels_map_shifted->size[0];
    idx = octomap_voxels_map_shifted->size[1];
    facetcountthisvoxel = octomap_voxels_map_shifted->size[2];
    loop_ub = nx * idx * facetcountthisvoxel;
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_shifted->data[i0] = (octomap_voxels_map_shifted->
        data[i0] + r3->data[i0]) + r5->data[i0];
    }

    emxFree_real_T(&r3);
    emxFree_real_T(&r5);
    if (2 > octomap_voxels_map_shifted->size[1] - 1) {
      i0 = 0;
      i1 = 1;
    } else {
      i0 = 1;
      i1 = octomap_voxels_map_shifted->size[1];
    }

    emxInit_real_T(&c_octomap_voxels_map_shifted, 3);
    nx = octomap_voxels_map_shifted->size[0];
    idx = octomap_voxels_map_shifted->size[2];
    i2 = c_octomap_voxels_map_shifted->size[0] *
      c_octomap_voxels_map_shifted->size[1] * c_octomap_voxels_map_shifted->
      size[2];
    c_octomap_voxels_map_shifted->size[0] = nx;
    c_octomap_voxels_map_shifted->size[1] = (i1 - i0) - 1;
    c_octomap_voxels_map_shifted->size[2] = idx;
    emxEnsureCapacity((emxArray__common *)c_octomap_voxels_map_shifted, i2,
                      (int32_T)sizeof(real_T));
    for (i2 = 0; i2 < idx; i2++) {
      loop_ub = i1 - i0;
      for (i3 = 0; i3 <= loop_ub - 2; i3++) {
        for (i4 = 0; i4 < nx; i4++) {
          c_octomap_voxels_map_shifted->data[(i4 +
            c_octomap_voxels_map_shifted->size[0] * i3) +
            c_octomap_voxels_map_shifted->size[0] *
            c_octomap_voxels_map_shifted->size[1] * i2] =
            octomap_voxels_map_shifted->data[(i4 +
            octomap_voxels_map_shifted->size[0] * (i0 + i3)) +
            octomap_voxels_map_shifted->size[0] *
            octomap_voxels_map_shifted->size[1] * i2];
        }
      }
    }

    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    octomap_voxels_map_shifted->size[0] = c_octomap_voxels_map_shifted->size[0];
    octomap_voxels_map_shifted->size[1] = c_octomap_voxels_map_shifted->size[1];
    octomap_voxels_map_shifted->size[2] = c_octomap_voxels_map_shifted->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = c_octomap_voxels_map_shifted->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      nx = c_octomap_voxels_map_shifted->size[1];
      for (i1 = 0; i1 < nx; i1++) {
        facetcountthisvoxel = c_octomap_voxels_map_shifted->size[0];
        for (i2 = 0; i2 < facetcountthisvoxel; i2++) {
          octomap_voxels_map_shifted->data[(i2 +
            octomap_voxels_map_shifted->size[0] * i1) +
            octomap_voxels_map_shifted->size[0] *
            octomap_voxels_map_shifted->size[1] * i0] =
            c_octomap_voxels_map_shifted->data[(i2 +
            c_octomap_voxels_map_shifted->size[0] * i1) +
            c_octomap_voxels_map_shifted->size[0] *
            c_octomap_voxels_map_shifted->size[1] * i0];
        }
      }
    }

    emxFree_real_T(&c_octomap_voxels_map_shifted);

    /* Remove border */
  }

  if (octomap_gridZ_size[1] > 2) {
    e_tmp_size[0] = octomap_gridX_size[1];
    e_tmp_size[1] = octomap_gridY_size[1];
    f_tmp_size[0] = octomap_gridX_size[1];
    f_tmp_size[1] = octomap_gridY_size[1];
    emxInit_real_T(&r6, 3);
    e_cat(e_tmp_size, b_octomap_voxels_map, f_tmp_size, r6);
    i0 = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    octomap_voxels_map_withborder->size[0] = r6->size[0];
    octomap_voxels_map_withborder->size[1] = r6->size[1];
    octomap_voxels_map_withborder->size[2] = r6->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_withborder, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = r6->size[0] * r6->size[1] * r6->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_withborder->data[i0] = r6->data[i0];
    }

    /* Add border */
    g_tmp_size[0] = octomap_gridX_size[1];
    g_tmp_size[1] = octomap_gridY_size[1];
    h_tmp_size[0] = octomap_gridX_size[1];
    h_tmp_size[1] = octomap_gridY_size[1];
    emxInit_real_T(&r7, 3);
    f_cat(g_tmp_size, octomap_voxels_map_shifted, h_tmp_size, r7);
    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    octomap_voxels_map_shifted->size[0] = r7->size[0];
    octomap_voxels_map_shifted->size[1] = r7->size[1];
    octomap_voxels_map_shifted->size[2] = r7->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = r7->size[0] * r7->size[1] * r7->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_shifted->data[i0] = r7->data[i0];
    }

    emxFree_real_T(&r7);

    /* Add border */
    i0 = r6->size[0] * r6->size[1] * r6->size[2];
    r6->size[0] = octomap_voxels_map_withborder->size[0];
    r6->size[1] = octomap_voxels_map_withborder->size[1];
    r6->size[2] = octomap_voxels_map_withborder->size[2];
    emxEnsureCapacity((emxArray__common *)r6, i0, (int32_T)sizeof(real_T));
    loop_ub = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r6->data[i0] = octomap_voxels_map_withborder->data[i0];
    }

    emxInit_real_T(&r8, 3);
    e_circshift(r6);
    i0 = r8->size[0] * r8->size[1] * r8->size[2];
    r8->size[0] = octomap_voxels_map_withborder->size[0];
    r8->size[1] = octomap_voxels_map_withborder->size[1];
    r8->size[2] = octomap_voxels_map_withborder->size[2];
    emxEnsureCapacity((emxArray__common *)r8, i0, (int32_T)sizeof(real_T));
    loop_ub = octomap_voxels_map_withborder->size[0] *
      octomap_voxels_map_withborder->size[1] *
      octomap_voxels_map_withborder->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r8->data[i0] = octomap_voxels_map_withborder->data[i0];
    }

    f_circshift(r8);
    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    nx = octomap_voxels_map_shifted->size[0];
    idx = octomap_voxels_map_shifted->size[1];
    facetcountthisvoxel = octomap_voxels_map_shifted->size[2];
    loop_ub = nx * idx * facetcountthisvoxel;
    for (i0 = 0; i0 < loop_ub; i0++) {
      octomap_voxels_map_shifted->data[i0] = (octomap_voxels_map_shifted->
        data[i0] + r6->data[i0]) + r8->data[i0];
    }

    emxFree_real_T(&r6);
    emxFree_real_T(&r8);
    if (2 > octomap_voxels_map_shifted->size[2] - 1) {
      i0 = 0;
      i1 = 1;
    } else {
      i0 = 1;
      i1 = octomap_voxels_map_shifted->size[2];
    }

    emxInit_real_T(&d_octomap_voxels_map_shifted, 3);
    nx = octomap_voxels_map_shifted->size[0];
    idx = octomap_voxels_map_shifted->size[1];
    i2 = d_octomap_voxels_map_shifted->size[0] *
      d_octomap_voxels_map_shifted->size[1] * d_octomap_voxels_map_shifted->
      size[2];
    d_octomap_voxels_map_shifted->size[0] = nx;
    d_octomap_voxels_map_shifted->size[1] = idx;
    d_octomap_voxels_map_shifted->size[2] = (i1 - i0) - 1;
    emxEnsureCapacity((emxArray__common *)d_octomap_voxels_map_shifted, i2,
                      (int32_T)sizeof(real_T));
    loop_ub = i1 - i0;
    for (i1 = 0; i1 <= loop_ub - 2; i1++) {
      for (i2 = 0; i2 < idx; i2++) {
        for (i3 = 0; i3 < nx; i3++) {
          d_octomap_voxels_map_shifted->data[(i3 +
            d_octomap_voxels_map_shifted->size[0] * i2) +
            d_octomap_voxels_map_shifted->size[0] *
            d_octomap_voxels_map_shifted->size[1] * i1] =
            octomap_voxels_map_shifted->data[(i3 +
            octomap_voxels_map_shifted->size[0] * i2) +
            octomap_voxels_map_shifted->size[0] *
            octomap_voxels_map_shifted->size[1] * (i0 + i1)];
        }
      }
    }

    i0 = octomap_voxels_map_shifted->size[0] * octomap_voxels_map_shifted->size
      [1] * octomap_voxels_map_shifted->size[2];
    octomap_voxels_map_shifted->size[0] = d_octomap_voxels_map_shifted->size[0];
    octomap_voxels_map_shifted->size[1] = d_octomap_voxels_map_shifted->size[1];
    octomap_voxels_map_shifted->size[2] = d_octomap_voxels_map_shifted->size[2];
    emxEnsureCapacity((emxArray__common *)octomap_voxels_map_shifted, i0,
                      (int32_T)sizeof(real_T));
    loop_ub = d_octomap_voxels_map_shifted->size[2];
    for (i0 = 0; i0 < loop_ub; i0++) {
      nx = d_octomap_voxels_map_shifted->size[1];
      for (i1 = 0; i1 < nx; i1++) {
        facetcountthisvoxel = d_octomap_voxels_map_shifted->size[0];
        for (i2 = 0; i2 < facetcountthisvoxel; i2++) {
          octomap_voxels_map_shifted->data[(i2 +
            octomap_voxels_map_shifted->size[0] * i1) +
            octomap_voxels_map_shifted->size[0] *
            octomap_voxels_map_shifted->size[1] * i0] =
            d_octomap_voxels_map_shifted->data[(i2 +
            d_octomap_voxels_map_shifted->size[0] * i1) +
            d_octomap_voxels_map_shifted->size[0] *
            d_octomap_voxels_map_shifted->size[1] * i0];
        }
      }
    }

    emxFree_real_T(&d_octomap_voxels_map_shifted);

    /* Remove border */
  }

  emxFree_real_T(&octomap_voxels_map_withborder);

  /* Identify the voxels which are at the boundary of the object: */
  i0 = x->size[0] * x->size[1] * x->size[2];
  x->size[0] = b_octomap_voxels_map->size[0];
  x->size[1] = b_octomap_voxels_map->size[1];
  x->size[2] = b_octomap_voxels_map->size[2];
  emxEnsureCapacity((emxArray__common *)x, i0, (int32_T)sizeof(boolean_T));
  loop_ub = b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
    b_octomap_voxels_map->size[2];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = ((b_octomap_voxels_map->data[i0] == 1) &&
                   (octomap_voxels_map_shifted->data[i0] < 6.0));
  }

  nx = x->size[0] * x->size[1] * x->size[2];
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = nx;
  emxEnsureCapacity((emxArray__common *)ii, i0, (int32_T)sizeof(int32_T));
  facetcountthisvoxel = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (facetcountthisvoxel <= nx)) {
    guard1 = FALSE;
    if (x->data[facetcountthisvoxel - 1]) {
      idx++;
      ii->data[idx - 1] = facetcountthisvoxel;
      if (idx >= nx) {
        exitg1 = TRUE;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      facetcountthisvoxel++;
    }
  }

  emxFree_boolean_T(&x);
  if (nx == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity((emxArray__common *)ii, i0, (int32_T)sizeof(int32_T));
    }
  } else {
    if (1 > idx) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    emxInit_int32_T(&d_ii, 1);
    i0 = d_ii->size[0];
    d_ii->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)d_ii, i0, (int32_T)sizeof(int32_T));
    for (i0 = 0; i0 < loop_ub; i0++) {
      d_ii->data[i0] = ii->data[i0];
    }

    i0 = ii->size[0];
    ii->size[0] = d_ii->size[0];
    emxEnsureCapacity((emxArray__common *)ii, i0, (int32_T)sizeof(int32_T));
    loop_ub = d_ii->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      ii->data[i0] = d_ii->data[i0];
    }

    emxFree_int32_T(&d_ii);
  }

  b_emxInit_real_T(&edgevoxelindices, 2);
  c_emxInit_real_T(&e_ii, 1);
  i0 = edgevoxelindices->size[0] * edgevoxelindices->size[1];
  edgevoxelindices->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)edgevoxelindices, i0, (int32_T)sizeof
                    (real_T));
  i0 = e_ii->size[0];
  e_ii->size[0] = ii->size[0];
  emxEnsureCapacity((emxArray__common *)e_ii, i0, (int32_T)sizeof(real_T));
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    e_ii->data[i0] = ii->data[i0];
  }

  c_emxInit_real_T(&f_ii, 1);
  facetcountthisvoxel = e_ii->size[0];
  i0 = edgevoxelindices->size[0] * edgevoxelindices->size[1];
  edgevoxelindices->size[1] = facetcountthisvoxel;
  emxEnsureCapacity((emxArray__common *)edgevoxelindices, i0, (int32_T)sizeof
                    (real_T));
  i0 = f_ii->size[0];
  f_ii->size[0] = ii->size[0];
  emxEnsureCapacity((emxArray__common *)f_ii, i0, (int32_T)sizeof(real_T));
  loop_ub = ii->size[0];
  emxFree_real_T(&e_ii);
  for (i0 = 0; i0 < loop_ub; i0++) {
    f_ii->data[i0] = ii->data[i0];
  }

  loop_ub = f_ii->size[0];
  emxFree_real_T(&f_ii);
  for (i0 = 0; i0 < loop_ub; i0++) {
    edgevoxelindices->data[i0] = ii->data[i0];
  }

  emxFree_int32_T(&ii);
  b_emxInit_real_T(&e_octomap_voxels_map_shifted, 2);

  /* Calculate the number of facets there wil be in the final STL mesh: */
  i0 = e_octomap_voxels_map_shifted->size[0] *
    e_octomap_voxels_map_shifted->size[1];
  e_octomap_voxels_map_shifted->size[0] = 1;
  e_octomap_voxels_map_shifted->size[1] = edgevoxelindices->size[1];
  emxEnsureCapacity((emxArray__common *)e_octomap_voxels_map_shifted, i0,
                    (int32_T)sizeof(real_T));
  loop_ub = edgevoxelindices->size[0] * edgevoxelindices->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    e_octomap_voxels_map_shifted->data[i0] = octomap_voxels_map_shifted->data
      [(int32_T)edgevoxelindices->data[i0] - 1];
  }

  emxFree_real_T(&octomap_voxels_map_shifted);
  emxInit_boolean_T(&neighbourlist, 2);
  facetcount = (real_T)edgevoxelindices->size[1] * 6.0 - sum
    (e_octomap_voxels_map_shifted);
  facetcount *= 2.0;

  /* Create an array to record... */
  /* Cols 1-6: Whether each edge voxel's 6 neighbours are inside or outside the object. */
  facetcountthisvoxel = edgevoxelindices->size[1];
  i0 = neighbourlist->size[0] * neighbourlist->size[1];
  neighbourlist->size[0] = facetcountthisvoxel;
  neighbourlist->size[1] = 6;
  emxEnsureCapacity((emxArray__common *)neighbourlist, i0, (int32_T)sizeof
                    (boolean_T));
  loop_ub = edgevoxelindices->size[1] * 6;
  emxFree_real_T(&e_octomap_voxels_map_shifted);
  for (i0 = 0; i0 < loop_ub; i0++) {
    neighbourlist->data[i0] = FALSE;
  }

  emxInit_real_T(&meshXYZ, 3);

  /* Initialise arrays to store the STL mesh data: */
  i0 = meshXYZ->size[0] * meshXYZ->size[1] * meshXYZ->size[2];
  meshXYZ->size[0] = (int32_T)facetcount;
  meshXYZ->size[1] = 3;
  meshXYZ->size[2] = 3;
  emxEnsureCapacity((emxArray__common *)meshXYZ, i0, (int32_T)sizeof(real_T));
  loop_ub = (int32_T)facetcount * 3 * 3;
  for (i0 = 0; i0 < loop_ub; i0++) {
    meshXYZ->data[i0] = 0.0;
  }

  /* Create a counter to keep track of how many facets have been written as the */
  /* following 'for' loop progresses: */
  facetcountsofar = 0.0;
  for (nx = 0; nx < edgevoxelindices->size[1]; nx++) {
    for (i0 = 0; i0 < 3; i0++) {
      octomap_gridX[i0] = b_octomap_voxels_map->size[i0];
    }

    b_ind2sub(octomap_gridX, edgevoxelindices->data[nx], &facetcount, &mtmp,
              &subZ);
    if (facetcount == 1.0) {
      neighbourlist->data[nx] = FALSE;
    } else {
      neighbourlist->data[nx] = b_octomap_voxels_map->data[(((int32_T)
        (facetcount - 1.0) + b_octomap_voxels_map->size[0] * ((int32_T)mtmp - 1))
        + b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
        ((int32_T)subZ - 1)) - 1];
    }

    if (mtmp == 1.0) {
      neighbourlist->data[nx + neighbourlist->size[0]] = FALSE;
    } else {
      neighbourlist->data[nx + neighbourlist->size[0]] =
        b_octomap_voxels_map->data[(((int32_T)facetcount +
        b_octomap_voxels_map->size[0] * ((int32_T)(mtmp - 1.0) - 1)) +
        b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
        ((int32_T)subZ - 1)) - 1];
    }

    if (subZ == octomap_gridZ_size[1]) {
      neighbourlist->data[nx + (neighbourlist->size[0] << 1)] = FALSE;
    } else {
      neighbourlist->data[nx + (neighbourlist->size[0] << 1)] =
        b_octomap_voxels_map->data[(((int32_T)facetcount +
        b_octomap_voxels_map->size[0] * ((int32_T)mtmp - 1)) +
        b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
        ((int32_T)(subZ + 1.0) - 1)) - 1];
    }

    if (mtmp == octomap_gridY_size[1]) {
      neighbourlist->data[nx + neighbourlist->size[0] * 3] = FALSE;
    } else {
      neighbourlist->data[nx + neighbourlist->size[0] * 3] =
        b_octomap_voxels_map->data[(((int32_T)facetcount +
        b_octomap_voxels_map->size[0] * ((int32_T)(mtmp + 1.0) - 1)) +
        b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
        ((int32_T)subZ - 1)) - 1];
    }

    if (subZ == 1.0) {
      neighbourlist->data[nx + (neighbourlist->size[0] << 2)] = FALSE;
    } else {
      neighbourlist->data[nx + (neighbourlist->size[0] << 2)] =
        b_octomap_voxels_map->data[(((int32_T)facetcount +
        b_octomap_voxels_map->size[0] * ((int32_T)mtmp - 1)) +
        b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
        ((int32_T)(subZ - 1.0) - 1)) - 1];
    }

    if (facetcount == octomap_gridX_size[1]) {
      neighbourlist->data[nx + neighbourlist->size[0] * 5] = FALSE;
    } else {
      neighbourlist->data[nx + neighbourlist->size[0] * 5] =
        b_octomap_voxels_map->data[(((int32_T)(facetcount + 1.0) +
        b_octomap_voxels_map->size[0] * ((int32_T)mtmp - 1)) +
        b_octomap_voxels_map->size[0] * b_octomap_voxels_map->size[1] *
        ((int32_T)subZ - 1)) - 1];
    }

    for (i0 = 0; i0 < 6; i0++) {
      b_neighbourlist[i0] = neighbourlist->data[nx + neighbourlist->size[0] * i0];
    }

    y = 2.0 * (6.0 - b_sum(b_neighbourlist));
    loop_ub = (int32_T)y * 3 * 3;
    for (i0 = 0; i0 < loop_ub; i0++) {
      facetCOtemp_data[i0] = 0.0;
    }

    facetcountthisvoxel = -1;
    if (neighbourlist->data[nx] == 0) {
      /* Neighbouring voxel in the -x direction */
      facetCOtemp_data[0] = octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(int32_T)y] = octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(int32_T)y << 1] = octomap_gridZlower_data[(int32_T)subZ
        - 1];
      facetCOtemp_data[(int32_T)y * 3] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[(int32_T)y + (int32_T)y * 3] = octomap_gridYlower_data
        [(int32_T)mtmp - 1];
      facetCOtemp_data[((int32_T)y << 1) + (int32_T)y * 3] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[(int32_T)y * 3 << 1] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[(int32_T)y + ((int32_T)y * 3 << 1)] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[((int32_T)y << 1) + ((int32_T)y * 3 << 1)] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetcountthisvoxel = 1;
      facetCOtemp_data[1] = octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[1 + (int32_T)y] = octomap_gridYupper_data[(int32_T)mtmp -
        1];
      facetCOtemp_data[1 + ((int32_T)y << 1)] = octomap_gridZupper_data[(int32_T)
        subZ - 1];
      facetCOtemp_data[1 + (int32_T)y * 3] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[((int32_T)y + (int32_T)y * 3) + 1] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(((int32_T)y << 1) + (int32_T)y * 3) + 1] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[1 + ((int32_T)y * 3 << 1)] = octomap_gridXlower_data
        [(int32_T)facetcount - 1];
      facetCOtemp_data[((int32_T)y + ((int32_T)y * 3 << 1)) + 1] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(((int32_T)y << 1) + ((int32_T)y * 3 << 1)) + 1] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetcountsofar += 2.0;
    }

    if (neighbourlist->data[nx + neighbourlist->size[0]] == 0) {
      /* Neighbouring voxel in the -y direction */
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXupper_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetcountsofar += 2.0;
    }

    if (neighbourlist->data[nx + (neighbourlist->size[0] << 1)] == 0) {
      /* Neighbouring voxel in the +z direction */
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXupper_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetcountsofar += 2.0;
    }

    if (neighbourlist->data[nx + neighbourlist->size[0] * 3] == 0) {
      /* Neighbouring voxel in the +y direction */
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXupper_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetcountsofar += 2.0;
    }

    if (neighbourlist->data[nx + (neighbourlist->size[0] << 2)] == 0) {
      /* Neighbouring voxel in the -z direction */
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXlower_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXupper_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXlower_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetcountsofar += 2.0;
    }

    if (neighbourlist->data[nx + neighbourlist->size[0] * 5] == 0) {
      /* Neighbouring voxel in the +x direction */
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXupper_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetcountthisvoxel++;
      facetCOtemp_data[facetcountthisvoxel] = octomap_gridXupper_data[(int32_T)
        facetcount - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y << 1)] =
        octomap_gridZupper_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + (int32_T)y * 3] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + (int32_T)y * 3] =
        octomap_gridYlower_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + (int32_T)y *
        3] = octomap_gridZlower_data[(int32_T)subZ - 1];
      facetCOtemp_data[facetcountthisvoxel + ((int32_T)y * 3 << 1)] =
        octomap_gridXupper_data[(int32_T)facetcount - 1];
      facetCOtemp_data[(facetcountthisvoxel + (int32_T)y) + ((int32_T)y * 3 << 1)]
        = octomap_gridYupper_data[(int32_T)mtmp - 1];
      facetCOtemp_data[(facetcountthisvoxel + ((int32_T)y << 1)) + ((int32_T)y *
        3 << 1)] = octomap_gridZupper_data[(int32_T)subZ - 1];
      facetcountsofar += 2.0;
    }

    facetcount = (facetcountsofar - ((real_T)facetcountthisvoxel + 1.0)) + 1.0;
    if (facetcount > facetcountsofar) {
      i0 = 0;
    } else {
      i0 = (int32_T)facetcount - 1;
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        loop_ub = (int32_T)y;
        for (i3 = 0; i3 < loop_ub; i3++) {
          meshXYZ->data[((i0 + i3) + meshXYZ->size[0] * i2) + meshXYZ->size[0] *
            meshXYZ->size[1] * i1] = facetCOtemp_data[(i3 + (int32_T)y * i2) +
            (int32_T)y * 3 * i1];
        }
      }
    }
  }

  emxFree_boolean_T(&b_octomap_voxels_map);
  emxFree_boolean_T(&neighbourlist);
  emxFree_real_T(&edgevoxelindices);
  b_emxInit_real_T(&normalsXYZ, 2);
  b_emxInit_real_T(&vertices, 2);

  /*    output and write the file */
  retrieve_meshformat(meshXYZ, normalsXYZ, vertices);
  if(MeshedOctomap != NULL) { // TODO(birchera): allocation of the right amount of memory does not work yet
    i0 = MeshedOctomap->faces->size[0] * MeshedOctomap->faces->size[1];
    MeshedOctomap->faces->size[0] = normalsXYZ->size[0];
    MeshedOctomap->faces->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)MeshedOctomap->faces, i0, (int32_T)
                      sizeof(real_T));
    loop_ub = normalsXYZ->size[0] * normalsXYZ->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      MeshedOctomap->faces->data[i0] = normalsXYZ->data[i0];
    }

    i0 = MeshedOctomap->vertices->size[0] * MeshedOctomap->vertices->size[1];
    MeshedOctomap->vertices->size[0] = vertices->size[0];
    MeshedOctomap->vertices->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)MeshedOctomap->vertices, i0, (int32_T)
                      sizeof(real_T));
    loop_ub = vertices->size[0] * vertices->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      MeshedOctomap->vertices->data[i0] = vertices->data[i0];
    }
  }
  
  //////////////////////////////
  std::fstream meshFile;
  meshFile.open((pkgPath+"/data/meshOutMeshXYZ.m").c_str(), std::ios::out);
  meshFile << "meshXYZ = [";
  for (int i = 0; i < meshXYZ->size[0]; i++) {
      meshFile << meshXYZ->data[i+meshXYZ->size[0]*0] << ", " << meshXYZ->data[i+meshXYZ->size[0]*1] << ", " << meshXYZ->data[i+meshXYZ->size[0]*2] << ", ";
      meshFile << meshXYZ->data[i+meshXYZ->size[0]*3] << ", " << meshXYZ->data[i+meshXYZ->size[0]*4] << ", " << meshXYZ->data[i+meshXYZ->size[0]*5] << ", ";
      meshFile << meshXYZ->data[i+meshXYZ->size[0]*6] << ", " << meshXYZ->data[i+meshXYZ->size[0]*7] << ", " << meshXYZ->data[i+meshXYZ->size[0]*8] << ";\n";
  }
  meshFile << "];";
  meshFile.close();
  /////////////////////////////
  emxFree_real_T(&meshXYZ);

  write_stl(vertices, normalsXYZ, fname_ply_data, fname_ply_size);
  emxFree_real_T(&vertices);
  emxFree_real_T(&normalsXYZ);
}

/* End of code generation (OctomapToMesh.cpp) */
