octomap_to_mesh
===============

This package supplies functions to generate a surface mesh from an octomap::OcTree. The use this package, add the following c++ snipplet to your program:


```cpp
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_to_mesh/rt_nonfinite.h>
#include <octomap_to_mesh/rtwtypes.h>
#include <octomap_to_mesh/OctomapToMesh_types.h>
#include <octomap_to_mesh/OctomapToMesh_struct.h>

...

OctomapToMesh_T data;

    if(OctomapToGrid (planner->octomap_, &data)) {
                      
      struct_T * MeshedOctomap = NULL;
      char_T * fileName = new char_T[50];
      strcpy(fileName, (pkgPath+"/data/meshOut.stl").c_str());
      OctomapToMesh(&data, fileName, MeshedOctomap);
}

...

```

Alternatively use the provided launchfile to save the octomap as a surface mesh.
