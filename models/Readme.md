# Models
This folder contains result textured meshes used in the paper. Currently some meshes are not latest, and will update them soon.

Here are screenshots of plane partition and face texture from the scan *copyroom*.
<img src="copyroom/planes.png" width="49%">
<img src="copyroom/texture.png" width="49%">

Each folder contains corresponding result textured obj meshes. You can view these models in MeshLab. Note to turn off the light (light on/off button in the render toolbar) when viewing them in Meshlab.

We also provide two original dense models of sequence `of_kr2` and `lr_kr2` from ICL-NUIM dataset, as `of_kt2_voxelhashing.ply` and `lr_kt2_voxelhashing.ply`. The two models are reconstructed by [VoxelHashing](https://github.com/niessner/VoxelHashing) with groundtruth camera poses. The original RGB-D sequences for the two models can be found in [ICL-NUIM website](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html).

All the other original dense models as well as original RGB-D sequences can be found in the [BundleFusion website](http://graphics.stanford.edu/projects/bundlefusion/).
