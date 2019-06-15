# PlaneRecon (on-going, will be ready soon)
Plane-Based RGB-D reconstruction of indoor scenes with geometry and texture optimization.

## Related publications
Please cite these two papers if you want to use the code and data:
```
@inproceedings{wang2018plane,
  title={[Plane-Based Optimization of Geometry and Texture for RGB-D Reconstruction of Indoor Scenes]},
  author={Wang, Chao and Guo, Xiaohu},
  booktitle={2018 International Conference on 3D Vision (3DV)},
  pages={533--541},
  year={2018},
  organization={IEEE}
}
```
([here is PDF](http://www.utdallas.edu/~xxg061000/3DV2018.pdf))

and
```
@InProceedings{Wang_2019_CVPR_Workshops,
author = {Wang, Chao and Guo, Xiaohu},
title = {Efficient Plane-Based Optimization of Geometry and Texture for Indoor RGB-D Reconstruction},
booktitle = {The IEEE Conference on Computer Vision and Pattern Recognition (CVPR) Workshops},
month = {June},
year = {2019}
}
```
([here is PDF](http://openaccess.thecvf.com/content_CVPRW_2019/papers/SUMO/Wang_Efficient_Plane-Based_Optimization_of_Geometry_and_Texture_for_Indoor_RGB-D_CVPRW_2019_paper.pdf))


## Notes

- `mesh_partition` code needs a lot of memory. For instance, it takes about 20G memory for a mesh with 10m vertices and 5m vertices.


## Other relevant code
### Plane detection on RGB-D frames

Source code can be found here: https://github.com/chaowang15/RGBDPlaneDetection
