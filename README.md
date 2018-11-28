# plane-opt-rgbd
This is the repo for the 3DV2018 paper:
```
@inproceedings{wang2018plane,
  title={Plane-Based Optimization of Geometry and Texture for RGB-D Reconstruction of Indoor Scenes},
  author={Wang, Chao and Guo, Xiaohu},
  booktitle={2018 International Conference on 3D Vision (3DV)},
  pages={533--541},
  year={2018},
  organization={IEEE}
}
```
This repo list relevant source codes and supplemental materials including video and result models for the paper.

Here is the video for the paper:

[![Plane opt video](https://img.youtube.com/vi/57GFQcebceY/0.jpg)](https://www.youtube.com/watch?v=57GFQcebceY)

## Plane extraction code

Part of the source codes about plane detection is listed here:

### 1. RGB-D plane detection on frames

Code and relevant reference can be found here: https://github.com/chaowang15/RGBDPlaneDetection 

### 2. Plane detection on a mesh

Source code can be downloaded here: http://graphics.utdallas.edu/sites/default/files/Cai_TVCG/Remeshing_PCA.zip

Code refers to this paper:
```
@article{cai2017surface,
  title={Surface approximation via asymptotic optimal geometric partition},
  author={Cai, Yiqi and Guo, Xiaohu and Liu, Yang and Wang, Wenping and Mao, Weihua and Zhong, Zichun},
  journal={IEEE Transactions on Visualization \& Computer Graphics},
  number={12},
  pages={2613--2626},
  year={2017},
  publisher={IEEE}
}
```

Please cite it and the 3DV2018 paper if you want to use the code.
