# Voxel- and graph-based point-cloud-segmentation
This is the source code for the algorithm of Voxel- and Graph-based Segmentation (VGS/SVGS).

##Environment:
The code is written in C++ and tested in VS2015.

##Dependence:
The code depends on the following third-party libraries:
-PCL 1.8.1.

##How to use these codes：
please refer to "Use_Test"

The "input_vector" encodes the input parameters (see "Task_File_VGS.txt" and "Task_File_SVGS.txt"), you can change it according to your demands.

Special thanks to Dong Lin from TU Dresden for correcting bugs in the codes!

##Reference:

Testing dataset (Town_Test.pcd) is cropped from ETH Zurich dataset： http://semantic3d.net/

You can find more details about the VGS/SVGS algorithm in our recent publications:

```
@article{xu2017geometric,  
  title={Geometric Primitive Extraction From Point Clouds of Construction Sites Using VGS},  
  author={Xu, Yusheng and Tuttas, Sebastian and Hoegner, Ludwig and Stilla, Uwe},  
  journal={IEEE Geoscience and Remote Sensing Letters},    
  volume={14},  
  number={3},  
  pages={424--428},  
  year={2017},  
  publisher={IEEE}  
}

@article{xu2017voxel,  
  title={Voxel- and Graph-Based Point Cloud Segmentation of 3D Scenes Using Perceptual Grouping Laws},  
  author={Xu, Yusheng and Hoegner, Ludwig and Tuttas, Sebastian and Stilla, Uwe},  
  journal={ISPRS Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences},  
  volume={4},  
  year={2017}  
}

@article{xu2018voxel,
  title={A Voxel-and Graph-Based Strategy for Segmenting Man-Made Infrastructures Using Perceptual Grouping Laws: Comparison and Evaluation},
  author={Xu, Yusheng and Hoegner, Ludwig and Tuttas, Sebastian and Stilla, Uwe},
  journal={Photogrammetric Engineering \& Remote Sensing},
  volume={84},
  number={6},
  pages={377--391},
  year={2018},
  publisher={American Society for Photogrammetry and Remote Sensing}
}
```
