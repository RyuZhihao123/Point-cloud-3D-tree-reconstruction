# Point-cloud-3D-tree-reconstruction

This is a program for reconstructing a 3D tree model from scanned point clouds. The program is fully developed in C++.

🎞️ **Demo video**: [[Video]](https://drive.google.com/file/d/1sX3tNEdxsmSTkAFL4GsnzzMajR-hw_qR/view?usp=sharing).

<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_Peach.png" alt="Description" width="400"/>
</p>

## Quick Start.

🎞️ **Demo video**: 

We strongly recommend first watching this [[Demo Video]](https://drive.google.com/file/d/1sX3tNEdxsmSTkAFL4GsnzzMajR-hw_qR/view?usp=sharing) to understand its usage quickly.

😄 **Software Download**: 

We released the ``exe`` program under this [[Folder]](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/TreeFromPoints_exe), so that users can directly execute the program on Windows PCs without any configuration or compilation.

Here are the [example point cloud files](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/Example_PointClouds) that I used in the video. You can use them to have a try.


📙 **Usage Instructions**: 

  1. Please first download the entire repository, then double-click the ``TreeFromPoints.exe`` to execute the program.
  2. Then, press the ``Load Point Data`` button to load a point cloud file from your local disk. Example point data is available [here](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/Example_PointClouds).
  3. Finally, sequentially press the seven buttons from ``(1) Remove noises`` to ``(7) Optimize``, and you will see the final 3D tree models in the right display panel.

❓ **How to export the 3D models?**

The results are automatically saved under the exe folder path with filename as ``bark_texture.obj``. Please refer to this [issue](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/issues/1) for details.

The following figures shows an exported result that is opened by 3D Viewer.

<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_export.png" alt="Description" width="400"/>
</p>

## More Experimental Results.
<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_Cercis.png" alt="Description" width="400"/>
</p>
<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_Maple.png" alt="Description" width="400"/>
</p>
<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_Peach.png" alt="Description" width="400"/>
</p>
<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_default.png" alt="Description" width="400"/>
</p>
