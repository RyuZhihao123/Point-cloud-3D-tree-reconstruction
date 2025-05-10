# Point-cloud-3D-tree-reconstruction

This is a program for reconstructing a 3D tree model from scanned point clouds. The program is fully developed in C++.

🎞️ **Demo video**: [[Video]](https://drive.google.com/file/d/1sX3tNEdxsmSTkAFL4GsnzzMajR-hw_qR/view?usp=sharing).

<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_Peach.png" alt="Description" width="400"/>
</p>

## Quick Start.

### 🎞️ Demo video: 

We recommend all users first watching this **[[Demo Video]](https://drive.google.com/file/d/1sX3tNEdxsmSTkAFL4GsnzzMajR-hw_qR/view?usp=sharing)** to understand its usage quickly. **(Strongly Suggest!🔥🔥)** 

### 🪴 Software:

We have released the ``exe`` program and you can [Download here](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/TreeFromPoints_exe). The users can directly run the program on Windows PCs without any configuration or compilation. We have sucessfully tested it for Win10 and Win11.

Here are also the [example Point-cloud Files](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/Example_PointClouds) that I used in the demo video. You can download them for a quick start.


**(1) Usage Instructions**: 

- Please first download the entire [[folder]](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/TreeFromPoints_exe), then double-click the ``TreeFromPoints.exe`` to execute the program.
- Then, press the ``Load Point Data`` button to load a point cloud file from your local disk. Example point data is available [here](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/Example_PointClouds).
- Finally, sequentially press the seven buttons from ``(1) Remove noises`` to ``(7) Optimize``, and you will see the final 3D tree models in the right display panel.

**(2) How to export the 3D models?**

Actually, the 3D results are automatically saved under the exe folder path with filename as ``bark_texture.obj``. Please refer to this [issue](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/issues/1) for more details.

The following figures shows an example of an exported result which is opened by 3D Viewer.

<p align="center">
<img src="https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/blob/main/Fig_export.png" alt="Description" width="600"/>
</p>

### ♻️ Code: 

For general use, we recommend using the exe program introduced above, which is very simple to use and there is no need for any configuration steps.

But if you want to customize the program for your own purposes, please [Download the Source Code here](https://github.com/RyuZhihao123/Point-cloud-3D-tree-reconstruction/tree/main/TreeFromPoints_codes). Our program is implemented in C++ codes, so you can compile with any C++ IDEs.



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
