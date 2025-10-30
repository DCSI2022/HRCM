# HRCM
[Datasets](https://drive.google.com/drive/folders/1aknX5qRzH96mUzXKG9H18TwjlUOdJf8g?usp=drive_link) and Code for Paper "HRCM：Human–Robot LiDAR-inertial Collaborative Mapping with robust triangulated planes"

_Abstract_: Multirobot Simultaneous localization and mapping (SLAM) has become a hot spot all over the world while there still exist some key issues to be addressed for collaborative mapping, especially for heterogenous platforms. This paper presents a self-designed LiDAR-inertial-mapping system on lightweight manned helmet and unmanned ground vehicle (UGV) platforms. A stable-triangular-descriptor-based place recognition method is proposed to detect overlap between varied field of view (FoV), followed by a Delaunay verification to restrain the global similarity. Based on the conjugated plane primitives across the platforms, bundle adjustment is derived to enhance the map consistency. All frames within the sliding window will be mutually constrained and optimized through the collective pose graph. Experiments on place recognition, pose estimation and point cloud mapping are conducted in different kinds of environments. Compared to other famous works (e.g. FastLIO2, LIOSAM, PointLIO), our system can attain the best results in almost all cases. The final improvements against them of mapping precision reach 15.71% with same LiDAR and 14.49% for heterogenous LiDARs on UGV. As for the helmet, this disparity varies from 0.6% for dataset with small intersection to 22.33% at most

<img width="542" height="380" alt="image" src="https://github.com/user-attachments/assets/93677a3d-bbc2-46bc-b180-b0af8c1cf903" />

<img width="860" height="392" alt="image" src="https://github.com/user-attachments/assets/192db5ed-eef8-4d4a-9486-5d5595adfab6" />

<img width="921" height="517" alt="image" src="https://github.com/user-attachments/assets/38d7ff10-e560-4a6b-92d3-d1effd097126" />
