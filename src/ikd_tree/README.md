# ikd-Tree
**ikd-Tree** is an incremental k-d tree designed for robotic applications. The ikd-Tree incrementally updates a k-d tree with new coming points only, leading to much lower computation time than existing static k-d trees. Besides point-wise operations, the ikd-Tree supports several features such as box-wise operations and down-sampling that are practically useful in robotic applications.

## What does ikd-Tree support?

- Build a balanced k-d tree - `Build()`

- Dynamically insert points to or delete points from the k-d tree - `Add_Points() / Delete_Points()`

- Delete points inside given axis-aligned bounding boxes - `Delete_Point_Boxes()`

- K Nearest Neighbor Search with range limitation - `Nearest_Search()`

- Acquire points inside a given axis-aligned bounding box on the k-d tree - `Box_Search()`

- Acquire points inside a ball with given radius on the k-d tree - `Radius_Search()`

## User Manual

- Browse the [User Manual](https://github.com/hku-mars/ikd-Tree/blob/main/documents/UserManual.pdf) for using our ikd-Tree.

## Developers

- [Yixi CAI 蔡逸熙](https://github.com/Ecstasy-EC): Data structure design and implementation

- [Wei XU 徐威](https://github.com/XW-HKU): Incorporation into  [LiDAR-inertial odometry package FAST_LIO2 (TRO, 2022)](https://github.com/hku-mars/FAST_LIO)


## Related paper

- [ikd-Tree: An Incremental K-D Tree for robotic applications](https://arxiv.org/abs/2102.10808)

- [FAST-LIO2: Fast Direct LiDAR-Inertial Odometry](https://ieeexplore.ieee.org/abstract/document/9697912)

If you are using any code of this repo in your research, please cite at least one of the articles as following:
- **ikd-Tree**
```
@article{cai2021ikd,
  title={ikd-Tree: An Incremental KD Tree for Robotic Applications},
  author={Cai, Yixi and Xu, Wei and Zhang, Fu},
  journal={arXiv preprint arXiv:2102.10808},
  year={2021}
}
```
- **FAST-LIO2**
```
@article{xu2022fast,
  title={Fast-lio2: Fast direct lidar-inertial odometry},
  author={Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal={IEEE Transactions on Robotics},
  year={2022},
  publisher={IEEE}
}
```

## How to build this project
```bash
cd ~/catkin_ws/src
git clone git@github.com:hku-mars/ikd-Tree.git
cd ikd-Tree/build
cmake ..
make -j 9
```

## Acknowledgments
- Thanks [Marcus Davi](https://github.com/Marcus-Davi) for helps in templating the ikd-Tree for more general applications.

- Thanks [Hyungtae Lim 임형태](https://github.com/LimHyungTae) for providing application examples on point clouds. 

## License

The source code of ikd-Tree is released under [GPLv2](http://www.gnu.org/licenses/old-licenses/gpl-2.0.html) license. For commercial use, please contact Mr. Yixi CAI (<yixicai@connect.hku.hk>) or Dr. Fu ZHANG (<fuzhang@hku.hk>).
