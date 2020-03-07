## kitti-velodyne-viewer

View kitti lidar point cloud with bounding box label by single file.

![teaser](/doc/teaser.png)

## Requirements

The code requires [pyqtgraph](https://github.com/pyqtgraph/pyqtgraph) and python 3.x

## Dataset

Download KITTI 3D object detection data and organize the folders as follows:

        dataset/KITTI/object/

            velodyne/
                training/
                    000003.bin
                testing/

            calib/
                training/
                    000003.txt
                testing/

            label/
                training/
                    000003.txt
                testing/

## convert `.pcd` to `.bin`
usage:
```bash
# pcdfolder is where `.pcd` files are stores
python pcd2bin.py covert pcdfolder binfolder
```
for example:

```bash
python pcd2bin.py convert pcdfiles outputfolder
```

## About converting `.bin` to `.pcd`
There are methods to do it, but all of them are in C++ and PCL lib is required.  
You can refer to this [kitti_velodyne_bin_to_pcd](https://github.com/HTLife/kitti_velodyne_bin_to_pcd)