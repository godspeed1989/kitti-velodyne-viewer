## kitti-velodyne-viewer

View kitti lidar point cloud with bounding box label by single file.

![teaser](https://github.com/godspeed1989/kitti-velodyne-viewer/blob/master/doc/teaser.png)

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
