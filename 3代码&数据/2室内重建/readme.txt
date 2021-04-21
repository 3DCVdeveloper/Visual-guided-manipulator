运行三维重建代码前，需要把数据以rgb和depth两个文件夹的形式放到"2室内重建/3室内重建代码/dataset"文件夹中。比如，把"2室内重建/2室内重建输入数据"中的rgb和depth文件夹放到"2室内重建/3室内重建代码/dataset/201222_Orbbec_200"中，并在orbbec.json中配置"path_dataset"项。
可以在camera_astrapro.json文件中修改相机内参。
修改完成后，即可在"2室内重建/3室内重建代码"中，运行代码进行三维重建：python start_3dReconstruction.py