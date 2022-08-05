# Point Cloud Library

<p align="center"><img src="pcl.png" height="100"></p>

[![Release][release-image]][releases]
[![License][license-image]][license]

[release-image]: https://img.shields.io/badge/release-1.12.1-green.svg?style=flat
[releases]: https://github.com/PointCloudLibrary/pcl/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt

Modifications
-------
1. Fix Gtest find_package
2. Fix CUDA find_package
3. Add custom allocator ```tk::tk_allocator``` instead ```tk::tk_allocator```

Usage
-------
```
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

std::array<pcl::PointXYZ, 460400> data;
cloud->points = std::vector<pcl::PointXYZ, tk::tk_allocator<pcl::PointXYZ>>(0, tk::tk_allocator<pcl::PointXYZ>(data.data(), 460400));
```