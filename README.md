# Point Cloud Library 1.13.0 fork

<p align="center"><img src="pcl.png" height="100"></p>

[![Release][release-image]][releases]
[![License][license-image]][license]

[release-image]: https://img.shields.io/badge/release-1.13.0-green.svg?style=flat
[releases]: https://github.com/PointCloudLibrary/pcl/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt

## Why this repo?
This repository aims to enable the use of Point Cloud Library (PCL) in a zero-copy manner with other cloud stacks. By using a container other than a std::vector, it reduces the overhead of constructors/destructors and resizing.

## Changes
- Resolved issue with finding CUDA libraries
- Implemented tk_allocator to allow mapping of pre-existing memory
- Fixed compilation errors on ARM architecture
- Fixed issue with finding Google Test framework

## Example
```
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

std::array<pcl::PointXYZ, 460400> data;
cloud->points = std::vector<pcl::PointXYZ, tk::tk_allocator<pcl::PointXYZ>>(0, tk::tk_allocator<pcl::PointXYZ>(data.data(), 460400));
```

