#pragma once

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> detectErrors(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> physicalPC, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> actualPC);