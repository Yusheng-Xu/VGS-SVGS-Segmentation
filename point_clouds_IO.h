////////////////////////////////////////////////////////////////////////////////
//	File:		 point_cloud_IO.h
//	Author:		 Yusheng Xu, PF_Technische Universitaet Muechen (yusheng.xu@tum.de)
//	Description: IO operation of the point clouds
//  Modified:    29.7.2016
//
//  Copyright (c) 2015-2017  Yusheng Xu (yusheng.xu@tum.de)
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  Version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <typeinfo>
#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

//Type definition
typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA> PCXYZRGBA;
typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB> PCXYZRGB;
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr PCXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ> PCXYZ;

using namespace std;

//Declaration 
template <typename PTTypePtr> 
int
inputPointCloudData(std::string dataName, PTTypePtr dataCloud);	//Input point clouds

template <typename PTTypePtr>
int
outputPointCloudData(string outName, PTTypePtr dataCloud);//Output point clouds

////////////////Remarks//////////////
//  For the use of "template" the 
//	declaration and definition of 
//	certain functions & classes 
//	should in same .h or .cpp files 
/////////////////////////////////////

//Input point cloud
template <typename PTTypePtr> 
int
inputPointCloudData(string dataName, PTTypePtr dataCloud)
{
	//Input the PCD file of datasets
	if( pcl::io::loadPCDFile(dataName, *dataCloud) == -1)
	{
		PCL_ERROR("Couldn't read the PCD file!");
		return(-1);
	}

	//Show the information of the points in the input PCD files
	size_t dataSize=dataCloud->points.size();  // Size of the point clouds

	return (0);
}

template <typename PTTypePtr>
int
inputPointCloudData2(string dataName, PTTypePtr dataCloud)
{
	//Input the PCD file of datasets
	if (pcl::io::loadPLYFile(dataName, *dataCloud) == -1)
	{
		PCL_ERROR("Couldn't read the PLY file!");
		return(-1);
	}

	//Show the information of the points in the input PLY files
	size_t dataSize = dataCloud->points.size();  // Size of the point clouds
	return (0);
}

//Output point cloud
template <typename PTTypePtr>
int
outputPointCloudData(string outName, PTTypePtr dataCloud)
{
	if( pcl::io::savePCDFile(outName, *dataCloud) == -1)
	{
		PCL_ERROR("Couldn't save the PCD file!");
		return(-1);
	}
	
	return (0);
}

//IO::saveTxT
void
saveTxtPoints(string path_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

//IO::save mesh consisting of colored lines
void
saveLinesOfMesh(string path_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

//IO::save lines representing normal vectors
void
saveNormalVectors(string pathname_file, pcl::PointCloud<pcl::Normal>::Ptr input_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr input_points, float length_normal);

//IO::save colored clusters in a point cloud
void
saveColoredClusters(string path_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,std::vector<vector<int> > clusters_points_idx);
void
saveColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);

//IO::show colored results
void
showColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, std::vector<vector<int> > clusters_points_idx);
void
showColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);
void
showColoredPolygons(string fileoutpath_name, const pcl::PolygonMesh::Ptr colored_polygons);



//IO::save segmented clusters
void
saveSegmentedClusters(string path_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,std::vector<vector<int> > clusters_points_idx, int min_points);

//IO::save gray image of the graphical model
void
saveGraphImage(string path_name, Eigen::MatrixXf graph_matrix);

//IO::read & save task file
std::vector<std::string>
inputTaskTxtFile(string pathname_file);//Read the task file

void
outputResultTxtFile(string pathname_file, std::vector<float> input_vector);//Output the results txt file

void
outputFeaturesTxtFile(string pathname_file, std::vector<std::vector<float>> input_vector);//Output the txt file recording the object features

