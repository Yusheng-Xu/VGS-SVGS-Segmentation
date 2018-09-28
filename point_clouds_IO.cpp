////////////////////////////////////////////////////////////////////////////////
//  File:        point_cloud_IO.cpp
//  Author:      Yusheng Xu, PF_Technische Universitaet Muenchen (yusheng.xu@tum.de)
//  Description: IO operation of the point clouds
//  Modified:    28.4.2018
//
//  Copyright (c) 2015-2018  Yusheng Xu (yusheng.xu@tum.de)
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

#include "point_clouds_IO.h"

//IO::save colored clusters in a point cloud
void
saveColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, std::vector<vector<int> > clusters_points_idx)
{
	//Settings
	int clusters_num=0;
	int points_num=0;
	std::vector<int > points_idx;

	pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
	pcl::PointXYZRGB colored_point;

	//Random colors
	std::vector<int> color_map(3);
	srand(static_cast<unsigned int> (time(0)));

	//Record points in each cluster
	clusters_num=clusters_points_idx.size();
	for(int i=0;i<clusters_num;i++)
	{
		std::vector<int > points_idx;
		points_idx=clusters_points_idx.at(i);
		points_num=points_idx.size();

		color_map[0]=static_cast<unsigned int> (rand()%256);
		color_map[1]=static_cast<unsigned int> (rand()%256);
		color_map[2]=static_cast<unsigned int> (rand()%256);

		for(int j=0;j<points_num;j++)
		{
			int point_idx=points_idx.at(j);
			colored_point.x=input_cloud->points[point_idx].x;
			colored_point.y=input_cloud->points[point_idx].y;
			colored_point.z=input_cloud->points[point_idx].z;
			colored_point.r=color_map[0];
			colored_point.g=color_map[1];
			colored_point.b=color_map[2];
			
			//Output		
			colored_cloud.points.push_back(colored_point);
		}	
	}

	//Output point cloud
	//std::string fileout_name("Supervoxel_Test_Clustered_Points.pcd");	
	//std::string fileoutpath_name=path_name+fileout_name;
	colored_cloud.width=colored_cloud.size();
	colored_cloud.height=1;
	pcl::io::savePCDFile(fileoutpath_name,colored_cloud);
}
void
saveColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{
	pcl::io::savePCDFile(fileoutpath_name, *colored_cloud);
}

//IO::show colored results
void
showColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, std::vector<vector<int> > clusters_points_idx)
{
	//Settings
	int clusters_num = 0;
	int points_num = 0;
	std::vector<int > points_idx;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB colored_point;

	//Random colors
	std::vector<int> color_map(3);
	srand(static_cast<unsigned int> (time(0)));

	//Record points in each cluster
	clusters_num = clusters_points_idx.size();
	for (int i = 0; i<clusters_num; i++)
	{
		std::vector<int > points_idx;
		points_idx = clusters_points_idx.at(i);
		points_num = points_idx.size();

		color_map[0] = static_cast<unsigned int> (rand() % 256);
		color_map[1] = static_cast<unsigned int> (rand() % 256);
		color_map[2] = static_cast<unsigned int> (rand() % 256);

		for (int j = 0; j<points_num; j++)
		{
			int point_idx = points_idx.at(j);
			colored_point.x = input_cloud->points[point_idx].x;
			colored_point.y = input_cloud->points[point_idx].y;
			colored_point.z = input_cloud->points[point_idx].z;
			colored_point.r = color_map[0];
			colored_point.g = color_map[1];
			colored_point.b = color_map[2];

			//Output		
			colored_cloud->points.push_back(colored_point);
		}
	}

	//Output point cloud
	colored_cloud->width = colored_cloud->size();
	colored_cloud->height = 1;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, fileoutpath_name);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}
void
showColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, fileoutpath_name);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}

//IO::read & save taskfile 
std::vector<std::string> 
inputTaskTxtFile(string pathname_file)//Read the task file
{
	//Parameters
	int type_method=0;
	string line_string, temp_string;
	std::vector<string> task_vector;

	//Settings
	task_vector.clear();
	
	//Input file 
    std::ifstream taskFile;
	taskFile.open(pathname_file);
	while (std::getline(taskFile, line_string))
    {    
		task_vector.push_back(line_string); 
    }
	taskFile.close();

	return(task_vector);
}
