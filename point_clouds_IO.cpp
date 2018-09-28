////////////////////////////////////////////////////////////////////////////////
//	File:		 point_cloud_IO.cpp
//	Author:		 Yusheng Xu, PF_Technische Universitaet Muenchen (yusheng.xu@tum.de)
//	Description: IO operation of the point clouds
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

//Definition
//IO::saveTxT: X, Y, Z, R, G, B
void
saveTxtPoints(string path_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
	int num_points = 0;
	num_points = input_cloud->points.size();
	pcl::PointXYZRGB temp_point;
	float point_x = 0, point_y = 0, point_z = 0, point_r = 0, point_g = 0, point_b = 0;

	//Write file 
	ofstream outFile;
	outFile.open(path_name);

	for (int i = 0; i < num_points; i++)
	{
		temp_point = input_cloud->points[i];
		point_x = temp_point.x;
		point_y = temp_point.y;
		point_z = temp_point.z;
		point_r = temp_point.r;
		point_g = temp_point.g;
		point_b = temp_point.b;

		string text_line;
		text_line = to_string(point_x) + "," + to_string(point_y) + "," + to_string(point_z) + "," + to_string(point_r) + "," + to_string(point_g) + "," + to_string(point_b);
		outFile << text_line << "\n";
	}
	outFile.close();
}

//IO::save mesh consisting of colored lines
void
saveLinesOfMesh(string path_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	//Settings
	pcl::PolygonMesh output_mesh;
	pcl::PointCloud<pcl::PointXYZRGB> clouds_vertices;
	pcl::Vertices line_vertices;
	pcl::PointXYZRGB colored_vertex1, colored_vertex2;
	
	int lines_num;
	lines_num=input_cloud->points.size()*0.5;

	//Set color of mesh
	std::vector<int> color_map(3);
	srand(static_cast<unsigned int> (time(0)));
	color_map[0]=static_cast<unsigned int> (rand()%256);
	color_map[1]=static_cast<unsigned int> (rand()%256);
	color_map[2]=static_cast<unsigned int> (rand()%256);

	//Record lines of mesh
	int vertex_id1=0,vertex_id2=0;
	for(int i=0;i<lines_num;i++)
	{
		//ID of vertice
		vertex_id1=i*2;
		vertex_id2=i*2+1;

		//Record points of vertice
		colored_vertex1.x=input_cloud->points[vertex_id1].x;
		colored_vertex1.y=input_cloud->points[vertex_id1].y;
		colored_vertex1.z=input_cloud->points[vertex_id1].z;
		colored_vertex1.r=color_map[0];
		colored_vertex1.g=color_map[1];
		colored_vertex1.b=color_map[2];
		
		colored_vertex2.x=input_cloud->points[vertex_id2].x;
		colored_vertex2.y=input_cloud->points[vertex_id2].y;
		colored_vertex2.z=input_cloud->points[vertex_id2].z;
		colored_vertex2.r=color_map[0];
		colored_vertex2.g=color_map[1];
		colored_vertex2.b=color_map[2];

		//Save
		clouds_vertices.points.push_back(colored_vertex1);
		clouds_vertices.points.push_back(colored_vertex2);

		line_vertices.vertices.push_back(vertex_id1);
		line_vertices.vertices.push_back(vertex_id2);
		line_vertices.vertices.push_back(vertex_id1);

		output_mesh.polygons.push_back(line_vertices);
		line_vertices.vertices.clear();
	}

	//Create polygonmesh
	pcl::toPCLPointCloud2(clouds_vertices,output_mesh.cloud);

	//Output mesh
	string fileout_name="Supervoxel_Test_Centers_Mesh.ply";	
	string fileoutpath_name=path_name+fileout_name;
	pcl::io::savePLYFile(fileoutpath_name,output_mesh);
}

//IO::save lines representing normal vectors
void
saveNormalVectors(string pathname_file, pcl::PointCloud<pcl::Normal>::Ptr input_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr input_points, float length_normal)
{
	int points_num = input_normals->points.size();
	pcl::PolygonMesh::Ptr normes_voxels(new pcl::PolygonMesh);

	//Setting
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZRGB> clouds_vertices;

	//Set random color of points in this voxel
	pcl::PointXYZRGB colored_vertex;
	std::vector<int> color_map(3);
	srand(static_cast<unsigned int> (time(0)));

	color_map[0] = static_cast<unsigned int> (rand() % 256);
	color_map[1] = static_cast<unsigned int> (rand() % 256);
	color_map[2] = static_cast<unsigned int> (rand() % 256);
	int k = 0;
	for (int i = 0; i<points_num; i++)
	{
		pcl::PointXYZ node_center = input_points->points[i];

		//Vertices of the voxel
		PCXYZRGBPtr voxel_vertices(new PCXYZRGB);
		voxel_vertices->points.resize(2);

		if (input_normals->points[i].normal_x * 0 == 0 && input_normals->points[i].normal_y * 0 == 0 && input_normals->points[i].normal_z * 0 == 0)
		{
			//Get the points in the voxel
			voxel_vertices->points[0].x = node_center.x;//X
			voxel_vertices->points[1].x = node_center.x + length_normal*input_normals->points[i].normal_x;

			voxel_vertices->points[0].y = node_center.y;//Y
			voxel_vertices->points[1].y = node_center.y + length_normal*input_normals->points[i].normal_y;

			voxel_vertices->points[0].z = node_center.z;//Z
			voxel_vertices->points[1].z = node_center.z + length_normal*input_normals->points[i].normal_z;

			//Color
			for (int j = 0; j<2; j++)
			{
				voxel_vertices->points[j].r = color_map[0];
				voxel_vertices->points[j].g = color_map[1];
				voxel_vertices->points[j].b = color_map[2];
			}

			//Input vertices points
			for (int j = 0; j<2; j++)
			{
				clouds_vertices.points.push_back(voxel_vertices->points[j]);
			}

			//Input vertices topology
			//center to norm
			pcl::Vertices vertice0;
			vertice0.vertices.push_back(k * 2 + 0); vertice0.vertices.push_back(k * 2 + 1); vertice0.vertices.push_back(k * 2 + 0);
			normes_voxels->polygons.push_back(vertice0);
			k++;
		}
	}

	//Create polygonmesh
	pcl::toPCLPointCloud2(clouds_vertices, normes_voxels->cloud);
	pcl::io::savePLYFile(pathname_file, *normes_voxels);
}

//IO::save colored clusters in a point cloud
void
saveColoredClusters(string fileoutpath_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,std::vector<vector<int> > clusters_points_idx)
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
void
showColoredPolygons(string fileoutpath_name, const pcl::PolygonMesh::Ptr colored_polygons)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1.0, 1.0, 1.0);
	viewer->addPolygonMesh(*colored_polygons, fileoutpath_name,0);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

}

//IO::save segmented clusters
void
saveSegmentedClusters(string fileout_name, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,std::vector<vector<int> > clusters_points_idx, int min_points)
{
	//Settings
	int clusters_num=clusters_points_idx.size();
	int points_num=0, out_num=0;

	std::vector<int > points_idx;

	pcl::PointCloud<pcl::PointXYZRGB> segmented_cluster;
	pcl::PointXYZRGB segmented_point;

	//Random colors
	std::vector<int> color_map(3);
	srand(static_cast<unsigned int> (time(0)));

	//Record points in each cluster
	for(int i=0;i<clusters_num;i++)
	{
		std::vector<int > points_idx;
		points_idx=clusters_points_idx.at(i);
		points_num=points_idx.size();

		//Find points in this cluster
		if(points_num>min_points)
		{
			out_num++;

			color_map[0]=static_cast<unsigned int> (rand()%256);
			color_map[1]=static_cast<unsigned int> (rand()%256);
			color_map[2]=static_cast<unsigned int> (rand()%256);

			for(int j=0;j<points_num;j++)
			{
				int point_idx=points_idx.at(j);
				segmented_point.x=input_cloud->points[point_idx].x;
				segmented_point.y=input_cloud->points[point_idx].y;
				segmented_point.z=input_cloud->points[point_idx].z;
				segmented_point.r=color_map[0];
				segmented_point.g=color_map[1];
				segmented_point.b=color_map[2];
				
				//Output		
				segmented_cluster.points.push_back(segmented_point);
			}	

			//Output point cloud
			//string path_name="D:\\Research\\VSGC\\Test\\Results\\";
			//string fileout_name="VSGC_Clustered_Points_";
			segmented_cluster.width=points_num;
			segmented_cluster.height=1;
			stringstream ss;
			ss<<out_num;
			string file_num= ss.str();
			string file_type=".pcd";	
			string fileoutpath_name=fileout_name+file_num+file_type;
			pcl::io::savePCDFile(fileoutpath_name,segmented_cluster);
			segmented_cluster.clear();
		}
	}

}

//IO::save gray image of the graphical model
void
saveGraphImage(string path_name, Eigen::MatrixXf graph_matrix)
{
	int height_size = graph_matrix.rows();
	int width_size= graph_matrix.cols();
	
	graph_matrix = graph_matrix * 255;

	cv::Mat output_matrix;

	cv::eigen2cv(graph_matrix, output_matrix);

	cv::imwrite(path_name, output_matrix);
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

void
outputResultTxtFile(string pathname_file, std::vector<float> input_vector)
{
	int data_size=input_vector.size();

	//Write file 
    ofstream outFile;
	outFile.open(pathname_file);
	for(int i=0;i<data_size;i++)
    {    
		outFile<<input_vector[i]<<"\n"; 
    }
	outFile.close();
}

void
outputFeaturesTxtFile(string pathname_file, std::vector<std::vector<float>> input_vector)
{
	int data_size=input_vector.size();

	//Test
	std::cout<<data_size<<std::endl;

	//Write file 
    ofstream outFile;
	outFile.open(pathname_file);

	for(int i=0;i<data_size;i++)
    {    

		//Test
		//std::cout<<input_vector[i][0]<<" "<<input_vector[i][1]<<" "<<input_vector[i][2]<<std::endl; 
		outFile<<input_vector[i][0]<<" "<<input_vector[i][1]<<" "<<input_vector[i][2]<< "\n"; 
    }
	
	outFile.close();
}
