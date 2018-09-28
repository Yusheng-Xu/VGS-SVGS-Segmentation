////////////////////////////////////////////////////////////////////////////////
//  File:	 supervoxel_segmentation.h
//  Author:      Yusheng Xu, PF_Technische Universitaet Muechen (yusheng.xu@tum.de)
//  Description: The supervoxel based segmentation methods for point cloud 
//  Modified:    29.05.2018, By Dong Lin TU Dresden, Bugs repaired
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
#pragma once

#ifndef PCL_SEGMENTATION_SUPERVOXEL_H_
#define PCL_SEGMENTATION_SUPERVOXEL_H_

#include <vector>
#include <math.h>
#include <limits.h>
#include <queue>
#include <algorithm>

#include <unsupported/Eigen/MatrixFunctions>

#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

//Type definition
typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA> PCXYZRGBA;
typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB> PCXYZRGB;
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr PCXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ> PCXYZ;

typedef  pcl::PointCloud<pcl::Normal>::Ptr PTNORMPtr;
typedef  pcl::PointCloud<pcl::Normal> PTNORM;

 namespace pcl
 {
   template<typename PointT>
   class  SuperVoxelBasedSegmentation: public pcl::octree::OctreePointCloud<PointT>
   {
	//Inherited 
    using pcl::octree::OctreePointCloud<PointT>::input_;
	
	//New ones
	public:
		struct 
		Weight_Index
		{
			float Weight;
			int Index;
		};

		static bool 
		godown(const Weight_Index & a, const Weight_Index & b)
		{
			return a.Weight > b.Weight;
		}

		static bool 
		riseup(const Weight_Index & a, const Weight_Index & b)
		{
			return a.Weight < b.Weight;
		}

		//Construction
		SuperVoxelBasedSegmentation(double input_resolution):OctreePointCloud(input_resolution)
		{
		}

		//Destrction
		~SuperVoxelBasedSegmentation()
		{
		}

		//Member functions
		int 
		getTaskVector(std::vector<std::string> input_vector)
		{
			this->task_vector_=input_vector;
		}

		int
		getCloudPointNum(PCXYZPtr input_data)
		{
			points_num_=input_data->points.size();

			points_cloud_= input_data;

			return (points_num_);
		}

		int
		getVoxelNum()
		{
			voxels_num_=this->voxel_centers_.size();
			return (voxels_num_);
		}

		int
		getSuperVoxelNum()
		{
			return (supervoxels_num_);
		}

		int
		getClusterNum()
		{
			return (clusters_num_);
		}

		std::vector<std::vector<int> >
		getClusterIdx()
		{
			return(clusters_point_idx_);
		}

		Eigen::MatrixXf
		getGlobalGraph()
		{
			return(global_adjacency_matrix_);
		}

		//Voxel & Supervoxel
		void
		setVoxelSize(double input_resolution,int points_num_min)
		{
			this->voxel_resolution_=input_resolution;
			this->voxel_points_min_=points_num_min;
		}

		void
		setSupervoxelSize(double input_resolution,int voxels_num_min, int points_num_min, int adjacency_num_min)
		{
			this->seed_resolution_=input_resolution;
			this->supervoxel_voxel_min_=voxels_num_min;
			this->supervoxel_point_min_=points_num_min;
			this->supervoxel_adjacency_min_=adjacency_num_min;
		}

		void
		setGraphSize(double small_resolution, double large_resolution)
		{
			this->graph_resolution_=large_resolution;		//Global, for the neighboring SV
			this->adjacent_resolution_=small_resolution;	//Local, for the adjacent SV
		}

		void
		setBoundingBox(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z)
		{
			this->min_x_=min_x;
			this->min_y_=min_y;
			this->min_z_=min_z;

			this->max_x_=max_x;
			this->max_y_=max_y;
			this->max_z_=max_z;
		}

		void
		setSupervoxelCentersCentroids()
		{
			//Setting
			SuperVoxelBasedSegmentation<pcl::PointXYZ>::LeafNodeIterator iter_leaf (this); //Iteration index for all leaf nodes in the tree

			PCXYZPtr temp_voxel_centers_cloud (new PCXYZ);
			pcl::PointXYZ voxel_center_pt;

			PCXYZPtr temp_voxel_centroids_cloud (new PCXYZ);
			pcl::PointXYZ voxel_centroid_pt;

			PCXYZPtr temp_voxel_points_cloud (new PCXYZ);

			//Tranverse the octree leaves
			while (*++iter_leaf)
			{
				//Search centers from the key of the leaf node
				pcl::PointXYZ node_center, node_centroid;
				pcl::octree::OctreeKey leaf_key;
				leaf_key=iter_leaf.getCurrentOctreeKey();

				// 根据 leaf node 编号, voxel分辨率 以及 boundary的X,Y,Z的最小值确定 该 leaf node 点的空间坐标
				this->getVoxelCenterFromOctreeKey(leaf_key, node_center);     
				this->voxel_centers_.push_back(node_center);					//Center of the voxel

				//Search idx all the points in this leaf node
				std::vector<int> points_idx;
				points_idx=iter_leaf.getLeafContainer().getPointIndicesVector();
				this->voxels_point_idx_.push_back(points_idx);

				// 根据 voxel 范围内点的坐标(X, Y, Z)平均值, 得到每个体素的质心位置
				//Calculate the centroid of the voxel (质心)
				int voxel_points_num=points_idx.size();
				for (int i=0;i<voxel_points_num;i++)
				{
					temp_voxel_points_cloud->push_back(this->points_cloud_->points[points_idx[i]]);
				}
				node_centroid=this->calculateVoxelCentroid(temp_voxel_points_cloud);
				this->voxel_centroids_.push_back(node_centroid);			   //Centroid of the voxel
				temp_voxel_points_cloud->clear();
			}
			
			//Build cloud
			int voxel_centers_cloud_size=this->voxel_centers_.size();
			this->centers_num_=voxel_centers_cloud_size;

			for (int i=0;i<voxel_centers_cloud_size;i++) 
			{
				voxel_center_pt=this->voxel_centers_[i];
				voxel_centroid_pt=this->voxel_centroids_[i];
				temp_voxel_centers_cloud->points.push_back(voxel_center_pt);
				temp_voxel_centroids_cloud->points.push_back(voxel_centroid_pt);
			}

			temp_voxel_centers_cloud->width = (int)temp_voxel_centers_cloud->points.size();  
			temp_voxel_centers_cloud->height = 1; 

			temp_voxel_centroids_cloud->width = (int)temp_voxel_centroids_cloud->points.size();  
			temp_voxel_centroids_cloud->height = 1; 

			//Assignment
			this->voxel_centers_cloud_=temp_voxel_centers_cloud;		
			this->voxel_centroids_cloud_=temp_voxel_centroids_cloud;	
		}

		//Super-voxelization
		void
		createSupervoxels()
		{
			//Point cloud draw
			pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud (new pcl::PointCloud<pcl::PointXYZL>);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr data_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud_insuper (new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::PointCloud<PointXYZ>::Ptr input_cloud (new pcl::PointCloud<PointXYZ>);

			//Point cloud copy
			input_cloud=this->points_cloud_;
			pcl::copyPointCloud(*input_cloud, *data_cloud);

			//Supervoxels' information
			std::vector<vector<int> > supervoxels_points_idx;

			//Creat supervoxel structure
			//源自 Papon CVPR 2013 的实现, 好像不能调, 只能后续分析结果...
			pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (this->voxel_resolution_, this->seed_resolution_);

			super.setInputCloud (data_cloud);
			super.setColorImportance (this->color_impt_);
			super.setSpatialImportance (this->spatial_impt_);
			super.setNormalImportance (this->normal_impt_);

			// map 的元素以 (key-value) 对的形式存贮
			std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;

			//Oversegmentation using supervoxel
			super.extract (supervoxel_clusters);
			super.refineSupervoxels(5,supervoxel_clusters);
			this->supervoxels_num_=supervoxel_clusters.size();
			pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
			
			//Record points and voxels of each supervoxel
			// labeled_points_cloud->points[点号].label 
			pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_points_cloud = super.getLabeledCloud ();
			int max_label=super.getMaxLabel();
			int size_labeled_points=labeled_points_cloud->points.size();
			std::vector<int> labels_voxels_idxs;

			//Create label maps
			std::vector<std::vector<int>> points_label_map, voxels_label_map;
			std::vector<int> label_list,label_temp;
			for(int g=0;g<max_label+1;g++)
			{
				points_label_map.push_back(label_list);
				voxels_label_map.push_back(label_list);
			}

			//Traverse all the points
			// points_label_ [点号] 表示 label
			int point_label;
			for (int j = 0; j < size_labeled_points; j++)
			{
				point_label = labeled_points_cloud->points[j].label;
				if (point_label>0)
				{
					points_label_.push_back(point_label);
					points_label_map[point_label].push_back(j);
				}
			}

			//Mapping the index between supervoxels, voxels, and points
			// points_label_map [label] 表示 一堆点号
			int temp_sv_num=0;
			for(int k=0;k<max_label;k++)
			{
				if(points_label_map[k].size()>0)
				{
					supervoxels_point_idx_.push_back(points_label_map[k]);
					supervoxels_label_.push_back(labeled_points_cloud->points[points_label_map[k][0]].label);
					// supervoxels_label_.push_back(k); 跟上面的结果应该一样....
					temp_sv_num++;
				}
			}
			supervoxels_num_=temp_sv_num;

			//Test
			//Output colored cloud: method 1
			pcl::PointCloud<PointXYZRGB>::Ptr supervoxel_cloud (new pcl::PointCloud<PointXYZRGB>);
			pcl::PointXYZRGB temp_colored_point;
			std::vector<int> color_map(3);
			srand(static_cast<unsigned int> (time(0)));
			for (int k=0;k<supervoxels_num_;k++)
			{
				//Set random color of points in this voxel
				color_map[0]=static_cast<unsigned int> (rand()%256);
				color_map[1]=static_cast<unsigned int> (rand()%256);
				color_map[2]=static_cast<unsigned int> (rand()%256);
			
				for(int l=0;l<this->supervoxels_point_idx_[k].size ();l++)
				{
					temp_colored_point.x=this->points_cloud_->points[supervoxels_point_idx_[k][l]].x;
					temp_colored_point.y=this->points_cloud_->points[supervoxels_point_idx_[k][l]].y;
					temp_colored_point.z=this->points_cloud_->points[supervoxels_point_idx_[k][l]].z;
					temp_colored_point.r=color_map[0];
					temp_colored_point.g=color_map[1];
					temp_colored_point.b=color_map[2];
				
					supervoxel_cloud->push_back(temp_colored_point);
				}
			}

			//// 点云漫游显示
			//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
			//viewer->setBackgroundColor(0, 0, 0);
			//viewer->addPointCloud<pcl::PointXYZRGB>(supervoxel_cloud, "supervoxel_cloud");
			//while (!viewer->wasStopped())
			//{
			//	viewer->spinOnce(100);
			//}
		}

		//Segmentation
		void
		segmentSupervoxelCloudWithGraphModel(float sig_a, float sig_b, float sig_l, float cut_thred,
		float sig_p, float sig_n, float sig_o, float sig_e, float sig_c, float sig_w)      //Segmentation
		{
			//Parameter setting
			this->color_impt_=sig_a;
			this->spatial_impt_=sig_b;
			this->normal_impt_=sig_l;

			//Create supervoxels
			this->createSupervoxels();

			//Attribute calculation
			this->calcualteSupervoxelCloudAttributes();

			//Find adjacencies
			this->findAllSupervoxelAdjacency(); 

			//Find neighbors
			this->findAllSupervoxelNeighbors();

			//Connectivity calculation
			for(int i=0;i<this->supervoxels_num_;i++)
			{
				//Judgement
				if(this->supervoxel_used_[i])//Supervoxel used or not
				{
					Eigen::MatrixXf supervoxel_adjgraph;
					std::vector<int> supervoxel_connect;
					std::vector<int> supervoxel_adjidx;
					
					//Get neighbors
					supervoxel_adjidx=this->getOneSupervoxelNeighbor(i);
					
					//Graph building
					supervoxel_adjgraph=this->buildAdjacencyGraph(supervoxel_adjidx, sig_p, sig_n, sig_o, sig_e, sig_c, sig_w);
					
					//Graph based segmentation
					supervoxel_connect=this->cutGraphSegmentation(cut_thred,supervoxel_adjgraph,supervoxel_adjidx);
					
					//Store the connection information
					supervoxels_connect_idx_.push_back(supervoxel_connect);

				}
				else
				{
					//Store the connection informaiton
					std::vector<int> supervoxel_connect;
					supervoxels_connect_idx_.push_back(supervoxel_connect);
				}

			}
	
			//Cross validation
 			this->crossValidation();
			//Closest checking
			this->closestCheck(sig_p, sig_n, sig_o, sig_e, sig_c, sig_w);
			//Voxel Clustering
			this->clusteringSupervoxels();
		}

		//Display
		void
		drawNormofVoxels(pcl::PolygonMesh::Ptr output_mesh)
		{
			//Setting
			std::vector<pcl::Vertices> polygons; 
			pcl::PointCloud<pcl::PointXYZRGB> clouds_vertices;

			int spvoxel_ID;
			int points_size;
			int points_min=this->supervoxel_point_min_;

			//Set random color of points in this voxel
			pcl::PointXYZRGB colored_vertex;
			std::vector<int> color_map(3);
			srand(static_cast<unsigned int> (time(0)));
			color_map[0]=static_cast<unsigned int> (rand()%256);
			color_map[1]=static_cast<unsigned int> (rand()%256);
			color_map[2]=static_cast<unsigned int> (rand()%256);

			//Tranverse all the supervoxels
			for(int i=0;i<this->supervoxels_num_;i++)
			{
				int point_num=0;
				int point_idx=0;
				int node_depth=0;
				pcl::PointXYZ spvoxel_center;

				//Vertices of the supervoxel
				PCXYZRGBPtr spvoxel_vertices (new PCXYZRGB);
				spvoxel_vertices->points.resize(2);

				//Find the atttributes of the supervoxel
				spvoxel_ID=i;				
				points_size=this->supervoxels_point_idx_[spvoxel_ID].size();
				spvoxel_center=this->supervoxel_centroids_cloud_->points[spvoxel_ID];

				//Traverse the points in node and color them
				if(points_size>points_min)
				{
					//Get the points in the voxel	
					spvoxel_vertices->points[0].x=spvoxel_center.x;//X
					spvoxel_vertices->points[1].x=spvoxel_center.x+this->seed_resolution_*this->supervoxel_norms_[spvoxel_ID].normal_x;
									
					spvoxel_vertices->points[0].y=spvoxel_center.y;//Y
					spvoxel_vertices->points[1].y=spvoxel_center.y+this->seed_resolution_*this->supervoxel_norms_[spvoxel_ID].normal_y;

					spvoxel_vertices->points[0].z=spvoxel_center.z;//Z
					spvoxel_vertices->points[1].z=spvoxel_center.z+this->seed_resolution_*this->supervoxel_norms_[spvoxel_ID].normal_z;

					//Color
					for(int j=0;j<2;j++)
					{
						spvoxel_vertices->points[j].r=color_map[0];
						spvoxel_vertices->points[j].g=color_map[1];
						spvoxel_vertices->points[j].b=color_map[2];
					}

					//Input vertices points
					for(int j=0;j<2;j++)
					{
						clouds_vertices.points.push_back(spvoxel_vertices->points[j]);
					}

					//Input vertices topology
					//center to norm
					pcl::Vertices vertice0;
					vertice0.vertices.push_back(i*2+0);vertice0.vertices.push_back(i*2+1);vertice0.vertices.push_back(i*2+0);
					output_mesh->polygons.push_back(vertice0);
				}
			}

			//Create polygonmesh
			pcl::toPCLPointCloud2(clouds_vertices,output_mesh->cloud);

		}

		void
		drawColorMapofPointsinVoxels(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud)
		{
			//Setting
			int points_size;
			int points_min=this->voxel_points_min_;

			pcl::PointXYZ original_point; 
			pcl::PointXYZRGB colored_point;
			std::vector<int> color_map(3);
			srand(static_cast<unsigned int> (time(0)));

			//VoxelBasedSegmentation<pcl::PointXYZ>::LeafNodeIterator iter_leaf (this); //Iteration index for all leaf nodes in the tree

			//Tranverse the octree voxel
			for(int i=0; i<this->voxels_num_;i++)//while (*++iter_leaf)
			{
				int point_num=0;
				int point_idx=0;

				//Set random color of points in this voxel
				color_map[0]=static_cast<unsigned int> (rand()%256);
				color_map[1]=static_cast<unsigned int> (rand()%256);
				color_map[2]=static_cast<unsigned int> (rand()%256);

				std::vector<int> index_vector;
				PCXYZPtr voxel_cloud (new PCXYZ);

				//Traverse the points in the voxel and color them
				bool temp_bool=true;
				int points_size=this->voxels_point_idx_[i].size();

				//if(points_size>points_min)
				//{
					//Get the points in the voxel
					while(point_num<points_size)
					{
						point_idx=this->voxels_point_idx_[i][point_num];
						original_point=this->points_cloud_->points[point_idx];//Find original point
						
						colored_point.x=original_point.x;
						colored_point.y=original_point.y;
						colored_point.z=original_point.z;
						colored_point.r=color_map[0];
						colored_point.g=color_map[1];
						colored_point.b=color_map[2];

						//Coloring point
						output_cloud->points.push_back(colored_point);//Put colored point in the cloud

						point_num++;
					}
				//}
			}

			output_cloud->width = (int)output_cloud->points.size();  
			output_cloud->height = 1;
		
		}

		void
		drawColorMapofPointsinSupervoxels(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud)
		{
			//Setting
			int points_size;
			int points_min=this->voxel_points_min_;

			pcl::PointXYZ original_point; 
			pcl::PointXYZRGB colored_point;
			std::vector<int> color_map(3);
			srand(static_cast<unsigned int> (time(0)));

			//Tranverse the octree voxel
			for(int i=0; i<this->supervoxels_num_;i++)
			{
				int point_num=0;
				int point_idx=0;

				//Set random color of points in this voxel
				color_map[0]=static_cast<unsigned int> (rand()%256);
				color_map[1]=static_cast<unsigned int> (rand()%256);
				color_map[2]=static_cast<unsigned int> (rand()%256);

				std::vector<int> index_vector;
				PCXYZPtr voxel_cloud (new PCXYZ);

				//Traverse the points in the voxel and color them
				bool temp_bool=true;
				int points_size=this->supervoxels_point_idx_[i].size();

				while(point_num<points_size)
				{
					point_idx=this->supervoxels_point_idx_[i][point_num];
					original_point=this->points_cloud_->points[point_idx];//Find original point
						
					colored_point.x=original_point.x;
					colored_point.y=original_point.y;
					colored_point.z=original_point.z;
					colored_point.r=color_map[0];
					colored_point.g=color_map[1];
					colored_point.b=color_map[2];

					//Coloring point
					output_cloud->points.push_back(colored_point);//Put colored point in the cloud

					point_num++;
				}
			}

			output_cloud->width = (int)output_cloud->points.size();  
			output_cloud->height = 1;
		}

		void
		drawColorMapofPointsinClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud)
		{
			//Geometry
			std::vector<pcl::Vertices> polygons; 
			pcl::PointCloud<pcl::PointXYZRGB> clouds_vertices;
			std::vector<int> use_or_not;

			//Setting
			int i=0;
			int points_size=0;
			int out_num=0;

			//Random color
			pcl::PointXYZRGB colored_point;
			pcl::PointXYZ original_point;
			std::vector<int> color_map(3);
			srand(static_cast<unsigned int> (time(0)));

			for (int m = 0; m<this->clusters_supervoxel_idx_.size(); m++)
			{
				color_map[0] = static_cast<unsigned int> (rand() % 256);
				color_map[1] = static_cast<unsigned int> (rand() % 256);
				color_map[2] = static_cast<unsigned int> (rand() % 256);

				int supervoxels_size = this->clusters_supervoxel_idx_[m].size();

				for (int n= 0; n<supervoxels_size; n++)
				{ 
					int supervoxel_idx = this->clusters_supervoxel_idx_[m][n];						 
					//Traverse the points in node and color them
					int point_idx = 0; //Get the points in the voxel
					std::vector<int> points_idx;
					points_idx = this->supervoxels_point_idx_[supervoxel_idx];
					points_size = points_idx.size();

					//Coloring
					for (int i = 0; i<points_size; i++)
					{
						original_point = this->points_cloud_->points[points_idx[i]];
						colored_point.x = original_point.x;
						colored_point.y = original_point.y;
						colored_point.z = original_point.z;
						colored_point.r = color_map[0];
						colored_point.g = color_map[1];
						colored_point.b = color_map[2];
						output_cloud->push_back(colored_point);
					}
				}
				
			}
		}

    private:
		
		//Member variables 
		int points_num_;				//Num of input points
		int voxels_num_;				//Num of segmented voxels
		int supervoxels_num_;			//Num of generated supervoxels
		int clusters_num_;				//Num of obtained clusters
		int segments_num_;
		int centers_num_;				//Centers of all the voxels
		
		int voxel_points_num_;			//Num of points in a voxel
		int supervoxel_points_num_;		//Num of points in a super voxel
		int cluster_points_num_;		//Num of points in a cluster

		float voxel_resolution_;		//Size of the voxel
		float seed_resolution_;			//Size of the supervoxel
		float adjacent_resolution_;		//Radius of the small local graph	
		float graph_resolution_;		//Radius of the large global graph

		int voxel_points_max_;			//Max num of points in a voxel
		int voxel_points_min_;			//Min num of points in a voxel
		int voxel_adjacency_max_;		//Max num of adjacent voxel
		int voxel_adjacency_min_;		//Min num of adjacent voxel
		int supervoxel_adjacency_max_;	//Max num of adjacent supervoxel
		int supervoxel_adjacency_min_;	//Min num of adjacent supervoxel
		int supervoxel_voxel_max_;		//Max num of voxels in a supervoxel
		int supervoxel_voxel_min_;		//Min num of voxels in a supervoxel
		int supervoxel_point_max_;      //Max num of points in a supervoxel
		int supervoxel_point_min_;		//Min num of points in a supervoxel

		float min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
		float color_impt_, spatial_impt_, normal_impt_;

		std::vector<std::string> task_vector_;

		Eigen::MatrixXf global_adjacency_matrix_;

		std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxel_centers_, supervoxel_centers_;					//Center point of the voxels
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxel_centroids_, supervoxel_centroids_;				//Centroid point of the voxels
		std::vector<bool> voxel_used_, supervoxel_used_, voxel_clustered_, supervoxel_clustered_;									//The voxel has been clutered or not
		
		std::vector<std::vector<float> > supervoxel_features_;																		//FPFHs of all the supervoxels
		std::vector<std::vector<float> > supervoxel_eigens_;																		//Eigen features of all the supervoxels
		std::vector<std::vector<float> > supervoxel_colors_;																		//Colors of all the supervoxels
		std::vector<pcl::Normal> supervoxel_norms_;																					//Norms of all the supervoxels
		std::vector<float> supervoxel_volumes_;																						//Volumes of all the supervoxels

		std::vector<std::vector<int> > voxels_adjacency_idx_, supervoxels_adjacency_idx_, supervoxels_neighbor_idx_;				//Idx of the adjacent voxels, adjacent supervoxels, neighboring supervoxels
		std::vector<std::vector<int> > voxels_connect_idx_, supervoxels_connect_idx_;												//Idx of the connected voxels, supervoxels
		std::vector<std::vector<int> > voxels_point_idx_, supervoxels_point_idx_, clusters_supervoxel_idx_, clusters_point_idx_;	//Idx of points, supervoxels, and cluster
		
		std::vector<int > supervoxels_label_, voxels_label_, points_label_;															//Labels of supervoxels and points

		pcl::KdTreeFLANN<pcl::PointXYZ> point_centers_kdtree_, voxel_centers_kdtree_, voxel_centroids_kdtree_, supervoxel_centers_kdtree_, supervoxel_centroids_kdtree_, cluster_centers_kdtree_;		//kd tree
		
		PCXYZPtr points_cloud_, voxel_centers_cloud_, voxel_centroids_cloud_, supervoxel_centers_cloud_, supervoxel_centroids_cloud_, cluster_centers_cloud_;			//PT Ptr of points and centers

		//Member functions
		void
		buildVoxelCentersKdtree()
		{
			//Input cloud of voxel centers
			this->voxel_centers_kdtree_.setInputCloud (this->voxel_centers_cloud_);

			//Input cloud of voxel centroids
			this->voxel_centroids_kdtree_.setInputCloud (this->voxel_centroids_cloud_);
		}
		
		void
		buildSupervoxelCentersKdtree()
		{
			//Input cloud of supervoxel centers
			//this->supervoxel_centers_kdtree_.setInputCloud (this->supervoxel_centers_cloud_);

			//Input cloud of supervoxel centroids
			this->supervoxel_centroids_kdtree_.setInputCloud (this->supervoxel_centroids_cloud_);
		}

		//Features
		pcl::PointXYZ
		calculateVoxelCentroid(PCXYZPtr input_cloud)
		{
			//Parameter setting
			pcl::PointXYZ output_centroid;
			int point_num=input_cloud->points.size();
			float x_sum=0,y_sum=0,z_sum=0;

			for(int i=0;i<point_num;i++)
			{
				x_sum=x_sum+input_cloud->points[i].x;
				y_sum=y_sum+input_cloud->points[i].y;
				z_sum=z_sum+input_cloud->points[i].z;
			}

			output_centroid.x=x_sum/point_num;
			output_centroid.y=y_sum/point_num;
			output_centroid.z=z_sum/point_num;

			return(output_centroid);
		}

		//Eight dimentional Eigen-based features (Weinmann et al. 2015 ISPRS Journal)
		std::vector<float>
		calculateEigenFeatures(PCXYZPtr input_cloud)//Eigen features
		{
			//Parameter setting
			Eigen::Vector3f eig_values;
			Eigen::Matrix3f eig_vectors;
			Eigen::Matrix3f *cor_matrix=new Eigen::Matrix3f;
			std::vector<float> output_features;

			int point_num=0;
			point_num=input_cloud->points.size();
			float eig_e1=0,eig_e2=0,eig_e3=0;
			//float *features=new float[8];

			// 自己构造好协方差矩阵就好, 特征值和特征向量的计算用Eigen库中自带就好
			//Weighted corvarance matrix
			//this->calculateWeightedCorvariance(input_cloud,cor_matrix);
			this->calculateCorvariance(input_cloud,cor_matrix);

			//EVD
			pcl::eigen33 (*cor_matrix, eig_vectors, eig_values);

			//Eigen values (normalized)
			//一共八维特征...完全按照 Weinmann 2015 ISPRS 文章上面的特征
			if(eig_values[0]==0 && eig_values[1]==0 && eig_values[2]==0)
			{
				for(int i=0;i<8;i++)
				{
					output_features.push_back(float(0));
				}
			}
			else
			{
				//e1>e2>e3 (跑程序时检查下是否满足这个规律...)
				eig_e3=(float)eig_values[0]/sqrt(pow(eig_values[0],2)+pow(eig_values[1],2)+pow(eig_values[2],2));
				eig_e2=(float)eig_values[1]/sqrt(pow(eig_values[0],2)+pow(eig_values[1],2)+pow(eig_values[2],2));
				eig_e1=(float)eig_values[2]/sqrt(pow(eig_values[0],2)+pow(eig_values[1],2)+pow(eig_values[2],2));


				//Feature calculation
				if(eig_e1==0) // e1 应该是最大的???
				{
					output_features.push_back(float(0));//Linearity
					output_features.push_back(float(1));//Planarity
					output_features.push_back(float(0));//Scattering
					output_features.push_back(float(0));//Anisotropy 各向异性
				}
				else
				{
					output_features.push_back(float(eig_e1-eig_e2)/eig_e1);//Linearity
					output_features.push_back(float(eig_e2-eig_e3)/eig_e1);//Planarity
					output_features.push_back(float(eig_e3)/eig_e1);//Scattering or Sphericity
					output_features.push_back(float(eig_e1 - eig_e3) / eig_e1);//Anisotropy 
					// 查 Weinmann (2015) ISPRS: 正确形式如上, 原错误代码 (e1 - e3) / e2 
				}

				output_features.push_back(float(eig_e3)/(eig_e1+eig_e2+eig_e3));//Change of curvature or Surface variation
				// 查 Weinmann (2015) ISPRS: 正确形式如上, 原错误代码 float(eig_e1)/(eig_e1+eig_e2+eig_e3)

				if(eig_e1*eig_e2*eig_e3==0)
				{
					output_features.push_back(float(0));//Eigenentropy
				}
				else
				{
					output_features.push_back(-1*(eig_e1*log(eig_e1)+eig_e2*log(eig_e2)+eig_e3*log(eig_e3)));//Eigenentropy
				}
				
				output_features.push_back(eig_e1+eig_e2+eig_e3);//Sum of eigen values
				output_features.push_back(pow(float(eig_e1*eig_e2*eig_e3),float(1.0/3)));//Omnivariance
			}

			////Test 
			//std::cout<<"Eigen Features: "<<std::endl;
			//std::cout<<" "<<output_features[0]<<" "<<output_features[1]<<" "<<output_features[2]<<" "<<output_features[3]<<std::endl;
			//std::cout<<" "<<output_features[4]<<" "<<output_features[5]<<" "<<output_features[6]<<" "<<output_features[7]<<std::endl;
			//std::cout<<""<<std::endl;
			
			return(output_features);
		}

		std::vector<float>
		calculateFPFHFeatures(PCXYZPtr input_cloud)
		{
			//Parameter setting
			std::vector<float> output_features;
			pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_descriptor;
			pcl::PointCloud<pcl::Normal>::Ptr input_norms (new pcl::PointCloud<pcl::Normal> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::Normal candidate_norm;
			pcl::PointXYZ candidate_point, sum_point, center_point;

			sum_point.x=0;sum_point.y=0;sum_point.z=0;

			int points_num=0;
			float norm_radius=this->voxel_resolution_;		//Norm radius = voxel size
			float fpfh_radius=this->seed_resolution_;	//FPFH radius = supervoxel size 
			std::vector<int> points_indices;

			//Building a kd tree
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud;
			kdtree_cloud.setInputCloud (input_cloud);

			//Calculate norms of voxels
			points_num=input_cloud->points.size();
			for(int i=0;i<points_num;i++)
			{
				//Find candidate point
				candidate_point=input_cloud->points[i];
				std::vector<int> candidate_cloud_idx;
				std::vector<float> candidate_cloud_dis;

				//Select candidate points 
				if(kdtree_cloud.radiusSearch(candidate_point,norm_radius, candidate_cloud_idx, candidate_cloud_dis)>0)
				{
					for(size_t j=0;j<candidate_cloud_idx.size();j++)
					{
						candidate_cloud->push_back(input_cloud->points[candidate_cloud_idx[j]]);
					}
				}

				//Calculate norm
				candidate_norm=this->calculateSupervoxelNorms(candidate_cloud);

				//Record norms
				input_norms->push_back(candidate_norm);
				points_indices.push_back(i);//Record the indices
				
				//Sum for the centroid point
				sum_point.x=sum_point.x+candidate_point.x;
				sum_point.y=sum_point.y+candidate_point.y;
				sum_point.z=sum_point.z+candidate_point.z;
			}
			
			//Centroid
			pcl::PointXYZ centroid_point;
			centroid_point.x=sum_point.x/points_num;
			centroid_point.y=sum_point.y/points_num;
			centroid_point.z=sum_point.z/points_num;

			//Center candidate selection
			std::vector<int> center_point_idx;
			std::vector<float> center_point_dis;
			kdtree_cloud.nearestKSearch(centroid_point,1,center_point_idx,center_point_dis);

			center_point=input_cloud->points[center_point_idx[0]];

			//Calculate the FPFH features
			Eigen::MatrixXf hist_f1, hist_f2, hist_f3,hist_spfh;
			Eigen::VectorXf hist_sum;
			hist_f1=Eigen::MatrixXf::Zero(points_num,11);
			hist_f2=Eigen::MatrixXf::Zero(points_num,11);
			hist_f3=Eigen::MatrixXf::Zero(points_num,11);
			hist_spfh=Eigen::MatrixXf::Zero(points_num,33);
			hist_sum=Eigen::VectorXf::Zero(33);
			std::vector<float> center_candidate_dis;
			for(int k=0;k<points_num;k++)
			{
				float temp_dis;
				pcl::PointXYZ temp_point;

				//Calculate SPFM feature
				fpfh_descriptor.computePointSPFHSignature(*input_cloud,*input_norms,k,k,points_indices, hist_f1, hist_f2, hist_f3);

				//Distance weight
				temp_point.x=center_point.x-input_cloud->points[k].x;
				temp_point.y=center_point.y-input_cloud->points[k].y;
				temp_point.z=center_point.z-input_cloud->points[k].z;
				
				temp_dis=sqrt(pow(temp_point.x,2)+pow(temp_point.y,2)+pow(temp_point.z,2));
				center_candidate_dis.push_back(temp_dis);
			
				//Weighted result
				for(int m=0;m<11;m++)
				{

					if(temp_dis!=0)
					{
						hist_spfh(k,m)=hist_f1(k,m)/temp_dis;
						hist_spfh(k,m+11)=hist_f2(k,m)/temp_dis;
						hist_spfh(k,m+22)=hist_f3(k,m)/temp_dis;

						hist_sum(m)=hist_sum(m)+hist_spfh(k,m);
						hist_sum(m+11)=hist_sum(m+11)+hist_spfh(k,m+11);
						hist_sum(m+22)=hist_sum(m+22)+hist_spfh(k,m+22);
					}
					/*std::cout<<hist_sum(m+11)<<"  "<<std::endl;*/
				}

				/*std::cout<<"  "<<std::endl;*/
			}

			//Record features
			for(int n=0;n<33;n++)
			{
				float temp_bin;

				temp_bin=hist_spfh(center_point_idx[0],n)+hist_sum(n)/(points_num-1);

				output_features.push_back(temp_bin);

				//std::cout<<temp_bin<<"  ";
			}

			float max_bin,sum_bin;
			//max_bin=*std::max_element(output_features.begin(),output_features.end());
			sum_bin=std::accumulate(output_features.begin(),output_features.end(),0);
			//std::cout<<max_bin<<"  "<<sum_bin<<std::endl;

			//Normalized
			if(sum_bin>0)
			{
				for(int n=0;n<33;n++)
				{
					output_features[n]/=sum_bin;
				}
			}
			return(output_features);
		}

		pcl::Normal
		calculateSupervoxelNorms(PCXYZPtr input_cloud)
		{
			//Parameter setting
			Eigen::Vector3f eig_values;
			Eigen::Matrix3f eig_vectors;
			Eigen::Matrix3f *cor_matrix=new Eigen::Matrix3f;
			pcl::Normal output_normal;
			pcl::PointXYZ view_point; // Why is [0, 0, 1.5]?

			int point_num=input_cloud->points.size();
			//view_point.x=0-input_cloud->points[0].x;
			//view_point.y=0-input_cloud->points[0].y;
			//view_point.z=0-input_cloud->points[0].z;
			view_point.x = 0 - input_cloud->points[0].x;
			view_point.y = 0 - input_cloud->points[0].y;
			view_point.z = 1.5 - input_cloud->points[0].z;
			float eig_e1=0,eig_e2=0,eig_e3=0;

			//Normal corvarance matrix
			this->calculateCorvariance(input_cloud,cor_matrix);
			//Weighted corvarance matrix
			//this->calculateWeightedCorvariance(input_cloud,cor_matrix);			
			//EVD
			pcl::eigen33 (*cor_matrix, eig_vectors, eig_values);

			//Eigen values (e1 > e2 > e3)
			eig_e1=eig_values[2];
			eig_e2=eig_values[1];
			eig_e3=eig_values[0];

			//Feature calculation
			//normal vector: the eigenvector corresponding to the smallest eigenvalue (eig_values[0]) of corvarance matrix
			output_normal.normal_x=eig_vectors(0,0);
			output_normal.normal_y=eig_vectors(1,0);
			output_normal.normal_z=eig_vectors(2,0);

			//Direction judgement
			//强制让 视向量 与 体素法向量 同向...
			if((output_normal.normal_x*view_point.x+output_normal.normal_y*view_point.y+output_normal.normal_z*view_point.z)<0)
			{
				output_normal.normal_x=output_normal.normal_x*-1;
				output_normal.normal_y=output_normal.normal_y*-1;
				output_normal.normal_z=output_normal.normal_z*-1;			
			}

			//Test 
			//std::cout<<"Norms: "<<std::endl;
			//std::cout<<" "<<output_normal.normal_x<<" "<<output_normal.normal_y<<" "<<output_normal.normal_z<<std::endl;
			//std::cout<<""<<std::endl;
			
			return(output_normal);
		}

		std::vector<float>
		calculateSupervoxelColors(PCXYZPtr input_cloud)
		{
			//Parameter setting
			std::vector<float> output_colors;
			
			//Test 
			//std::cout<<"Norms: "<<std::endl;
			//std::cout<<" "<<output_normal.normal_x<<" "<<output_normal.normal_y<<" "<<output_normal.normal_z<<std::endl;
			//std::cout<<""<<std::endl;
			return(output_colors);
		}

		float
		calculateSupervoxelVolume(PCXYZPtr input_cloud)
		{
			//Parameter setting
			float output_size=0;
			float x_min=0,x_max=0,y_min=0,y_max=0,z_min=0,z_max=0;
			float x_temp=0, y_temp=0, z_temp=0;

			if(input_cloud->points.size()>0)
			{
				x_min=input_cloud->points[0].x;
				x_max=input_cloud->points[0].x;
				y_min=input_cloud->points[0].y;
				y_max=input_cloud->points[0].y;
				z_min=input_cloud->points[0].z;
				z_max=input_cloud->points[0].z;
			}

			for(int i=1;i<input_cloud->points.size();i++)
			{
				x_temp=input_cloud->points[i].x;
				y_temp=input_cloud->points[i].y;			
				z_temp=input_cloud->points[i].z;

				if(x_temp<x_min)
				{
					x_min=x_temp;
				}

				if(x_temp>x_max)
				{
					x_max=x_temp;
				}

				if(y_temp<y_min)
				{
					y_min=y_temp;
				}

				if(y_temp>y_max)
				{
					y_max=y_temp;
				}

				if(z_temp<z_min)
				{
					z_min=z_temp;
				}

				if(z_temp>z_max)
				{
					z_max=z_temp;
				}
			}

			if(z_max==z_min)
			{
				output_size=(x_max-x_min)*(y_max-y_min);
			}
			else if (y_max==y_min)
			{
				output_size=(x_max-x_min)*(z_max-z_min);
			}
			else if (x_max==x_min)
			{
				output_size=(y_max-y_min)*(z_max-z_min);
			}
			else
			{
				output_size=(x_max-x_min)*(y_max-y_min)*(z_max-z_min);
			}
			return(output_size);
		}

		void
		setSupervoxelCentroid(int supervoxel_id, pcl::PointXYZ supervoxel_centroid)
		{
			this->supervoxel_centroids_.at(supervoxel_id)=supervoxel_centroid;
			this->supervoxel_centroids_cloud_->points.push_back(supervoxel_centroid);
		}

		void
		setSupervoxelCenter(int supervoxel_id, pcl::PointXYZ supervoxel_center)
		{
			this->supervoxel_centers_.at(supervoxel_id)=supervoxel_center;
			this->supervoxel_centers_cloud_->points.push_back(supervoxel_center);
		}

		void
		setSupervoxelNorms(int supervoxel_id, pcl::Normal supervoxel_norm)
		{
			this->supervoxel_norms_.at(supervoxel_id)=supervoxel_norm;
		}

		void
		setSupervoxelFeatures(int supervoxel_id, std::vector<float> input_features)
		{
			//int features_num=input_features.size();
			this->supervoxel_features_[supervoxel_id].clear();
			//for(int i=0;i<features_num;i++)
			//{
			//	voxel_features_[voxel_id].push_back(input_features[i]);
			//}
			this->supervoxel_features_[supervoxel_id]=input_features;

		}

		void
		setSupervoxelEigens(int supervoxel_id, std::vector<float> input_eigens)
		{
			//int eigens_num=input_eigens.size();
			this->supervoxel_eigens_[supervoxel_id].clear();
			//for(int i=0;i<eigens_num;i++)
			//{
			//	voxel_eigens_[voxel_id].push_back(input_eigens[i]);
			//}
			this->supervoxel_eigens_[supervoxel_id]=input_eigens;
		}

		void
		setSupervoxelColors(int supervoxel_id, std::vector<float> input_colors)
		{
			this->supervoxel_colors_.at(supervoxel_id)=input_colors;
		}

		void
		setSupervoxelVolume(int supervoxel_id, float input_vol)
		{	
			this->supervoxel_volumes_.at(supervoxel_id)=input_vol;
		}

		void
		setSupervoxelChosen(int supervoxel_idx,  int chosen_ornot)
		{
			if(chosen_ornot==0)
			{
				this->supervoxel_used_.push_back(true);
			}
			else
			{
				this->supervoxel_used_.push_back(false);			
			}
		}

		void
		setSupervoxelClustered(int supervoxel_idx,  int clustered_ornot)
		{
			if(clustered_ornot==0)
			{
				this->supervoxel_clustered_.push_back(true);
			}
			else
			{
				this->supervoxel_clustered_.push_back(false);			
			}
		}

		void
		initialSupervoxelAttributes()
		{
			std::vector<float> empty_attribute(1,0);
			pcl::Normal empty_norm;
			pcl::PointXYZ empty_point;
			float empty_value;

			for(int i=0;i<this->supervoxels_num_;i++)
			{
				supervoxel_features_.push_back(empty_attribute); //FPFH
				supervoxel_eigens_.push_back(empty_attribute);	 //Eigens
				supervoxel_colors_.push_back(empty_attribute);	 //Color
				supervoxel_norms_.push_back(empty_norm);		 //Norm
				supervoxel_centroids_.push_back(empty_point);	 //Centroid
				supervoxel_centers_.push_back(empty_point);		 //Center
				supervoxel_volumes_.push_back(empty_value);		 //Volume
			}

			//Initialization of clouds
			PCXYZPtr temp_cloud1 (new PCXYZ);
			PCXYZPtr temp_cloud2 (new PCXYZ);
			this->supervoxel_centroids_cloud_=temp_cloud1;
			this->supervoxel_centers_cloud_=temp_cloud2;
		}

		void
		calcualteSupervoxelCloudAttributes()
		{
			//Parameters and settings
			int sv_num = 0;
			std::vector<float> supervoxel_features; // FPFH, 33 values
			std::vector<float> supervoxel_eigens;  // L, P, S, C, 4 values + the rest of the 5 eigen features from Dr. Martin Weinmann 
			std::vector<float> supervoxel_colors; // R,G,B color, 3 values
			pcl::Normal supervoxel_norm;		 // x,y,z, 3 values
			pcl::PointXYZ supervoxel_centroid;	// x,y,z, 3 values
			float supervoxel_volume;		  // vol, 1 value

			//Initial feature
			this->initialSupervoxelAttributes();

			//Traverse all the supervoxels
			int valid_supervoxels_num = 0;
 			for (int i=0;i<this->supervoxels_num_;i++)
			{

				int points_num=this->supervoxels_point_idx_[i].size();

				valid_supervoxels_num++;

				PCXYZPtr supervoxel_cloud (new PCXYZ);

				//Get the points in this supervoxel
				for(int j=0;j<points_num;j++)
				{
					supervoxel_cloud->push_back(this->points_cloud_->points[this->supervoxels_point_idx_[i][j]]);
				}

				//Spatial position
				supervoxel_centroid=this->calculateVoxelCentroid(supervoxel_cloud);
				this->setSupervoxelCentroid(i,supervoxel_centroid);
				this->setSupervoxelCenter(i, supervoxel_centroid);

				//Normal vector
				supervoxel_norm=this->calculateSupervoxelNorms(supervoxel_cloud);
				this->setSupervoxelNorms(i, supervoxel_norm);

				//Eigen feature
				supervoxel_eigens=this->calculateEigenFeatures(supervoxel_cloud);
				this->setSupervoxelEigens(i, supervoxel_eigens);

				//Volume
				supervoxel_volume=this->calculateSupervoxelVolume(supervoxel_cloud);
				this->setSupervoxelVolume(i,supervoxel_volume);

				//Set supervoxel chosen or not
				this->setSupervoxelChosen(i, 0);

				//Set supervoxel clustered or not
				this->setSupervoxelClustered(i,1);

				//Test
				//std::cout<<"Attributes of SV: "<<i<<std::endl;
				//std::cout<<"Centroid:         "<<supervoxel_centroid.x<<"   "<<supervoxel_centroid.y<<"   "<<supervoxel_centroid.z<<std::endl;
				//std::cout<<"Normal:           "<<supervoxel_norm.normal_x<<"   "<<supervoxel_norm.normal_y<<"   "<<supervoxel_norm.normal_z<<std::endl;
				//std::cout<<"Eigens:           "<<supervoxel_eigens[0]<<"   "<<supervoxel_eigens[1]<<"   "<<supervoxel_eigens[2]<<"   "<<supervoxel_eigens[3]<<std::endl;
				//std::cout<<"                  "<<supervoxel_eigens[4]<<"   "<<supervoxel_eigens[5]<<"   "<<supervoxel_eigens[6]<<"   "<<supervoxel_eigens[7]<<std::endl;
				//std::cout<<"Volume:           "<<supervoxel_volume<<std::endl;
				//std::cout<<"Chosen or not?    "<<this->supervoxel_used_[i]<<std::endl;
				//std::cout<<" "<<std::endl;
			}
		}

		void
		calculateWeightedCorvariance(PCXYZPtr input_cloud, Eigen::Matrix3f *output_cor)
		{
			//Settings 
			pcl::PointXYZ point_can;	// Candidate point
			Eigen::Vector3f key_coor=Eigen::Vector3f::Zero(3,1);	// Coordinates of key point 
			Eigen::Vector3f can_coor=Eigen::Vector3f::Zero(3,1);	// Coordinates of candidate point
			Eigen::Vector3f sum_coor=Eigen::Vector3f::Zero(3,1);	// Sum of coordinates
			Eigen::Vector3f diff_coor=Eigen::Vector3f::Zero(3,1);	// Coordinates difference
			Eigen::Matrix3f cor_single=Eigen::Matrix3f::Zero(3,3);	// CTC for a point
			Eigen::Matrix3f cor_sum=Eigen::Matrix3f::Zero(3,3);		// Sum of all CTC

			std::vector<int> points_id_support; 	// Neighbors within radius search
			std::vector<float> points_dis_support; 	// Distance of these neighbors 

			int num_support=input_cloud->points.size();	//Num of input point
			int num_support_min=3;
			float point_dis=0;
			float sum_dis=0;

			//Tranverse in the support region
			if(num_support>num_support_min)
			{
				for (size_t i = 0; i < num_support; i++)
				{
					point_can=input_cloud->points[i];
					sum_coor[0]=sum_coor[0]+point_can.x;
					sum_coor[1]=sum_coor[1]+point_can.y;
					sum_coor[2]=sum_coor[2]+point_can.z;
				}

				//key point
				key_coor[0]=sum_coor[0]/num_support;
				key_coor[1]=sum_coor[1]/num_support;
				key_coor[2]=sum_coor[2]/num_support;

				for (size_t j = 0; j < num_support; j++)
				{
					//Get candidate point in support
					point_can=input_cloud->points[j]; 
					can_coor[0]=point_can.x;
					can_coor[1]=point_can.y;
					can_coor[2]=point_can.z;

					//Coordinate differences
					diff_coor=can_coor-key_coor;

					//Distance between the candidate and key points
					point_dis=diff_coor.norm();
					sum_dis=sum_dis+point_dis;

					//CTC
					cor_single=point_dis*diff_coor*diff_coor.transpose();
					cor_sum=cor_sum+cor_single;
				}
			}
			else
			{
				sum_dis=1;
				cor_sum=Eigen::Matrix3f::Zero(3,3);
			}

			//Final covariance matrix
			cor_sum=cor_sum/sum_dis;
			*output_cor=cor_sum;
		}

		void
		calculateCorvariance(PCXYZPtr input_cloud, Eigen::Matrix3f *output_cor)
		{
			//Settings 
			pcl::PointXYZ point_can;	// Candidate point
			Eigen::Vector3f key_coor=Eigen::Vector3f::Zero(3,1);	// Coordinates of key point 
			Eigen::Vector3f can_coor=Eigen::Vector3f::Zero(3,1);	// Coordinates of candidate point
			Eigen::Vector3f sum_coor=Eigen::Vector3f::Zero(3,1);	// Sum of coordinates
			Eigen::Vector3f diff_coor=Eigen::Vector3f::Zero(3,1);	// Coordinates difference
			Eigen::Matrix3f cor_single=Eigen::Matrix3f::Zero(3,3);	// CTC for a point
			Eigen::Matrix3f cor_sum=Eigen::Matrix3f::Zero(3,3);		// Sum of all CTC

			std::vector<int> points_id_support; 	// Neighbors within radius search
			std::vector<float> points_dis_support; 	// Distance of these neighbors 

			int num_support=input_cloud->points.size();	//Num of input point
			int num_support_min=3;
			float point_dis=0;
			//float sum_dis=0;

			//Tranverse in the support region
			if(num_support>num_support_min)
			{
				for (size_t i = 0; i < num_support; i++)
				{
					point_can=input_cloud->points[i];
					sum_coor[0]=sum_coor[0]+point_can.x;
					sum_coor[1]=sum_coor[1]+point_can.y;
					sum_coor[2]=sum_coor[2]+point_can.z;
				}

				//key point
				key_coor[0]=sum_coor[0]/num_support;
				key_coor[1]=sum_coor[1]/num_support;
				key_coor[2]=sum_coor[2]/num_support;

				for (size_t j = 0; j < num_support; j++)
				{
					//Get candidate point in support
					point_can=input_cloud->points[j]; 
					can_coor[0]=point_can.x;
					can_coor[1]=point_can.y;
					can_coor[2]=point_can.z;

					//Coordinate differences
					diff_coor=can_coor-key_coor;

					//CTC
					cor_single=diff_coor*diff_coor.transpose();
					cor_sum=cor_sum+cor_single;
				}

				//理论上应该除以点的数量
				cor_sum = cor_sum / num_support;
			}
			else
			{
				//sum_dis=1;
				cor_sum=Eigen::Matrix3f::Zero(3,3);
			}

			//Final covariance matrix
			*output_cor=cor_sum;
		}

		//Graph
		void
		findAllSupervoxelAdjacency()
		{
			//Build kd-tree of supervoxels' centroids
			this->buildSupervoxelCentersKdtree();

			//Search radius
			double search_radius;
			search_radius=this->adjacent_resolution_; //Here, the adjacent ones mean the ones that really closely stick to the center one

			//Tranverse all the centers in the tree
			pcl::PointXYZ search_point;
			for (int i = 0; i<this->supervoxels_num_; i++)
			{
				//Searching points
				search_point=this->supervoxel_centroids_cloud_->points[i];   

				//Defining search radius
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				//Searching results
				//Achtung!!! The FLANN 1.7.1 has a known bug for the search_radius function! Use FLANN 1.8.0! 
				if ( supervoxel_centroids_kdtree_.radiusSearch (search_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
				{
					//Add elements in the vector
					this->supervoxels_adjacency_idx_.push_back(vector<int>(pointIdxRadiusSearch.size()+1,0));
					this->supervoxels_adjacency_idx_[i][0]=pointIdxRadiusSearch.size();//The first element records the num of adjacencies

					for (size_t j = 1; j < pointIdxRadiusSearch.size()+1; j++)//The rest records the idx 
					{
						//Records
						this->supervoxels_adjacency_idx_[i][j]=pointIdxRadiusSearch[j-1];
						//std::cout	<< "    "  << supervoxels_adjacency_idx_[i][j]<< std::endl;
					}
				}
			}
		}

		void
		findAllSupervoxelNeighbors()
		{
			//Build kd-tree of supervoxels' centroids
			this->buildSupervoxelCentersKdtree();

			//Search radius
			double search_radius;
			search_radius=this->graph_resolution_;

			//Tranverse all the centers in the tree
			pcl::PointXYZ search_point;
			for (int i = 0; i<this->supervoxels_num_; i++)
			{

				//Searching points
				search_point=this->supervoxel_centroids_cloud_->points[i];   

				//Defining search radius
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				//Searching results
				//Achtung!!! The FLANN 1.7.1 has a known bug for the search_radius function! Use FLANN 1.8.0! 
				if ( supervoxel_centroids_kdtree_.radiusSearch (search_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
				{
					//Add elements in the vector
					this->supervoxels_neighbor_idx_.push_back(vector<int>(pointIdxRadiusSearch.size()+1,0));
					this->supervoxels_neighbor_idx_[i][0]=pointIdxRadiusSearch.size();//The first element records the num of adjacencies

					for (size_t j = 1; j < pointIdxRadiusSearch.size()+1; j++)//The rest records the idx 
					{
						//Records
						this->supervoxels_neighbor_idx_[i][j]=pointIdxRadiusSearch[j-1];
						//std::cout	<< "    "  << supervoxels_neighbor_idx_[i][j]<< std::endl;
					}
				}
				else
				{
					//Add elements in the vector
					this->supervoxels_neighbor_idx_.push_back(vector<int>(pointIdxRadiusSearch.size() + 1, 0));
					this->supervoxels_neighbor_idx_[i][0] = 1;
				}
			}		
		}

		std::vector<int>
		getOneSupervoxelAdjacency(int supervoxel_id)
		{
			//Test
			//std::cout	<< " "<< std::endl;
			//std::cout	<< " Find adjacent supervoxels of supervoxel  "<< supervoxel_id <<std::endl;
			//std::cout	<< " SV:   "<< supervoxel_id<< "  having  "<< supervoxels_adjacency_idx_[supervoxel_id][0] <<" adjacent supervoxels "<<std::endl;
			std::vector<int> adjacency_idx;
			
			//Traverse
			for (size_t j = 1; j < this->supervoxels_adjacency_idx_[supervoxel_id][0]+1; j++)//Find the idx of recorded adjacent supervoxels 
			{
				//Records
				//std::cout	<< "    "  << supervoxels_adjacency_idx_[supervoxel_id][j]<< std::endl;					
				adjacency_idx.push_back(this->supervoxels_adjacency_idx_[supervoxel_id][j]);
				//std::cout	<< "    "  << adjacency_idx[j-1]<< std::endl;
			}

			return(adjacency_idx);
		}

		std::vector<int>
		getOneSupervoxelNeighbor(int supervoxel_id)
		{
			//Test
			std::cout	<< " "<< std::endl;
			std::cout	<< " Find neighbors of supervoxel  "<< supervoxel_id <<std::endl;
			std::cout	<< " SV:   "<< supervoxel_id<< "  having  "<< supervoxels_neighbor_idx_[supervoxel_id][0] <<" neighboring sv "<<std::endl;
			std::vector<int> neighbor_idx;
			
			//Traverse
			for (size_t j = 1; j < this->supervoxels_neighbor_idx_[supervoxel_id][0]+1; j++)//Find the idx of recorded neighboring supervoxels 
			{
				//Records
				//std::cout	<< "    "  << supervoxels_neighbor_idx_[supervoxel_id][j]<< std::endl;					
				neighbor_idx.push_back(this->supervoxels_neighbor_idx_[supervoxel_id][j]);
				//std::cout	<< "    "  << neighbor_idx[j-1]<< std::endl;
			}

			return(neighbor_idx);
		}	

		Eigen::MatrixXf
		buildAdjacencyGraph(std::vector<int> spvoxels_idx, float sig_p, float sig_n, float sig_o, float sig_e, float sig_c, float sig_w)//Building simple adjacency Graph: static graph only connectivity considered
		{
			//Settings
			Eigen::MatrixXf adj_matrix; //Adjacency in a n*n*n cube
			std::vector<float> spvoxel1_position, spvoxel1_normal, spvoxel1_eigen;
			std::vector<float> spvoxel2_position, spvoxel2_normal, spvoxel2_eigen;
			pcl::PointXYZ temp_center, empty_center;
			pcl::Normal temp_norm, empty_norm;
			float spvoxel1_vol,spvoxel2_vol;

			std::vector<float> dist_all; //Affinity calculated from different cues

			double similarity_spvoxels=0;
			int neighbor_num=0;
			empty_center.x=0;empty_center.y=0;empty_center.z=0;
			empty_norm.normal_x=0;empty_norm.normal_y=0;empty_norm.normal_z=0;

			//Find idx of adjacent voxels
			neighbor_num=spvoxels_idx.size();

			//Initialization
			adj_matrix=Eigen::MatrixXf::Zero(neighbor_num,neighbor_num); 

			//Assigning
			for (int i=0;i<neighbor_num;i++)
			{
				spvoxel1_position.clear();
				spvoxel1_normal.clear();
				spvoxel1_eigen.clear();
				temp_center=this->supervoxel_centroids_[spvoxels_idx[i]];
				temp_norm=this->supervoxel_norms_[spvoxels_idx[i]];

				if(temp_center.x!=empty_center.x && temp_center.y!=empty_center.y && temp_center.z!=empty_center.z)
				{		
					spvoxel1_position.push_back(temp_center.x);
					spvoxel1_position.push_back(temp_center.y);
					spvoxel1_position.push_back(temp_center.z);
				}
				else
				{
					spvoxel1_position.push_back(0);
				}

				if(temp_norm.normal_x!=empty_norm.normal_x && temp_norm.normal_y!=empty_norm.normal_y && temp_norm.normal_z!=empty_norm.normal_z)
				{
					spvoxel1_normal.push_back(temp_norm.normal_x);
					spvoxel1_normal.push_back(temp_norm.normal_y);
					spvoxel1_normal.push_back(temp_norm.normal_z);
				}
				else
				{
					spvoxel1_normal.push_back(0);
				}

				spvoxel1_eigen=this->supervoxel_eigens_[spvoxels_idx[i]];
				spvoxel1_vol=this->supervoxel_volumes_[spvoxels_idx[i]];

				for(int j=0;j<neighbor_num;j++)
				{
					spvoxel2_position.clear();
					spvoxel2_normal.clear();
					spvoxel2_eigen.clear();
					temp_center=this->supervoxel_centroids_[spvoxels_idx[j]];
					temp_norm=this->supervoxel_norms_[spvoxels_idx[j]];

					if(temp_center.x!=empty_center.x && temp_center.y!=empty_center.y && temp_center.z!=empty_center.z)
					{		
						spvoxel2_position.push_back(temp_center.x);
						spvoxel2_position.push_back(temp_center.y);
						spvoxel2_position.push_back(temp_center.z);
					}
					else
					{
						spvoxel2_position.push_back(0);
					}

					if(temp_norm.normal_x!=empty_norm.normal_x && temp_norm.normal_y!=empty_norm.normal_y && temp_norm.normal_z!=empty_norm.normal_z)
					{
						spvoxel2_normal.push_back(temp_norm.normal_x);
						spvoxel2_normal.push_back(temp_norm.normal_y);
						spvoxel2_normal.push_back(temp_norm.normal_z);
					}
					else
					{
						spvoxel2_normal.push_back(0);
					}

					spvoxel2_eigen=this->supervoxel_eigens_[spvoxels_idx[j]];
					spvoxel2_vol=this->supervoxel_volumes_[spvoxels_idx[j]];

					//Similarity measuring
					if(i!=j)
					{
						dist_all=measuringDistance(spvoxel1_position, spvoxel2_position, spvoxel1_normal, spvoxel2_normal, spvoxel1_eigen, spvoxel2_eigen);
						
						similarity_spvoxels=distanceWeight(dist_all, sig_p, sig_n, sig_o, sig_e, sig_c, sig_w);
						
						adj_matrix(i,j)=similarity_spvoxels;
					}
					else
					{
						adj_matrix(i,j)=1;
					}

				}
			}

			return(adj_matrix);
		}

		Eigen::MatrixXf
		findRowMemberMatrix(Eigen::MatrixXf input_matrix, int row_idx)
		{
			Eigen::MatrixXf output_matrix;
			int matrix_size=input_matrix.rows();
			output_matrix=Eigen::MatrixXf::Zero(matrix_size-1,1); 
			
			//Assigning
			int j=0;
			for(int i=0;i<matrix_size;i++)
			{
				if(i!=row_idx)
				{
					output_matrix(j,0)=input_matrix(i,row_idx);
					j++;
				}
			}

			return(output_matrix);
		}

		std::vector<std::vector<float> >
		supervoxelAttributes(int sv_idx)
		{
			//Parameters and settings
			std::vector<std::vector<float> > sv_attributes;
			std::vector<float> sv_position, sv_normal, sv_eigen, sv_features, sv_colors, sv_vol;; //Attributes of supervoxels			
			pcl::PointXYZ temp_center, empty_center;
			pcl::Normal temp_norm, empty_norm;

			empty_center.x=0;empty_center.y=0;empty_center.z=0;
			empty_norm.normal_x=0;empty_norm.normal_y=0;empty_norm.normal_z=0;

			temp_center=this->supervoxel_centroids_[sv_idx];
			temp_norm=this->supervoxel_norms_[sv_idx];

			//Position
			if(temp_center.x!=empty_center.x && temp_center.y!=empty_center.y && temp_center.z!=empty_center.z)
			{		
				sv_position.push_back(temp_center.x);
				sv_position.push_back(temp_center.y);
				sv_position.push_back(temp_center.z);
			}
			else
			{
				sv_position.push_back(0);
			}
			//Normal
			if(temp_norm.normal_x!=empty_norm.normal_x && temp_norm.normal_y!=empty_norm.normal_y && temp_norm.normal_z!=empty_norm.normal_z)
			{
				sv_normal.push_back(temp_norm.normal_x);
				sv_normal.push_back(temp_norm.normal_y);
				sv_normal.push_back(temp_norm.normal_z);
			}
			else
			{
				sv_normal.push_back(0);
			}


			//Eigen
			sv_eigen=this->supervoxel_eigens_[sv_idx];
			//Feature
			sv_features=this->supervoxel_features_[sv_idx];
			//Color
			sv_colors=this->supervoxel_colors_[sv_idx];
			//Volumn
			sv_vol.push_back(this->supervoxel_volumes_[sv_idx]);

			//Assigning
			sv_attributes.push_back(sv_position); //0
			sv_attributes.push_back(sv_normal);	  //1
			sv_attributes.push_back(sv_eigen);    //2
			sv_attributes.push_back(sv_features); //3
			sv_attributes.push_back(sv_colors);   //4
			sv_attributes.push_back(sv_vol);      //5
			return(sv_attributes);
		}

		//Graphical method
		std::vector<float>
		measuringDistance(std::vector<float> v1_center, std::vector<float> v2_center, std::vector<float> v1_norm, std::vector<float> v2_norm, std::vector<float> v1_eigen, std::vector<float> v2_eigen)
		{
			//Parameters and settings
			std::vector<float> similarity_dist;
			
			// 初始化一个很大的值, 这样当碰到很小的点集时, 可以直接赋很大的距离值给这些变量....
			float dist_space = 100;	//S: space distance
			float dist_angle = 100;	//A: norm angles
			float dist_stair = 100;	//T: stair effect
			float dist_eigen = 100;	//E: eigen feature
			float dist_convx = 100;	//C: convex

			float dist_v1_v2=0,dist_v1=0,dist_v2=0,dist_o1=0,dist_o2=0;
			float cos_v1_dist=0, cos_v2_dist=0,cos_v1_v2=0, cos_d_s=0;
			float dist_cos_dist1=0,dist_cos_dist2=0, thred_singular=0, max_singular=0;
			double a_1=0, a_2=0, a_1_2=0, a_d_s=0, a_d_s1=0, a_d_s2=0, PI=3.1415926;
			std::vector<float> norm_v1_v2, product_v1_v2, norm_v2_v1;

			//Spacial distance
			if(v1_center.size()>1 && v2_center.size()>1) 
			{
				dist_v1_v2=sqrt(pow(v1_center[0]-v2_center[0],2)+pow(v1_center[1]-v2_center[1],2)+pow(v1_center[2]-v2_center[2],2));
				dist_space=dist_v1_v2;

				if(dist_v1_v2!=0)
				{
					norm_v1_v2.clear();
					norm_v1_v2.push_back((v1_center[0]-v2_center[0])/dist_v1_v2);
					norm_v1_v2.push_back((v1_center[1]-v2_center[1])/dist_v1_v2);
					norm_v1_v2.push_back((v1_center[2]-v2_center[2])/dist_v1_v2);

					norm_v2_v1.clear();
					norm_v2_v1.push_back((v2_center[0]-v1_center[0])/dist_v1_v2);
					norm_v2_v1.push_back((v2_center[1]-v1_center[1])/dist_v1_v2);
					norm_v2_v1.push_back((v2_center[2]-v1_center[2])/dist_v1_v2);

					// X1×X2
					product_v1_v2.clear();
					product_v1_v2.push_back(v1_center[1]*v2_center[2]-v1_center[2]*v2_center[1]);
					product_v1_v2.push_back(v1_center[2]*v2_center[0]-v1_center[0]*v2_center[2]);
					product_v1_v2.push_back(v1_center[0]*v2_center[1]-v1_center[1]*v2_center[0]);
				}
			}

			//Normal vectors angle, stair-like distance, and convex
			if(v1_norm.size()>1 && v2_norm.size()>1)
			{
				if (dist_v1_v2 != 0)
				{
					cos_v1_v2 = (v1_norm[0] * v2_norm[0] + v1_norm[1] * v2_norm[1] + v1_norm[2] * v2_norm[2]); // <N1,N2>
					cos_v1_dist = (v1_norm[0] * norm_v1_v2[0] + v1_norm[1] * norm_v1_v2[1] + v1_norm[2] * norm_v1_v2[2]); // <N1,d>
					cos_v2_dist = (v2_norm[0] * norm_v1_v2[0] + v2_norm[1] * norm_v1_v2[1] + v2_norm[2] * norm_v1_v2[2]); // <N2,d>
					cos_d_s = (product_v1_v2[0] * norm_v1_v2[0] + product_v1_v2[1] * norm_v1_v2[1] + product_v1_v2[2] * norm_v1_v2[2]); //<X1×X2,d>

					a_1 = acos(cos_v1_dist); // alpha1
					a_2 = acos(cos_v2_dist); // alpha2
					a_1_2 = acos(cos_v1_v2);

					a_d_s1 = acos(cos_d_s);
					a_d_s2 = PI - a_d_s1;

					dist_angle = acos(cos_v1_v2);// norm angle <N1,N2>

					dist_v1 = v1_norm[0] * v1_center[0] + v1_norm[1] * v1_center[1] + v1_norm[2] * v1_center[2]; // <N1,X1> = D1
					dist_v2 = v2_norm[0] * v2_center[0] + v2_norm[1] * v2_center[1] + v2_norm[2] * v2_center[2]; // <N2,X2> = D2
					dist_o1 = v1_norm[0] * v2_center[0] + v1_norm[1] * v2_center[1] + v1_norm[2] * v2_center[2]; // <N1,X2>
					dist_o2 = v2_norm[0] * v1_center[0] + v2_norm[1] * v1_center[1] + v2_norm[2] * v1_center[2]; // <N2,X1>

					dist_stair = sqrt(pow((dist_o1 - dist_v1), 2) + pow((dist_o2 - dist_v2), 2)); // sqrt((<N1,X2> - D1)^2 + (<N2,X1> - D2)^2)
				}
				else
				{
					dist_stair = 0;
				}

				//Singular judgement
				double temp_a=0.5, temp_off=PI/6, 
				max_singular=PI/2;
				thred_singular=(float)max_singular/(1+exp(-1*temp_a*(a_1_2-temp_off)));

				a_d_s=a_d_s1;
				if(a_d_s1>a_d_s2)
				{
					a_d_s=a_d_s2;
				}
				
				if(a_d_s>thred_singular) //closure: convexity 
				{
					dist_convx = abs(a_1 - a_2); 
				}
				else
				{
					dist_convx = PI;
				}
			}

			//Eigen features similarity (这里来看其实就用点积的方法,并没有用 histogram intersection kernel )
			if(v1_eigen.size()>1 && v2_eigen.size()>1)  
			{
				float eigen_diff=0, eigen_cos=0, eigen_abs1=0, eigen_abs2=0; 
				for(int i=0;i<v1_eigen.size();i++)
				{
					eigen_cos=eigen_cos + v1_eigen[i] * v2_eigen[i];
					eigen_abs1=eigen_abs1 + v1_eigen[i] * v1_eigen[i];
					eigen_abs2=eigen_abs2 + v2_eigen[i] * v2_eigen[i];
				}

				if(eigen_abs1!=0 && eigen_abs2!=0)
				{
					dist_eigen=(1.0f - eigen_cos / (sqrt(eigen_abs1)*sqrt(eigen_abs2)));
				}
			}

			//Output similarity distances
			similarity_dist.push_back(dist_space);	//P: proximity
			similarity_dist.push_back(dist_angle);	//S: smoothness
			similarity_dist.push_back(dist_stair);	//O: co-plane
			similarity_dist.push_back(dist_eigen);	//E: similarity
			similarity_dist.push_back(dist_convx);	//L: closure

			return(similarity_dist);
		}

		float
		distanceWeight(std::vector<float> dist_all, float sig_p, float sig_n, float sig_o, float sig_e, float sig_c, float sig_weight)
		{
			//Minkovski distance measure (Chebychev distance)

			//Settings
			float similarity_weight=0;
			float similarity_dist=0;
			float dist_space = 1;	//Space
			float dist_angle = 1;	//Angle
			float dist_stair = 1;	//sTair
			float dist_eigen = 1;	//Eigen
			float dist_convx = 1;	//Convex

			dist_space = dist_all[0];
			dist_angle = dist_all[1];
			dist_stair = dist_all[2];
			dist_eigen = dist_all[3];
			dist_convx = dist_all[4];

			similarity_dist = sqrt(pow(dist_space, 2) / sig_p + pow(dist_angle, 2) / sig_n + pow(dist_eigen, 2) / sig_e + pow(dist_stair, 2) / sig_o);
			
			similarity_weight=exp(-0.5*similarity_dist/pow(sig_weight,2));

			return(similarity_weight);		
		}

		//Graph segmentation
		std::vector<int>
		cutGraphSegmentation(float thrd_value, Eigen::MatrixXf graph_matrix, std::vector<int> voxel_adjidx)       //Efficient graph segmentation algorithm
		{
			//Parameters and settings
			int graph_size=0, weight_size=0;
			float initial_thrd=1;
			graph_size=graph_matrix.rows();
			weight_size=graph_size*graph_size;
			//std::vector<float> weight_array;
			std::vector<int> connected_voxels; // original_idx, sorted_idx;
			graph_matrix.resize(1,weight_size);

			// Sort, descending using sort function
			std::vector<Weight_Index> weight_index_array;
			Weight_Index temp;
			for (int i = 0; i < weight_size; i++)
			{
				temp.Weight = graph_matrix(0, i);
				temp.Index = i;
				weight_index_array.push_back(temp);
			}
			sort(weight_index_array.begin(), weight_index_array.end(), godown); // riseup

			//Sort, descending （bug: sorted_idx is not correct）
			/*for (int i = 0; i < weight_size; i++) 
			{
				original_idx.push_back(i);
				sorted_idx.push_back(i);
				weight_array.push_back(graph_matrix(0, i));
			}
			int c_max,i_temp;
			float w_temp;
			for (int i = 0; i < weight_size - 1; i++)
			{
				c_max = i;
				for (int j = i + 1; j < weight_size; j++)
				{
					if (weight_array[c_max] < weight_array[j])
					{	
						c_max = j;
					}
				}
				i_temp = original_idx[c_max];
				sorted_idx[i]=i_temp;
				sorted_idx[c_max]=original_idx[i];

				w_temp = weight_array[c_max];
				weight_array[c_max] = weight_array[i];
				weight_array[i] = w_temp;
			}*/

			//Merging
			std::vector<float> seg_int;
			std::vector< std::vector<int> > seg_ver_idx;
			std::vector<int> seg_size, ver_seg_idx, vec_temp; 
			for (int i = 0; i < graph_size; i++) 
			{
				vec_temp.clear();//Temp
				vec_temp.push_back(i);

				seg_ver_idx.push_back(vec_temp);	// segment has which vertice, if the vertice has been merged, than empty
				ver_seg_idx.push_back(i);			// vertice belong to which segment
				seg_size.push_back(1);				// How many vertex in the segment
				seg_int.push_back(initial_thrd);	// Initial of mint
			}

			// 边权值从大到小进行合并...
			// seg_ver_idx [面号]: 当前面包含的所有点
			// ver_seg_idx [点号]: 当前点所属的面片
			// seg_size [面号]: 当前面具有点的数量
			// seg_int [面号]: 当前面的异质性测度(初始值是 1, )
			for (int i = 0; i < weight_size; i++) 
			{
				float w_temp, thred_temp;
				int v1_idx, v2_idx, mint_idx, mint_not_idx;
				v1_idx = floor(double(weight_index_array[i].Index / graph_size));
				v2_idx = weight_index_array[i].Index - v1_idx*graph_size;
				w_temp = weight_index_array[i].Weight;

				if (ver_seg_idx[v1_idx] != ver_seg_idx[v2_idx])
				{
					float v1_mint, v2_mint;
					int v1_size, v2_size;
					v1_size = seg_size[ver_seg_idx[v1_idx]];
					v2_size = seg_size[ver_seg_idx[v2_idx]];
					v1_mint = seg_int[ver_seg_idx[v1_idx]] - thrd_value / v1_size;
					v2_mint = seg_int[ver_seg_idx[v2_idx]] - thrd_value / v2_size;

					if (v1_mint >= v2_mint)
					{
						mint_idx = ver_seg_idx[v1_idx];
						mint_not_idx = ver_seg_idx[v2_idx];
						thred_temp = v1_mint;//seg_mint[ver_seg_idx[v1_idx]];
					}
					else
					{
						mint_idx = ver_seg_idx[v2_idx];
						mint_not_idx = ver_seg_idx[v1_idx];
						thred_temp = v2_mint;//seg_mint[ver_seg_idx[v2_idx]];
					}

					if (w_temp > thred_temp)//Judgement, merge or not
					{
						seg_int[mint_idx] = w_temp;

						for (int j = 0; j < seg_size[mint_not_idx]; j++)
						{
							seg_ver_idx[mint_idx].push_back(seg_ver_idx[mint_not_idx][j]);  //Rearrange vertex of merged one
							ver_seg_idx[seg_ver_idx[mint_not_idx][j]]=mint_idx;				//Remark the vertex of merged one					
						}

						seg_size[mint_idx]=seg_size[mint_idx]+seg_size[mint_not_idx]; //Change sizes
						seg_size[mint_not_idx]=0;
						
						seg_ver_idx[mint_not_idx].clear();

					}
				}
			}

			//Select
			int seg_con=0;
			for(int i=0;i<graph_size;i++)//Start from 1, because zero is itself
			{
				if(seg_ver_idx[i].size()!=0)
				{
					for(int j=0; j<seg_ver_idx[i].size(); j++)
					{
						// 令 seg_ver_idx[i][j]==0 的意思是找到当前图模型中含有中心体素的图模型
						// 因为实际情况下, 该图模型可能含有多个不相连的图模型, 其中一般最大的那个含有目标体素...
						if(seg_ver_idx[i][j]==0)
						{
							seg_con=i;
							// 感觉找到了就可以不同继续循环了....
							break;
						}
					}
				}			
			}

			for(int k=0; k<seg_ver_idx[seg_con].size(); k++)
			{
				connected_voxels.push_back(voxel_adjidx[seg_ver_idx[seg_con][k]]);
			}
			
			return(connected_voxels);
		}

		//Clustering
		void
		recursionSearch(std::vector<int> connects_idx, std::vector<int> *inthis_cluster_idx )//recursion search for finding voxels belonging to one cluster
		{
			int connect_id;

			for(int i=0;i<connects_idx.size();i++)// loop + recrusion
			{

				if(!this->supervoxel_clustered_[connects_idx[i]])
				{	
				
					inthis_cluster_idx->push_back(connects_idx[i]);		//Record the idx of this voxel
					
					this->supervoxel_clustered_[connects_idx[i]]=true;	//change the status

					this->recursionSearch(this->supervoxels_connect_idx_[connects_idx[i]],inthis_cluster_idx);	//recursion
				}

			}

		}

		void
		clusteringSupervoxels()		//Clustering of supervoxels		
		{
			//Setting
			int clusters_num=0;
			std::vector<int> cluster_supervoxel;

			//Clustering
			int i=0;
			for(int i=0;i<this->supervoxels_num_;i++)// loop + recrusion
			{
				if(!this->supervoxel_clustered_[i])
				{	
					//std::cout<<i<<std::endl;

					std::vector<int> *inthis_cluster_idx=new std::vector<int>;

					this->supervoxel_clustered_.at(i)=true;

					this->recursionSearch(this->supervoxels_connect_idx_[i],inthis_cluster_idx);	

					inthis_cluster_idx->push_back(i); //Don't forget to 

					this->clusters_supervoxel_idx_.push_back(*inthis_cluster_idx);
				}
			}

			//Output
			this->clusters_num_=this->clusters_supervoxel_idx_.size();

			for(int m=0;m<this->clusters_num_;m++)
			{
				int supervoxels_size=this->clusters_supervoxel_idx_[m].size();

				if(supervoxels_size>0)
				{
					std::vector<int> cluster_points;
					for(int n=0;n<supervoxels_size;n++)
					{
						int points_size=this->supervoxels_point_idx_[this->clusters_supervoxel_idx_[m][n]].size();
						for(int k=0;k<points_size;k++)
						{
							cluster_points.push_back(this->supervoxels_point_idx_[this->clusters_supervoxel_idx_[m][n]][k]);
						}
					}
					this->clusters_point_idx_.push_back(cluster_points);
				}
			}

			//Test
			std::cout << "In total " << this->clusters_num_ << " segments\n";
		}

		//Other
		void
		getVoxelCenterFromOctreeKey (pcl::octree::OctreeKey key, pcl::PointXYZ & point)
		{
			// define point to leaf node voxel center
			point.x = static_cast<float> ((static_cast<double> (key.x) + 0.5f) * this->voxel_resolution_ + this->min_x_);
			point.y = static_cast<float> ((static_cast<double> (key.y) + 0.5f) * this->voxel_resolution_ + this->min_y_);
			point.z = static_cast<float> ((static_cast<double> (key.z) + 0.5f) * this->voxel_resolution_ + this->min_z_);
		}

		void
		crossValidation()
		{
			//Traverse
			for(int i=0;i<this->supervoxels_num_;i++)// loop all the supervoxels
			{
				int inthis_num=this->supervoxels_connect_idx_[i].size();
				std::vector<int> new_connect_idx;

				if(inthis_num>1)
				{
					for(int j=0;j<inthis_num;j++)// check its connections
					{
						bool find_ornot=false;
						int find_idx=i;
						int search_idx=this->supervoxels_connect_idx_[i][j];
						int search_size=this->supervoxels_connect_idx_[search_idx].size();
						
						if(search_size>0)
						{
							for(int k=0;k<search_size;k++)
							{
								if(this->supervoxels_connect_idx_[search_idx][k]==find_idx)
								{
									find_ornot=true;
									// 感觉搜索到了就不用再搜索了...
									break;
								}						
							}
						}
						//Judgement
						if(find_ornot==true)
						{
							new_connect_idx.push_back(search_idx);
						}
					}

					//Replace
					this->supervoxels_connect_idx_[i].clear();
					this->supervoxels_connect_idx_[i]=new_connect_idx;
				}
			}
		}

		void
		closestCheck(float sig_p, float sig_n, float sig_o, float sig_e, float sig_c, float sig_weight)
		{
			//Settings
			std::vector<float> voxel1_position, voxel1_normal, voxel1_eigen, voxel1_features, voxel1_colors;
			std::vector<float> voxel2_position, voxel2_normal, voxel2_eigen, voxel2_features, voxel2_colors;
			pcl::PointXYZ temp_center, empty_center;
			pcl::Normal temp_norm, empty_norm;
			std::vector<float> dist_vector;
			empty_center.x = 0; empty_center.y = 0; empty_center.z = 0;
			empty_norm.normal_x = 0; empty_norm.normal_y = 0; empty_norm.normal_z = 0;

			//Closet neighbor check
			for (int i = 0; i<this->supervoxels_num_; i++)
			{
				int inthis_num = this->supervoxels_connect_idx_[i].size();
				std::vector<int> new_connect_idx;

				if (inthis_num>0 && inthis_num<2)
				{
					if (this->supervoxels_neighbor_idx_[i].size()>this->supervoxel_adjacency_min_)
					{
						float min_dis = 0, temp_dis = 0;
						int min_idx;
						pcl::PointXYZ temp_p1, temp_p2;

						voxel1_position.clear();
						voxel1_normal.clear();
						voxel1_eigen.clear();
						temp_center = this->supervoxel_centroids_[i];
						temp_norm = this->supervoxel_norms_[i];

						if (temp_center.x != empty_center.x && temp_center.y != empty_center.y && temp_center.z != empty_center.z)
						{
							voxel1_position.push_back(temp_center.x);
							voxel1_position.push_back(temp_center.y);
							voxel1_position.push_back(temp_center.z);
						}
						else
						{
							voxel1_position.push_back(0);
						}

						if (temp_norm.normal_x != empty_norm.normal_x && temp_norm.normal_y != empty_norm.normal_y && temp_norm.normal_z != empty_norm.normal_z)
						{
							voxel1_normal.push_back(temp_norm.normal_x);
							voxel1_normal.push_back(temp_norm.normal_y);
							voxel1_normal.push_back(temp_norm.normal_z);
						}
						else
						{
							voxel1_normal.push_back(0);
						}
						voxel1_eigen = this->supervoxel_eigens_[i];

						//Initialization
						min_dis = 0; min_idx = -1;

						//Check neighbors
						for (int j = 0; j<this->supervoxels_neighbor_idx_[i].size(); j++)
						{
							if (this->supervoxels_connect_idx_[this->supervoxels_neighbor_idx_[i][j]].size()>1)
							{
								temp_p2 = this->supervoxel_centroids_[this->supervoxels_neighbor_idx_[i][j]];

								voxel2_position.clear();
								voxel2_normal.clear();
								voxel2_eigen.clear();
								temp_center = this->supervoxel_centroids_[this->supervoxels_neighbor_idx_[i][j]];//temp_center=voxel_centers_[voxels_idx[j]];
								temp_norm = this->supervoxel_norms_[this->supervoxels_neighbor_idx_[i][j]];

								if (temp_center.x != empty_center.x && temp_center.y != empty_center.y && temp_center.z != empty_center.z)
								{
									voxel2_position.push_back(temp_center.x);
									voxel2_position.push_back(temp_center.y);
									voxel2_position.push_back(temp_center.z);
								}
								else
								{
									voxel2_position.push_back(0);
								}

								if (temp_norm.normal_x != empty_norm.normal_x && temp_norm.normal_y != empty_norm.normal_y && temp_norm.normal_z != empty_norm.normal_z)
								{
									voxel2_normal.push_back(temp_norm.normal_x);
									voxel2_normal.push_back(temp_norm.normal_y);
									voxel2_normal.push_back(temp_norm.normal_z);
								}
								else
								{
									voxel2_normal.push_back(0);
								}
								voxel2_eigen = this->supervoxel_eigens_[this->supervoxels_neighbor_idx_[i][j]];


								dist_vector = measuringDistance(voxel1_position, voxel2_position, voxel1_normal, voxel2_normal, voxel1_eigen, voxel2_eigen);
								// 测量边权值的方法有所不同
								temp_dis = distanceWeight(dist_vector, sig_p, sig_n, sig_o, sig_e, sig_c, sig_weight);

								if (temp_dis >= min_dis)
								{
									min_dis = temp_dis;
									min_idx = this->supervoxels_neighbor_idx_[i][j];
								}

							}
						}

						if (min_idx != -1)
						{
							this->supervoxels_connect_idx_[i].push_back(min_idx);
							this->supervoxels_connect_idx_[min_idx].push_back(i);
						}
					}
				}

			}


		}
   };

 }

 #endif // PCL_SEGMENTATION_SUPERVOXEL_H_
