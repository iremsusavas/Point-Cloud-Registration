#include "Registration.h"

#include "open3d/Open3D.h"

Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename)
{
  // TO COMPLETE
  open3d::io::ReadPointCloud(cloud_source_filename , source_);
  open3d::io::ReadPointCloud(cloud_target_filename, target_);

}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target)
{
  // TO COMPLETE
  
  source_ = cloud_source;  
  target_ = cloud_target; 
}


void Registration::draw_registration_result()
{

  //visualize target and source with two different colors
  // TO COMPLETE

std::shared_ptr<open3d::geometry::PointCloud>  pc0_pointer = std::make_shared<open3d::geometry::PointCloud>(source_);
std::shared_ptr<open3d::geometry::PointCloud> pc1_pointer= std::make_shared<open3d::geometry::PointCloud>(target_);

pc0_pointer ->PaintUniformColor({1.000, 0.706, 0.000});  //source point cloud yellow
pc1_pointer ->PaintUniformColor({0.000, 0.906, 0.100});  //target point cloud green

pc0_pointer ->Transform(transformation_);
open3d::visualization::DrawGeometries({pc0_pointer,pc1_pointer});
  
}


void Registration::preprocess(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down_ptr, std::shared_ptr<open3d::pipelines::registration::Feature> &pcd_fpfh)
{

  //downsample, estimate normals and compute FPFH features
	pcd_down_ptr = pcd.VoxelDownSample(voxel_size); //downsample
	pcd_down_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30)); //estimate normals
	
	//compute fpfh features search
	pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
                *pcd_down_ptr, open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));  
	

  // TO COMPLETE

  return;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_global_registration(double voxel_size)
{// remember to apply the transformation_ matrix to source_cloud
	source_.Transform(transformation_);  //transform source point cloud
	
	
	// create two point cloud to contain the downsampled point cloud and two structure to contain the features
	std::shared_ptr<open3d::geometry::PointCloud> source_down;
	std::shared_ptr<open3d::geometry::PointCloud> target_down;
	std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh;
	std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh;
	
	
	// call the Registration::preprocess function on target and transformed source
	Registration::preprocess(source_,voxel_size,source_down,source_fpfh);
	Registration::preprocess(target_,voxel_size,target_down,target_fpfh);
	
	
	//prepare checkers for ransac based feature matcher
	float distance_threshold = voxel_size * 1.5;  //liberal distance treshold
	
	std::vector<std::reference_wrapper<
const open3d::pipelines::registration::CorrespondenceChecker>>
correspondence_checker;
auto correspondence_checker_edge_length =
open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
0.9); //lenght checker
auto correspondence_checker_distance =
open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
distance_threshold); //distance checker

correspondence_checker.push_back(correspondence_checker_edge_length);
correspondence_checker.push_back(correspondence_checker_distance);

bool filter = true;
	  open3d::pipelines::registration::RegistrationResult result;
result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(*source_down, 
                                                                            *target_down,
                                                                            *source_fpfh,
                                                                            *target_fpfh,
                                                                            filter,
                                                                            distance_threshold,
                                                                            open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
                                                                            3,
correspondence_checker,                                                                        open3d::pipelines::registration::RANSACConvergenceCriteria(4000000, 500));


  transformation_ = result.transformation_;

std::cout << "inlier(correspondence_set size):"
<< result.correspondence_set_.size() << std::endl;

  return result;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_icp_registration(double threshold, double relative_fitness, double relative_rmse, int max_iteration)
{
  open3d::pipelines::registration::RegistrationResult result;
  result = open3d::pipelines::registration::RegistrationICP(
      source_,
      target_,
    threshold,
    transformation_,
    open3d::pipelines::registration::TransformationEstimationPointToPoint(false), //false to force scaling to be 1
    open3d::pipelines::registration::ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration));
  
transformation_ = result.transformation_;
  return result;
}


void Registration::set_transformation(Eigen::Matrix4d init_transformation)
{
  transformation_=init_transformation;
}


Eigen::Matrix4d  Registration::get_transformation()
{
  return transformation_;
}


void Registration::write_tranformation_matrix(std::string filename)
{
  std::ofstream outfile (filename);
  if (outfile.is_open())
  {
    outfile << transformation_;
    outfile.close();
  }
}

void Registration::save_merged_cloud(std::string filename)
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  source_clone.Transform(transformation_);
  open3d::geometry::PointCloud merged = target_clone+source_clone;
  open3d::io::WritePointCloud(filename, merged );
}

