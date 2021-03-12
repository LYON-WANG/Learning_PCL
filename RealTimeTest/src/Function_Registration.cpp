#include "RealTimeTest.h"
/* \author Leo Wang */
// Customized Registration function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
template<typename PointT> // Print Rotation Matrix & Translation Vector
 void Registration<PointT>::print4x4Matrix(const Eigen::Matrix4f &matrix){
    std::cout << "-----------------" << std::endl;
    std::cout << "Rotation matrix: " << std::endl;
    std::cout << "R = " << matrix (0, 0) << " " << matrix (0, 1) << " " << matrix (0, 2) << std::endl;
    std::cout << "    " << matrix (1, 0) << " " << matrix (1, 1) << " " << matrix (1, 2) << std::endl;
    std::cout << "    " <<matrix (2, 0) << " " << matrix (2, 1) << " " << matrix (2, 2) << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "T = " << matrix (0, 3) << " " << matrix (1, 3) << " " << matrix (2, 3) << std::endl;
    std::cout << "-----------------" << std::endl;
}

template<typename PointT>  //ICP point-to-point
 std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
 Registration<PointT>::ICP_Point2Point( const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                                        const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
                                        const Eigen::Matrix4f &init_transform,
                                        const int &MaxIteration,
                                        const float &Epsilon,
                                        const float &MaxCorrespondenceDistance){
    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    typename pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(MaxIteration);
    //icp.setEuclideanFitnessEpsilon(Epsilon);  // Convergence condition: The smaller the accuracy, the slower the convergence
	//icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
    icp.setInputSource(cloud_source);      
    icp.setInputTarget(cloud_target); 
    icp.align(*output, init_transform);  
    Eigen::Matrix4f SourceToTarget = Eigen::Matrix4f::Identity();
    if (icp.hasConverged()){
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        SourceToTarget = icp.getFinalTransformation();
        std::cout << SourceToTarget << std::endl;
    }
    return std::make_tuple(output, SourceToTarget);
}

template<typename PointT>  //ICP point-to-plane
 std::tuple<typename pcl::PointCloud<pcl::PointNormal>::Ptr, Eigen::Matrix4f> 
 Registration<PointT>::ICP_Point2Plane( const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source,
                                        typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target, 
                                        const Eigen::Matrix4f &init_transform,
                                        const int &MaxIteration,
                                        const float &Epsilon,
                                        const float &MaxCorrespondenceDistance){
    typename pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setMaximumIterations(MaxIteration);
    icp.setEuclideanFitnessEpsilon(Epsilon);  // Convergence condition: The smaller the accuracy, the slower the convergence
	icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
    icp.setInputSource(cloud_source);      
    icp.setInputTarget(cloud_target); 
    icp.align(*cloud_target, init_transform);  
    Eigen::Matrix4f SourceToTarget = Eigen::Matrix4f::Identity();
    if(icp.hasConverged()){
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        SourceToTarget = icp.getFinalTransformation();
        std::cout << SourceToTarget << std::endl;
    }
    return std::make_tuple(cloud_target, SourceToTarget);
}

template<typename PointT>  //ICP plane-to-plane
 std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
 Registration<PointT>::ICP_Plane2Plane( const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                                        const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
                                        const int &MaxIteration,
                                        const float &Epsilon,
                                        const float &MaxCorrespondenceDistance){
    typename pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(MaxIteration);
    icp.setEuclideanFitnessEpsilon(Epsilon);  // Convergence condition: The smaller the accuracy, the slower the convergence
	icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
    icp.setInputSource(cloud_source);      
    icp.setInputTarget(cloud_target); 
    icp.align(*cloud_source);  
    Eigen::Matrix4f SourceToTarget = Eigen::Matrix4f::Identity();
    if(icp.hasConverged()){
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        SourceToTarget = icp.getFinalTransformation();
        std::cout << SourceToTarget << std::endl;
    }
    return std::make_tuple(cloud_source, SourceToTarget);
}

template<typename PointT>  //NDT Registration
 std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> 
 Registration<PointT>::NDT_Registration(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                                        const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
                                        const Eigen::Matrix4f &init_guess,
                                        const float &tTransformationEpsilon,
                                        const float &StepSize,
                                        const float &Resolution,
                                        const int &MaxIteration){
    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    pcl::NormalDistributionsTransform<PointT, PointT> NDT;
    NDT.setTransformationEpsilon(tTransformationEpsilon);
    NDT.setStepSize(StepSize);           
    NDT.setResolution(Resolution);        
    NDT.setMaximumIterations(MaxIteration);
    NDT.setInputSource(cloud_source); 
    NDT.setInputTarget(cloud_target); 
    NDT.align(*output, init_guess);
    Eigen::Matrix4f SourceToTarget = Eigen::Matrix4f::Identity();
    if(NDT.hasConverged()){
        std::cout << "NDP has converged, score is " << NDT.getFitnessScore () << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        SourceToTarget = NDT.getFinalTransformation();
        std::cout << SourceToTarget << std::endl;
        pcl::transformPointCloud(*cloud_source, *output, SourceToTarget);
    }
    return std::make_tuple(output, SourceToTarget);
}

template<typename PointT>  // Sample Consensus based Initial Alignment algorithm (SAC-IA)
 std::tuple<typename pcl::PointCloud<pcl::PointNormal>::Ptr, Eigen::Matrix4f> 
 Registration<PointT>::SAC_IA(  const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source,
                                const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target, 
                                const float &SearchRadius,
                                const int &MaxIteration,
                                const int &NumberOfSamples){
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> FPFH;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    FPFH.setRadiusSearch(SearchRadius);
    FPFH.setInputCloud(cloud_source);
    FPFH.setInputNormals(cloud_source);
    FPFH.compute(*source_features);
    FPFH.setInputCloud(cloud_target);
    FPFH.setInputNormals(cloud_target);
    FPFH.compute(*target_features);
    // Sample Consensus Prerejective
    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> SAC;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointNormal>);
    SAC.setInputSource(cloud_source); // Source
    SAC.setSourceFeatures(source_features); //Source FPFH
    SAC.setInputTarget(cloud_target); // Target
    SAC.setTargetFeatures(target_features); //Target FPFH
    //SAC.setMaximumIterations(MaxIteration);
    //SAC.setNumberOfSamples(NumberOfSamples);
    //SAC.setCorrespondenceRandomness(5);
    //SAC.setSimilarityThreshold(0.9f);
    //SAC.setMaxCorrespondenceDistance (2.5f * 0.005f);
    //SAC.setInlierFraction (0.25f);
    SAC.align (*cloud_aligned);
    Eigen::Matrix4f SourceToTarget = Eigen::Matrix4f::Identity();
    if(SAC.hasConverged()){
        std::cout << "SAC-IA has converged, score is " << SAC.getFitnessScore () << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        SourceToTarget = SAC.getFinalTransformation();
        std::cout << SourceToTarget << std::endl;
    }
    else 
        std::cout << "Alignment failed!\n" << std::endl;
    return std::make_tuple(cloud_aligned, SourceToTarget);
 }