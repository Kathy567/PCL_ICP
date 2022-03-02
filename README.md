Iterative Closest Point

PCL 库中提供了以下 ICP 的接口及其变种：

点到点：pcl::IterativeClosestPoint< PointSource, PointTarget, Scalar >
点到面：pcl::IterativeClosestPointWithNormals< PointSource, PointTarget, Scalar >
面到面：pcl::GeneralizedIterativeClosestPoint< PointSource, PointTarget >
……

其中，IterativeClosestPoint 模板类是 ICP 算法的一个基本实现，其优化求解方法基于 Singular Value Decomposition (SVD)，算法迭代结束条件包括：
最大迭代次数：Number of iterations has reached the maximum user imposed number of iterations (via setMaximumIterations)
两次变换矩阵之间的差值：The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via setTransformationEpsilon)
均方误差：The sum of Euclidean squared errors is smaller than a user defined threshold (via setEuclideanFitnessEpsilon)

IterativeClosestPoint<PointXYZ, PointXYZ> icp;

// Set the input source and target
icp.setInputCloud (cloud_source);
icp.setInputTarget (cloud_target);

// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
icp.setMaxCorrespondenceDistance (0.05);
// Set the maximum number of iterations (criterion 1) 最大迭代次数
icp.setMaximumIterations (50);
// Set the transformation epsilon (criterion 2) 两次变换矩阵之间的差值
icp.setTransformationEpsilon (1e-8);
// Set the euclidean distance difference epsilon (criterion 3) 均方误差
icp.setEuclideanFitnessEpsilon (1);

// Perform the alignment
icp.align (cloud_source_registered);
// Obtain the transformation that aligned cloud_source to cloud_source_registered
Eigen::Matrix4f transformation = icp.getFinalTransformation ();


