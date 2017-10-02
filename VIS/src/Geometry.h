#pragma once

namespace vis
{

	using Mesh = pcl::geometry::TriangleMesh<pcl::geometry::DefaultMeshTraits<pcl::PointXYZ>>;
	using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

	///
	/// The bounding sphere of a point cloud
	///
	struct BoundingSphere
	{
		Eigen::Vector3f center;
		float radius;
	};

	///
	/// Construct a point cloud representing the surface of a mesh. The point cloud
	/// will have a uniform density on the surface.
	/// @param idealMesh the mesh whose surface will be constructed
	/// @param density the number of points per square mesh unit
	///
	boost::shared_ptr<PointCloud> constructSurfaceCloud(const Mesh& idealMesh, float density);


	///
	/// Calculate the distance between two points in a three dimensional triangle.
	/// @param points the points in the triangle
	/// @param i1 the index of the first edge point
	/// @param i2 the index of the second edge point
	///
	static float triangleEdgeDistance(const std::array<pcl::PointXYZ, 3>& points, int i1, int i2);


	///
	/// Calculate the area of a 3D triangle using Heron's formula
	/// @param facePoints the points in the tirangular face
	///
	static float triangleFaceArea(const std::array<pcl::PointXYZ, 3>& facePoints);


	///
	/// Spray points in uniform density across a triangle represented in 3D space
	/// @param pOutCloud the cloud to add points to
	/// @param facePoints the face representing the bounding triangle
	/// @param numPoints the number of points to spray
	///
	static void sprayTrianglePoints(PointCloud* pOutCloud, const std::array<pcl::PointXYZ, 3>& facePoints, int numPoints);


	///
	/// Find the furthest neighbore from a given point in a point cloud. Runs in linear
	/// time relative to the number of points in the cloud
	/// @cloud the given point cloud
	/// @point the givevn point
	///
	static pcl::PointXYZ furthestNeighbor(const PointCloud& cloud, const pcl::PointXYZ point);


	///
	/// Parameters describing the tradeoff between speed and quality of alignment
	///
	struct AlignmentQuality
	{
		/// The ratio of points to spray onto the ideal surface relative to the capture
		double surfaceSprayRatio = 1.0;

		/// The number of points to sample per square error iteration
		int squareErrorBatch = 500;

		/// The maximum acceptable difference ratio in error before the final iteration
		double squareErrorEpsilon = 0.02;

		/// The number of Y axis rotations to attempt
		int numRotations = 20;

		/// Whether to use exact bounding spheres
		bool useExactBoundingSpheres = true;

		/// Use a variant of ICP which may alter scale
		bool allowIcpScaleChange = true;
	};


	///
	/// Calculate a bounding sphere of the point cloud according to provided quality
	/// settings
	/// @param cloud the given point cloud
	/// @param qualitySettings the settings to use
	///
	BoundingSphere calculateBoundingSphere(const PointCloud& cloud, const AlignmentQuality& qualitySettings);


	///
	/// User Ritter's algorithm to compute an approximate bounding sphere of a point
	/// cloud See https://en.wikipedia.org/wiki/Bounding_sphere#Ritter.27s_bounding_sphere
	/// @param cloud the given point cloud
	///
	static BoundingSphere estimateBoundingSphere(const PointCloud& cloud);


	///
	/// Use Kutz's algorithm to compute an exact bounding sphere of a point cloud
	/// @param cloud the given point cloud
	///
	static BoundingSphere exactBoundingSphere(const PointCloud& cloud);


	///
	/// Calculate the surface area of a mesh
	/// @param mesh the given mesh
	///
	double surfaceArea(const Mesh& mesh);


	///
	/// Estimate the mean squared error from a point cloud to a reference using an
	/// iterative algorithm
	/// @param referenceTree A KD Tree with points from a reference point cloud
	/// @param target the point cloud containing errors
	/// @param qualitySettings settings describing quality
	///
	double estimateMeanSquaredError(const pcl::KdTreeFLANN<pcl::PointXYZ>& referenceTree, const PointCloud& target, const AlignmentQuality& qualitySettings);


	///
	/// Centers a pointcloud using its precomputed bounding sphere, which will be modified
	/// @param cloud the reference point cloud
	/// @param bounds the bounding sphere of the reference point cloud
	///
	boost::shared_ptr<PointCloud> centerPointCloud(const PointCloud& cloud, BoundingSphere& bounds);


	///
	/// Coarsely align a target point cloud to a reference
	/// @param spReference the reference point cloud
	/// @param spTarget the cloud to align to the reference
	/// @param qualitySettings settings describing quality
	///
	static void coarseAlignCentered(
			const boost::shared_ptr<PointCloud>& spReference,
			const BoundingSphere& refBounds,
			PointCloud& target,
			const BoundingSphere& targetBounds,
			const AlignmentQuality& qualitySettings);


	///
	/// Attempt to finely align a target point cloud to a reference. Both will be
	/// modified to be centered at the origin. The ideal surface cloud will be
	/// returned
	/// @param reference the reference mesh
	/// @param spTarget the cloud to align to the reference
	/// @param qualitySettings settings describing quality
	///
	boost::shared_ptr<PointCloud> alignPointCloud(Mesh& reference, boost::shared_ptr<PointCloud>& spTarget, const AlignmentQuality& qualitySettings);
}