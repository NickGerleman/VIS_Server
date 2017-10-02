#include "pch.h"
#include "Geometry.h"

using namespace Eigen;

namespace vis
{

	boost::shared_ptr<PointCloud> constructSurfaceCloud(const Mesh& idealMesh, float density)
	{
		auto originalCloud = idealMesh.getVertexDataCloud();
		auto spOutputCloud = boost::make_shared<PointCloud>();

		for (Mesh::FaceIndex i(0); idealMesh.isValid(i); i++)
		{
			auto itVertex = idealMesh.getVertexAroundFaceCirculator(i);

			std::array<pcl::PointXYZ, 3> facePoints;
			for (size_t i = 0; i < 3; i++) {
				auto vertexIndex = itVertex.getTargetIndex();
				facePoints[i] = (originalCloud)[vertexIndex.get()];
				itVertex++;
			}

			float numPoints = std::fmax(1.0f, density * triangleFaceArea(facePoints));
			sprayTrianglePoints(spOutputCloud.get(), facePoints, static_cast<int>(numPoints));
		}

		return spOutputCloud;
	}


	static float triangleEdgeDistance(const std::array<pcl::PointXYZ, 3>& points, int i1, int i2)
	{
		float dx = points[i1].x - points[i2].x;
		float dy = points[i1].y - points[i2].y;
		float dz = points[i1].z - points[i2].z;

		return sqrt((dx * dx) + (dy * dy) + (dz*dz));
	}


	static float triangleFaceArea(const std::array<pcl::PointXYZ, 3>& facePoints)
	{
		float d1 = triangleEdgeDistance(facePoints, 0, 1);
		float d2 = triangleEdgeDistance(facePoints, 1, 2);
		float d3 = triangleEdgeDistance(facePoints, 2, 0);

		float s = 0.5f * (d1 + d2 + d3);
		return sqrt(s * (s - d1) * (s - d2) * (s - d3));
	}


	static void sprayTrianglePoints(PointCloud* pOutCloud, const std::array<pcl::PointXYZ, 3>& facePoints, int numPoints)
	{
		Vector3f v1
		{
			facePoints[1].x - facePoints[0].x,
			facePoints[1].y - facePoints[0].y,
			facePoints[1].z - facePoints[0].z,
		};

		Vector3f v2
		{
			facePoints[2].x - facePoints[0].x,
			facePoints[2].y - facePoints[0].y,
			facePoints[2].z - facePoints[0].z,
		};

		for (int i = 0; i < numPoints; i++)
		{
			float randA, randB;
			do
			{
				randA = ((float)std::rand() / (RAND_MAX));
				randB = ((float)std::rand() / (RAND_MAX));
			} while (randA + randB > 1.0);

			float px = (randA * v1)[0] + (randB * v2)[0] + facePoints[0].x;
			float py = (randA * v1)[1] + (randB * v2)[1] + facePoints[0].y;
			float pz = (randA * v1)[2] + (randB * v2)[2] + facePoints[0].z;

			pOutCloud->push_back({ px, py, pz });
		}

	}


	static pcl::PointXYZ furthestNeighbor(const PointCloud& cloud, const pcl::PointXYZ point)
	{
		Vector3f egPoint(point.x, point.y, point.z);

		float maxDist = 0;
		auto furthest = point;
		for (const auto& point : cloud)
		{
			float distFromRand = (egPoint - Eigen::Vector3f(point.x, point.y, point.z)).norm();
			if (distFromRand > maxDist)
			{
				maxDist = distFromRand;
				furthest = point;
			}
		}

		return furthest;
	}


	BoundingSphere calculateBoundingSphere(const PointCloud& cloud, const AlignmentQuality& qualitySettings)
	{
		return qualitySettings.useExactBoundingSpheres
			? exactBoundingSphere(cloud)
			: estimateBoundingSphere(cloud);
	}


	BoundingSphere estimateBoundingSphere(const PointCloud& cloud)
	{
		auto pointX = cloud[std::rand() % cloud.size()];
		auto pointY = furthestNeighbor(cloud, pointX);
		auto pointZ = furthestNeighbor(cloud, pointY);
	
		Eigen::Vector3f y(pointY.x, pointY.y, pointY.z);
		Eigen::Vector3f z(pointZ.x, pointZ.y, pointZ.z);
		auto midPoint = (y + z) / 2;
		auto radius = (y - z).norm();
	
		for (auto& point : cloud)
		{
			auto dist = (midPoint - Eigen::Vector3f(point.x, point.y, point.z)).norm();
			radius = std::max(radius, dist);
		}
	
		return { midPoint, radius };
	}


	class SebAccessor
	{
	public:
		SebAccessor(const PointCloud& cloud)
			: m_cloud(cloud) {}

		Seb::Point<float> operator[](size_t idx) const
		{
			Seb::Point<float> pt(3);
			pt[0] = m_cloud[idx].x;
			pt[1] = m_cloud[idx].y;
			pt[2] = m_cloud[idx].z;

			return pt;
		}

		size_t size() const
		{
			return m_cloud.size();
		}

	private:
		const PointCloud& m_cloud;

	};


	BoundingSphere exactBoundingSphere(const PointCloud& cloud)
	{
		SebAccessor cloudAccessor(cloud);
		Seb::Smallest_enclosing_ball<float, Seb::Point<float>, SebAccessor> ball(3, cloudAccessor);

		Eigen::Vector3f center
		(
			ball.center_begin()[0],
			ball.center_begin()[1],
			ball.center_begin()[2]
		);
		return { center, ball.radius() };
	}


	double surfaceArea(const Mesh& mesh)
	{
		double surfaceArea = 0;

		for (Mesh::FaceIndex i(0); mesh.isValid(i); i++)
		{
			auto itVertex = mesh.getVertexAroundFaceCirculator(i);

			std::array<pcl::PointXYZ, 3> facePoints;
			for (size_t i = 0; i < 3; i++) {
				auto vertexIndex = itVertex.getTargetIndex();
				facePoints[i] = mesh.getVertexDataCloud()[vertexIndex.get()];
				itVertex++;
			}

			surfaceArea += triangleFaceArea(facePoints);
		}

		return surfaceArea;
	}


	double estimateMeanSquaredError(const pcl::KdTreeFLANN<pcl::PointXYZ>& referenceTree, const PointCloud& target, const AlignmentQuality& qualitySettings)
	{
		double squareError = 0;
		size_t numPointsSampled = 0;

		std::vector<int> nearest(1);
		std::vector<float> distSquare(1);

		while (true)
		{
			double lastMeanSquared = squareError / numPointsSampled;

			for (int i = 0; i < qualitySettings.squareErrorBatch; i++)
			{
				size_t idx = rand() % target.size();
				int numFound = referenceTree.nearestKSearch(target[idx], 1, nearest, distSquare);
				squareError += distSquare[0];
			}

			numPointsSampled += qualitySettings.squareErrorBatch;
			double meanSquared = squareError / numPointsSampled;
			double pctChange = std::abs(1.0 - (meanSquared / lastMeanSquared));

			if (pctChange < qualitySettings.squareErrorEpsilon)
				break;
		}

		return squareError / numPointsSampled;
	}


	boost::shared_ptr<PointCloud> centerPointCloud(const PointCloud& cloud, BoundingSphere& bounds)
	{
		auto translate = Eigen::Translation3f
		(
			-bounds.center.x(),
			-bounds.center.y(),
			-bounds.center.z()
		);

		auto spCenteredCloud = boost::make_shared<PointCloud>();
		pcl::transformPointCloud(cloud, *spCenteredCloud, Eigen::Affine3f(translate));
		bounds.center = Eigen::Vector3f(0, 0, 0);
		return spCenteredCloud;

	}



	static void coarseAlignCentered(
		const boost::shared_ptr<PointCloud>& spReference,
		const BoundingSphere& refBounds,
		PointCloud& target,
		const BoundingSphere& targetBounds,
		const AlignmentQuality& qualitySettings)
	{
		// Scale the target to match the reference size
		Eigen::Affine3f transform(Affine3d::Identity());
		transform.scale(refBounds.radius / targetBounds.radius);

		// Test for the ideal Y axis rotation
		float bestRotation = 0;
		double bestError = std::numeric_limits<double>::max();
		PointCloud alignedTarget;

		pcl::KdTreeFLANN<pcl::PointXYZ> referenceTree;
		referenceTree.setInputCloud(spReference);
		for (int i = 0; i < qualitySettings.numRotations; i++)
		{
			float angle = static_cast<float>(2.0 * M_PI) * (static_cast<float>(i) / qualitySettings.numRotations);
			Eigen::Affine3f candidateTransform(transform);
			candidateTransform.rotate(AngleAxisf(angle, Vector3f::UnitY()));
			pcl::transformPointCloud(target, alignedTarget, candidateTransform);

			double error = estimateMeanSquaredError(referenceTree, alignedTarget, qualitySettings);
			if (error < bestError)
			{
				bestError = error;
				bestRotation = angle;
			}
		}

		// Apply the final transform
		transform.rotate(AngleAxisf(bestRotation, Vector3f::UnitY()));
		pcl::transformPointCloud(target, alignedTarget, transform);
		target = alignedTarget;
	}


	boost::shared_ptr<PointCloud> alignPointCloud(Mesh& reference, boost::shared_ptr<PointCloud>& spTarget, const AlignmentQuality& qualitySettings)
	{
		// Center our existing mesh and get an ideal cloud
		auto refBounds = calculateBoundingSphere(reference.getVertexDataCloud(), qualitySettings);
		auto spCenteredMeshCloud = centerPointCloud(reference.getVertexDataCloud(), refBounds);
		reference.setVertexDataCloud(*spCenteredMeshCloud);

		double sprayDensity = (qualitySettings.surfaceSprayRatio / surfaceArea(reference)) * spTarget->size();
		auto spReferenceCloud = constructSurfaceCloud(reference, static_cast<float>(sprayDensity));

		// Center our target
		auto targetBounds = calculateBoundingSphere(*spTarget, qualitySettings);
		auto spCenteredTarget = centerPointCloud(*spTarget, targetBounds);
		*(spTarget) = *spCenteredTarget;

		// Get translation, scale, and coarse rotation in check
		coarseAlignCentered(spReferenceCloud, refBounds, *spTarget, targetBounds, qualitySettings);

		// Use ICP to get finer alignment
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(spTarget);
		icp.setInputTarget(spReferenceCloud);

		using TransformSvd = pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>;
		using TransformSvdScale = pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ, pcl::PointXYZ>;

		if (qualitySettings.allowIcpScaleChange)
			icp.setTransformationEstimation(boost::make_shared<TransformSvdScale>());
		else
			icp.setTransformationEstimation(boost::make_shared<TransformSvd>());
		
		auto spAlignedCloud = boost::make_shared<PointCloud>();
		icp.align(*spAlignedCloud);
		(*spTarget) = *spAlignedCloud;

		return spReferenceCloud;
	}
}
