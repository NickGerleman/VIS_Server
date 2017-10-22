#include "pch.h"
#include "MeshIO.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <lib/TinyObjLoader.h>

namespace vis
{

	using Vertex = std::array<float, 3>;


	std::vector<uint8_t> convertCloudToSpc(const PointCloud& cloud)
	{
		std::vector<uint8_t> buffer;

		std::array<uint32_t, 2> header
		{
			3, static_cast<uint32_t>(cloud.size())
		};
		auto* headerBytes = reinterpret_cast<uint8_t*>(&header);
		buffer.insert(buffer.end(), headerBytes, headerBytes + sizeof(header));

		for (auto& point : cloud)
		{
			std::array<float, 3> ptArray
			{
				point.x, point.y, point.z
			};

			const auto* pointBytes = reinterpret_cast<const uint8_t*>(&ptArray);
			buffer.insert(buffer.end(), pointBytes, pointBytes + sizeof(ptArray));
		}

		return buffer;
	}


	std::vector<uint8_t> convertCloudToSpc(const ErrorPointCloud& cloud)
	{
		std::vector<uint8_t> buffer;

		std::array<uint32_t, 2> header
		{
			4, static_cast<uint32_t>(cloud.size())
		};
		auto* headerBytes = reinterpret_cast<uint8_t*>(&header);
		buffer.insert(buffer.end(), headerBytes, headerBytes + sizeof(header));

		for (auto& point : cloud)
		{
			std::array<float, 4> ptArray
			{
				point.x, point.y, point.z, point.intensity
			};

			const auto* pointBytes = reinterpret_cast<const uint8_t*>(&ptArray);
			buffer.insert(buffer.end(), pointBytes, pointBytes + sizeof(ptArray));
		}

		return buffer;
	}


	std::shared_ptr<Mesh> tryLoadObjFile(const std::string& filename)
	{
		tinyobj::attrib_t attribs;
		std::vector<tinyobj::shape_t> meshShapes;
		std::vector<tinyobj::material_t> meshMaterials;
		std::string loadMessage;

		bool success = tinyobj::LoadObj(&attribs, &meshShapes,
			&meshMaterials, &loadMessage, filename.c_str());

		// We can get a warning message even if we successfully load the file
		if (!loadMessage.empty())
			std::cerr << "tinyobjloader: " << loadMessage << std::endl;

		if (!success || meshShapes.empty())
		{
			std::cerr << "Unable to load OBJ file" << std::endl;
			return nullptr;
		}

		auto& objMesh = meshShapes[0].mesh;
		auto spPclMesh = std::make_shared<Mesh>();


		// Loosley based on (horribly stolen from) the TinyObjLoader example page
		size_t indexOffset = 0;
		for (size_t faceIndex = 0; faceIndex < objMesh.num_face_vertices.size(); faceIndex++)
		{
			int numVertices = objMesh.num_face_vertices[faceIndex];
			if (numVertices != 3)
			{
				std::cerr << "OBJ file has more than 3 vertices per face" << std::endl;
				return nullptr;
			}


			std::array<Mesh::VertexIndex, 3> addedVertices;
			for (size_t vertexIndex = 0; vertexIndex < 3; vertexIndex++)
			{
				tinyobj::index_t idx = objMesh.indices[indexOffset + vertexIndex];
				float x = attribs.vertices[3 * idx.vertex_index + 0];
				float y = attribs.vertices[3 * idx.vertex_index + 1];
				float z = attribs.vertices[3 * idx.vertex_index + 2];

				addedVertices[vertexIndex] = spPclMesh->addVertex(pcl::PointXYZ(x, y, z));
			}

			spPclMesh->addFace(addedVertices[0], addedVertices[1], addedVertices[2]);
			indexOffset += 3;
		}

		return spPclMesh;
	}


#pragma pack(1)
	struct StlHeader
	{
		uint8_t metadata[80];
		uint32_t numTriangles;
	};


#pragma pack(1)
	struct StlTriangle
	{
		std::array<float, 3> normalVector;
		std::array<Vertex, 3> vertices;
		uint16_t attrib;
	};


	struct VertexHash
	{
		size_t operator()(const Vertex v) const
		{
			std::hash<float> fHash;
			return fHash(v[0]) ^ fHash(v[1]) ^ fHash(v[2]);
		}
	};


	static std::shared_ptr<Mesh> meshFromStlTriangles(const StlTriangle* triangles, uint32_t numTriangles)
	{
		// PCL stores vertices by index, we need to manually track this
		auto spPclMesh = std::make_shared<Mesh>();
		std::unordered_map<Vertex, Mesh::VertexIndex, VertexHash> vertexIndices;

		for (uint32_t idxTriangle = 0; idxTriangle < numTriangles; idxTriangle++)
		{
			const StlTriangle& triangle = triangles[idxTriangle];
			std::array<Mesh::VertexIndex, 3> addedVertices;

			// Create a triangle of PCL vertex indices added
			for (int idxVert = 0; idxVert < 3; idxVert++)
			{
				const auto& vert = triangle.vertices[idxVert];
				auto vertIt = vertexIndices.find(vert);
				if (vertIt == vertexIndices.end())
				{
					auto idx = spPclMesh->addVertex(pcl::PointXYZ(vert[0], vert[1], vert[2]));
					addedVertices[idxVert] = idx;
					vertexIndices[vert] = idx;
				}
				else
				{
					addedVertices[idxVert] = vertIt->second;
				}
			}

			spPclMesh->addFace(addedVertices[0], addedVertices[1], addedVertices[2]);
		}

		return spPclMesh;
	}


	std::shared_ptr<Mesh> tryLoadBinaryStlFile(const std::string& filename)
	{
		if (!boost::filesystem::exists(filename))
			return nullptr;

		auto fileSize = boost::filesystem::file_size(filename);
		if (fileSize < sizeof(StlHeader))
			return nullptr;

		boost::iostreams::mapped_file_source mappedFile(filename);
		auto headerData = reinterpret_cast<const StlHeader *>(mappedFile.data());
		auto minSize = sizeof(StlHeader) + headerData->numTriangles * sizeof(StlTriangle);
		if (fileSize < minSize)
			return nullptr;

		auto fileTriangles = reinterpret_cast<const StlTriangle *>(mappedFile.data() + sizeof(StlHeader));
		return meshFromStlTriangles(fileTriangles, headerData->numTriangles);
	}

}
