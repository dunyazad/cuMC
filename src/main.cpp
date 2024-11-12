#include <Common.h>
#include <App/App.h>
#include <App/AppEventHandlers.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include <Algorithm/SVO.h>
#include <Algorithm/Octree.hpp>

#include <Algorithm/CustomPolyDataFilter.h>
#include <Algorithm/vtkMedianFilter.h>
#include <Algorithm/vtkQuantizingFilter.h>

#include <Debugging/VisualDebugging.h>

#include <CUDA/HashTable.cuh>
#include <CUDA/KDTree.cuh>
#include <CUDA/KDITree.cuh>
#include <CUDA/SVO.cuh>
#include <CUDA/AOctree.cuh>

#include <CUDA/Processing.cuh>

#include <CUDA/CUDA.cuh>

int pid = 0;
size_t size_0 = 0;
size_t size_45 = 0;
float transform_0[16];
float transform_45[16];
unsigned char image_0[400 * 480];
unsigned char image_45[400 * 480];
Eigen::Vector3f points_0[400 * 480];
Eigen::Vector3f points_45[400 * 480];
Eigen::AlignedBox3f aabb_0(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
Eigen::AlignedBox3f aabb_45(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
Eigen::AlignedBox3f gaabb_0(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
Eigen::AlignedBox3f gaabb_45(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
Eigen::AlignedBox3f taabb(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
Eigen::AlignedBox3f lmax(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

vector<Eigen::Vector3f> patchPoints_0;
vector<Eigen::Vector3f> patchPoints_45;
vector<Eigen::Vector3f> inputPoints;

void LoadPatch(int patchID, vtkRenderer* renderer)
{
	stringstream ss;
	ss << "C:\\Debug\\Patches\\patch_" << patchID << ".pat";

	ifstream ifs;
	ifs.open(ss.str(), ios::in | ios::binary);

	ifs.read((char*)&pid, sizeof(int));
	ifs.read((char*)&size_0, sizeof(size_t));
	ifs.read((char*)&size_45, sizeof(size_t));
	ifs.read((char*)&transform_0, sizeof(float) * 16);
	ifs.read((char*)&transform_45, sizeof(float) * 16);
	ifs.read((char*)&image_0, sizeof(unsigned char) * 400 * 480);
	ifs.read((char*)&image_45, sizeof(unsigned char) * 400 * 480);
	ifs.read((char*)&points_0, sizeof(Eigen::Vector3f) * size_0);
	ifs.read((char*)&points_45, sizeof(Eigen::Vector3f) * size_45);

	ifs.close();

	vtkNew<vtkPoints> points;
	//points->SetNumberOfPoints(size_0);

	Eigen::Matrix4f t0(transform_0);
	Eigen::Matrix4f t45(transform_45);

	aabb_0 = Eigen::AlignedBox3f(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
	aabb_45 = Eigen::AlignedBox3f(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
	gaabb_0 = Eigen::AlignedBox3f(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
	gaabb_45 = Eigen::AlignedBox3f(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

	int count = 0;

	patchPoints_0.clear();
	patchPoints_45.clear();

	for (size_t i = 0; i < size_0; i++)
	{
		auto& p = points_0[i];

		if ((p.x() > -1000 && p.y() > -1000 && p.z() > -1000) &&
			(p.x() < 1000 && p.y() < 1000 && p.z() < 1000))
		{
			patchPoints_0.push_back(p);
			
			Eigen::Vector4f p4(p.x(), p.y(), p.z(), 1.0f);
			Eigen::Vector4f tp = t0 * p4;
			Eigen::Vector3f tp3 = Eigen::Vector3f(tp.x(), tp.y(), tp.z());

			points->InsertNextPoint(tp3.data());

			aabb_0.extend(p);
			gaabb_0.extend(tp3);
			taabb.extend(tp3);

			lmax.extend(aabb_0.max() - aabb_0.min());

			inputPoints.push_back(tp3);

			count++;
		}
	}

	for (size_t i = 0; i < size_45; i++)
	{
		auto& p = points_45[i];

		if ((p.x() > -1000 && p.y() > -1000 && p.z() > -1000) &&
			(p.x() < 1000 && p.y() < 1000 && p.z() < 1000))
		{
			patchPoints_45.push_back(p);
	
			Eigen::Vector4f p4(p.x(), p.y(), p.z(), 1.0f);
			Eigen::Vector4f tp = t45 * p4;
			Eigen::Vector3f tp3 = Eigen::Vector3f(tp.x(), tp.y(), tp.z());

			points->InsertNextPoint(tp3.data());

			aabb_45.extend(p);
			gaabb_45.extend(tp3);
			taabb.extend(tp3);

			lmax.extend(aabb_45.max() - aabb_45.min());
	
			inputPoints.push_back(tp3);

			count++;
		}
	}

	std::cout << aabb_0.min().transpose() << std::endl;
	std::cout << aabb_0.max().transpose() << std::endl;
	std::cout << gaabb_0.min().transpose() << std::endl;
	std::cout << gaabb_0.max().transpose() << std::endl;
	std::cout << aabb_45.min().transpose() << std::endl;
	std::cout << aabb_45.max().transpose() << std::endl;
	std::cout << gaabb_45.min().transpose() << std::endl;
	std::cout << gaabb_45.max().transpose() << std::endl;

	return;

	//VisualDebugging::AddCube("aabb", (aabb.min() + aabb.max()) * 0.5f, aabb.max() - aabb.min(), { 0.0f, 0.0f, 0.0f }, Color4::Red);

	vtkNew<vtkPolyData> polyData;
	polyData->SetPoints(points);

	//WritePLY(polyData, "C:\\Debug\\GPV\\Original.ply");

	//double spatialSigma = 0.5;  // adjust this based on the point cloud scale
	//double featureSigma = 0.1;  // adjust based on feature variance
	//double neighborhoodSize = 0.5;  // adjust based on the density of the point cloud

	//// Apply bilateral filter
	//vtkSmartPointer<vtkPoints> newPoints = BilateralFilter(polyData, spatialSigma, featureSigma, neighborhoodSize);

	//vtkNew<vtkPolyData> newPolyData;
	//newPolyData->SetPoints(newPoints);

	//WritePLY(newPolyData, "C:\\Debug\\GPV\\Filtered.ply");

	vtkNew<vtkVertexGlyphFilter> vertexFilter;
	vertexFilter->SetInputData(polyData);
	vertexFilter->Update();

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(vertexFilter->GetOutput());

	vtkNew<vtkActor> actor;
	actor->SetObjectName("points");
	actor->SetMapper(mapper);

	actor->GetProperty()->SetPointSize(5.0f);
	actor->GetProperty()->SetColor(1.0, 1.0, 1.0);

	renderer->AddActor(actor);
}

int main()
{
	App app;
	app.AddAppStartCallback([&](App* pApp) {
		auto renderer = pApp->GetRenderer();

		SVO::Octree octree;
		SVO::InitializeSVO(&octree, 3000000);

		{
			Processing::PatchProcessor pp;
			float sampleSize = 0.1f;
			pp.Initialize(2000, 2000, 256 * 480);

			int i = 3;
			//for (int i = 3; i < 244; i++)
				//for (int i = 3; i < 57; i++)
				//for (int cnt = 0; cnt < 4; cnt++)
			{
				printf("======= [%4d] =======\n", i);

				auto te = Time::Now();
				LoadPatch(i, renderer);
				Time::End(te, "Loading PointCloud Patch");

				//for (size_t i = 0; i < patchPoints_0.size(); i++)
				//{
				//	auto& p = patchPoints_0[i];
				//	VisualDebugging::AddSphere("Original_0", p, { 0.05f, 0.05f, 0.05f }, { 0.0f, 0.0f, 0.0f }, Color4::Red);
				//}
				//for (size_t i = 0; i < patchPoints_45.size(); i++)
				//{
				//	auto& p = patchPoints_45[i];
				//	VisualDebugging::AddSphere("Original_45", p, { 0.05f, 0.05f, 0.05f }, { 0.0f, 0.0f, 0.0f }, Color4::Blue);

				//SVO::IntegratePointCloud(&octree, aabb_0.center(), Eigen::Matrix4f(transform_0), patchPoints_0.data(), patchPoints_0.size(), 200, 14);
				//SVO::IntegratePointCloud(&octree, aabb_45.center(), Eigen::Matrix4f(transform_45), patchPoints_45.data(), patchPoints_45.size(), 200, 14);

				//SVO::IntegratePointCloudWithTSDF(&octree, aabb_0.center(), Eigen::Matrix4f(transform_0), patchPoints_0.data(), patchPoints_0.size(), 200, 14, 1.0f);
				//SVO::IntegratePointCloudWithTSDF(&octree, aabb_45.center(), Eigen::Matrix4f(transform_45), patchPoints_45.data(), patchPoints_45.size(), 200, 14, 1.0f);

				//SVO::IntegratePointCloudWithNeighborUpdate(&octree, aabb_0.center(), Eigen::Matrix4f(transform_0), patchPoints_0.data(), patchPoints_0.size(), 200, 13, 1.0f);
				//SVO::IntegratePointCloudWithNeighborUpdate(&octree, aabb_45.center(), Eigen::Matrix4f(transform_45), patchPoints_45.data(), patchPoints_45.size(), 200, 13, 1.0f);

				/*
				pp.DownSample(patchPoints_0.data(), patchPoints_0.size(), sampleSize, aabb_0.min(), aabb_0.max());
				pp.MedianFilter(pp.h_resultPoints, pp.h_numberOfResultPoints, sampleSize, aabb_0.min(), aabb_0.max());

				SVO::IntegratePointCloudWithTSDF(&octree, aabb_0.center(), Eigen::Matrix4f(transform_0), pp.h_resultPoints, pp.h_numberOfResultPoints, 200, 13, 1.0f);

				pp.DownSample(patchPoints_45.data(), patchPoints_45.size(), sampleSize, aabb_45.min(), aabb_45.max());
				pp.MedianFilter(pp.h_resultPoints, pp.h_numberOfResultPoints, sampleSize, aabb_45.min(), aabb_45.max());

				SVO::IntegratePointCloudWithTSDF(&octree, aabb_45.center(), Eigen::Matrix4f(transform_45), pp.h_resultPoints, pp.h_numberOfResultPoints, 200, 13, 1.0f);
				*/

				SVO::IntegratePointCloudWithTSDF(&octree, aabb_0.center(), Eigen::Matrix4f(transform_0), patchPoints_0.data(), patchPoints_0.size(), 200, 13, 1.0f);
				SVO::IntegratePointCloudWithTSDF(&octree, aabb_45.center(), Eigen::Matrix4f(transform_45), patchPoints_45.data(), patchPoints_45.size(), 200, 13, 1.0f);
			}

			//{
			//	float sampleSize = 0.1f;

			//	printf("[[[ patchPoints.size() ]]] : %llu\n", patchPoints_0.size());

			//	auto t = Time::Now();

			//	pp.DownSample(patchPoints_0.data(), patchPoints_0.size(), sampleSize, aabb_0.min(), aabb_0.max());

			//	t = Time::End(t, "DownSampling");

			//	printf("[[[ *pp.h_numberOfResultPoints ]]] : %llu\n", pp.h_numberOfResultPoints);

			//	//for (size_t i = 0; i < pp.h_numberOfResultPoints; i++)
			//	//{
			//	//	auto& p = pp.h_resultPoints[i];

			//	//	//p += Eigen::Vector3f(0.0f, 0.0f, 10.0f);

			//	//	VisualDebugging::AddSphere("DownSample", p, { 0.05f, 0.05f, 0.05f }, { 0.0f, 0.0f, 0.0f }, Color4::Blue);
			//	//}
			//}

			//{
			//	float sampleSize = 0.1f;

			//	printf("[[[ patchPoints.size() ]]] : %llu\n", patchPoints_0.size());

			//	auto t = Time::Now();

			//	pp.MedianFilter(patchPoints_0.data(), patchPoints_0.size(), sampleSize, aabb_45.min(), aabb_45.max());

			//	t = Time::End(t, "MedianFilter");

			//	printf("[[[ *pp.h_numberOfResultPoints ]]] : %llu\n", pp.h_numberOfResultPoints);

			//	//for (size_t i = 0; i < pp.h_numberOfResultPoints; i++)
			//	//{
			//	//	auto& p = pp.h_resultPoints[i];

			//	//	//p += Eigen::Vector3f(0.0f, 0.0f, 10.0f);

			//	//	VisualDebugging::AddSphere("Filter", p, { 0.05f, 0.05f, 0.05f }, { 0.0f, 0.0f, 0.0f }, Color4::Red);
			//	//}
			//}
		}

		SVO::TraverseOctree(&octree, [&](const SVO::SVONode& node, int depth) {

			//std::cout << "Node TSDF Value: " << node.tsdfValue << std::endl;

			//if (node.occupied)
			{
				stringstream ss;
				ss << "Cubes_" << depth;
				VisualDebugging::AddCube(ss.str(), node.center, { node.size, node.size, node.size }, { 0.0f, 0.0f, 1.0f }, Color4::White);
			}
			});

		vector<SVO::Triangle> triangles;
		SVO::ExtractTrianglesFromOctree(&octree, triangles);
		printf("triangles : %llu\n", triangles.size());

		for (auto& t : triangles)
		{
			VisualDebugging::AddTriangle("t", t.vertices[0], t.vertices[1], t.vertices[2], Color4::White);
		}

		VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 100.0f, 0.0f, 0.0f }, Color4::Red);
		VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 100.0f, 0.0f }, Color4::Green);
		VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 0.0f, 100.0f }, Color4::Blue);
		});

	app.AddAppUpdateCallback([&](App* pApp) {
		//cout << "App Update" << endl;
		});

	app.AddKeyPressCallback(OnKeyPress);
	app.Run();

	return 0;
}
