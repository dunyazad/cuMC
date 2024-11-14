#include <Common.h>
#include <App/App.h>
#include <App/AppEventHandlers.h>

#include <Algorithm/SVO.h>
#include <Algorithm/Octree.hpp>

#include <Algorithm/CustomPolyDataFilter.h>
#include <Algorithm/vtkMedianFilter.h>
#include <Algorithm/vtkQuantizingFilter.h>

#include <Debugging/VisualDebugging.h>
using VD = VisualDebugging;

#include <CUDA/HashTable.cuh>
#include <CUDA/KDTree.cuh>
#include <CUDA/KDITree.cuh>
#include <CUDA/SVO.cuh>
#include <CUDA/AOctree.cuh>

#include <CUDA/Processing.cuh>

#include <CUDA/CUDA.cuh>
#include <CUDA/IndexedTriangleSet.cuh>

int pid = 0;
size_t size_0 = 0;
size_t size_45 = 0;
float transform_0[16];
float transform_45[16];
Eigen::Vector3f cameraPosition;
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

vector<Eigen::Matrix4f> cameraTransforms;

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
	ifs.read((char*)&cameraPosition, sizeof(float) * 3);
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

tuple<Eigen::Matrix4f, Eigen::Vector3f> LoadPatchTransform(int patchID)
{
	stringstream ss;
	ss << "C:\\Debug\\Patches\\patch_" << patchID << ".pat";

	ifstream ifs;
	ifs.open(ss.str(), ios::in | ios::binary);

	ifs.read((char*)&pid, sizeof(int));
	ifs.read((char*)&size_0, sizeof(size_t));
	ifs.read((char*)&size_45, sizeof(size_t));
	ifs.read((char*)&transform_0, sizeof(float) * 16);
	ifs.read((char*)&cameraPosition, sizeof(float) * 3);

	ifs.close();

	return make_tuple(Eigen::Matrix4f(transform_0), Eigen::Vector3f(cameraPosition));
}

void SaveTRNFile()
{
	ofstream ofs;
	ofs.open("C:\\Debug\\Patches\\transforms.trn", ios::out | ios::binary);

	int numberOfTransforms = 4252;
	ofs.write((char*)&numberOfTransforms, sizeof(int));

	for (size_t i = 0; i < 4252; i++)
	{
		printf("Patch : %4d\n", i);

		auto [transform, cameraPosition] = LoadPatchTransform(i);

		ofs.write((char*)transform.data(), sizeof(float) * 16);
		ofs.write((char*)cameraPosition.data(), sizeof(float) * 3);
	}
	ofs.close();
}

void LoadTRNFile()
{
	ifstream ifs;
	ifs.open("C:\\Debug\\Patches\\transforms.trn", ios::in | ios::binary);
	
	int numberOfTransforms = 0;
	ifs.read((char*)&numberOfTransforms, sizeof(int));

	for (size_t i = 0; i < numberOfTransforms; i++)
	{
		printf("Patch %4d\n", i);

		ifs.read((char*)&transform_0, sizeof(float) * 16);
		ifs.read((char*)&cameraPosition, sizeof(float) * 3);
		Eigen::Matrix4f transform(transform_0);

		cameraTransforms.push_back(transform);

		Eigen::Vector3f zero = (transform * Eigen::Vector4f(0.0f, 0.0f, 20.0f, 1.0f)).head<3>();
		Eigen::Vector3f right = (transform * Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f)).head<3>();
		Eigen::Vector3f up = (transform * Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.0f)).head<3>();
		Eigen::Vector3f front = (transform * Eigen::Vector4f(0.0f, 0.0f, -1.0f, 0.0f)).head<3>();
		Eigen::Vector3f cam = (transform * Eigen::Vector4f(cameraPosition.x(), cameraPosition.y(), cameraPosition.z(), 1.0f)).head<3>();

		//VisualDebugging::AddSphere("sphere", zero, { 0.5f, 0.5f, 0.5f }, { 0.0f, 0.0f, 1.0f }, Color4::Red);
		//VisualDebugging::AddLine("transform", zero, zero + right, Color4::Red);
		//VisualDebugging::AddLine("transform", zero, zero + up, Color4::Green);
		//VisualDebugging::AddLine("transform", zero, zero + (front * 10.0f), Color4::Yellow);
	}
}

void LoadModel(vtkRenderer* renderer, const string& filename)
{
	vtkNew<vtkPLYReader> reader;
	reader->SetFileName(filename.c_str());
	reader->Update();

	vtkPolyData* polyData = reader->GetOutput();
	
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(polyData);

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);

	renderer->AddActor(actor);

	renderer->ResetCamera();

	return;

	vtkPoints* points = polyData->GetPoints();
	vtkFloatArray* floatArray = vtkArrayDownCast<vtkFloatArray>(points->GetData());
	float* pointData = static_cast<float*>(floatArray->GetPointer(0));
	size_t numPoints = points->GetNumberOfPoints();

	vtkCellArray* cells = polyData->GetPolys();
	vtkIdType npts;
	const vtkIdType* pts;

	std::vector<uint32_t> triangleIndices;

	cells->InitTraversal();
	while (cells->GetNextCell(npts, pts))
	{
		if (npts == 3)
		{
			for (vtkIdType i = 0; i < 3; ++i)
			{
				triangleIndices.push_back(static_cast<uint32_t>(pts[i]));
			}
		}
	}

	CUDA::Mesh mesh = CUDA::AllocateMesh(pointData, numPoints, triangleIndices.data(), triangleIndices.size() / 3);
	CUDA::DeallocMesh(&mesh);

	vector<Algorithm::kdNode> kdNodes(numPoints);
	for (size_t i = 0; i < numPoints; i++)
	{
		kdNodes[i].id = i;
		kdNodes[i].x[0] = pointData[i * 3 + 0];
		kdNodes[i].x[1] = pointData[i * 3 + 1];
		kdNodes[i].x[2] = pointData[i * 3 + 2];
	}
	Algorithm::kdTree tree;
	tree.init(numPoints); // Allocates memory and prepares the tree

	auto t = Time::Now();
	// Build the KD-tree
	tree.kdRoot = tree.buildTree(kdNodes.data(), numPoints, 0, 3);
	t = Time::End(t, "KDTree Build");
}

int transformIndex = 0;
void MoveCamera(App* pApp, vtkCamera* camera, const Eigen::Matrix4f& tm)
{
	Eigen::Vector3f forward = tm.block<3, 1>(0, 2);
	Eigen::Vector3f position = tm.block<3, 1>(0, 3);

	Eigen::Vector3f cameraPosition = position + forward * 20.0f;

	camera->SetPosition(cameraPosition.x(), cameraPosition.y(), cameraPosition.z());
	camera->SetFocalPoint(position.x(), position.y(), position.z());
	//camera->SetParallelScale(10.0);

	//float pixel_to_world_ratio = 0.1;
	//float world_height = 480.0f * pixel_to_world_ratio;
	//camera->SetParallelScale(world_height / 2);

	camera->Modified();

	pApp->GetRenderer()->ResetCameraClippingRange();
	pApp->GetRenderWindow()->Render();

}

void CaptureNextFrame(App* pApp)
{
	if (transformIndex >= cameraTransforms.size()) return;

	vtkCamera* camera = pApp->GetRenderer()->GetActiveCamera();

	auto& tm = cameraTransforms[transformIndex];

	MoveCamera(pApp, camera, tm);

	pApp->CaptureColorAndDepth("C:\\Resources\\2D\\Captured\\RGBD");
	printf("Saved %d\n", transformIndex);

	transformIndex++;
}

void LoadDepthImage()
{
	std::string depthmapFileName = "C:\\Resources\\2D\\Captured\\RGBD\\depth_0.png";

	vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
	reader->SetFileName(depthmapFileName.c_str());
	reader->Update();

	vtkImageData* imageData = reader->GetOutput();
	int* dims = imageData->GetDimensions();

	vtkUnsignedCharArray* pixelArray = vtkUnsignedCharArray::SafeDownCast(imageData->GetPointData()->GetScalars());

	if (!pixelArray) {
		std::cerr << "Failed to get pixel data from image!" << std::endl;
		return;
	}

	for (int z = 0; z < dims[2]; z++) {
		for (int y = 0; y < dims[1]; y++) {
			for (int x = 0; x < dims[0]; x++) {
				int idx = z * dims[0] * dims[1] + y * dims[0] + x;
				unsigned char pixelValue = pixelArray->GetValue(idx);

				double depthValue = static_cast<double>(pixelValue) / 255.0;

				depthValue *= 200;

				// Print out depth value if needed
				//std::cout << "Depth value at (" << x << ", " << y << "): " << depthValue << std::endl;

				Eigen::Vector4f p4(x * 0.05f - 12.8f, y * 0.05f - 24.0f, -depthValue, 1.0f);
				Eigen::Vector3f p = (cameraTransforms[0] * p4).head<3>();

				VisualDebugging::AddSphere("depth", p, { 0.05f, 0.05f, 0.05f }, { 0.0f, 0.0f, 1.0f }, Color4::Red);
			}
		}
	}
}

bool enabledToCapture = false;
int main()
{
	vtkActor* planeActor = nullptr;

	App app;
	app.AddAppStartCallback([&](App* pApp) {
		auto renderer = pApp->GetRenderer();

		LoadModel(renderer, "C:\\Resources\\3D\\PLY\\Complete\\Lower.ply");

		auto camera = renderer->GetActiveCamera();
		camera->SetParallelProjection(true);
		// Parallel Scale은 카메라 절반 높이
		// 픽셀당 3D 공간의 유닛 * 창 높이 / 2
		// 여기에선 256 x 480이므로 픽셀당 0.1, 창높이 480
		// 480 * 0.1 / 2 = 24
		camera->SetParallelScale(24);

		LoadPatch(0, renderer);

		//SaveTRNFile();

		LoadTRNFile();

		//LoadDepthImage();

		MoveCamera(pApp, camera, cameraTransforms[0]);

		{
			auto& tm = cameraTransforms[0];

#pragma region Drawing ViewPort
			/*
			Eigen::Vector3f cd = -tm.block<3, 1>(0, 2);

			//float ratio = 0.05f;
			float ratio = 0.1f;

			Eigen::Vector3f topLeft = (tm * Eigen::Vector4f(-128.0f * ratio, 240.0f * ratio, 20.0f, 1.0f)).head<3>();
			Eigen::Vector3f topRight = (tm * Eigen::Vector4f(128.0f * ratio, 240.0f * ratio, 20.0f, 1.0f)).head<3>();
			Eigen::Vector3f bottomLeft = (tm * Eigen::Vector4f(-128.0f * ratio, -240.0f * ratio, 20.0f, 1.0f)).head<3>();
			Eigen::Vector3f bottomRight = (tm * Eigen::Vector4f(128.0f * ratio, -240.0f * ratio, 20.0f, 1.0f)).head<3>();

			Eigen::Vector3f ttl = topLeft + cd * 20.0f;
			Eigen::Vector3f ttr = topRight + cd * 20.0f;
			Eigen::Vector3f tbl = bottomLeft + cd * 20.0f;
			Eigen::Vector3f tbr = bottomRight + cd * 20.0f;

			VD::AddLine("rays", topLeft, ttl, Color4::Red);
			VD::AddLine("rays", topRight, ttr, Color4::Red);
			VD::AddLine("rays", bottomLeft, tbl, Color4::Red);
			VD::AddLine("rays", bottomRight, tbr, Color4::Red);

			VD::AddLine("rays", ttl, ttr, Color4::Red);
			VD::AddLine("rays", tbl, tbr, Color4::Red);
			VD::AddLine("rays", ttl, tbl, Color4::Red);
			VD::AddLine("rays", ttr, tbr, Color4::Red);
			*/
#pragma endregion


			//for (float y = -24.0f; y < 24.0f; y += 0.1f)
			//{
			//	for (float x = -12.8f; x < 12.8f; x += 0.1f)
			//	{
			//		auto ps = Transform(tm, { x, y, 0.0f });
			//		auto pe = Transform(tm, { x, y, 20.0f });

			//		//VD::AddLine("rays", ps, pe, Color4::Red);
			//		VD::AddSphere("rayorigin", ps, { 0.1f, 0.1f, 0.1f }, {0.0f, 0.0f, 1.0f}, Color4::Red);
			//	}
			//}
		}


		{
			auto& tm = cameraTransforms[0];

			vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
			windowToImageFilter->SetInput(pApp->GetRenderWindow());
			//windowToImageFilter->SetMagnification(1); // No magnification
			windowToImageFilter->SetInputBufferTypeToZBuffer(); // Get the depth buffer
			windowToImageFilter->Update();

			// Access the depth buffer as an image
			vtkSmartPointer<vtkImageData> depthImage = windowToImageFilter->GetOutput();

			// Get the depth values as a float array
			vtkSmartPointer<vtkFloatArray> depthArray = vtkFloatArray::SafeDownCast(depthImage->GetPointData()->GetScalars());

			double* clippingRange = camera->GetClippingRange();
			float depthRatio = (float)(clippingRange[1] - clippingRange[0]);

			if (depthArray) {
				vtkIdType index = 0;
				for (float y = -24.0f; y < 24.0f; y += 0.1f)
				{
					for (float x = -12.8f; x < 12.8f; x += 0.1f)
					{
						float depth = depthArray->GetValue(index++);

						auto ps = Transform(tm, { x, y, -depth * depthRatio + 20.0f });
						auto pe = Transform(tm, { x, y, 20.0f });

						//VD::AddLine("rays", ps, pe, Color4::Red);
						VD::AddSphere("rayorigin", ps, { 0.1f, 0.1f, 0.1f }, {0.0f, 0.0f, 1.0f}, Color4::Red);
					}
				}

				//// Iterate over the depth values (if needed)
				//for (vtkIdType i = 0; i < depthArray->GetNumberOfTuples(); i++) {
				//	float depth = depthArray->GetValue(i);
				//	// Use depth as needed (for example, printing)
				//	//std::cout << "Depth at point " << i << ": " << depth << std::endl;

				//	float x = (float)(i % 256) * 0.1f;
				//	float y = (float)(i / 256) * 0.1f;

				//	Eigen::Vector3f p = (tm * Eigen::Vector4f(x, y, -depth, 1.0f)).head<3>();

				//	VD::AddSphere("points", p, { 0.05f, 0.05f , 0.05f }, { 0.0f, 0.0f, 1.0f }, Color4::Red);
				//}
			}
		}

		//VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 100.0f, 0.0f, 0.0f }, Color4::Red);
		//VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 100.0f, 0.0f }, Color4::Green);
		//VisualDebugging::AddLine("axes", { 0, 0, 0 }, { 0.0f, 0.0f, 100.0f }, Color4::Blue);

		enabledToCapture = true;
		});

	app.AddAppUpdateCallback([&](App* pApp) {
		if (enabledToCapture)
		{
			//CaptureNextFrame(pApp);
		}
	});

	app.AddKeyPressCallback(OnKeyPress);
	//app.Run();
	app.Run(256, 480, false);

	return 0;
}
