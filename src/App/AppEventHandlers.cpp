#include <App/App.h>
#include <App/AppEventHandlers.h>
#include <Debugging/VisualDebugging.h>

int depthIndex = 0;

//void OnKeyPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
//{
//	vtkRenderWindowInteractor* interactor = static_cast<vtkRenderWindowInteractor*>(caller);
//	vtkRenderWindow* renderWindow = interactor->GetRenderWindow();
//	vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
//	vtkCamera* camera = renderer->GetActiveCamera();
//
//	std::string key = interactor->GetKeySym();
//
//	printf("%s\n", key.c_str());
//
//	if (key == "r")
//	{
//		std::cout << "Key 'r' was pressed. Resetting camera." << std::endl;
//		vtkRenderer* renderer = static_cast<vtkRenderer*>(clientData);
//
//		vtkCamera* camera = renderer->GetActiveCamera();
//		renderer->ResetCamera();
//
//		// Reset the camera position, focal point, and view up vector to their defaults
//		camera->SetPosition(0, 0, 1);          // Reset position to default
//		camera->SetFocalPoint(0, 0, 0);        // Reset focal point to origin
//		camera->SetViewUp(0, 1, 0);            // Reset the up vector to default (Y-axis up)
//
//		interactor->Render();
//	}
//	else if (key == "c")
//	{
//		// Generate a filename using the current time
//		std::string filename = "C:\\Resources\\2D\\Captured\\screenshot_" + Time::DateTime() + ".png";
//
//		// Capture the frame
//		vtkNew<vtkWindowToImageFilter> windowToImageFilter;
//		windowToImageFilter->SetInput(renderWindow);
//		windowToImageFilter->Update();
//
//		// Write the captured frame to a PNG file with the timestamped filename
//		vtkNew<vtkPNGWriter> pngWriter;
//		pngWriter->SetFileName(filename.c_str());
//		pngWriter->SetInputConnection(windowToImageFilter->GetOutputPort());
//		pngWriter->Write();
//
//		std::cout << "Screenshot saved as " << filename << std::endl;
//	}
//	else if (key == "p")
//	{
//		vtkRenderer* renderer = static_cast<vtkRenderer*>(clientData);
//
//		vtkCamera* camera = renderer->GetActiveCamera();
//		camera->SetParallelProjection(!camera->GetParallelProjection());
//
//		interactor->Render();
//	}
//	else if (key == "Escape")
//	{
//		std::cout << "Key 'Escape' was pressed. Exiting." << std::endl;
//		interactor->TerminateApp();
//	}
//	else if (key == "minus")
//	{
//		VisualDebugging::SetLineWidth("Spheres", 1);
//		vtkSmartPointer<vtkActor> actor = VisualDebugging::GetSphereActor("Spheres");
//		vtkSmartPointer<vtkMapper> mapper = actor->GetMapper();
//		vtkSmartPointer<vtkPolyDataMapper> polyDataMapper =
//			vtkPolyDataMapper::SafeDownCast(mapper);
//		vtkSmartPointer<vtkGlyph3DMapper> glyph3DMapper = vtkGlyph3DMapper::SafeDownCast(mapper);
//		vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(glyph3DMapper->GetInputDataObject(0, 0));
//		vtkSmartPointer<vtkPointData> pointData = polyData->GetPointData();
//		vtkSmartPointer<vtkDoubleArray> scaleArray =
//			vtkDoubleArray::SafeDownCast(pointData->GetArray("Scales"));
//		for (vtkIdType i = 0; i < scaleArray->GetNumberOfTuples(); ++i)
//		{
//			double scale[3]; // Assuming 3-component scale array (X, Y, Z)
//			scaleArray->GetTuple(i, scale);
//			//std::cout << "Scale for point " << i << ": "
//			//	<< scale[0 ] << ", " << scale[1] << ", " << scale[2] << std::endl;
//			scale[0] *= 0.9;
//			scale[1] *= 0.9;
//			scale[2] *= 0.9;
//			scaleArray->SetTuple(i, scale);
//		}
//		polyData->Modified();
//		glyph3DMapper->SetScaleArray("Scales");
//		glyph3DMapper->Update();
//	}
//	else if (key == "equal")
//	{
//		VisualDebugging::SetLineWidth("Spheres", 1);
//		vtkSmartPointer<vtkActor> actor = VisualDebugging::GetSphereActor("Spheres");
//		vtkSmartPointer<vtkMapper> mapper = actor->GetMapper();
//		vtkSmartPointer<vtkPolyDataMapper> polyDataMapper =
//			vtkPolyDataMapper::SafeDownCast(mapper);
//		vtkSmartPointer<vtkGlyph3DMapper> glyph3DMapper = vtkGlyph3DMapper::SafeDownCast(mapper);
//		vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(glyph3DMapper->GetInputDataObject(0, 0));
//		vtkSmartPointer<vtkPointData> pointData = polyData->GetPointData();
//		vtkSmartPointer<vtkDoubleArray> scaleArray =
//			vtkDoubleArray::SafeDownCast(pointData->GetArray("Scales"));
//		for (vtkIdType i = 0; i < scaleArray->GetNumberOfTuples(); ++i)
//		{
//			double scale[3]; // Assuming 3-component scale array (X, Y, Z)
//			scaleArray->GetTuple(i, scale);
//			//std::cout << "Scale for point " << i << ": "
//			//	<< scale[0 ] << ", " << scale[1] << ", " << scale[2] << std::endl;
//			scale[0] *= 1.1;
//			scale[1] *= 1.1;
//			scale[2] *= 1.1;
//			scaleArray->SetTuple(i, scale);
//		}
//		polyData->Modified();
//		glyph3DMapper->SetScaleArray("Scales");
//		glyph3DMapper->Update();
//	}
//	else if (key == "Return")
//	{
//		vtkRenderer* renderer = static_cast<vtkRenderer*>(clientData);
//		vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
//
//		// Grab the depth buffer (z-values)
//		int width = renderWindow->GetSize()[0];
//		int height = renderWindow->GetSize()[1];
//		vtkSmartPointer<vtkFloatArray> depthBuffer = vtkSmartPointer<vtkFloatArray>::New();
//		depthBuffer->SetNumberOfComponents(1);
//		depthBuffer->SetNumberOfTuples(width * height);
//
//		renderWindow->GetZbufferData(0, 0, width - 1, height - 1, depthBuffer);
//
//		// Save depth map to an image
//		float minDepth = std::numeric_limits<float>::max();
//		float maxDepth = std::numeric_limits<float>::lowest();
//
//		for (vtkIdType i = 0; i < depthBuffer->GetNumberOfTuples(); i++) {
//			float depthValue = depthBuffer->GetValue(i);
//			minDepth = std::min(minDepth, depthValue);
//			maxDepth = std::max(maxDepth, depthValue);
//		}
//
//		vtkSmartPointer<vtkImageData> depthImage = vtkSmartPointer<vtkImageData>::New();
//		depthImage->SetDimensions(width, height, 1);
//		depthImage->AllocateScalars(VTK_FLOAT, 1);
//
//		for (int y = 0; y < height; ++y)
//		{
//			for (int x = 0; x < width; ++x)
//			{
//				float depthValue = depthBuffer->GetValue((height - y - 1) * width + x);
//				float normalizedDepth = (depthValue - minDepth) / (maxDepth - minDepth);
//
//				depthImage->SetScalarComponentFromFloat(x, y, 0, 0, normalizedDepth * 255);
//
//				if (0.0f != normalizedDepth)
//				{
//					VisualDebugging::AddSphere("Depth", { (float)x / (float)width * 100.0f, (float)y / (float)height * 100.0f, normalizedDepth * 100.0f }, { 0.1f, 0.1f, 0.1f }, { 0.0f, 0.0f, 0.0f }, Color4::White);
//				}
//			}
//		}
//
//		// Optional: Cast the float image to unsigned char to save as PNG
//		vtkSmartPointer<vtkImageCast> castFilter = vtkSmartPointer<vtkImageCast>::New();
//		castFilter->SetInputData(depthImage);
//		castFilter->SetOutputScalarTypeToUnsignedChar();
//		castFilter->Update();
//
//		vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
//		writer->SetFileName("c:\\Debug\\2D\\depthmap.png");
//		writer->SetInputData(castFilter->GetOutput());
//		writer->Write();
//	}
//	else if (key == "1")
//	{
//		VisualDebugging::ToggleVisibility("Original");
//	}
//	else if (key == "2")
//	{
//		VisualDebugging::ToggleVisibility("DownSample");
//	}
//	else if (key == "3")
//	{
//		VisualDebugging::ToggleVisibility("Filter");
//	}
//	else if (key == "4")
//	{
//		VisualDebugging::ToggleVisibility("OctreeNode");
//	}
//	else if (key == "5")
//	{
//		VisualDebugging::ToggleVisibility("WiredBox_4");
//	}
//	else if (key == "6")
//	{
//		VisualDebugging::ToggleVisibility("WiredBox_5");
//	}
//	else if (key == "7")
//	{
//		VisualDebugging::ToggleVisibility("WiredBox_6");
//	}
//	else if (key == "8")
//	{
//		VisualDebugging::ToggleVisibility("WiredBox_7");
//	}
//	else if (key == "9")
//	{
//		VisualDebugging::ToggleVisibility("WiredBox_8");
//	}
//	else if (key == "0")
//	{
//		VisualDebugging::ToggleVisibility("WiredBox_9");
//	}
//	else if (key == "Left")
//	{
//		depthIndex--;
//		if (depthIndex < 0) depthIndex = 0;
//
//		for (int i = 0; i < 14; i++)
//		{
//			stringstream ss;
//			ss << "Cubes_" << i;
//			VisualDebugging::SetVisibility(ss.str(), false);
//		}
//		{
//			stringstream ss;
//			ss << "Cubes_" << depthIndex;
//			VisualDebugging::SetVisibility(ss.str(), true);
//		}
//
//		printf("Depth : %d\n", depthIndex);
//	}
//	else if (key == "Right")
//	{
//		depthIndex++;
//		if (depthIndex > 14) depthIndex = 13;
//
//		for (int i = 0; i < 14; i++)
//		{
//			stringstream ss;
//			ss << "Cubes_" << i;
//			VisualDebugging::SetVisibility(ss.str(), false);
//		}
//		{
//			stringstream ss;
//			ss << "Cubes_" << depthIndex;
//			VisualDebugging::SetVisibility(ss.str(), true);
//		}
//
//		printf("Depth : %d\n", depthIndex);
//	}
//	else if (key == "quoteleft")
//	{
//		VisualDebugging::ToggleVisibility("axes");
//	}
//	else if (key == "BackSpace")
//	{
//		auto viewMatrix = vtkToEigen(camera->GetViewTransformMatrix());
//		Eigen::Matrix4f inverseViewMatrix = viewMatrix.inverse();
//
//		Eigen::Vector3f viewDirection = -inverseViewMatrix.block<3, 1>(0, 2);
//		viewDirection.normalize();
//		Eigen::Vector3f cameraPosition = inverseViewMatrix.block<3, 1>(0, 3);
//
//		//VisualDebugging::AddLine("ViewDirection", cameraPosition, cameraPosition + viewDirection * 100, Color4::Red);
//
//		Eigen::Vector3f zero = (inverseViewMatrix * Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f)).head<3>();
//		Eigen::Vector3f right = (inverseViewMatrix * Eigen::Vector4f(10.0f, 0.0f, 0.0f, 1.0f)).head<3>();
//		Eigen::Vector3f up = (inverseViewMatrix * Eigen::Vector4f(0.0f, 10.0f, 0.0f, 1.0f)).head<3>();
//		Eigen::Vector3f front = (inverseViewMatrix * Eigen::Vector4f(0.0f, 0.0f, 10.0f, 1.0f)).head<3>();
//
//		VisualDebugging::AddLine("ViewDirection", zero, right, Color4::Red);
//		VisualDebugging::AddLine("ViewDirection", zero, up, Color4::Green);
//		VisualDebugging::AddLine("ViewDirection", zero, front, Color4::Blue);
//	}
//	else if (key == "space")
//	{
//		static int captureCount = 0;
//		stringstream ss;
//		ss << captureCount;
//
//		string depthmapFileName = "C:\\Resources\\2D\\Captured\\RGBD\\depth_" + ss.str() + ".png";
//		string colormapFileName = "C:\\Resources\\2D\\Captured\\RGBD\\color_" + ss.str() + ".png";
//
//		// Create a sub-camera with orthogonal projection
//		vtkNew<vtkCamera> subCamera;
//		subCamera->SetPosition(camera->GetPosition()); // Adjust position for orthographic view
//		subCamera->SetFocalPoint(camera->GetFocalPoint());
//		subCamera->SetViewUp(camera->GetViewUp());
//		subCamera->ParallelProjectionOn(); // Enable orthogonal projection
//		subCamera->SetParallelScale(1.0); // Adjust scale for the scene as needed
//
//		// Set the sub-camera as the active camera for the renderer
//		renderer->SetActiveCamera(subCamera);
//		renderer->Render();
//
//		// Calculate magnification factor to achieve 256x480 resolution without changing render window size
//		int originalWidth = renderWindow->GetSize()[0];
//		int originalHeight = renderWindow->GetSize()[1];
//		int magnificationX = std::ceil(256.0 / originalWidth);
//		int magnificationY = std::ceil(480.0 / originalHeight);
//		int magnification = std::max(magnificationX, magnificationY);
//
//		// Render window to image for color capture
//		vtkNew<vtkWindowToImageFilter> colorFilter;
//		colorFilter->SetInput(renderWindow);
//		colorFilter->SetInputBufferTypeToRGB(); // Capture color
//		colorFilter->SetScale(magnificationX, magnificationY); // Apply magnification for desired resolution
//		colorFilter->Update();
//
//		// Save color map to file
//		vtkNew<vtkPNGWriter> colorWriter;
//		colorWriter->SetFileName(colormapFileName.c_str());
//		colorWriter->SetInputConnection(colorFilter->GetOutputPort());
//		colorWriter->Write();
//
//		// Render window to image for depth capture
//		vtkNew<vtkWindowToImageFilter> depthFilter;
//		depthFilter->SetInput(renderWindow);
//		depthFilter->SetInputBufferTypeToZBuffer(); // Capture depth
//		depthFilter->SetScale(magnificationX, magnificationY); // Apply magnification for desired resolution
//		depthFilter->Update();
//
//		// Optionally, shift and scale depth for visibility
//		vtkNew<vtkImageShiftScale> shiftScale;
//		shiftScale->SetInputConnection(depthFilter->GetOutputPort());
//		shiftScale->SetOutputScalarTypeToUnsignedChar();
//		shiftScale->SetShift(0);
//		shiftScale->SetScale(255); // Adjust scale factor as needed for visualization
//		shiftScale->Update();
//
//		// Save depth map to file
//		vtkNew<vtkPNGWriter> depthWriter;
//		depthWriter->SetFileName(depthmapFileName.c_str());
//		depthWriter->SetInputConnection(shiftScale->GetOutputPort());
//		depthWriter->Write();
//
//		renderer->SetActiveCamera(camera);
//		renderer->Render();
//	}
//	else if (key == "Insert")
//	{
//		printf("Camera Distance : %f\n", camera->GetDistance());
//	}
//}

void OnKeyPress(App* app)
{
	vtkRenderWindowInteractor* interactor = app->GetInteractor();
	vtkRenderWindow* renderWindow = interactor->GetRenderWindow();
	vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
	vtkCamera* camera = renderer->GetActiveCamera();

	std::string key = interactor->GetKeySym();

	printf("%s\n", key.c_str());

	if (key == "r")
	{
		std::cout << "Key 'r' was pressed. Resetting camera." << std::endl;
		renderer->ResetCamera();

		// Reset the camera position, focal point, and view up vector to their defaults
		camera->SetPosition(0, 0, 1);          // Reset position to default
		camera->SetFocalPoint(0, 0, 0);        // Reset focal point to origin
		camera->SetViewUp(0, 1, 0);            // Reset the up vector to default (Y-axis up)

		interactor->Render();
	}
	else if (key == "c")
	{
		// Generate a filename using the current time
		std::string filename = "C:\\Resources\\2D\\Captured\\screenshot_" + Time::DateTime() + ".png";

		// Capture the frame
		vtkNew<vtkWindowToImageFilter> windowToImageFilter;
		windowToImageFilter->SetInput(renderWindow);
		windowToImageFilter->Update();

		// Write the captured frame to a PNG file with the timestamped filename
		vtkNew<vtkPNGWriter> pngWriter;
		pngWriter->SetFileName(filename.c_str());
		pngWriter->SetInputConnection(windowToImageFilter->GetOutputPort());
		pngWriter->Write();

		std::cout << "Screenshot saved as " << filename << std::endl;
	}
	else if (key == "p")
	{
		camera->SetParallelProjection(!camera->GetParallelProjection());

		interactor->Render();
	}
	else if (key == "Escape")
	{
		std::cout << "Key 'Escape' was pressed. Exiting." << std::endl;
		interactor->TerminateApp();
	}
	else if (key == "minus")
	{
		VisualDebugging::SetLineWidth("Spheres", 1);
		vtkSmartPointer<vtkActor> actor = VisualDebugging::GetSphereActor("Spheres");
		vtkSmartPointer<vtkMapper> mapper = actor->GetMapper();
		vtkSmartPointer<vtkPolyDataMapper> polyDataMapper =
			vtkPolyDataMapper::SafeDownCast(mapper);
		vtkSmartPointer<vtkGlyph3DMapper> glyph3DMapper = vtkGlyph3DMapper::SafeDownCast(mapper);
		vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(glyph3DMapper->GetInputDataObject(0, 0));
		vtkSmartPointer<vtkPointData> pointData = polyData->GetPointData();
		vtkSmartPointer<vtkDoubleArray> scaleArray =
			vtkDoubleArray::SafeDownCast(pointData->GetArray("Scales"));
		for (vtkIdType i = 0; i < scaleArray->GetNumberOfTuples(); ++i)
		{
			double scale[3]; // Assuming 3-component scale array (X, Y, Z)
			scaleArray->GetTuple(i, scale);
			//std::cout << "Scale for point " << i << ": "
			//	<< scale[0 ] << ", " << scale[1] << ", " << scale[2] << std::endl;
			scale[0] *= 0.9;
			scale[1] *= 0.9;
			scale[2] *= 0.9;
			scaleArray->SetTuple(i, scale);
		}
		polyData->Modified();
		glyph3DMapper->SetScaleArray("Scales");
		glyph3DMapper->Update();
	}
	else if (key == "equal")
	{
		VisualDebugging::SetLineWidth("Spheres", 1);
		vtkSmartPointer<vtkActor> actor = VisualDebugging::GetSphereActor("Spheres");
		vtkSmartPointer<vtkMapper> mapper = actor->GetMapper();
		vtkSmartPointer<vtkPolyDataMapper> polyDataMapper =
			vtkPolyDataMapper::SafeDownCast(mapper);
		vtkSmartPointer<vtkGlyph3DMapper> glyph3DMapper = vtkGlyph3DMapper::SafeDownCast(mapper);
		vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(glyph3DMapper->GetInputDataObject(0, 0));
		vtkSmartPointer<vtkPointData> pointData = polyData->GetPointData();
		vtkSmartPointer<vtkDoubleArray> scaleArray =
			vtkDoubleArray::SafeDownCast(pointData->GetArray("Scales"));
		for (vtkIdType i = 0; i < scaleArray->GetNumberOfTuples(); ++i)
		{
			double scale[3]; // Assuming 3-component scale array (X, Y, Z)
			scaleArray->GetTuple(i, scale);
			//std::cout << "Scale for point " << i << ": "
			//	<< scale[0 ] << ", " << scale[1] << ", " << scale[2] << std::endl;
			scale[0] *= 1.1;
			scale[1] *= 1.1;
			scale[2] *= 1.1;
			scaleArray->SetTuple(i, scale);
		}
		polyData->Modified();
		glyph3DMapper->SetScaleArray("Scales");
		glyph3DMapper->Update();
	}
	else if (key == "1")
	{
		VisualDebugging::ToggleVisibility("Original");
	}
	else if (key == "2")
	{
		VisualDebugging::ToggleVisibility("DownSample");
	}
	else if (key == "3")
	{
		VisualDebugging::ToggleVisibility("Filter");
	}
	else if (key == "4")
	{
		VisualDebugging::ToggleVisibility("OctreeNode");
	}
	else if (key == "5")
	{
		VisualDebugging::ToggleVisibility("WiredBox_4");
	}
	else if (key == "6")
	{
		VisualDebugging::ToggleVisibility("WiredBox_5");
	}
	else if (key == "7")
	{
		VisualDebugging::ToggleVisibility("WiredBox_6");
	}
	else if (key == "8")
	{
		VisualDebugging::ToggleVisibility("WiredBox_7");
	}
	else if (key == "9")
	{
		VisualDebugging::ToggleVisibility("WiredBox_8");
	}
	else if (key == "0")
	{
		VisualDebugging::ToggleVisibility("WiredBox_9");
	}
	else if (key == "Left")
	{
		depthIndex--;
		if (depthIndex < 0) depthIndex = 0;

		for (int i = 0; i < 14; i++)
		{
			stringstream ss;
			ss << "Cubes_" << i;
			VisualDebugging::SetVisibility(ss.str(), false);
		}
		{
			stringstream ss;
			ss << "Cubes_" << depthIndex;
			VisualDebugging::SetVisibility(ss.str(), true);
		}

		printf("Depth : %d\n", depthIndex);
	}
	else if (key == "Right")
	{
		depthIndex++;
		if (depthIndex > 14) depthIndex = 13;

		for (int i = 0; i < 14; i++)
		{
			stringstream ss;
			ss << "Cubes_" << i;
			VisualDebugging::SetVisibility(ss.str(), false);
		}
		{
			stringstream ss;
			ss << "Cubes_" << depthIndex;
			VisualDebugging::SetVisibility(ss.str(), true);
		}

		printf("Depth : %d\n", depthIndex);
	}
	else if (key == "quoteleft")
	{
		VisualDebugging::ToggleVisibility("axes");
	}
	else if (key == "BackSpace")
	{
		auto viewMatrix = vtkToEigen(camera->GetViewTransformMatrix());
		Eigen::Matrix4f inverseViewMatrix = viewMatrix.inverse();

		Eigen::Vector3f viewDirection = -inverseViewMatrix.block<3, 1>(0, 2);
		viewDirection.normalize();
		Eigen::Vector3f cameraPosition = inverseViewMatrix.block<3, 1>(0, 3);

		//VisualDebugging::AddLine("ViewDirection", cameraPosition, cameraPosition + viewDirection * 100, Color4::Red);

		Eigen::Vector3f zero = (inverseViewMatrix * Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f)).head<3>();
		Eigen::Vector3f right = (inverseViewMatrix * Eigen::Vector4f(10.0f, 0.0f, 0.0f, 1.0f)).head<3>();
		Eigen::Vector3f up = (inverseViewMatrix * Eigen::Vector4f(0.0f, 10.0f, 0.0f, 1.0f)).head<3>();
		Eigen::Vector3f front = (inverseViewMatrix * Eigen::Vector4f(0.0f, 0.0f, 10.0f, 1.0f)).head<3>();

		VisualDebugging::AddLine("ViewDirection", zero, right, Color4::Red);
		VisualDebugging::AddLine("ViewDirection", zero, up, Color4::Green);
		VisualDebugging::AddLine("ViewDirection", zero, front, Color4::Blue);
	}
	else if (key == "space")
	{
		app->CaptureColorAndDepth("C:\\Resources\\2D\\Captured\\RGBD");
	}
	else if (key == "Insert")
	{
		printf("Camera Distance : %f\n", camera->GetDistance());
		printf("Camera Parallel Scale : %f\n", camera->GetParallelScale());
	}
	else if (key == "Return")
	{
		auto viewMatrix = vtkToEigen(camera->GetViewTransformMatrix());
		Eigen::Matrix4f inverseViewMatrix = viewMatrix.inverse();

		Eigen::Vector3f viewDirection = -inverseViewMatrix.block<3, 1>(0, 2);
		viewDirection.normalize();
		Eigen::Vector3f cameraPosition = inverseViewMatrix.block<3, 1>(0, 3);

		VisualDebugging::AddLine("ViewDirection", cameraPosition, cameraPosition + viewDirection * 1000, Color4::Red);
	}
}
