#include <App/App.h>
#include <App/AppEventHandlers.h>
#include <Debugging/VisualDebugging.h>

int depthIndex = 0;

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
	}
}

void OnMouseButtonPress(App* app, int button)
{
}

void OnMouseButtonRelease(App* app, int button)
{
	vtkRenderWindowInteractor* interactor = app->GetInteractor();
	vtkRenderWindow* renderWindow = interactor->GetRenderWindow();
	vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
	vtkCamera* camera = renderer->GetActiveCamera();

	int* mousePos = interactor->GetEventPosition();
	int mouseX = mousePos[0];
	int mouseY = mousePos[1];

	int* size = renderWindow->GetSize();
	int screenWidth = size[0];
	int screenHeight = size[1];

	float depth = 0.5f;

	printf("mouseX : %d, mouseY : %d\n", mouseX, mouseY);

	if (0 == button)
	{
		float ndcX = (2.0f * mouseX) / screenWidth - 1.0f;
		float ndcY = (2.0f * mouseY) / screenHeight - 1.0f;

		Eigen::Vector4f clipSpacePoint(ndcX, ndcY, depth, 1.0f);

		auto viewMatrix = vtkToEigen(camera->GetViewTransformMatrix());
		auto projectionMatrix = vtkToEigen(camera->GetProjectionTransformMatrix((float)screenWidth / (float)screenHeight, -1, 1));

		Eigen::Matrix4f viewProjectionMatrix = projectionMatrix * viewMatrix;
		Eigen::Matrix4f inverseVPMatrix = viewProjectionMatrix.inverse();
		Eigen::Vector4f worldSpacePoint4 = inverseVPMatrix * clipSpacePoint;
		if (worldSpacePoint4.w() != 0.0f) {
			worldSpacePoint4 /= worldSpacePoint4.w();
		}
		Eigen::Vector3f worldSpacePoint = worldSpacePoint4.head<3>();

		Eigen::Matrix4f inverseViewMatrix = viewMatrix.inverse();

		/*Eigen::Vector3f viewDirection = -inverseViewMatrix.block<3, 1>(0, 2);
		viewDirection.normalize();*/

		Eigen::Vector3f cameraPosition = inverseViewMatrix.block<3, 1>(0, 3);

		Eigen::Vector3f rayDirection = worldSpacePoint - cameraPosition;
		rayDirection.normalize();

		VisualDebugging::AddLine("ViewDirection", cameraPosition, cameraPosition + rayDirection * 1000, Color4::Red);
	}
}