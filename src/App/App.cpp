#include <App/App.h>

set<App*> App::s_instances;

TimerCallback* TimerCallback::New() { return new TimerCallback; }
void TimerCallback::SetApp(App* app) { this->app = app; }

void TimerCallback::Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
{
    if (eventId == vtkCommand::TimerEvent)
    {
        OnTimer();
    }
    else
    {
        std::cerr << "Unexpected event ID: " << eventId << std::endl;
    }
}

void TimerCallback::OnTimer()
{
    app->OnUpdate();

    VisualDebugging::Update();
}

PostRenderCallback* PostRenderCallback::New() { return new PostRenderCallback; }
void PostRenderCallback::SetApp(App* app) { this->app = app; }

void PostRenderCallback::Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
{
    if (eventId == vtkCommand::EndEvent)
    {
        OnPostRender();
    }
    else
    {
        std::cerr << "Unexpected event ID: " << eventId << std::endl;
    }
}

void PostRenderCallback::OnPostRender()
{
    app->OnPostRender();
}


App::App()
{
    s_instances.insert(this);
}

App::~App()
{
    s_instances.erase(this);
}

void App::Run(int windowWidth, int windowHeight, bool maximizeRenderWindow, bool maximizeConsoleWindow)
{
    if (maximizeConsoleWindow)
    {
        MaximizeConsoleWindowOnMonitor(1);
    }

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.3, 0.5, 0.7);

    renderer->GetActiveCamera()->SetClippingRange(0.001, 40.0);

    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(windowWidth, windowHeight);
    renderWindow->AddRenderer(renderer);

    interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    customTrackballStyle = vtkSmartPointer<CustomTrackballStyle>::New();
    interactor->SetInteractorStyle(customTrackballStyle);
    interactor->SetRenderWindow(renderWindow);
    interactor->Initialize();

    VisualDebugging::Initialize(renderer);

    if (maximizeRenderWindow)
    {
        MaximizeVTKWindowOnMonitor(renderWindow, 2);
    }

    timerCallback = vtkSmartPointer<TimerCallback>::New();
    timerCallback->SetApp(this);

    interactor->AddObserver(vtkCommand::TimerEvent, timerCallback);
    int timerId = interactor->CreateRepeatingTimer(16);
    if (timerId < 0) {
        std::cerr << "Error: Timer was not created!" << std::endl;
    }

    postRenderCallback = vtkSmartPointer<PostRenderCallback>::New();
    postRenderCallback->SetApp(this);

    renderWindow->AddObserver(vtkCommand::EndEvent, postRenderCallback);
    
    for (auto& kvp : appStartCallbacks)
    {
        kvp.second(this);
    }

    renderWindow->Render();
    interactor->Start();

    VisualDebugging::Terminate();
}

void App::AddAppStartCallback(function<void(App*)> f)
{
    AddAppStartCallback("Default", f);
}

void App::AddAppStartCallback(const string& name, function<void(App*)> f)
{
    if (0 != appStartCallbacks.count(name))
    {
        printf("[Error] same name callback exists!");
    }
    appStartCallbacks[name] = f;
}

void App::RemoveAppStartCallback()
{
    RemoveAppStartCallback("Default");
}

void App::RemoveAppStartCallback(const string& name)
{
    if (0 != appStartCallbacks.count(name))
    {
        appStartCallbacks.erase(name);
    }
}

void App::AddAppUpdateCallback(function<void(App*)> f)
{
    AddAppUpdateCallback("Default", f);
}

void App::AddAppUpdateCallback(const string& name, function<void(App*)> f)
{
    if (0 != appUpdateCallbacks.count(name))
    {
        printf("[Error] same name callback exists!");
    }
    appUpdateCallbacks[name] = f;
}

void App::RemoveAppUpdateCallback()
{
    RemoveAppUpdateCallback("Default");
}

void App::RemoveAppUpdateCallback(const string& name)
{
    if (0 != appUpdateCallbacks.count(name))
    {
        appUpdateCallbacks.erase(name);
    }
}

void App::AddAppPostRenderCallback(function<void(App*)> f)
{
    AddAppPostRenderCallback("Default", f);
}

void App::AddAppPostRenderCallback(const string& name, function<void(App*)> f)
{
    if (0 != appPostRenderCallbacks.count(name))
    {
        printf("[Error] same name callback exists!");
    }
    appPostRenderCallbacks[name] = f;
}

void App::RemoveAppPostRenderCallback()
{
    RemoveAppPostRenderCallback("Default");
}

void App::RemoveAppPostRenderCallback(const string& name)
{
    if (0 != appPostRenderCallbacks.count(name))
    {
        appPostRenderCallbacks.erase(name);
    }
}

void App::AddKeyPressCallback(function<void(App*)> f)
{
    AddKeyPressCallback("Default", f);
}

void App::AddKeyPressCallback(const string& name, function<void(App*)> f)
{
    if (0 != keyPressCallbacks.count(name))
    {
        printf("[Error] same name callback exists!");
    }
	keyPressCallbacks[name] = f;
}

void App::RemoveKeyPressCallback()
{
    RemoveKeyPressCallback("Default");
}

void App::RemoveKeyPressCallback(const string& name)
{
	if (0 != keyPressCallbacks.count(name))
	{
		keyPressCallbacks.erase(name);
	}
}

void App::OnUpdate()
{
    for (auto& kvp : appUpdateCallbacks)
    {
        kvp.second(this);
    }
}

void App::OnPostRender()
{
    for (auto& kvp : appPostRenderCallbacks)
    {
        kvp.second(this);
    }
}

void App::OnKeyPress()
{
	for (auto& instance : s_instances)
	{
		for (auto& kvp : instance->keyPressCallbacks)
		{
			kvp.second(instance);
		}
	}
}

void App::CaptureColorAndDepth(const string& saveDirectory)
{
    static int captureCount = 0;
    std::stringstream ss;
    ss << captureCount++;

    std::string depthDataFileName = saveDirectory + "\\depth_data_" + ss.str() + ".dpt";
    std::string depthmapFileName = saveDirectory + "\\depth_" + ss.str() + ".png";
    std::string colormapFileName = saveDirectory + "\\color_" + ss.str() + ".png";

    {
        vtkNew<vtkWindowToImageFilter> colorFilter;
        colorFilter->SetInput(renderWindow);
        colorFilter->SetInputBufferTypeToRGB();
        colorFilter->Update();

        vtkNew<vtkPNGWriter> colorWriter;
        colorWriter->SetFileName(colormapFileName.c_str());
        colorWriter->SetInputConnection(colorFilter->GetOutputPort());
        colorWriter->Write();
    }

    {
        vtkNew<vtkWindowToImageFilter> depthFilter;
        depthFilter->SetInput(renderWindow);
        depthFilter->SetInputBufferTypeToZBuffer();
        depthFilter->Update();

        vtkNew<vtkImageShiftScale> shiftScale;
        shiftScale->SetInputConnection(depthFilter->GetOutputPort());
        shiftScale->SetOutputScalarTypeToUnsignedChar();
        shiftScale->SetShift(0);
        shiftScale->SetScale(255);
        shiftScale->Update();

        vtkNew<vtkPNGWriter> depthWriter;
        depthWriter->SetFileName(depthmapFileName.c_str());
        depthWriter->SetInputConnection(shiftScale->GetOutputPort());
        depthWriter->Write();
    }

    {
        ofstream ofs;
        ofs.open(depthDataFileName, ios::out | ios::binary);

        int width = 256;
        int height = 480;
        ofs.write((char*)&width, sizeof(int));
        ofs.write((char*)&height, sizeof(int));

        Eigen::Matrix4f viewMatrix = vtkToEigen(renderer->GetActiveCamera()->GetViewTransformMatrix());
        Eigen::Matrix4f tm = viewMatrix.inverse();
        //auto& tm = cameraTransforms[0];

        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInput(GetRenderWindow());
        windowToImageFilter->SetInputBufferTypeToZBuffer(); // Get the depth buffer
        windowToImageFilter->Update();

        // Access the depth buffer as an image
        vtkSmartPointer<vtkImageData> depthImage = windowToImageFilter->GetOutput();

        // Get the depth values as a float array
        vtkSmartPointer<vtkFloatArray> depthArray = vtkFloatArray::SafeDownCast(depthImage->GetPointData()->GetScalars());

        double* clippingRange = renderer->GetActiveCamera()->GetClippingRange();
        float depthRatio = (float)(clippingRange[1] - clippingRange[0]);

        vtkIdType index = 0;
        for (float y = -24.0f; y < 24.0f; y += 0.1f)
        {
            for (float x = -12.8f; x < 12.8f; x += 0.1f)
            {
                float depth = depthArray->GetValue(index++);
                auto p = Transform(tm, { x, y, -depth * depthRatio });
                ofs.write((char*)p.data(), sizeof(Eigen::Vector3f));
            }
        }

        ofs.close();
    }
}
