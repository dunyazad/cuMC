#include <App/App.h>

App* TimerCallback::s_app = nullptr;

set<App*> App:: s_instances;

TimerCallback* TimerCallback::New() { return new TimerCallback; }
void TimerCallback::SetApp(App* app) { s_app = app; }

void TimerCallback::Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
{
    if (eventId == vtkCommand::TimerEvent)
    {
        animate();
    }
    else
    {
        std::cerr << "Unexpected event ID: " << eventId << std::endl;
    }
}

void TimerCallback::animate()
{
    s_app->OnUpdate();

    VisualDebugging::Update();
}

App::App()
{
    s_instances.insert(this);
}

App::~App()
{
    s_instances.erase(this);
}

void App::Run()
{
    MaximizeConsoleWindowOnMonitor(1);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.3, 0.5, 0.7);

    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(1920, 1080);
    //renderWindow->SetSize(256, 480);
    renderWindow->AddRenderer(renderer);

    interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    customTrackballStyle = vtkSmartPointer<CustomTrackballStyle>::New();
    interactor->SetInteractorStyle(customTrackballStyle);
    interactor->SetRenderWindow(renderWindow);
    interactor->Initialize();

    VisualDebugging::Initialize(renderer);

    MaximizeVTKWindowOnMonitor(renderWindow, 2);

    keyPressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    keyPressCallback->SetCallback(OnKeyPress);
    keyPressCallback->SetClientData(renderer);

    timerCallback = vtkSmartPointer<TimerCallback>::New();
    timerCallback->SetApp(this);

    interactor->AddObserver(vtkCommand::TimerEvent, timerCallback);
    int timerId = interactor->CreateRepeatingTimer(16);
    if (timerId < 0) {
        std::cerr << "Error: Timer was not created!" << std::endl;
    }

    interactor->AddObserver(vtkCommand::KeyPressEvent, keyPressCallback);

    //vtkCamera* camera = renderer->GetActiveCamera();
    //camera->SetParallelProjection(true);
    //renderer->ResetCamera();

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

void App::AddKeyPressCallback(function<void(vtkObject*, long unsigned int, void*, void*)> f)
{
    AddKeyPressCallback("Default", f);
}

void App::AddKeyPressCallback(const string& name, function<void(vtkObject*, long unsigned int, void*, void*)> f)
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

void App::OnKeyPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	for (auto& instance : s_instances)
	{
		for (auto& kvp : instance->keyPressCallbacks)
		{
			kvp.second(caller, eventId, clientData, callData);
		}
	}
}