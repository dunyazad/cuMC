#include <Common.h>

#include <App/CustomTrackballStyle.h>
#include <App/Utility.h>

#include <Debugging/VisualDebugging.h>

class App;

class TimerCallback : public vtkCommand
{
public:
    static App* s_app;
    static TimerCallback* New();
    static void SetApp(App* app);

    TimerCallback() = default;

    virtual void Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData)) override;

private:
    void animate();
};

class App
{
public:
	App();
	~App();

	void Run(int windowWidth = 1920, int windowHeight = 1080, bool maximizeRenderWindow = true, bool maximizeConsoleWindow = true);

    void AddAppStartCallback(function<void(App*)> f);
    void AddAppStartCallback(const string& name, function<void(App*)> f);
    void RemoveAppStartCallback();
    void RemoveAppStartCallback(const string& name);

    void AddAppUpdateCallback(function<void(App*)> f);
    void AddAppUpdateCallback(const string& name, function<void(App*)> f);
    void RemoveAppUpdateCallback();
    void RemoveAppUpdateCallback(const string& name);

    void AddKeyPressCallback(function<void(App*)> f);
    void AddKeyPressCallback(const string& name, function<void(App*)> f);
    void RemoveKeyPressCallback();
    void RemoveKeyPressCallback(const string& name);

    void OnUpdate();

    void CaptureColorAndDepth(const string& saveDirectory);

    static void OnKeyPress();

    inline vtkSmartPointer<vtkRenderer> GetRenderer() const { return renderer; }
    inline vtkSmartPointer<vtkRenderWindow> GetRenderWindow() const { return renderWindow; }
    inline vtkSmartPointer<vtkRenderWindowInteractor> GetInteractor() const { return interactor; }

private:
    static set<App*> s_instances;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    vtkSmartPointer<CustomTrackballStyle> customTrackballStyle;

    vtkSmartPointer<vtkCallbackCommand> keyPressCallback;

    vtkSmartPointer<TimerCallback> timerCallback;

    map<string, function<void(App*)>> appStartCallbacks;
    map<string, function<void(App*)>> appUpdateCallbacks;
    //vtkObject* caller, long unsigned int eventId, void* clientData, void* callData
    map<string, function<void(App*)>> keyPressCallbacks;
};
