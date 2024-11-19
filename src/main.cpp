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

void AppStartCallback_Capture(App* pApp);

int main()
{
	vtkActor* planeActor = nullptr;

	App app;
	app.AddAppStartCallback(AppStartCallback_Capture);
	app.AddKeyPressCallback(OnKeyPress);
	app.Run();

	return 0;
}
