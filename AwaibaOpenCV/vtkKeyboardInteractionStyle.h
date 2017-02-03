#pragma once
// System includes
#include <windows.h> // Sleep
#include <stdint.h>
#include <stdlib.h>

#include <iostream>

 #include <vtkAutoInit.h>
#include <vtkObject.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkObjectFactory.h>
#include <vtkRenderingCoreModule.h>
#include <vtkProperty.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
	
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);
 
    virtual void OnKeyPress() 
    {
      // Get the keypress
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();
	  
	  // set the camera to desired values -> this is just for demonstration
	  if (key == "b")
	  {
		  vtkCamera* camera = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
 		  camera->Yaw(-20);
		  camera->Pitch(20);
		  ::std::cout << "change camera view" << ::std::endl;
	  }
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
 
};
vtkStandardNewMacro(KeyPressInteractorStyle);