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
#include <vtkTransform.h>


void SetCameraPositionOrientation( vtkCamera* cam, double position[3], double orientation[3] );

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
		  this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
		  vtkCamera* camera = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();

		  camera->SetPosition(0, -90, 0);
		  camera->SetViewUp(1, 0, 0);
		  camera->Yaw(-45);
		  this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
		  ::std::cout << "change camera view" << ::std::endl;
	  }
	  else if (key == "r")
	  {
			this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
			vtkCamera* camera = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();

			camera->SetPosition(-30, 0, -30);
			camera->SetViewUp(1, 0, 0);
			camera->Yaw(0);
			camera->Pitch(-30);
			this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
			::std::cout << "change camera view" << ::std::endl;
	  }

	  vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
 
};
vtkStandardNewMacro(KeyPressInteractorStyle);

void SetCameraPositionOrientation( vtkCamera* cam, double position[3], double orientation[3] )
{
    double focus[3];
    double viewup[3];

    if( cam == NULL )
        return;

    focus[0] = position[0] - -cos(orientation[0])*sin(orientation[1]);
    focus[1] = position[1] - sin(orientation[0]);
    focus[2] = position[2] - cos(orientation[0])*cos(orientation[1]);

    viewup[0] = cos(orientation[1])*sin(orientation[2])+
                sin(orientation[1])*sin(orientation[2])*cos(orientation[2]);
    viewup[1] = cos(orientation[0])*cos(orientation[2]);
    viewup[2] = sin(orientation[1])*sin(orientation[2])-
                cos(orientation[1])*sin(orientation[0])*cos(orientation[2]);

    //set the camera position and orientation
    cam->SetPosition( position );
    cam->SetViewUp( viewup );
    cam->SetFocalPoint( focus );

}


