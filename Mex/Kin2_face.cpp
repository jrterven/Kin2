#include "Kin2.h"
#include "mex.h"
#include "class_handle.hpp"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <Kinect.Face.h>
#include <vector>

/***********************************************************************/
/********************  Face Processing functions *************************/
/***********************************************************************/
void Kin2::getFaces(std::vector<k2::FaceData>& facesData)
{
    if (!(m_flags & k2::FACE))
    {
        mexPrintf("ERROR: NO FACE FUNCTIONALITY SELECTED!\n");
        return;
    }
        
	HRESULT hr;
	facesData.clear();

	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		if (SUCCEEDED(hr))
		{
			// If face tracked, save its data on the facesData structure array
			if (bFaceTracked)
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				k2::FaceData faceData;

				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
				{
					hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceData.faceBox);

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, faceData.facePoints);
					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceData.faceRotation);
					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceData.faceProperties);
					}

					facesData.push_back(faceData);
				}

				SafeRelease(pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (m_bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = m_ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}

		SafeRelease(pFaceFrame);
	}
}


void Kin2::getHDFaces(bool withVertices, std::vector<k2::HDFaceData>& facesData)
{
    if (!(m_flags & k2::HD_FACE))
	{
        mexPrintf("ERROR: NO HD-FACE FUNCTIONALITY SELECTED!\n");
        return;
    }
        
	HRESULT hr;
	facesData.clear();

	// iterate through each HD face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IHighDefinitionFaceFrame *pHDFaceFrame = nullptr;
		
		hr = m_pHDFaceFrameReaders[iFace]->AcquireLatestFrame(&pHDFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pHDFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pHDFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		// If face tracked, save its data on the facesData structure array
		if (bFaceTracked)
		{		
            float animationUnits[FaceShapeAnimations_Count]={0};					
			UINT32 vertexCount;
            
            // Here we save the HD face data
			k2::HDFaceData faceData;
            
			hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment[iFace]);

            if (SUCCEEDED(hr) && m_pFaceAlignment[iFace] != nullptr)
			{	
                // Get the Animation units
                hr = m_pFaceAlignment[iFace]->GetAnimationUnits(FaceShapeAnimations_Count, animationUnits);

                if (SUCCEEDED(hr))
                {
                    for (int vi = 0; vi < FaceShapeAnimations_Count; vi++)
                        faceData.animationUnits[vi] = animationUnits[vi];
                }

                // If HD face model vertices are requested
				if (withVertices)
				{
                    hr = GetFaceModelVertexCount(&vertexCount);
                    //mexPrintf("Number of Vertices: %d", vertexCount);

					// If there is no model ready, issue a warning message (just once)
					if (!m_faceModelReady[iFace] && !m_faceModelWarning[iFace])
					{
						mexPrintf("WARNING: No personal model has been created. An average face model will be used\n");
						m_faceModelWarning[iFace] = true;
					}
                    
                    CameraSpacePoint *vertices = new CameraSpacePoint[vertexCount];

					// Get the vertices (HD points)
					if (SUCCEEDED(hr))
						hr = m_pFaceModel[iFace]->CalculateVerticesForAlignment(m_pFaceAlignment[iFace], vertexCount, vertices);

					if (SUCCEEDED(hr))
                    {
						faceData.faceModel.resize(vertexCount);

						for (int vi = 0; vi < vertexCount; vi++)
							faceData.faceModel[vi] = vertices[vi];
					}

					if (vertices)
					{
						delete[] vertices;
						vertices = NULL;
					}
                } // if withVertices	
				
                // Get the facebox
                if (SUCCEEDED(hr))
                    hr = m_pFaceAlignment[iFace]->get_FaceBoundingBox(&faceData.faceBox);

                // Get the face rotation
                if (SUCCEEDED(hr))
                    hr = m_pFaceAlignment[iFace]->get_FaceOrientation(&faceData.faceRotation);

                // Get the head pivot
                if (SUCCEEDED(hr))
                {
                    hr = m_pFaceAlignment[iFace]->get_HeadPivotPoint(&faceData.headPivot);
                }

                // Save the HD face data in the member variable m_HDfacesData
                facesData.push_back(faceData);			
            }  // if face alignment	
        } // If face tracked
		else
		{
			// face tracking is not valid - attempt to fix the issue
			// a valid body is required to perform this step
			if (m_bHaveBodyData)
			{
				// check if the corresponding body is tracked 
				// if this is true then update the face frame source to track this body
				IBody* pBody = m_ppBodies[iFace];
				if (pBody != nullptr)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					UINT64 bodyTId;
					if (SUCCEEDED(hr) && bTracked)
					{
						// get the tracking ID of this body
						hr = pBody->get_TrackingId(&bodyTId);
						if (SUCCEEDED(hr))
						{
							// update the face frame source with the tracking ID
							m_pHDFaceFrameSources[iFace]->put_TrackingId(bodyTId);
						}
					}
				} // if (pBody != nullptr)
			} // if (m_bHaveBodyData)
		} // if face tracked

		SafeRelease(pHDFaceFrame);		
	} // for each face reader
} // end getHDFaces function

void Kin2::buildHDFaceModels(int &collectionStatus, int &captureStatus)
{
    collectionStatus = -1;
    captureStatus = -1;
    
	if (!(m_flags & k2::HD_FACE))
	{
        mexPrintf("ERROR: NO HD-FACE FUNCTIONALITY SELECTED!\n");
        return;
    }
    
	HRESULT hr;

	// iterate through each HD face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IHighDefinitionFaceFrame *pHDFaceFrame = nullptr;

		hr = m_pHDFaceFrameReaders[iFace]->AcquireLatestFrame(&pHDFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pHDFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pHDFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		// If face tracked, try to align it
		if (SUCCEEDED(hr) && bFaceTracked)
		{
			IFaceModel *pFaceModel = nullptr;

			hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment[iFace]);

			// If face aligned, continue building the model
			if (SUCCEEDED(hr) && m_pFaceAlignment[iFace] != nullptr)
			{
				// If face model not ready
				if (!m_faceModelReady[iFace])
				{
					FaceModelBuilderCollectionStatus collection;
					hr = m_pFaceModelBuilder[iFace]->get_CollectionStatus(&collection);
                    collectionStatus = (int)collection;

					// If model completed
					if (collection == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete)
					{
						mexPrintf("Face Model Completed!\n");						

						IFaceModelData* pFaceModelData = nullptr;
						hr = m_pFaceModelBuilder[iFace]->GetFaceData(&pFaceModelData);

						// Produce the model
						if (SUCCEEDED(hr) && pFaceModelData != nullptr)
						{
                            mexPrintf("Producing model...\n");
							hr = pFaceModelData->ProduceFaceModel(&m_pFaceModel[iFace]);
                            mexPrintf("Model Ready!\n");

							// Set the model ready flag
							if (SUCCEEDED(hr) && m_pFaceModel[iFace] != nullptr)
							{
								m_faceModelReady[iFace] = true;
							}
						}
						SafeRelease(pFaceModelData);

						// Get the shape units (SU) i.e. the deformations wrt the base face model
                        /*
						if (SUCCEEDED(hr))
						{
							float deformations[FaceShapeDeformations_Count];
							hr = m_pFaceModel[iFace]->GetFaceShapeDeformations(FaceShapeDeformations_Count, deformations);										
						}
                        */
					}
					// if model not completed yet
					else
					{
						// Display Collection Status
                        /*
						if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded)
						{
							mexPrintf("Need : Tilted Up Views\n");							
						}


						else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded)
						{
							mexPrintf("Need : Right Views\n");							
						}

						else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded)
						{
							mexPrintf("Need : Left Views\n");							
						}

						else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded)
						{
							mexPrintf("Need : Front ViewFrames\n");							
						}
                        */ 

						// Display Capture Status
						FaceModelBuilderCaptureStatus capture;
						hr = m_pFaceModelBuilder[iFace]->get_CaptureStatus(&capture);

                        captureStatus = (int)capture;
                        
                        /*
						switch (capture)
						{
						case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_OtherViewsNeeded:
							std::cout << "Other views needed" << std::endl;
							break;
						case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
							std::cout << "Face Too Far from Camera" << std::endl;
							break;
						case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
							std::cout << "Face Too Near to Camera" << std::endl;
							break;
						case FaceModelBuilderCaptureStatus_MovingTooFast:
							std::cout << "Moving Too Fast" << std::endl;
							break;
						case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_LostFaceTrack:
							std::cout << "Lost Face Track" << std::endl;
							break;
						case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_SystemError:
							std::cout << "ERROR: System Error" << std::endl;
							break;

						default:
							break;
						}
                         */
					} // collection not complete
				} // If face model not ready
			} // If face aligned
		} // If face tracked
		else
		{
			// face tracking is not valid - attempt to fix the issue
			// a valid body is required to perform this step
			if (m_bHaveBodyData)
			{
				// check if the corresponding body is tracked 
				// if this is true then update the face frame source to track this body
				IBody* pBody = m_ppBodies[iFace];
				if (pBody != nullptr)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					UINT64 bodyTId;
					if (SUCCEEDED(hr) && bTracked)
					{
						// get the tracking ID of this body
						hr = pBody->get_TrackingId(&bodyTId);
						if (SUCCEEDED(hr))
						{
							// update the face frame source with the tracking ID
							m_pHDFaceFrameSources[iFace]->put_TrackingId(bodyTId);
						}
					}
				} // if (pBody != nullptr)
			} // if (m_bHaveBodyData)
		} // if face tracked
	}// for each face reader

} // end buildHDFaceModels
