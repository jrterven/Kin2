#include "Kin2.h"
#include "mex.h"
#include "class_handle.hpp"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <Kinect.Face.h>
#include <vector>

/***********************************************************************/
/********************  Kinect Fusion functions *************************/
/***********************************************************************/
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

void Kin2::KF_init(int voxelsPerMeter, int voxelsX, int voxelsY, int voxelsZ, bool processorType, bool autoReset)
{
	// Check if depth source was activated
	if (!(m_flags & k2::DEPTH))
	{
		mexPrintf("ERROR Initializing Kinec Fusion. No depth source activated.\n");
		mexPrintf("Select depth when creating Kin2 object.\n");
		return;
	}
    
    /*
    mexPrintf("voxelsPerMeter: %d\n",voxelsPerMeter);
    mexPrintf("voxelsX: %d\n",voxelsX);
    mexPrintf("voxelsY: %d\n",voxelsY);
    mexPrintf("voxelsZ: %d\n",voxelsZ);
    mexPrintf("processorType: %d\n",processorType);
    */

	m_bInitializeError = false;
	m_pVolume = nullptr;
	m_bMirrorDepthFrame = false;
	m_bTranslateResetPoseByMinDepthThreshold = true;
	m_bAutoResetReconstructionWhenLost = autoReset;
	m_bResetReconstruction = false;
	m_cLostFrameCounter = 0;
	m_bTrackingFailed = false;
	m_cFrameCounter = 0;
	m_pDepthImagePixelBuffer = nullptr;
	m_pDepthDistortionMap = nullptr;
	m_pDepthDistortionLT = nullptr;
	m_pDepthFloatImage = nullptr;
	m_pPointCloud = nullptr;
	m_pShadedSurface = nullptr;
	m_bInitializeError = false;
	m_bHaveValidCameraParameters = false;
	m_cDepthImagePixels = cDepthWidth * cDepthHeight;

	// Define a cubic Kinect Fusion reconstruction volume,
	// with the Kinect at the center of the front face and the volume directly in front of Kinect.
	m_reconstructionParams.voxelsPerMeter = voxelsPerMeter;// 1000mm / 256vpm = ~3.9mm/voxel    
	m_reconstructionParams.voxelCountX = voxelsX;   // 384 / 256vpm = 1.5m wide reconstruction
	m_reconstructionParams.voxelCountY = voxelsY;   // Memory = 384*384*384 * 4bytes per voxel
	m_reconstructionParams.voxelCountZ = voxelsZ;   // This will require a GPU with at least 256MB

	// These parameters are for optionally clipping the input depth image 
	m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
	m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

	// This parameter is the temporal averaging parameter for depth integration into the reconstruction
	m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes

	// This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
	// too slow for real-time processing.
	if (processorType)
        m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;
    else
		m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU;			

	// If GPU processing is selected, we can choose the index of the device we would like to
	// use for processing by setting this zero-based index parameter. Note that setting -1 will cause
	// automatic selection of the most suitable device (specifically the DirectX11 compatible device 
	// with largest memory), which is useful in systems with multiple GPUs when only one reconstruction
	// volume is required. Note that the automatic choice will not load balance across multiple 
	// GPUs, hence users should manually select GPU indices when multiple reconstruction volumes 
	// are required, each on a separate device.
	m_deviceIndex = -1;    // automatically choose device index for processing

	SetIdentityMatrix(m_worldToCameraTransform);
	SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	// We don't know these at object creation time, so we use nominal values.
	// These will later be updated in using the coordinate mapping.
	m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
	m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
	m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
	m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

	// Look for a connected Kinect, and create it if found
	HRESULT hr;
	hr = m_pCoordinateMapper->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);

	if (SUCCEEDED(hr))
	{
		hr = KF_InitializeKinectFusion();

		if (FAILED(hr))
		{
			m_bInitializeError = true;
		}
	}
	else
		m_bInitializeError = true;
}// end KF_init

void Kin2::KF_update()
{
	if (nullptr == m_pKinectSensor)
	{
		mexPrintf("KF: No Kinect sensor detected\n");
		return;
	}

	if (m_coordinateMappingChangedEvent != NULL &&
		WAIT_OBJECT_0 == WaitForSingleObject((HANDLE)m_coordinateMappingChangedEvent, 0))
	{
		OnCoordinateMappingChanged();
		ResetEvent((HANDLE)m_coordinateMappingChangedEvent);
	}

	if (!m_bHaveValidCameraParameters)
	{
		mexPrintf("KF: No valid camera parameters\n");
		return;
	}

	m_bResetReconstruction = false;

	if (!m_pMultiSourceFrameReader)
	{
		mexPrintf("KF: No depth frame reader\n");
		return;
	}

	if (m_pDepthArray16U)
	{        
		//copy and remap depth
        
		const UINT bufferLength = m_cDepthImagePixels;
		UINT16 * pDepth = m_pDepthImagePixelBuffer;
		for (UINT i = 0; i < bufferLength; i++, pDepth++)
		{
			const UINT id = m_pDepthDistortionLT[i];
			*pDepth = id < bufferLength ? m_pDepthArray16U[id] : 0;
		}
        
		KF_ProcessDepth();
	}
} // end KF_update


void UpdateIntrinsics(NUI_FUSION_IMAGE_FRAME * pImageFrame, NUI_FUSION_CAMERA_PARAMETERS * params)
{
	if (pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr)
	{
		pImageFrame->pCameraParameters->focalLengthX = params->focalLengthX;
		pImageFrame->pCameraParameters->focalLengthY = params->focalLengthY;
		pImageFrame->pCameraParameters->principalPointX = params->principalPointX;
		pImageFrame->pCameraParameters->principalPointY = params->principalPointY;
	}

	// Confirm we are called correctly
	_ASSERT(pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr);
} // end UpdateIntrinsics

HRESULT Kin2::KF_SetupUndistortion()
{
	HRESULT hr = E_UNEXPECTED;

	if (m_cameraParameters.principalPointX != 0)
	{

		CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
		{
			/*LT*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
			/*RT*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
			/*LB*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f },
			/*RB*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }
		};

		for (UINT rowID = 0; rowID < cDepthHeight; rowID++)
		{
			const float rowFactor = float(rowID) / float(cDepthHeight - 1);
			const CameraSpacePoint rowStart =
			{
				cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
				cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
				1.f
			};

			const CameraSpacePoint rowEnd =
			{
				cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
				cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
				1.f
			};

			const float stepFactor = 1.f / float(cDepthWidth - 1);
			const CameraSpacePoint rowDelta =
			{
				(rowEnd.X - rowStart.X) * stepFactor,
				(rowEnd.Y - rowStart.Y) * stepFactor,
				0
			};

			_ASSERT(cDepthWidth == NUI_DEPTH_RAW_WIDTH);
			CameraSpacePoint cameraCoordsRow[NUI_DEPTH_RAW_WIDTH];

			CameraSpacePoint currentPoint = rowStart;
			for (UINT i = 0; i < cDepthWidth; i++)
			{
				cameraCoordsRow[i] = currentPoint;
				currentPoint.X += rowDelta.X;
				currentPoint.Y += rowDelta.Y;
			}

			hr = m_pCoordinateMapper->MapCameraPointsToDepthSpace(cDepthWidth, cameraCoordsRow, cDepthWidth, &m_pDepthDistortionMap[rowID * cDepthWidth]);
			if (FAILED(hr))
			{
				mexPrintf("KF: Failed to initialize Kinect Coordinate Mapper.\n");
				return hr;
			}
		}

		if (nullptr == m_pDepthDistortionLT)
		{
			mexPrintf("KF:Failed to initialize Kinect Fusion depth image distortion Lookup Table.\n");
			return E_OUTOFMEMORY;
		}

		UINT* pLT = m_pDepthDistortionLT;
		for (UINT i = 0; i < m_cDepthImagePixels; i++, pLT++)
		{
			//nearest neighbor depth lookup table 
			UINT x = UINT(m_pDepthDistortionMap[i].X + 0.5f);
			UINT y = UINT(m_pDepthDistortionMap[i].Y + 0.5f);

			*pLT = (x < cDepthWidth && y < cDepthHeight) ? x + y * cDepthWidth : UINT_MAX;
		}
		m_bHaveValidCameraParameters = true;
	}
	else
	{
		m_bHaveValidCameraParameters = false;
	}
    return S_OK;
} // end KF_SetupUndistortion


HRESULT Kin2::OnCoordinateMappingChanged()
{
	HRESULT hr = E_UNEXPECTED;

	// Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
	CameraIntrinsics intrinsics = {};

	m_pCoordinateMapper->GetDepthCameraIntrinsics(&intrinsics);	

	float focalLengthX = intrinsics.FocalLengthX / NUI_DEPTH_RAW_WIDTH;
	float focalLengthY = intrinsics.FocalLengthY / NUI_DEPTH_RAW_HEIGHT;
	float principalPointX = intrinsics.PrincipalPointX / NUI_DEPTH_RAW_WIDTH;
	float principalPointY = intrinsics.PrincipalPointY / NUI_DEPTH_RAW_HEIGHT;

	if (m_cameraParameters.focalLengthX == focalLengthX && m_cameraParameters.focalLengthY == focalLengthY &&
		m_cameraParameters.principalPointX == principalPointX && m_cameraParameters.principalPointY == principalPointY)
		return S_OK;

	m_cameraParameters.focalLengthX = focalLengthX;
	m_cameraParameters.focalLengthY = focalLengthY;
	m_cameraParameters.principalPointX = principalPointX;
	m_cameraParameters.principalPointY = principalPointY;

	_ASSERT(m_cameraParameters.focalLengthX != 0);

	UpdateIntrinsics(m_pDepthFloatImage, &m_cameraParameters);
	UpdateIntrinsics(m_pPointCloud, &m_cameraParameters);
	UpdateIntrinsics(m_pShadedSurface, &m_cameraParameters);

	if (nullptr == m_pDepthDistortionMap)
	{
		mexPrintf("Failed to initialize Kinect Fusion depth image distortion buffer.\n");
		return E_OUTOFMEMORY;
	}

	hr = KF_SetupUndistortion();
	return hr;
} // end OnCoordinateMappingChanged

// Initialize Kinect Fusion volume and images for processing
// <returns>S_OK on success, otherwise failure code</returns>
HRESULT Kin2::KF_InitializeKinectFusion()
{
	HRESULT hr = S_OK;

	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
	WCHAR description[MAX_PATH];
	WCHAR instancePath[MAX_PATH];
	UINT memorySize = 0;

	if (FAILED(hr = NuiFusionGetDeviceInfo(
		m_processorType,
		m_deviceIndex,
		&description[0],
		ARRAYSIZE(description),
		&instancePath[0],
		ARRAYSIZE(instancePath),
		&memorySize)))
	{
		if (hr == E_NUI_BADINDEX)
		{
			// This error code is returned either when the device index is out of range for the processor 
			// type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
			// for the device index in the parameters, this indicates that there is no DirectX11 capable 
			// device. The options for users in this case are to either install a DirectX11 capable device
			// (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
			// reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
			mexPrintf("No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction.\n");
		}
		else
		{
			mexPrintf("Failed in call to NuiFusionGetDeviceInfo.\n");
		}
		return hr;
	}

	// Create the Kinect Fusion Reconstruction Volume
	hr = NuiFusionCreateReconstruction(
		&m_reconstructionParams,
		m_processorType, m_deviceIndex,
		&m_worldToCameraTransform,
		&m_pVolume);

	if (FAILED(hr))
	{
		if (E_NUI_GPU_FAIL == hr)
		{			
			mexPrintf("Device %d not able to run Kinect Fusion, or error initializing.\n",m_deviceIndex);			
		}
		else if (E_NUI_GPU_OUTOFMEMORY == hr)
		{
			mexPrintf("Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.\n",m_deviceIndex);
		}
		else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
		{
			mexPrintf("Failed to initialize Kinect Fusion reconstruction volume on device %d\n",m_deviceIndex);
		}
		else
		{
			mexPrintf("Failed to initialize Kinect Fusion reconstruction volume on CPU.\n");;
		}

		return hr;
	}

	// Save the default world to volume transformation to be optionally used in ResetReconstruction
	hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
	if (FAILED(hr))
	{
		mexPrintf("Failed in call to GetCurrentWorldToVolumeTransform.\n");
		return hr;
	}

	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		// This call will set the world-volume transformation
		KF_reset();
	}

	// Frames generated from the depth input
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, cDepthWidth, cDepthHeight, &m_cameraParameters, &m_pDepthFloatImage);
	if (FAILED(hr))
	{
		mexPrintf("Failed to initialize Kinect Fusion image.\n");
		return hr;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, cDepthWidth, cDepthHeight, &m_cameraParameters, &m_pPointCloud);
	if (FAILED(hr))
	{
		mexPrintf("Failed to initialize Kinect Fusion image.\n");
		return hr;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, cDepthWidth, cDepthHeight, &m_cameraParameters, &m_pShadedSurface);
	if (FAILED(hr))
	{
		mexPrintf("Failed to initialize Kinect Fusion image.\n");
		return hr;
	}

	_ASSERT(m_pDepthImagePixelBuffer == nullptr);
	m_pDepthImagePixelBuffer = new(std::nothrow) UINT16[m_cDepthImagePixels];
	if (nullptr == m_pDepthImagePixelBuffer)
	{
		mexPrintf("Failed to initialize Kinect Fusion depth image pixel buffer.\n");
		return hr;
	}

	_ASSERT(m_pDepthDistortionMap == nullptr);
	m_pDepthDistortionMap = new(std::nothrow) DepthSpacePoint[m_cDepthImagePixels];
	if (nullptr == m_pDepthDistortionMap)
	{
		mexPrintf("Failed to initialize Kinect Fusion depth image distortion buffer.\n");
		return E_OUTOFMEMORY;
	}

	SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
	m_pDepthDistortionLT = new(std::nothrow) UINT[m_cDepthImagePixels];

	if (nullptr == m_pDepthDistortionLT)
	{
		mexPrintf("Failed to initialize Kinect Fusion depth image distortion Lookup Table.\n");
		return E_OUTOFMEMORY;
	}

	// If we have valid parameters, let's go ahead and use them.
	if (m_cameraParameters.focalLengthX != 0)
	{
		KF_SetupUndistortion();
	}	  

    return hr;
} // end KF_InitializeKinectFusion

// Handle new depth data and perform Kinect Fusion processing
void Kin2::KF_ProcessDepth()
{
	if (m_bInitializeError)
	{
		return;
	}

	HRESULT hr = S_OK;

	// To enable playback of a .xef file through Kinect Studio and reset of the reconstruction
	// if the .xef loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
	if (m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0 && m_bResetReconstruction)
	{
		KF_reset();

		if (FAILED(hr))
		{
			return;
		}
	}

	// Return if the volume is not initialized
	if (nullptr == m_pVolume)
	{
		mexPrintf("Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting.\n");
		return;
	}

	////////////////////////////////////////////////////////
	// Depth to DepthFloat

	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(UINT16), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);

	if (FAILED(hr))
	{
		mexPrintf("Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed.\n");
		return;
	}

	////////////////////////////////////////////////////////
	// ProcessFrame

	// Perform the camera tracking and update the Kinect Fusion Volume
	// This will create memory on the GPU, upload the image, run camera tracking and integrate the
	// data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
	// parameter will use and update the internal camera pose.
	hr = m_pVolume->ProcessFrame(m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, nullptr, &m_worldToCameraTransform);

	// Test to see if camera tracking failed. 
	// If it did fail, no data integration or raycast for reference points and normals will have taken 
	//  place, and the internal camera pose will be unchanged.
	if (FAILED(hr))
	{
		if (hr == E_NUI_FUSION_TRACKING_ERROR)
		{
			m_cLostFrameCounter++;
			m_bTrackingFailed = true;
			mexPrintf("Kinect Fusion camera tracking failed! Align the camera to the last tracked position.\n");
		}
		else
		{
			mexPrintf("Kinect Fusion ProcessFrame call failed!\n");
			return;
		}
	}
	else
	{
		Matrix4 calculatedCameraPose;
		hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);

		if (SUCCEEDED(hr))
		{
			// Set the pose
			m_worldToCameraTransform = calculatedCameraPose;
			m_cLostFrameCounter = 0;
			m_bTrackingFailed = false;
		}
	}

	if (m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
	{
		// Automatically clear volume and reset tracking if tracking fails
		KF_reset();

		// Set bad tracking message
		mexPrintf("Kinect Fusion camera tracking failed, automatically reset volume.\n");
	}

	////////////////////////////////////////////////////////
	// CalculatePointCloud

	// Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
	hr = m_pVolume->CalculatePointCloud(m_pPointCloud, &m_worldToCameraTransform);

	if (FAILED(hr))
	{
		mexPrintf("Kinect Fusion CalculatePointCloud call failed.\n");
		return;
	}

	////////////////////////////////////////////////////////
	// ShadePointCloud and render

	hr = NuiFusionShadePointCloud(m_pPointCloud, &m_worldToCameraTransform, nullptr, m_pShadedSurface, nullptr);

	if (FAILED(hr))
	{
		mexPrintf("Kinect Fusion NuiFusionShadePointCloud call failed.\n");
		return;
	}
} // end KF_ProcessDepth

// cDepthWidth * cDepthHeight * cBytesPerPixel
void Kin2::KF_getVolumeImage(BYTE volumeImg[])
{
    BYTE * pBuffer = m_pShadedSurface->pFrameBuffer->pBits;
    
    int pixCount = cDepthWidth * cDepthHeight;
    
    // copy data (424x512) BYTE buffer to Matlab output
    // sweep the entire matrix copying data to output matrix
    int size = 4*cDepthWidth;
    for (int x=0, k=0; x < cDepthWidth*4; x+=4)
        for (int y=0; y <cDepthHeight; y++,k++)
        {
            int idx = y * size + x;
            volumeImg[k] = pBuffer[idx];
            volumeImg[pixCount + k] = pBuffer[++idx];
            volumeImg[pixCount*2 + k] = pBuffer[++idx];                   
        }
    
//    if(pBuffer)
//        delete pBuffer;
} // end KF_getVolumeImage

HRESULT Kin2::KF_getMesh(INuiFusionMesh **ppMesh)
{
	return m_pVolume->CalculateMesh(1, ppMesh);
} // end KF_getMesh

// Reset the reconstruction camera pose and clear the volume.
void Kin2::KF_reset()
{
    HRESULT hr = S_OK;
    
	SetIdentityMatrix(m_worldToCameraTransform);

	// Translate the reconstruction volume location away from the world origin by an amount equal
	// to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
	// If set false, the default world origin is set to the center of the front face of the 
	// volume, which has the effect of locating the volume directly in front of the initial camera
	// position with the +Z axis into the volume along the initial camera direction of view.
	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

		// Translate the volume in the Z axis by the minDepthThreshold distance
		float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
		worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);

		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	}
	else
	{
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
	}

	m_cLostFrameCounter = 0;
	m_cFrameCounter = 0;

	if (SUCCEEDED(hr))
	{
		m_bTrackingFailed = false;

		mexPrintf("Reconstruction has been reset.");
	}
	else
	{
		mexPrintf("Failed to reset reconstruction.");
	}
} // end KF_reset
