///////////////////////////////////////////////////////////////////////////
///		Kin2_mex.cpp
///
///		Description: 
///			Kin2 class implementations.
///			Define methods to initialize, and get images from sensor.
///			It uses Kinect2 SDK from Microsoft.
///			Copyright (c) Microsoft Corporation.  All rights reserved.
///
///		Usage:
///			Run demos
///
///		Authors: 
///			Juan R. Terven
///         Diana M. Cordova
///
///     Citation:
///     Terven J. Cordova D.M., "Kin2. A Kinect 2 Toolbox for MATLAB", Science of Computer Programming. 
///     https://github.com/jrterven/Kin2, 2016.
///		
///		Creation Date: Oct/07/2015
///     Modifications: 
///         Oct/08/2015: Reserve heap space only if required (using flags)
///         Oct/09/2015: Add coordinate mapping between depth and color
///         Oct/24/2015: Add skeleton tracking
///         Oct/28/2015: Add coordinate mapping between depth, color and camera
///         Oct/29/2015: Add coordinate mapping between camera and depth and color
///         Jan/05/2016: Add getPointCloud function
///			Jan/23/2016: Kinect Fusion
///			Jan/25/2016: Face processing
///			Jan/25/2016: Body index
///         Jan/31/2016: HD Face processing
///         Mar/01/2016: Depth camera intrinsics
///         Mar/03/2016: Add Joints orientations
///         Mar/09/2016: Add pointclouds with color and pointCloud MATLAB object
///         Mar/15/2016: Add radial distortion to the color camera calibration
///         Mar/18/2016: Fix HD face shape units
///         Mar/31/2016: Add floor clip plane to the body data
///////////////////////////////////////////////////////////////////////////
#include "Kin2.h"
#include "mex.h"
#include "class_handle.hpp"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <Kinect.Face.h>
#include <vector>

 // Constructor
Kin2::Kin2(UINT16 sources):
    m_pKinectSensor(NULL),
    m_pMultiSourceFrameReader(NULL),
    m_pDepthArray16U(NULL),
    m_pColor(NULL),    
    m_pInfraredArray16U(NULL),
    m_pBodyIndex(NULL),
    m_depthTimeStamp(0),
    m_colorTimeStamp(0),
    m_infraredTimeStamp(0),
    m_bodiesTimeStamp(0),
    m_newDepthData(false),
    m_newColorData(false),
    m_newInfraredData(false),
    m_bHaveBodyData(false),
    m_newBodyIndex(false),
    m_newPointCloudData(false),
    // Kinect Fusion variables
    m_pVolume(nullptr),
    m_cDepthImagePixels(0),
    m_bMirrorDepthFrame(false),
    m_bTranslateResetPoseByMinDepthThreshold(true),
    m_bAutoResetReconstructionWhenLost(false),
    m_bAutoResetReconstructionOnTimeout(true),
    m_bResetReconstruction(false),
    m_cLostFrameCounter(0),
    m_bTrackingFailed(false),
    m_cFrameCounter(0),
    m_pDepthImagePixelBuffer(nullptr),
    m_pDepthDistortionMap(nullptr),
    m_pDepthDistortionLT(nullptr),
    m_pDepthFloatImage(nullptr),
    m_pPointCloud(nullptr),
    m_pShadedSurface(nullptr),
    m_bInitializeError(false),
    m_coordinateMappingChangedEvent(NULL),
    m_bHaveValidCameraParameters(false)
{
    m_flags = (k2::Flags)sources;
    
    for (int i = 0; i < BODY_COUNT; ++i)
		m_ppBodies[i] = NULL;
    
    for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
		m_pHDFaceFrameSources[i] = nullptr;
		m_pHDFaceFrameReaders[i] = nullptr;		
		m_pFaceAlignment[i] = nullptr;
		m_pFaceModel[i] = nullptr;		
		m_pFaceModelBuilder[i] = nullptr;
		m_faceModelReady[i] = false;
		m_faceModelWarning[i] = false;
	}
        
    // Initialize Kinect2
    init();
} // end constructor
        
// Destructor. Release all buffers
Kin2::~Kin2()
{    
    // Clean up Kinect Fusion
	//SafeRelease(m_pVolume);
    if(m_pVolume != NULL )
    {
        m_pVolume->Release();
        m_pVolume = NULL;
    }
    
    if(m_pDepthArray16U)
    {
        delete [] m_pDepthArray16U;
        m_pDepthArray16U = NULL;
    }

    if (m_pColor)
    {
        delete [] m_pColor;
        m_pColor = NULL;
    }
    
    if(m_pInfraredArray16U)
	{
		delete [] m_pInfraredArray16U;
		m_pInfraredArray16U = NULL;
	}
    
    if (m_pBodyIndex)
	{
		delete[] m_pBodyIndex;
		m_pBodyIndex = NULL;
	}

    for (int i = 0; i < _countof(m_ppBodies); ++i)
	{
		SafeRelease(m_ppBodies[i]);
	}
    
    // done with face sources and readers
	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
		SafeRelease(m_pHDFaceFrameSources[i]);
		SafeRelease(m_pHDFaceFrameReaders[i]);
	
		SafeRelease(m_pFaceAlignment[i]);
		SafeRelease(m_pFaceModel[i]);

		SafeRelease(m_pFaceModelBuilder[i]);
	}
        
    // done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	if (nullptr != m_pCoordinateMapper)
		m_pCoordinateMapper->UnsubscribeCoordinateMappingChanged(m_coordinateMappingChangedEvent);        

	//SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedSurface);
	//SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pPointCloud);
	//SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);
    
    if (m_pShadedSurface) 
    { 
        static_cast<void>(NuiFusionReleaseImageFrame(m_pShadedSurface)); 
        m_pShadedSurface = NULL;
    }
    
    if (m_pPointCloud) 
    { 
        static_cast<void>(NuiFusionReleaseImageFrame(m_pPointCloud)); 
        m_pPointCloud = NULL;
    }
    
    if (m_pDepthFloatImage) 
    { 
        static_cast<void>(NuiFusionReleaseImageFrame(m_pDepthFloatImage)); 
        m_pDepthFloatImage = NULL;
    }
    
    // done with frame reader
    SafeRelease(m_pMultiSourceFrameReader);
       

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
        m_pKinectSensor->Release();
    }
    
    // clean up the depth pixel array
	//SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
	//SAFE_DELETE_ARRAY(m_pDepthDistortionMap);
	//SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
    if(m_pDepthImagePixelBuffer)
    {
        delete[] m_pDepthImagePixelBuffer;
        m_pDepthImagePixelBuffer = NULL;
    }
    
    if(m_pDepthDistortionMap)
    {
        delete[] m_pDepthDistortionMap;
        m_pDepthDistortionMap = NULL;
    }
    
    if(m_pDepthDistortionLT)
    {
        delete[] m_pDepthDistortionLT;
        m_pDepthDistortionLT = NULL;
    }

    mexPrintf("Kinect Object destroyed\n");

} // end destructor

///////// Function: init ///////////////////////////////////////////
// Initialize Kinect2 and frame reader
//////////////////////////////////////////////////////////////////////////
void Kin2::init()
{
    HRESULT hr;
    
    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        mexPrintf("Failed getting Kinect Sensor!\n");
        return;
    }
    
    if (m_pKinectSensor)
    {
        // Get Coordinate mapper
		if (SUCCEEDED(hr))
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        
        // Initialize the Kinect and get frame reader
        if (SUCCEEDED(hr))
            hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            mexPrintf("Kinect Sensor Opened succesfully!\n");
            
            k2::Flags validFlags = m_flags;
			// remove FACE flags because it is not a valid flag for the OpenMultiSourceFrameReader
			if (m_flags & k2::FACE)
			{
				validFlags = m_flags & ~k2::FACE | k2::BODY;
				m_flags = m_flags | k2::BODY;
			}

			// remove HD_FACE flags because it is not a valid flag for the OpenMultiSourceFrameReader
			if (m_flags & k2::HD_FACE)
			{
				validFlags = validFlags & ~k2::HD_FACE | k2::BODY;
				m_flags = m_flags | k2::BODY;
			}            
                        
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(
                validFlags,&m_pMultiSourceFrameReader);
        }
        if (SUCCEEDED(hr))
            mexPrintf("Kinect sources initialized successfully!\n");
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        mexPrintf("Failed Initializing Kinect Sensor!\n");
        return;
    }
    
    // create heap storage for the 16bit depth image
    if (m_flags & k2::DEPTH)
        m_pDepthArray16U = new UINT16[cDepthWidth * cDepthHeight];

    // create heap storage for color pixel data 
    if (m_flags & k2::COLOR)
        m_pColor = new BYTE[cColorWidth * cColorHeight * 4];
    
    // create heap storage for infrared data in UINT16 format 
    if (m_flags & k2::INFRARED)
        m_pInfraredArray16U = new UINT16[cInfraredWidth * cInfraredHeight];
    
    // create heap storage for body index data
	if (m_flags & k2::BODY_INDEX)
		m_pBodyIndex = new BYTE[cDepthWidth * cDepthHeight];
    
    if (m_flags & k2::FACE)
	{
		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
			}
		}
	}
    
    if (m_flags & k2::HD_FACE)
	{        
        // create a face frame source + reader to track each body in the fov
        for (int i = 0; i < BODY_COUNT; i++)
        {
            // create the face frame source by specifying the required face frame features
            hr = CreateHighDefinitionFaceFrameSource(m_pKinectSensor, &m_pHDFaceFrameSources[i]);

            if (FAILED(hr))
            {
                mexPrintf("Error : CreateHighDefinitionFaceFrameSource()\n");
                return;
            }

            // open the corresponding reader
            hr = m_pHDFaceFrameSources[i]->OpenReader(&m_pHDFaceFrameReaders[i]);
            if (FAILED(hr))
            {
                mexPrintf("Error : IHighDefinitionFaceFrameSource::OpenReader()\n");
            }		

            // Create Face Alignment
            hr = CreateFaceAlignment(&m_pFaceAlignment[i]);
            if (FAILED(hr))
            {
                mexPrintf("Error : CreateFaceAlignment()\n");
            }

            // Create base Face Models
            float deformations[FaceShapeDeformations_Count] = { 0 };
            hr = CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, deformations, &m_pFaceModel[i]);
            if (FAILED(hr))
            {
                mexPrintf("Error : CreateFaceModel()\n");
            }

            // Open Face Model Builder
            hr = m_pHDFaceFrameSources[i]->OpenModelBuilder(FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &m_pFaceModelBuilder[i]);
            if (FAILED(hr))
            {
                mexPrintf("Error : IHighDefinitionFaceFrameSource::OpenModelBuilder()\n");
                return;
            }

            // Start Collection Face Data
            hr = m_pFaceModelBuilder[i]->BeginFaceDataCollection();
            if (FAILED(hr))
            {
                mexPrintf("Error : IFaceModelBuilder::BeginFaceDataCollection()\n");
                return;
            }
        } // for each body
	} // if (m_flags & k2::HD_FACE)        
} // end init

///////// Function: updateData ///////////////////////////////////////////
// Get current data from Kinect and save it in the member variables
//////////////////////////////////////////////////////////////////////////
void Kin2::updateData(INT8 valid[])
{
    if (!m_pMultiSourceFrameReader)
    {
        mexPrintf("No Kinect Reader initialized!\n");
        return;
    }
    
    IMultiSourceFrame* pMultiSourceFrame = NULL;
    HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

    // Get Depth frame
    if (SUCCEEDED(hr))
    {   
        if (m_flags & k2::DEPTH)
		{
            IDepthFrame* pDepthFrame;
            IDepthFrameReference* pDepthFrameReference = NULL;

            hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
            if (SUCCEEDED(hr))
                hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);

            // get the timestamp
            if (SUCCEEDED(hr))
                hr = pDepthFrame->get_RelativeTime(&m_depthTimeStamp);
                
            // Copy underlying buffer to member variable
            if (SUCCEEDED(hr))
                hr = pDepthFrame->CopyFrameDataToArray(cDepthWidth * cDepthHeight, m_pDepthArray16U);

            // If successful copy, set indicator of new depth data
            if (SUCCEEDED(hr))
            {
                m_newDepthData = true;
                m_newPointCloudData = true;
            }
            
            SafeRelease(pDepthFrame);
            SafeRelease(pDepthFrameReference);
        }
    }

    // Get Color frame
    if (SUCCEEDED(hr))
    {        
        if (m_flags & k2::COLOR)
		{
            IColorFrame* pColorFrame;
            IColorFrameReference* pColorFrameReference = NULL;

            hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
            if (SUCCEEDED(hr))
                hr = pColorFrameReference->AcquireFrame(&pColorFrame);
            
            if (SUCCEEDED(hr))
                hr = pColorFrame->get_RelativeTime(&m_colorTimeStamp);
                
            if (SUCCEEDED(hr))
            {
                UINT nColorBufferSize = cColorWidth * cColorHeight * 4;
                hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, m_pColor, ColorImageFormat_Rgba);
            }
            
            if (SUCCEEDED(hr))
                m_newColorData = true;

            SafeRelease(pColorFrame);
            SafeRelease(pColorFrameReference);
            
        }
    }
    
    // Get Infrared frame
	if (SUCCEEDED(hr))
    {    
        if (m_flags & k2::INFRARED)
		{
            IInfraredFrame* pInfraredFrame;
            IInfraredFrameReference* pInfraredReference = NULL;

            hr = pMultiSourceFrame->get_InfraredFrameReference(&pInfraredReference);

            if (SUCCEEDED(hr))
                hr = pInfraredReference->AcquireFrame(&pInfraredFrame);
            
            if (SUCCEEDED(hr))
                hr = pInfraredFrame->get_RelativeTime(&m_infraredTimeStamp);
             
            if (SUCCEEDED(hr))
               hr = pInfraredFrame->CopyFrameDataToArray(cInfraredHeight*cInfraredWidth, m_pInfraredArray16U);

            if (SUCCEEDED(hr))
                m_newInfraredData = true;
                
            SafeRelease(pInfraredFrame);
            SafeRelease(pInfraredReference);
        }
    }
    
    // Get Body Index
	if (SUCCEEDED(hr))
	{
		if (m_flags & k2::BODY_INDEX)
		{
			IBodyIndexFrame* pBodyIndexFrame = NULL;
			IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

			hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
			}

			// Copy data to member variable
			if (SUCCEEDED(hr))
				hr = pBodyIndexFrame->CopyFrameDataToArray(cDepthWidth*cDepthHeight, m_pBodyIndex);
            
            if (SUCCEEDED(hr))
                m_newBodyIndex = true;
			
			SafeRelease(pBodyIndexFrame);
			SafeRelease(pBodyIndexFrameReference);
		}
	}
    
    // Get Body data
	if (SUCCEEDED(hr))
	{
		if ((m_flags & k2::BODY) || (m_flags & k2::FACE) || (m_flags & k2::HD_FACE))
		{
			IBodyFrame* pBodyFrame = NULL;
			IBodyFrameReference* pBodyFrameReference = NULL;

			hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
			}

            if (SUCCEEDED(hr))
                hr = pBodyFrame->get_RelativeTime(&m_bodiesTimeStamp);
            
			if (SUCCEEDED(hr))
			{
				for (int i = 0; i < BODY_COUNT; ++i)
					m_ppBodies[i] = NULL;

				for (int i = 0; i < _countof(m_ppBodies); ++i)
					SafeRelease(m_ppBodies[i]);

				hr = pBodyFrame->GetAndRefreshBodyData(_countof(m_ppBodies), m_ppBodies);
                m_bHaveBodyData = SUCCEEDED(hr);
                
                if (SUCCEEDED(hr))
				{
					pBodyFrame->get_FloorClipPlane(&m_floorClipPlane);
				}
			}

			SafeRelease(pBodyFrame);
			SafeRelease(pBodyFrameReference);
		}
	}        
  
    if (SUCCEEDED(hr))
    {
        valid[0] = 1;
    }
    else
        valid[0] = 0;
    
    SafeRelease(pMultiSourceFrame);
} // end updateData

///////// Function: getDepth ///////////////////////////////////////////
// Copy depth frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void Kin2::getDepth(UINT16 depth[], INT64& time, bool& validDepth)
{
    if(m_newDepthData)
    {
        // Copy Depth frame to output matrix
        for (int x=0, k=0;x<cDepthWidth;x++)
            for (int y=0;y<cDepthHeight;y++,k++)              
                depth[k] = m_pDepthArray16U[y*cDepthWidth+x];
        
        time = m_depthTimeStamp;
    }
    
    validDepth = m_newDepthData;
    m_newDepthData = false;   
} // end getDepth

///////// Function: getColor ///////////////////////////////////////////
// Copy color frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void Kin2::getColor(unsigned char rgbImage[], INT64& time, bool& validColor)
{
    if(m_newColorData)
    {         
        // copy color (1080x1920) BYTE buffer to Matlab output
        // sweep the entire matrix copying data to output matrix
        int colSize = 4*cColorWidth;
        for (int x=0, k=0; x < cColorWidth*4; x+=4)
            for (int y=0; y <cColorHeight; y++,k++)
            {
                int idx = y * colSize + x;
                rgbImage[k] = m_pColor[idx];
                rgbImage[cNumColorPix + k] = m_pColor[++idx];
                rgbImage[cNumColorPix*2 + k] = m_pColor[++idx];                   
            }
        
        time = m_colorTimeStamp;
    }
    validColor = m_newColorData;
    m_newColorData = false;
} // end getColor

///////// Function: getInfrared ///////////////////////////////////////////
// Copy infrared frame to Matlab matrix
// You must call updateData first
///////////////////////////////////////////////////////////////////////////
void Kin2::getInfrared(UINT16 infrared[], INT64& time, bool& validInfrared)
{
    if (m_newInfraredData)
    {
        // sweep the entire matrix copying data to output matrix
        for (int x=0, k=0;x<cInfraredWidth;x++)
            for (int y=0;y<cInfraredHeight;y++,k++)              
                infrared[k] = m_pInfraredArray16U[y*cInfraredWidth+x];  
        
        time = m_infraredTimeStamp;
    }

     validInfrared = m_newInfraredData;
     m_newInfraredData = false;
} // end getInfrared

///////// Function: getBodyIndex ///////////////////////////////////////////
// Copy getBodyIndex frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void Kin2::getBodyIndex(BYTE bodyIndex[], bool& validBodyIndex)
{
    if(m_newBodyIndex)
    {
        for (int x=0, k=0;x<cDepthWidth;x++)
            for (int y=0;y<cDepthHeight;y++,k++)              
                bodyIndex[k] = m_pBodyIndex[y*cDepthWidth+x];
    }
    
    validBodyIndex = m_newBodyIndex;
    m_newBodyIndex = false;   
} // end getBodyIndex


void Kin2::getBodies(std::vector<std::vector<Joint> >& bodiesJoints,
        std::vector<std::vector<JointOrientation> >& bodiesJointsOrientations,
        std::vector<HandState>& lhs, std::vector<HandState>& rhs, 
        Vector4 &fcp, INT64& time)
{
	HRESULT hr;

    if(m_bHaveBodyData)
    {
        fcp = m_floorClipPlane;
        time = m_bodiesTimeStamp;
    }
    else
    {
        time = 0;
    }
        
	for (int i = 0; i < BODY_COUNT; ++i)
	{
		IBody* pBody = m_ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];
                JointOrientation jointsOrientations[JointType_Count];
				HandState leftHandState = HandState_Unknown;
				HandState rightHandState = HandState_Unknown;

				pBody->get_HandLeftState(&leftHandState);
				pBody->get_HandRightState(&rightHandState);

				hr = pBody->GetJoints(_countof(joints), joints);
                if (SUCCEEDED(hr))
                    hr = pBody->GetJointOrientations(_countof(joints), jointsOrientations);

				// Copy array of Joints to vector of joints
				std::vector<Joint> vJoints(std::begin(joints), std::end(joints));
                
                // Copy array of Joints orientations to vector
                std::vector<JointOrientation> vJointsOrientations
                        (std::begin(jointsOrientations), std::end(jointsOrientations));

				if (SUCCEEDED(hr))
				{
					bodiesJoints.push_back(vJoints);
                    bodiesJointsOrientations.push_back(vJointsOrientations);
					lhs.push_back(leftHandState);
					rhs.push_back(rightHandState);
				}
			} // if body tracked
		} // if body detected
	} // for each body
}// end getBodies



void Kin2::getDepthIntrinsics(CameraIntrinsics &intrinsics)
{
    m_pCoordinateMapper->GetDepthCameraIntrinsics(&intrinsics);
}

void Kin2::extractRotationInDegrees(Vector4& pQuaternion, double& dPitch, double& dYaw, double& dRoll)
{
	double x = pQuaternion.x;
	double y = pQuaternion.y;
	double z = pQuaternion.z;
	double w = pQuaternion.w;

	// convert rotation quaternion to Euler angles in degrees		
	dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / 3.141593 * 180.0;
	dYaw = asin(2 * (w * y - x * z)) / 3.141593 * 180.0;
	dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / 3.141593 * 180.0;
}
