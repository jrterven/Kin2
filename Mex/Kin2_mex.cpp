///////////////////////////////////////////////////////////////////////////
///		Kin2_mex.cpp
///
///		Description: 
///			Kin2 class encapsulates the funtionality of Kinect2 Sensor.
///			Define methods to initialize, and get images from sensor.
///			It uses Kinect2 SDK from Microsoft.
///			Copyright (c) Microsoft Corporation.  All rights reserved.
///
///		Usage:
///			Run demos
///
///		Author: 
///			Juan R. Terven.	jrterven@hotmail.com
///         Diana M. Cordova, diana_mce@hotmail.com
///
///     Citation:
///     J. R. Terven, D. M. Cordova, "A Kinect 2 Toolbox for MATLAB", 
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
///////////////////////////////////////////////////////////////////////////
#include "mex.h"
#include "class_handle.hpp"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <Kinect.Face.h>
#include <vector>

#define SAFE_FUSION_RELEASE_IMAGE_FRAME(p) { if (p) { static_cast<void>(NuiFusionReleaseImageFrame(p)); (p)=NULL; } }
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }

namespace k2
{
    // Sources of Kinect data. These are selected when creating the Kin2 object
    enum{ 
		COLOR = FrameSourceTypes::FrameSourceTypes_Color,
		DEPTH = FrameSourceTypes::FrameSourceTypes_Depth,
		INFRARED = FrameSourceTypes::FrameSourceTypes_Infrared,
        BODY_INDEX = FrameSourceTypes::FrameSourceTypes_BodyIndex,
		BODY = FrameSourceTypes::FrameSourceTypes_Body,
		FACE = 0x80,
        HD_FACE = 0x100
    };
    typedef unsigned short int Flags;
    
    // FaceData structure returned by the getFaces function
    typedef struct _FaceData
	{
		RectI faceBox;
		PointF facePoints[FacePointType::FacePointType_Count];
		Vector4 faceRotation;
		DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
	}FaceData;
    
    // HDFaceData structure returned by the getHDFaces function
    typedef struct _HDFaceData
	{
		RectI faceBox;		
		Vector4 faceRotation;
		CameraSpacePoint headPivot;
		float animationUnits[FaceShapeAnimations_Count];
		float shapeUnits[FaceShapeDeformations_Count];
		CameraSpacePoint faceModel[1347];

	}HDFaceData;
}

/*********** Kin2 Class **********************/
class Kin2
{
    static const int        cDepthWidth  = 512;     // depth image width
    static const int        cDepthHeight = 424;     // depth image height
    static const int        cInfraredWidth = 512;   // infrared image width
	static const int        cInfraredHeight = 424;  // infrared image height
    static const int        cColorWidth  = 1920;    // color image width
    static const int        cColorHeight = 1080;    // color image height
    static const int        cNumColorPix = cColorWidth*cColorHeight; // number of color pixels
public:   
    Kin2(UINT16 sources);   // Constructor    
    ~Kin2();                // Destructor
    
    // Initialize Kinect2
    void init();    
    
    /************ Video Sources *************/
    void updateData(INT8 valid[]);
    void getDepth(UINT16 depth[],bool& validDepth);
    void getColor(unsigned char rgbImage[], bool& validColor);
    void getInfrared(UINT16 infrared[],bool& validInfrared);
    void getBodyIndex(BYTE bodyIndex[],bool& validBodyIndex);
    void getPointCloud(double pointCloud[], int size, bool& validData);
    void getFaces(std::vector<k2::FaceData>& facesData);
    void getHDFaces(std::vector<k2::HDFaceData>& facesData);
    
    /************ Mappings **************/
    void mapDepthPoints2Color(double depthCoords[], int size, UINT16 colorCoords[]);
    void mapDepthPoints2Camera(double depthCoords[], int size, double cameraCoords[]);
    bool mapDepthFrame2Color(ColorSpacePoint* depth2ColorMapping);

	void mapColorPoints2Depth(double colorCoords[], int size, UINT16 depthCoords[]);
    void mapColorPoints2Camera(double colorCoords[], int size, double cameraCoords[]);
    
    void mapCameraPoints2Depth(double cameraCoords[], int size, UINT16 depthCoords[]);
    void mapCameraPoints2Color(double cameraCoords[], int size, UINT16 colorCoords[]);
    
    /************ Body Tracking *****************/
    void getBodies(std::vector<std::vector<Joint> >& bodiesJoints, 
        std::vector<HandState>& lhs, std::vector<HandState>& rhs);
    
    /*************** Kinect Fusion***************/
	void KF_init(int voxelsPerMeter = 64, int voxelsX = 256, int voxelsY = 256, int voxelsZ = 256, bool processorType = true, bool autoReset = true);
	void KF_update();
	void KF_getVolumeImage(BYTE volumeImg[]);
	void KF_reset();
	HRESULT KF_getMesh(INuiFusionMesh **ppMesh);
        
private:
    // Current Kinect
    IKinectSensor*          m_pKinectSensor;		// The Kinect sensor
	ICoordinateMapper*      m_pCoordinateMapper;	// The coordinate mapper
    
    // Frame reader
    IMultiSourceFrameReader* m_pMultiSourceFrameReader; // Kinect data reader

	// Heap storage for images
	UINT16*     m_pDepthArray16U;       // 16-bit depth image
    UINT16*		m_pInfraredArray16U;    // 16-bit infrared image 
	BYTE*       m_pColor;
    BYTE*		m_pBodyIndex;
    
    // Heap storage for bodies
	IBody*		m_ppBodies[BODY_COUNT];
    bool		m_bHaveBodyData;
    
	// Face sources
	IFaceFrameSource*       m_pFaceFrameSources[BODY_COUNT];

	// Face readers
	IFaceFrameReader*       m_pFaceFrameReaders[BODY_COUNT];
    
    // HD Face sources
	IHighDefinitionFaceFrameSource* m_pHDFaceFrameSources[BODY_COUNT];

	// HDFace readers
	IHighDefinitionFaceFrameReader*	m_pHDFaceFrameReaders[BODY_COUNT];
    
    // HD face data
    float*                  m_pAnimationUnits;
    float*                  m_pShapeUnits;
    CameraSpacePoint*       m_pFaceModelVertices;
             
    // Indicators of data available
    bool        m_newDepthData;
    bool        m_newColorData;
    bool        m_newInfraredData;
    bool        m_newBodyIndex;
    bool        m_newPointCloudData;
    
    // Initialization flags
    k2::Flags       m_flags;
    
    /************************************************************/
	/******  Face Processing private variables and functions ******/
	/************************************************************/
    // Store faces data
	std::vector<k2::FaceData> m_facesData;
    std::vector<k2::HDFaceData> m_HDfacesData;

	// define the face frame features required to be computed by this application
	static const DWORD c_FaceFrameFeatures =
		FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

	void ProcessFaces();
    void ProcessHDFaces();
    
    /************************************************************/
	/******  Kinect Fusion private variables and functions ******/
	/************************************************************/
	static const int            cResetOnTimeStampSkippedMilliseconds = 1000;  // ms
	static const int            cResetOnNumberOfLostFrames = 100;

	/// Setup or update the Undistortion calculation for the connected camera
	HRESULT		KF_SetupUndistortion();

	
	HRESULT		OnCoordinateMappingChanged();

	/// Initialize Kinect Fusion volume and images for processing
	HRESULT		KF_InitializeKinectFusion();

	/// Handle new depth data
	void		KF_ProcessDepth();

	UINT		m_cDepthImagePixels;

	/// The Kinect Fusion Reconstruction Volume
	INuiFusionReconstruction*   m_pVolume;

	/// The Kinect Fusion Volume Parameters
	NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;

	// The Kinect Fusion Camera Transform
	Matrix4                     m_worldToCameraTransform;

	// The default Kinect Fusion World to Volume Transform
	Matrix4                     m_defaultWorldToVolumeTransform;

	/// Frames from the depth input
	UINT16*                     m_pDepthImagePixelBuffer;
	NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;

	/// For depth distortion correction
	DepthSpacePoint*            m_pDepthDistortionMap;
	UINT*                       m_pDepthDistortionLT;
	WAITABLE_HANDLE             m_coordinateMappingChangedEvent;

	/// Kinect camera parameters.
	NUI_FUSION_CAMERA_PARAMETERS m_cameraParameters;
	bool                        m_bHaveValidCameraParameters;

	/// Frames generated from ray-casting the Reconstruction Volume
	NUI_FUSION_IMAGE_FRAME*     m_pPointCloud;

	/// Images for display
	NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;

	/// Camera Tracking parameters
	int                         m_cLostFrameCounter;
	bool                        m_bTrackingFailed;

	bool                        m_bResetReconstruction;
	/// Parameter to turn automatic reset of the reconstruction when camera tracking is lost on or off.
	/// Set to true in the KF_init to enable auto reset on cResetOnNumberOfLostFrames lost frames,
	/// or set false to never automatically reset.
	bool                        m_bAutoResetReconstructionWhenLost;

	/// Parameter to enable automatic reset of the reconstruction when there is a large
	/// difference in timestamp between subsequent frames. This should usually be set true as 
	/// default to enable recorded .xef files to generate a reconstruction reset on looping of
	/// the playback or scrubbing, however, for debug purposes, it can be set false to prevent
	/// automatic reset on timeouts.
	bool                        m_bAutoResetReconstructionOnTimeout;

	/// Processing parameters
	int                         m_deviceIndex;
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;
	bool                        m_bInitializeError;
	float                       m_fMinDepthThreshold;
	float                       m_fMaxDepthThreshold;
	bool                        m_bMirrorDepthFrame;
	unsigned short              m_cMaxIntegrationWeight;
	int                         m_cFrameCounter;

	/// Parameter to translate the reconstruction based on the minimum depth setting. When set to
	/// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
	/// Setting this true in the constructor will move the volume forward along +Z away from the
	/// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
	/// by setting a non-identity camera transformation in the ResetReconstruction call.
	/// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
	/// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
	/// when the majority of a small volume is inside this distance.
	bool                        m_bTranslateResetPoseByMinDepthThreshold;
}; // Kin2 class

 // Constructor
Kin2::Kin2(UINT16 sources):
    m_pKinectSensor(NULL),
    m_pMultiSourceFrameReader(NULL),
    m_pDepthArray16U(NULL),
    m_pColor(NULL),    
    m_pInfraredArray16U(NULL),
    m_pBodyIndex(NULL),
    m_newDepthData(false),
    m_newColorData(false),
    m_newInfraredData(false),
    m_bHaveBodyData(false),
    m_newBodyIndex(false),
    m_newPointCloudData(false),
    m_pAnimationUnits(NULL),
    m_pShapeUnits(NULL),
    m_pFaceModelVertices(NULL),
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
	}
    
    if(m_pAnimationUnits)
    {
        delete[] m_pAnimationUnits;
        m_pAnimationUnits = NULL;
    }
    
    if(m_pShapeUnits)
    {
        delete[] m_pShapeUnits;
        m_pShapeUnits = NULL;
    }
    
    if(m_pFaceModelVertices)
    {
        delete[] m_pFaceModelVertices;
        m_pFaceModelVertices = NULL;
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
        m_pAnimationUnits = new float[FaceShapeAnimations_Count]();
        m_pShapeUnits = new float[FaceShapeDeformations_Count]();
        m_pFaceModelVertices = new CameraSpacePoint[1347]();
        
		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateHighDefinitionFaceFrameSource(m_pKinectSensor, &m_pHDFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pHDFaceFrameSources[i]->OpenReader(&m_pHDFaceFrameReaders[i]);
				}
			}
		}
	}
} // end init

///////// Function: updateData ///////////////////////////////////////////
// Get current data from Kinect and save it in the member variables
//////////////////////////////////////////////////////////////////////////
void Kin2::updateData(INT8 valid[])
{
    if (!m_pMultiSourceFrameReader)
    {
        mexPrintf("No depth source initialized!\n");
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
			{
				for (int i = 0; i < BODY_COUNT; ++i)
					m_ppBodies[i] = NULL;

				for (int i = 0; i < _countof(m_ppBodies); ++i)
					SafeRelease(m_ppBodies[i]);

				hr = pBodyFrame->GetAndRefreshBodyData(_countof(m_ppBodies), m_ppBodies);
                m_bHaveBodyData = SUCCEEDED(hr);
			}

			SafeRelease(pBodyFrame);
			SafeRelease(pBodyFrameReference);
		}
	}
    
    // Get Face data
	if (SUCCEEDED(hr))
	{
		if (m_flags & k2::FACE)
		{
			ProcessFaces();
		}
	}
    
    // Get HD Face data
	if (SUCCEEDED(hr))
	{
		if (m_flags & k2::HD_FACE)
		{
			ProcessHDFaces();
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
void Kin2::getDepth(UINT16 depth[],bool& validDepth)
{
    if(m_newDepthData)
    {
 //       mexPrintf("bufferSize=%d\n",m_dBufferSize);
        // Copy Depth frame to output matrix
        for (int x=0, k=0;x<cDepthWidth;x++)
            for (int y=0;y<cDepthHeight;y++,k++)              
                depth[k] = m_pDepthArray16U[y*cDepthWidth+x];
//                  depth[k] = m_pdBuffer[y*cDepthWidth+x];
    }
    
    validDepth = m_newDepthData;
    m_newDepthData = false;   
} // end getDepth

///////// Function: getColor ///////////////////////////////////////////
// Copy color frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void Kin2::getColor(unsigned char rgbImage[], bool& validColor)
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
    }
    validColor = m_newColorData;
    m_newColorData = false;
} // end getColor

///////// Function: getInfrared ///////////////////////////////////////////
// Copy infrared frame to Matlab matrix
// You must call updateData first
///////////////////////////////////////////////////////////////////////////
void Kin2::getInfrared(UINT16 infrared[],bool& validInfrared)
{
    if (m_newInfraredData)
    {
        // sweep the entire matrix copying data to output matrix
        for (int x=0, k=0;x<cInfraredWidth;x++)
            for (int y=0;y<cInfraredHeight;y++,k++)              
                infrared[k] = m_pInfraredArray16U[y*cInfraredWidth+x];           
    }

     validInfrared = m_newInfraredData;
     m_newInfraredData = false;
} // end getInfrared

///////// Function: getBodyIndex ///////////////////////////////////////////
// Copy getBodyIndex frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void Kin2::getBodyIndex(BYTE bodyIndex[],bool& validBodyIndex)
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

///////// Function: getPointCloud ///////////////////////////////////////////
// Get camera points from depth frame and copy them to Matlab matrix
// You must call updateData first and have depth activated
///////////////////////////////////////////////////////////////////////////
void Kin2::getPointCloud(double pointCloud[], int size, bool& validData)
{
    // Create coordinate mapping from depth to camera
	HRESULT hr;
	int numDepthPoints = size;

    if(m_newPointCloudData)
    {
        CameraSpacePoint* cameraPoints = new CameraSpacePoint[numDepthPoints];	
        hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(numDepthPoints,
            (UINT16*)m_pDepthArray16U, numDepthPoints, cameraPoints);

	
        if (SUCCEEDED(hr))
        {
            // Fill-up the point cloud with x,y,z values from camera space
            for (int i = 0; i < numDepthPoints; i++)
            {
                float X, Y, Z;
                X = cameraPoints[i].X;
                //if (X - X == 0) // if is not infinite
                //{
                    Y = cameraPoints[i].Y;
                    Z = cameraPoints[i].Z;

                    pointCloud[i] = X;
                    pointCloud[i + size] = Y;
                    pointCloud[i + size + size] = Z;
                //}
                /*else
                {
                    pointCloud[i] = 0;
                    pointCloud[i + size] = 0;
                    pointCloud[i + size + size] = 0;
                }*/
            }
        }    
        else
        {
            pointCloud[0] = 0;
            pointCloud[1] = 0;
            pointCloud[2] = 0;
            mexPrintf("Error getting depth-to-camera mapping.\n");
        }

        delete[] cameraPoints; 
        cameraPoints = NULL;
    }
    validData = m_newPointCloudData;
    m_newPointCloudData = false;
}

///////// Function: mapDepthPoints2Color //////////////////////////////
// Map the input points in depth coordinates to points in color coordinates
///////////////////////////////////////////////////////////////////////////
void Kin2::mapDepthPoints2Color(double depthCoords[], int size, UINT16 colorCoords[])
{
    int numDepthPoints = size;
    
    // Depth coordinates
	DepthSpacePoint* depthPoints = new DepthSpacePoint[numDepthPoints];
    
    // Depth values
	UINT16* depthValues = new UINT16[numDepthPoints];
    
    DepthSpacePoint d;
    
    for(int i=0; i<size; i++)
    {
        int x = (int)depthCoords[i];
        int y = (int)depthCoords[i+size];
        
        d.X = (float)x; d.Y = (float)y;
		depthPoints[i] = d;
        
        // Get Depths
		int idx = y*cDepthWidth + x;
		depthValues[i] = m_pDepthArray16U[idx];
    }

    ColorSpacePoint* colorPoints = new ColorSpacePoint[numDepthPoints];

    HRESULT hr;
    hr = m_pCoordinateMapper->MapDepthPointsToColorSpace(numDepthPoints, depthPoints, 
    numDepthPoints, depthValues, numDepthPoints, colorPoints);
    
    if (SUCCEEDED(hr))
	{		
		for (int i = 0; i < numDepthPoints; i++)
		{       
            colorCoords[i] = (int)colorPoints[i].X;
            colorCoords[i+size] = (int)colorPoints[i].Y;
        }
    }    
	else
    {
        colorCoords[0] = 0;
		colorCoords[1] = 0;
		mexPrintf("Mapping error.\n");
    }
    
    delete[] depthPoints; depthPoints = NULL;
	delete[] depthValues; depthValues = NULL;
	delete[] colorPoints; colorPoints = NULL;    
} // end mapDepthPoints2Color

///////// Function: mapDepthPoints2Camera //////////////////////////////
// Map the input points in depth coordinates to points in camera coordinates
///////////////////////////////////////////////////////////////////////////
void Kin2::mapDepthPoints2Camera(double depthCoords[], int size, double cameraCoords[])
{
    int numDepthPoints = size;
    
    // Depth coordinates
	DepthSpacePoint* depthPoints = new DepthSpacePoint[numDepthPoints];
    
    // Depth values
	UINT16* depthValues = new UINT16[numDepthPoints];
    
    DepthSpacePoint d;
    
    for(int i=0; i<size; i++)
    {
        int x = (int)depthCoords[i];
        int y = (int)depthCoords[i+size];
        
        d.X = (float)x; d.Y = (float)y;
		depthPoints[i] = d;
        
        // Get Depths
		int idx = y*cDepthWidth + x;
		depthValues[i] = m_pDepthArray16U[idx];
    }

    CameraSpacePoint* cameraPoints = new CameraSpacePoint[numDepthPoints];

    HRESULT hr;
    hr = m_pCoordinateMapper->MapDepthPointsToCameraSpace(numDepthPoints, depthPoints, 
    numDepthPoints, depthValues, numDepthPoints, cameraPoints);
    
    if (SUCCEEDED(hr))
	{		
		for (int i = 0; i < numDepthPoints; i++)
		{       
            cameraCoords[i] = cameraPoints[i].X;
            cameraCoords[i+size] = cameraPoints[i].Y;
            cameraCoords[i+size+size] = cameraPoints[i].Z;
        }
    }    
	else
    {
        cameraCoords[0] = 0;
		cameraCoords[1] = 0;
        cameraCoords[2] = 0;
		mexPrintf("Mapping error.\n");
    }
    
    delete[] depthPoints; depthPoints = NULL;
	delete[] depthValues; depthValues = NULL;
	delete[] cameraPoints; cameraPoints = NULL;    
} // end mapDepthPoints2Camera

bool Kin2::mapDepthFrame2Color(ColorSpacePoint* depth2ColorMapping)
{
	// Create coordinate mapping from depth to color
	HRESULT hr;
	hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(cDepthWidth * cDepthHeight,
		(UINT16*)m_pDepthArray16U, cDepthWidth * cDepthHeight, depth2ColorMapping);

	if (SUCCEEDED(hr)) return true;
	else return false;
} // end mapDepthFrame2Color


///////// Function: mapColorPoint2Depth //////////////////////////////
// Map points in color coordinates to points in depth coordinates
///////////////////////////////////////////////////////////////////////////
void Kin2::mapColorPoints2Depth(double colorCoords[], int size, UINT16 depthCoords[])
{
    // create heap storage for the coordinate mapping from color to depth
	DepthSpacePoint*  pColor2Depth = new DepthSpacePoint[cColorWidth * cColorHeight];

	// Get the mapping from color to depth
	HRESULT hr;
	hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(cDepthWidth * cDepthHeight,
		(UINT16*)m_pDepthArray16U, cColorWidth * cColorHeight, pColor2Depth);

	if (SUCCEEDED(hr))
	{
        int x, y, idx, depthX, depthY;

        for(int i=0; i<size; i++)
        {
            x = (int)colorCoords[i];
            y = (int)colorCoords[i+size];
            idx = y*cColorWidth + x;

            // Map high res color coordinates to depth coordinates
            depthX = (int)pColor2Depth[y*cColorWidth + x].X;
            depthY = (int)pColor2Depth[y*cColorWidth + x].Y;

            depthCoords[i] = depthX;
            depthCoords[i+size] = depthY;
        }
	}
	else
    {
        depthCoords[0] = 0;
		depthCoords[1] = 0;
		mexPrintf("No valid mapping. Call updateColor2DepthMapping first\n");
    }
    
    delete[] pColor2Depth;
	pColor2Depth = NULL;
} // end mapColorPoints2Depth

///////// Function: mapColorPoint2Camera //////////////////////////////
// Map points in color coordinates to points in camera coordinates
///////////////////////////////////////////////////////////////////////////
void Kin2::mapColorPoints2Camera(double colorCoords[], int size, double cameraCoords[])
{
    // create heap storage for the coordinate mapping from color to camera
	CameraSpacePoint* pColor2Camera = new CameraSpacePoint[cColorWidth * cColorHeight];

	// Get the mapping from color to camera
	HRESULT hr;
	hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(cDepthWidth * cDepthHeight,
		(UINT16*)m_pDepthArray16U, cColorWidth * cColorHeight, pColor2Camera);

	if (SUCCEEDED(hr))
	{
        int x, y, idx;
        double camX, camY, camZ;

        for(int i=0; i<size; i++)
        {
            x = (int)colorCoords[i];
            y = (int)colorCoords[i+size];
            idx = y*cColorWidth + x;

            // Map high res color coordinates to camera coordinates
            camX = pColor2Camera[idx].X;
            camY = pColor2Camera[idx].Y;
            camZ = pColor2Camera[idx].Z;

            cameraCoords[i] = camX;
            cameraCoords[i+size] = camY;
            cameraCoords[i+size+size] = camZ;
        }
	}
	else
    {
        cameraCoords[0] = 0;
		cameraCoords[1] = 0;
        cameraCoords[2] = 0;
		mexPrintf("Color2Camera mapping error\n");
    }
    
    delete[] pColor2Camera;
	pColor2Camera = NULL;
} // end mapColorPoints2Camera

void Kin2::mapCameraPoints2Depth(double cameraCoords[], int size, UINT16 depthCoords[])
{
    int numPoints = size;
    
    CameraSpacePoint* camPoints = new CameraSpacePoint[numPoints];

    CameraSpacePoint c;
    
    for(int i=0; i<size; i++)
    {
        c.X = (float)cameraCoords[i];
        c.Y = (float)cameraCoords[i+size];
        c.Z = (float)cameraCoords[i+size+size];

		camPoints[i] = c;
    }

	DepthSpacePoint* depthPoints = new DepthSpacePoint[numPoints];

    HRESULT hr;
    hr = m_pCoordinateMapper->MapCameraPointsToDepthSpace(numPoints, 
            camPoints, numPoints, depthPoints);
    
    if (SUCCEEDED(hr))
	{		
		for (int i = 0; i < numPoints; i++)
		{       
            depthCoords[i] = (UINT16)depthPoints[i].X;
            depthCoords[i+size] = (UINT16)depthPoints[i].Y;
        }
    }    
	else
    {
        depthCoords[0] = 0;
		depthCoords[1] = 0;
		mexPrintf("Camera to Depth Mapping error.\n");
    }
    
    delete[] depthPoints; depthPoints = NULL;
	delete[] camPoints; camPoints = NULL;    
} // end mapCameraPoints2Depth

void Kin2::mapCameraPoints2Color(double cameraCoords[], int size, UINT16 colorCoords[])
{
    int numPoints = size;
    
    CameraSpacePoint* camPoints = new CameraSpacePoint[numPoints];

    CameraSpacePoint c;
    
    for(int i=0; i<size; i++)
    {
        c.X = (float)cameraCoords[i];
        c.Y = (float)cameraCoords[i+size];
        c.Z = (float)cameraCoords[i+size+size];
        
		camPoints[i] = c;
    }

	ColorSpacePoint* colorPoints = new ColorSpacePoint[numPoints];

    HRESULT hr;
    hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(numPoints, 
            camPoints, numPoints, colorPoints);
    
    if (SUCCEEDED(hr))
	{		
		for (int i = 0; i < numPoints; i++)
		{       
            colorCoords[i] = (UINT16)colorPoints[i].X;
            colorCoords[i+size] = (UINT16)colorPoints[i].Y;
        }
    }    
	else
    {
        colorCoords[0] = 0;
		colorCoords[1] = 0;
		mexPrintf("Camera to Color Mapping error.\n");
    }
    
    delete[] colorPoints; colorPoints = NULL;
	delete[] camPoints; camPoints = NULL; 
} // end mapCameraPoints2Color    

void Kin2::getBodies(std::vector<std::vector<Joint> >& bodiesJoints, 
        std::vector<HandState>& lhs, std::vector<HandState>& rhs)
{
	HRESULT hr;

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
				HandState leftHandState = HandState_Unknown;
				HandState rightHandState = HandState_Unknown;

				pBody->get_HandLeftState(&leftHandState);
				pBody->get_HandRightState(&rightHandState);

				hr = pBody->GetJoints(_countof(joints), joints);

				// Copy array of Joints to vector of joints
				std::vector<Joint> vJoints(std::begin(joints), std::end(joints));

				if (SUCCEEDED(hr))
				{
					bodiesJoints.push_back(vJoints);
					lhs.push_back(leftHandState);
					rhs.push_back(rightHandState);
				}
			} // if body tracked
		} // if body detected
	} // for each body
}// end getBodies

/***********************************************************************/
/********************  Face Processing functions *************************/
/***********************************************************************/
void Kin2::ProcessFaces()
{
	HRESULT hr;
	m_facesData.clear();

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
			// If face tracked, save its data on the m_facesData structure array
			if (bFaceTracked)
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				//RectI faceBox = { 0 };
				//PointF facePoints[FacePointType::FacePointType_Count];
				//Vector4 faceRotation;
				//DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
				//D2D1_POINT_2F faceTextLayout;

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

					m_facesData.push_back(faceData);
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

void Kin2::getFaces(std::vector<k2::FaceData>& facesData)
{
	facesData = m_facesData;
}

void Kin2::ProcessHDFaces()
{
	HRESULT hr;
	m_HDfacesData.clear();

	// iterate through each HD face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IHighDefinitionFaceFrame * pHDFaceFrame = nullptr;
		
		hr = m_pHDFaceFrameReaders[iFace]->AcquireLatestFrame(&pHDFaceFrame);

		//std::cout << "HD Face frame acquired\n";

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pHDFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pHDFaceFrame->get_IsTrackingIdValid(&bFaceTracked);

			//std::cout << "HD Face tracking valid\n";
		}

		// If face tracked, save its data on the m_facesData structure array
		if (bFaceTracked)
		{
			//std::cout << "HD Face: " << iFace << " is tracked\n";				
			
			IFaceAlignment* pFaceAlignment = nullptr;
			hr = CreateFaceAlignment(&pFaceAlignment);			
			//UINT32 vertexCount;
			IFaceModel *pFaceModel = nullptr;
			
			// Here we save the HD face data
			k2::HDFaceData faceData;

			if (SUCCEEDED(hr))
				hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(pFaceAlignment);

			// Get the Animation units
			if (SUCCEEDED(hr))
				hr = pFaceAlignment->GetAnimationUnits(FaceShapeAnimations_Count, m_pAnimationUnits);

			if (SUCCEEDED(hr))
			{
				for (int vi = 0; vi < FaceShapeAnimations_Count; vi++)
					faceData.animationUnits[vi] = m_pAnimationUnits[vi];
			}

			// Get the face model
			if (SUCCEEDED(hr))
			{
				hr = CreateFaceModel(1, FaceShapeDeformations_Count, m_pShapeUnits, &pFaceModel);					

				//GetFaceModelVertexCount(&vertexCount);
				//vertices = new CameraSpacePoint[vertexCount];
			}

			if (SUCCEEDED(hr))
				hr = pFaceModel->CalculateVerticesForAlignment(pFaceAlignment, 1347, m_pFaceModelVertices);

			if (SUCCEEDED(hr))
			{
				for (int vi = 0; vi < 1347; vi++)
					faceData.faceModel[vi] = m_pFaceModelVertices[vi];
			}

			// Get the shape units 
			if (SUCCEEDED(hr))
				hr = pFaceModel->GetFaceShapeDeformations(FaceShapeDeformations_Count, m_pShapeUnits);

			if (SUCCEEDED(hr))
			for (int vi = 0; vi < FaceShapeDeformations_Count; vi++)
				faceData.shapeUnits[vi] = m_pShapeUnits[vi];
				
			// Get the facebox
			if (SUCCEEDED(hr))
				hr = pFaceAlignment->get_FaceBoundingBox(&faceData.faceBox);

			// Get the face rotation
			if (SUCCEEDED(hr))
				hr = pFaceAlignment->get_FaceOrientation(&faceData.faceRotation);

			// Get the head pivot
			if (SUCCEEDED(hr))
			{
				hr = pFaceAlignment->get_HeadPivotPoint(&faceData.headPivot);
			}
				
			// Save the HD face data in the member variable m_HDfacesData
			m_HDfacesData.push_back(faceData);
			SafeRelease(pFaceAlignment);
			SafeRelease(pFaceModel);

			/*if (pDeformations)
			{
				delete[] pDeformations;
				pDeformations = NULL;
			}
			if (pAnimationUnits)
			{
				delete[] pAnimationUnits;
				pAnimationUnits = NULL;
			}
			if (vertices)
			{
				delete[] vertices;
				vertices = NULL;
			}*/
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
							m_pHDFaceFrameSources[iFace]->put_TrackingId(bodyTId);
						}
					}
				}
			}
		}

		SafeRelease(pHDFaceFrame);
		
	}
}

void Kin2::getHDFaces(std::vector<k2::HDFaceData>& facesData)
{
	facesData = m_HDfacesData;
}


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
    
    mexPrintf("voxelsPerMeter: %d\n",voxelsPerMeter);
    mexPrintf("voxelsX: %d\n",voxelsX);
    mexPrintf("voxelsY: %d\n",voxelsY);
    mexPrintf("voxelsZ: %d\n",voxelsZ);
    mexPrintf("processorType: %d\n",processorType);

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

void extractRotationInDegrees(Vector4& pQuaternion, double& dPitch, double& dYaw, double& dRoll)
{
	double x = pQuaternion.x;
	double y = pQuaternion.y;
	double z = pQuaternion.z;
	double w = pQuaternion.w;

	// convert face rotation quaternion to Euler angles in degrees		
	dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / 3.141593 * 180.0;
	dYaw = asin(2 * (w * y - x * z)) / 3.141593 * 180.0;
	dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / 3.141593 * 180.0;
}

    
///////// Function: mexFunction ///////////////////////////////////////////
// Provides the interface of Matlab code with C++ code
///////////////////////////////////////////////////////////////////////////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{   
    // Get the command string
    char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    {
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
        return;
    }
        
    // New
    if (!strcmp("new", cmd)) 
    {
        // Check output parameters
        if (nlhs != 1)
            mexErrMsgTxt("New: One output expected.");
        
        // Check input parameters        
        if(nrhs < 2)
        {
            mexErrMsgTxt("You must specify at least one video source");
            return;
        }
        
        // Get input parameter (flags)
        unsigned short int *pr;
        pr = (unsigned short int *)mxGetData(prhs[1]);         
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<Kin2>(new Kin2(*pr));
               
        return;
    }
    
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<Kin2>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    Kin2 *Kin2_instance = convertMat2Ptr<Kin2>(prhs[1]);
    
    // Call the Kin2 methods
    
    // updateData method   
    if (!strcmp("updateData", cmd)) 
    {        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("updateData: Unexpected arguments.");
              
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
        INT8 *valid = (INT8*)mxGetPr(plhs[0]);
        
        // Call the class function
        Kin2_instance->updateData(valid);
        
        return;
    }
    
    // getDepth method
    if (!strcmp("getDepth", cmd)) 
    {        
         UINT16 *depth; // pointer to output data
         int depthDim[2]={424,512};
         int invalidDepth[2] = {0,0};
         
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getDepth: Unexpected arguments.");
        
        // Call the method
        plhs[0] = mxCreateNumericArray(2, depthDim, mxUINT16_CLASS, mxREAL);
        
        // Assign pointers to the output parameters
        depth = (UINT16*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validDepth;
        Kin2_instance->getDepth(depth,validDepth);
        
        if(!validDepth)
            plhs[0] = mxCreateNumericArray(2, invalidDepth, mxUINT16_CLASS, mxREAL);
        
        return;
    }
    
    // getColor method
    if (!strcmp("getColor", cmd)) 
    {        
        unsigned char *rgbImage;    // pointer to output data
        int colorDim[3]={1080,1920,3};
        int invalidColor[3] = {0,0,0};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getColor: Unexpected arguments.");
        
        // Call the method
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        
        // Assign pointers to the output parameters
        rgbImage = (unsigned char*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validColor;
        Kin2_instance->getColor(rgbImage,validColor);
        
        if(!validColor)
            plhs[0] = mxCreateNumericArray(3, invalidColor, mxUINT8_CLASS, mxREAL);
        
        return;
    }
    
    // getInfrared method
    if (!strcmp("getInfrared", cmd)) 
    {        
        UINT16 *infrared;   // pointer to output data
        int infraredDim[2]={424,512};
        int invalidInfrared[2] = {0,0};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getInfrared: Unexpected arguments.");
        
        // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, infraredDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        infrared = (UINT16*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validInfrared;
        Kin2_instance->getInfrared(infrared, validInfrared);
        
        if(!validInfrared)
            plhs[0] = mxCreateNumericArray(2, invalidInfrared, mxUINT16_CLASS, mxREAL);
        
        return;
    }
    
    // getBodyIndex method
    if (!strcmp("getBodyIndex", cmd)) 
    {        
        BYTE *bodyIndex;   // pointer to output data
        int bodyIDim[2]={424,512};
        int invalidBodyI[2] = {0,0};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getBodyIndex: Unexpected arguments.");
        
        // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, bodyIDim, mxUINT8_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        bodyIndex = (BYTE*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validBodyIndex;
        Kin2_instance->getBodyIndex(bodyIndex, validBodyIndex);
        
        if(!validBodyIndex)
            plhs[0] = mxCreateNumericArray(2, invalidBodyI, mxUINT8_CLASS, mxREAL);
        
        return;
    }
    
    // getPointCloud method
    if (!strcmp("getPointCloud", cmd)) 
    {        
        int depthWidth  = 512;
        int depthHeight = 424;
        
         // Prepare output array
        double *pointCloud;   // pointer to output data
        int size = depthWidth * depthHeight;
        int outDim[2]={size,3};    // three values (row vector)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        pointCloud = (double*)mxGetPr(plhs[0]);     
        
        int invalidData[3] = {0,0,0};
         
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getPointCloud: Unexpected arguments.");
      
        // Call the class function
        bool validData;
        Kin2_instance->getPointCloud(pointCloud,size,validData);
        
        if(!validData)
            plhs[0] = mxCreateNumericArray(2, invalidData, mxDOUBLE_CLASS, mxREAL);
        
        return;
 
    }
    
    // mapDepthPoints2Color method
    if (!strcmp("mapDepthPoints2Color", cmd)) 
    { 
        // Check input parameters        
        if(nrhs < 3)
        {
            mexErrMsgTxt("You must specify the input depth coordinates");
            return;
        }
        
         // Get input parameter (depth coordinates)
        double *depthCoords;
        depthCoords = (double*)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]);       
        
        // Prepare output array
        UINT16 *colorCoordinates;   // pointer to output data
        int outDim[2]={*size,2};        // two values (row vector)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        colorCoordinates = (UINT16*)mxGetPr(plhs[0]);     
        
        // call the class method
        Kin2_instance->mapDepthPoints2Color(depthCoords,*size,colorCoordinates);
               
        return;
    }
    
    // mapDepthFrame2Color method
//    if (!strcmp("mapDepthFrame2Color", cmd)) 
//    {      
        /*
         // Get input parameter (depth coordinates)
        double *depthCoords;
        depthCoords = (double*)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]);       
        
        // Prepare output array
        UINT16 *colorCoordinates;   // pointer to output data
        int outDim[2]={*size,2};        // two values (row vector)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        colorCoordinates = (UINT16*)mxGetPr(plhs[0]);     
        */
//        ColorSpacePoint* pDepth2Color = new ColorSpacePoint[217088];
        
        // call the class method
//        bool resutl = Kin2_instance->mapDepthFrame2Color();
               
//        return;
//    }
    
    // mapDepthPoints2Camera method
    if (!strcmp("mapDepthPoints2Camera", cmd)) 
    { 
        // Check input parameters        
        if(nrhs < 3)
        {
            mexErrMsgTxt("You must specify the input depth coordinates");
            return;
        }
        
         // Get input parameter (depth coordinates)
        double *depthCoords;
        depthCoords = (double*)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]);       
        
        // Prepare output array
        double *cameraCoordinates;   // pointer to output data
        int outDim[2]={*size,3};    // three values (row vector)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        cameraCoordinates = (double*)mxGetPr(plhs[0]);     
        
        // call the class method
        Kin2_instance->mapDepthPoints2Camera(depthCoords,*size,cameraCoordinates);
               
        return;
    }
    
    // mapColorPoints2Depth method
    if (!strcmp("mapColorPoints2Depth", cmd)) 
    { 
        // Check input parameters        
        if(nrhs < 3)
        {
            mexErrMsgTxt("You must specify the input color coordinates");
            return;
        }
        
         // Get input parameter (color coordinates)
        double *colorCoords;
        colorCoords = (double *)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]); 
        
        // Prepare output array
        UINT16 *depthCoordinates;   // pointer to output data
        int outDim[2]={*size,2};    // two values (row vectors)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        depthCoordinates = (UINT16*)mxGetPr(plhs[0]);     
        
        // call the class method
        Kin2_instance->mapColorPoints2Depth(colorCoords, *size, depthCoordinates);
               
        return;
    }
    
    // mapColorPoints2Camera method
    if (!strcmp("mapColorPoints2Camera", cmd)) 
    { 
        // Check input parameters        
        if(nrhs < 3)
        {
            mexErrMsgTxt("You must specify the input color coordinates");
            return;
        }
        
         // Get input parameter (color coordinates)
        double *colorCoords;
        colorCoords = (double *)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]); 
        
        // Prepare output array
        double *cameraCoordinates;   // pointer to output data
        int outDim[2]={*size,3};    // three values (row vectors)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        cameraCoordinates = (double*)mxGetPr(plhs[0]);     
        
        // call the class method
        Kin2_instance->mapColorPoints2Camera(colorCoords, *size, cameraCoordinates);
               
        return;
    }
    
    // mapCameraPoints2Depth method
    if (!strcmp("mapCameraPoints2Depth", cmd)) 
    { 
        // Check input parameters        
        if(nrhs < 3)
        {
            mexErrMsgTxt("You must specify the input in camera coordinates");
            return;
        }
        
         // Get input parameter (color coordinates)
        double *cameraCoords;
        cameraCoords = (double *)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]); 
        
        // Prepare output array
        UINT16 *depthCoords;        // pointer to output data
        int outDim[2]={*size,2};    // two values (row vectors)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        depthCoords = (UINT16*)mxGetPr(plhs[0]);     
        
        // call the class method
        Kin2_instance->mapCameraPoints2Depth(cameraCoords, *size, depthCoords);
               
        return;
    }
    
    // mapCameraPoints2Color method
    if (!strcmp("mapCameraPoints2Color", cmd)) 
    { 
        // Check input parameters        
        if(nrhs < 3)
        {
            mexErrMsgTxt("You must specify the input in camera coordinates");
            return;
        }
        
         // Get input parameter (color coordinates)
        double *cameraCoords;
        cameraCoords = (double *)mxGetData(prhs[2]); 
        
        // Get input parameter (size)
        int *size;
        size = (int*)mxGetData(prhs[3]); 
        
        // Prepare output array
        UINT16 *colorCoords;   // pointer to output data
        int outDim[2]={*size,2};    // two values (row vectors)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        colorCoords = (UINT16*)mxGetPr(plhs[0]);     
        
        // call the class method
        Kin2_instance->mapCameraPoints2Color(cameraCoords, *size, colorCoords);
               
        return;
    }
    
    // getBodies method
    if (!strcmp("getBodies", cmd)) 
    {
        //Assign field names
        const char *field_names[] = {"Position", "TrackingState",
                                "LeftHandState","RightHandState"};                
        
        
        // And for the second output parameter
        //plhs[1] = mxCreateNumericArray(1, numBodiesDim, mxINT32_CLASS, mxREAL);
        //numBodies = (int*)mxGetPr(plhs[1]);
        
        // Call the class function
        std::vector<std::vector<Joint> > bodiesJoints;
        std::vector<HandState> lhs;
        std::vector<HandState> rhs;
                
        Kin2_instance->getBodies(bodiesJoints,lhs,rhs);
        
        int bodiesSize = bodiesJoints.size();
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, bodiesSize};
        plhs[0] = mxCreateStructArray(2,dims,4,field_names);
        
        // Copy number of bodies to output variable
        //numBodies[0] = (int)bodiesJoints.size();
                
        // Copy the body data to the output matrices
        for(unsigned int i=0; i < bodiesJoints.size(); i++)
        {
            // output data
            mxArray *position,*trackingState, *leftHandState, *rightHandState;
            
            //Create mxArray data structures to hold the data
            //to be assigned for the structure.
            position  = mxCreateDoubleMatrix(3,25,mxREAL);
            double* posptr = (double*)mxGetPr(position);
            int trackStateDim[2]={1,25};
            trackingState  = mxCreateNumericArray(2, trackStateDim, mxINT32_CLASS, mxREAL);
            int* trackStatePtr = (int*)mxGetPr(trackingState);
            int handsDim[1] = {1};
            leftHandState = mxCreateNumericArray(1, handsDim, mxINT32_CLASS, mxREAL);
            rightHandState = mxCreateNumericArray(1, handsDim, mxINT32_CLASS, mxREAL);
            int *lhsptr = (int*)mxGetPr(leftHandState);
            int *rhsptr = (int*)mxGetPr(rightHandState);
        
            // Get next body
            std::vector<Joint> curBody = bodiesJoints[i];
            
            // For each joint
            for(int j=0; j<JointType_Count; j++)
            {
                // Copy joints position to output matrix 
                posptr[j*3] = curBody[j].Position.X;
                posptr[j*3 + 1] = curBody[j].Position.Y;
                posptr[j*3 + 2] = curBody[j].Position.Z; 
                
                // Copy joints tracking state to output matrix
                trackStatePtr[j] = curBody[j].TrackingState;
            }
            
            lhsptr[0] = (int)lhs[i];
            rhsptr[0] = (int)rhs[i];
               
            //Assign the output matrices to the struct
            mxSetFieldByNumber(plhs[0],i,0, position);
            mxSetFieldByNumber(plhs[0],i,1, trackingState);
            mxSetFieldByNumber(plhs[0],i,2, leftHandState);
            mxSetFieldByNumber(plhs[0],i,3, rightHandState);
        }
        
        return;
    }
    
    // getFaces method
    if (!strcmp("getFaces", cmd)) 
    {
        //Assign field names
        const char *field_names[] = {"FaceBox", "FacePoints",
                                "FaceRotation","FaceProperties"};  
                                
        std::vector<k2::FaceData> facesData;
        
        // call the class method
        Kin2_instance->getFaces(facesData);
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, facesData.size()};
        plhs[0] = mxCreateStructArray(2,dims,4,field_names);
        
        // Copy the faces data to the output matrices
        for(unsigned int i=0; i < facesData.size(); i++)
        {
            // output data
            mxArray *faceBox,*facePoints, *faceRotation, *faceProperties;
            
            //Create mxArray data structures to hold the data
            //to be assigned for the structure.
            faceBox  = mxCreateDoubleMatrix(1,4,mxREAL);
            double* faceboxptr = (double*)mxGetPr(faceBox);
            
            facePoints  = mxCreateDoubleMatrix(2,5,mxREAL);
            double* facepointsptr = (double*)mxGetPr(facePoints);
            
            faceRotation  = mxCreateDoubleMatrix(1,3,mxREAL);
            double* facerotationptr = (double*)mxGetPr(faceRotation);
            
            int facePropsDim[2]={1,FaceProperty::FaceProperty_Count};
            faceProperties  = mxCreateNumericArray(2, facePropsDim, mxINT32_CLASS, mxREAL);
            int* facepropsptr = (int*)mxGetPr(faceProperties);
  
            // Get next face
            k2::FaceData curFace = facesData[i];
            
            // Get facebox position
            faceboxptr[0] = curFace.faceBox.Left;
            faceboxptr[1] = curFace.faceBox.Top;
            faceboxptr[2] = curFace.faceBox.Right;
            faceboxptr[3] = curFace.faceBox.Bottom;
                    
            // Get face points
            for(int j=0; j<FacePointType::FacePointType_Count; j++)
            {               
                // Copy joints position to output matrix 
                facepointsptr[j*2] = curFace.facePoints[j].X;
                facepointsptr[j*2 + 1] = curFace.facePoints[j].Y;                 
            }
            
            // Get face rotation
            double pitch, yaw, roll;
            extractRotationInDegrees(curFace.faceRotation, pitch, yaw, roll);
            facerotationptr[0] = pitch;
            facerotationptr[1] = yaw;
            facerotationptr[2] = roll;
            
            // Get face properties
            for(int j=0; j<FaceProperty::FaceProperty_Count; j++)
            {               
                // Copy joints position to output matrix 
                facepropsptr[j] = (int)curFace.faceProperties[j];                
            }
               
            //Assign the output matrices to the struct
            mxSetFieldByNumber(plhs[0],i,0, faceBox);
            mxSetFieldByNumber(plhs[0],i,1, facePoints);
            mxSetFieldByNumber(plhs[0],i,2, faceRotation);
            mxSetFieldByNumber(plhs[0],i,3, faceProperties);
        }
                
        return;
    }
    
    // getHDFaces method
    if (!strcmp("getHDFaces", cmd)) 
    {
        //Assign field names
        const char *field_names[] = {"FaceBox", "FaceRotation",
                                "HeadPivot","AnimationUnits","ShapeUnits","FaceModel"};  
                                
        std::vector<k2::HDFaceData> facesData;
        
        // call the class method
        Kin2_instance->getHDFaces(facesData);
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, facesData.size()};
        plhs[0] = mxCreateStructArray(2,dims,6,field_names);
        
        // Copy the faces data to the output matrices
        for(unsigned int i=0; i < facesData.size(); i++)
        {
            // output data
            mxArray *faceBox, *faceRotation, *headPivot, *animationUnits;
            mxArray *shapeUnits, *faceModel;
            
            //Create mxArray data structures to hold the data
            //to be assigned for the structure.
            faceBox  = mxCreateDoubleMatrix(1,4,mxREAL);
            double* faceBoxPtr = (double*)mxGetPr(faceBox);
            
            faceRotation  = mxCreateDoubleMatrix(1,3,mxREAL);
            double* faceRotationPtr = (double*)mxGetPr(faceRotation);
            
            headPivot  = mxCreateDoubleMatrix(1,3,mxREAL);
            double* headPivotPtr = (double*)mxGetPr(headPivot);
            
            animationUnits  = mxCreateDoubleMatrix(1,FaceShapeAnimations_Count,mxREAL);
            double* animationUnitsPtr = (double*)mxGetPr(animationUnits);
            
            shapeUnits  = mxCreateDoubleMatrix(1,FaceShapeDeformations_Count,mxREAL);
            double* shapeUnitsPtr = (double*)mxGetPr(shapeUnits);
            
            faceModel  = mxCreateDoubleMatrix(3,1347,mxREAL);
            double* faceModelPtr = (double*)mxGetPr(faceModel);
  
            // Get next face
            k2::HDFaceData curFace = facesData[i];
            
            // Get facebox position
            faceBoxPtr[0] = curFace.faceBox.Left;
            faceBoxPtr[1] = curFace.faceBox.Top;
            faceBoxPtr[2] = curFace.faceBox.Right;
            faceBoxPtr[3] = curFace.faceBox.Bottom;
                              
            // Get face rotation
            double pitch, yaw, roll;
            extractRotationInDegrees(curFace.faceRotation, pitch, yaw, roll);
            faceRotationPtr[0] = pitch;
            faceRotationPtr[1] = yaw;
            faceRotationPtr[2] = roll;
            
            // Get head pivot
            headPivotPtr[0] = curFace.headPivot.X;
            headPivotPtr[1] = curFace.headPivot.Y;
            headPivotPtr[2] = curFace.headPivot.Z;
            
            // Get animation units
            for(int j=0; j<FaceShapeAnimations_Count; j++)
                animationUnitsPtr[j] = curFace.animationUnits[j];
            
            // Get shape units
            for(int j=0; j<FaceShapeDeformations_Count; j++)
                shapeUnitsPtr[j] = curFace.shapeUnits[j];
            
            // Get face points
            for(int j=0; j<1347; j++)
            {               
                // Copy joints position to output matrix 
                faceModelPtr[j*3] = curFace.faceModel[j].X;
                faceModelPtr[j*3 + 1] = curFace.faceModel[j].Y;
                faceModelPtr[j*3 + 2] = curFace.faceModel[j].Z;
            }
               
            //Assign the output matrices to the struct
            mxSetFieldByNumber(plhs[0],i,0, faceBox);
            mxSetFieldByNumber(plhs[0],i,1, faceRotation);
            mxSetFieldByNumber(plhs[0],i,2, headPivot);
            mxSetFieldByNumber(plhs[0],i,3, animationUnits);
            mxSetFieldByNumber(plhs[0],i,4, shapeUnits);
            mxSetFieldByNumber(plhs[0],i,5, faceModel);
        }
                
        return;
    }
    
    // KF_init method
    if (!strcmp("KF_init", cmd)) 
    {       
        int voxelsPerMeter;
        int voxelsX, voxelsY, voxelsZ;
        bool processorType;
        
        mexPrintf("nrhs:%d\n",nrhs);
        if(nrhs > 2)
        {        
            // Get voxels per meter input parameter            
            voxelsPerMeter = (int)mxGetScalar(prhs[2]); 
        }
        else
            voxelsPerMeter = 64;
        
        if(nrhs > 3)
        {        
            // Get voxels X input parameter            
            voxelsX = (int)mxGetScalar(prhs[3]); 
        }
        else
            voxelsX = 256;
        
        if(nrhs > 4)
        {        
            // Get voxels Y input parameter
            voxelsY = (int)mxGetScalar(prhs[4]); 
        }
        else
            voxelsY = 256;
        
        if(nrhs > 5)
        {        
            // Get voxels Z input parameter
            voxelsZ = (int)mxGetScalar(prhs[4]); 
        }
        else
            voxelsZ = 256;
        
        if(nrhs > 6)
        {        
            // Get processorType input parameter
            processorType = (bool)mxGetScalar(prhs[5]); 
        }
        else
            processorType = true;
        
        // call the class method
        Kin2_instance->KF_init(voxelsPerMeter,voxelsX,voxelsY,voxelsZ,processorType);
               
        return;
    }
    
    // KF_update method
    if (!strcmp("KF_update", cmd)) 
    {        
        Kin2_instance->KF_update();
        
        return;
    }
    
    // KF_getVolumeImage method
    if (!strcmp("KF_getVolumeImage", cmd)) 
    {        
        unsigned char *volImage;    // pointer to output data
        int dims[3]={424,512,3};
        
        // Call the method
        plhs[0] = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);
        
        // Assign pointers to the output parameters
        volImage = (unsigned char*)mxGetPr(plhs[0]);
      
        // Call the class function
        Kin2_instance->KF_getVolumeImage(volImage);
        
        return;
    }
    
    // KF_reset method
    if (!strcmp("KF_reset", cmd)) 
    {        
        Kin2_instance->KF_reset();
        
        return;
    }
    
    // KF_getMesh method
    if (!strcmp("KF_getMesh", cmd)) 
    {        
        // Output Structure field names
        const char *field_names[] = {"Vertices","TriangleIndices","Normals"};
                                
        INuiFusionMesh *ppMesh = NULL;
        HRESULT hr;
        hr = Kin2_instance->KF_getMesh(&ppMesh);
        
        int numMeshes = 0;
        int numVertices = 0;
        int numTriangleVertexIndices = 0;
        int numNormals = 0;
        const Vector3* pVertices = NULL;
        const Vector3* pNormals = NULL;
        const int* pTriangleVertexIndices;
        
        if (SUCCEEDED(hr))
        {            
            numMeshes = sizeof(ppMesh) / sizeof(*ppMesh);
            mexPrintf("Meshes:%d\n",numMeshes);	
        }        
        else
            mexPrintf("Error getting mesh\n");
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, numMeshes};
        plhs[0] = mxCreateStructArray(2,dims,3,field_names);

       // Copy the data to the output matrices
        for(unsigned int i=0; i < numMeshes; i++)
        {              
            numVertices = ppMesh->VertexCount();
            numTriangleVertexIndices = ppMesh->TriangleVertexIndexCount();
            numNormals = ppMesh->NormalCount();

            mexPrintf("Vertices:%d\n",numVertices);					

            hr = ppMesh->GetVertices(&pVertices);
            hr = ppMesh->GetNormals(&pNormals);
            hr = ppMesh->GetTriangleIndices(&pTriangleVertexIndices);    
            
            // output data
            mxArray *vertices,*triangleVertexIndices, *normals;
            
            //Create mxArray data structures to hold the data
            //to be assigned for the structure.
            vertices  = mxCreateDoubleMatrix(3,numVertices,mxREAL);
            double* vertptr = (double*)mxGetPr(vertices);
            int triangleVertIndSize[2]={1,numTriangleVertexIndices};
            triangleVertexIndices  = mxCreateNumericArray(2, triangleVertIndSize, mxINT32_CLASS, mxREAL);
            int* triVertexIndPtr = (int*)mxGetPr(triangleVertexIndices);
            normals  = mxCreateDoubleMatrix(3,numNormals,mxREAL);
            double* normalsptr = (double*)mxGetPr(normals);
            
            // For each vertex
            for(int j=0; j<numVertices; j++)
            {
                // Copy vertex to output matrix 
                vertptr[j*3] = pVertices[j].x;
                vertptr[j*3 + 1] = pVertices[j].y;
                vertptr[j*3 + 2] = pVertices[j].z;                 
            }
            
            // For each normal
            for(int j=0; j<numNormals; j++)
            {
                // Copy vertex to output matrix 
                normalsptr[j*3] = pNormals[j].x;
                normalsptr[j*3 + 1] = pNormals[j].y;
                normalsptr[j*3 + 2] = pNormals[j].z;                 
            }
            
            // For triangle index
            for(int j=0; j<numTriangleVertexIndices; j++)
            {
                // Copy vertex to output matrix 
                triVertexIndPtr[j] = pTriangleVertexIndices[j];                
            }           
               
            //Assign the output matrices to the struct
            mxSetFieldByNumber(plhs[0],i,0, vertices);
            mxSetFieldByNumber(plhs[0],i,1, triangleVertexIndices);
            mxSetFieldByNumber(plhs[0],i,2, normals);
        } // for each mesh			
        return;
    }
                       
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
