///////////////////////////////////////////////////////////////////////////
///		Kin2.h
///
///		Description: 
///			Kin2 class encapsulates the funtionality of Kinect2 Sensor.
///         It uses Kinect2 SDK from Microsoft.
///			Copyright (c) Microsoft Corporation.  All rights reserved.
///			
///         Define methods to:
///          * Initialize, and get images from the depth, color, and infrared cameras.
///          * Coordinate Mapping between cameras.
///          * Body tracking
///          * Face and HD face processing
///          * 3D reconstruction
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
///         Mar/25/2016: Update documentation
///         Mar/31/2016: Add floor clip plane to the body data
///////////////////////////////////////////////////////////////////////////
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
		std::vector<CameraSpacePoint> faceModel;

	}HDFaceData;
}

/*************************************************************************/
/************************** Kin2 Class ***********************************/
/*************************************************************************/
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
    
    /************ Data Sources *************/
    void updateData(INT8 valid[]);
    void getDepth(UINT16 depth[],INT64& time, bool& validDepth);
    void getColor(unsigned char rgbImage[], INT64& time, bool& validColor);
    void getInfrared(UINT16 infrared[], INT64& time, bool& validInfrared);
    void getBodyIndex(BYTE bodyIndex[],bool& validBodyIndex);
    void getPointCloud(double pointCloud[], unsigned char colors[], bool color, bool& validData);    
    void getDepthIntrinsics(CameraIntrinsics &intrinsics);
    
    /************ Mappings **************/
    void mapDepthPoints2Color(double depthCoords[], int size, UINT16 colorCoords[]);
    void mapDepthPoints2Camera(double depthCoords[], int size, double cameraCoords[]);
    //bool mapDepthFrame2Color(ColorSpacePoint* depth2ColorMapping);

	void mapColorPoints2Depth(double colorCoords[], int size, UINT16 depthCoords[]);
    void mapColorPoints2Camera(double colorCoords[], int size, double cameraCoords[]);
    
    void mapCameraPoints2Depth(double cameraCoords[], int size, UINT16 depthCoords[]);
    void mapCameraPoints2Color(double cameraCoords[], int size, UINT16 colorCoords[]);
    
    void alignColor2Depth(unsigned char alignedImage[], bool& validData);
    
    /************ Body Tracking *****************/
    void getBodies(std::vector<std::vector<Joint> >& bodiesJoints,
        std::vector<std::vector<JointOrientation> >& bodiesJointsOrientations,
        std::vector<HandState>& lhs, std::vector<HandState>& rhs, Vector4 &fcp, INT64& time);
    
    /************ Face Processing public functions *****************/
    void getFaces(std::vector<k2::FaceData>& facesData);
    void getHDFaces(bool withVertices, std::vector<k2::HDFaceData>& facesData);
    void buildHDFaceModels(int &collectionStatus, int &captureStatus);
    
    /*************** Kinect Fusion public functions ***************/
	void KF_init(int voxelsPerMeter = 64, int voxelsX = 256, int voxelsY = 256, int voxelsZ = 256, bool processorType = true, bool autoReset = true);
	void KF_update();
	void KF_getVolumeImage(BYTE volumeImg[]);
	void KF_reset();
	HRESULT KF_getMesh(INuiFusionMesh **ppMesh);
    
    /*************** Other methods ***************/
    void extractRotationInDegrees(Vector4& pQuaternion, double& dPitch, double& dYaw, double& dRoll);
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
    
    // Bodies floor clip plane
	Vector4		m_floorClipPlane;
    
    // Timestamps
    INT64       m_depthTimeStamp;
    INT64       m_colorTimeStamp;
    INT64       m_infraredTimeStamp;
    INT64       m_bodiesTimeStamp;
    
	// Face sources
	IFaceFrameSource*       m_pFaceFrameSources[BODY_COUNT];

	// Face readers
	IFaceFrameReader*       m_pFaceFrameReaders[BODY_COUNT];
    
    // HD Face sources
	IHighDefinitionFaceFrameSource* m_pHDFaceFrameSources[BODY_COUNT];

	// HDFace readers
	IHighDefinitionFaceFrameReader*	m_pHDFaceFrameReaders[BODY_COUNT];
    
    // HD Face models
    IFaceModelBuilder*		m_pFaceModelBuilder[BODY_COUNT];
	bool					m_faceModelReady[BODY_COUNT];
	bool					m_faceModelWarning[BODY_COUNT];		// keep track of the face model warning message
	IFaceAlignment*			m_pFaceAlignment[BODY_COUNT];
	IFaceModel*				m_pFaceModel[BODY_COUNT];	    
             
    // Flags of available data
    bool        m_newDepthData;
    bool        m_newColorData;
    bool        m_newInfraredData;
    bool        m_newBodyIndex;
    bool        m_newPointCloudData;
    
    // Initialization flags
    k2::Flags       m_flags;
    
    /************ Face Processing private functions *****************/

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
    
     /************************************************************/
	/******  Kinect Fusion variables and functions ******/
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
}; // Kin2 class definition