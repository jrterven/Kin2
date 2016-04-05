#include "Kin2.h"
#include <mex.h>
#include "class_handle.hpp"

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
         UINT16 *depth; // pointer to output data 0
         int depthDim[2]={424,512};
         int invalidDepth[2] = {0,0};
         int timeDim[2] = {1,1};
         
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getDepth: Unexpected arguments.");
        
        // Reserve space for output variables
        plhs[0] = mxCreateNumericArray(2, depthDim, mxUINT16_CLASS, mxREAL);
        depth = (UINT16*)mxGetPr(plhs[0]);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        INT64 *timeStamp = (INT64*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validDepth;
        Kin2_instance->getDepth(depth,*timeStamp,validDepth);
        
        if(!validDepth)
        {
            plhs[0] = mxCreateNumericArray(2, invalidDepth, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }
    
    // getColor method
    if (!strcmp("getColor", cmd)) 
    {        
        unsigned char *rgbImage;    // pointer to output data
        int colorDim[3]={1080,1920,3};
        int invalidColor[3] = {0,0,0};
        int timeDim[2] = {1,1};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getColor: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        INT64 *timeStamp = (INT64*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        rgbImage = (unsigned char*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validColor;
        Kin2_instance->getColor(rgbImage,*timeStamp,validColor);
        
        if(!validColor)
        {
            plhs[0] = mxCreateNumericArray(3, invalidColor, mxUINT8_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }
    
    // getInfrared method
    if (!strcmp("getInfrared", cmd)) 
    {        
        UINT16 *infrared;   // pointer to output data
        int infraredDim[2]={424,512};
        int invalidInfrared[2] = {0,0};
        int timeDim[2] = {1,1};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getInfrared: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(2, infraredDim, mxUINT16_CLASS, mxREAL); 
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        INT64 *timeStamp = (INT64*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        infrared = (UINT16*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validInfrared;
        Kin2_instance->getInfrared(infrared, *timeStamp, validInfrared);
        
        if(!validInfrared)
        {
            plhs[0] = mxCreateNumericArray(2, invalidInfrared, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
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
        
        // Get input parameter:
        // 0 = no color
        // 1 = with color
        int *withColor;
        bool bwithColor = false;
        withColor = (int*)mxGetData(prhs[2]);
        if(*withColor == 0)
            bwithColor = false;
        else
            bwithColor = true;
                
        // Prepare output arrays
        double *pointCloud;   // pointer to output data
        unsigned char *colors;
        int size = depthWidth * depthHeight;
        int outDim[2]={size,3};    // three values (row vector)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
        plhs[1] = mxCreateNumericArray(2, outDim, mxUINT8_CLASS, mxREAL);
        
        // Assign pointers to the output parameters
        pointCloud = (double*)mxGetPr(plhs[0]);   
        colors = (unsigned char*)mxGetPr(plhs[1]);
                
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getPointCloud: Unexpected arguments.");
      
        // Call the class function
        bool validData;
        Kin2_instance->getPointCloud(pointCloud,colors,bwithColor, validData);
        
        if(!validData)
        {
            plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
            plhs[1] = mxCreateNumericArray(2, outDim, mxUINT8_CLASS, mxREAL);
        }
        
        return;
 
    }    
    
    // getDepthIntrinsics method
    if (!strcmp("getDepthIntrinsics", cmd)) 
    { 
        //Assign field names
        const char *field_names[] = {"FocalLengthX", "FocalLengthY",
                                "PrincipalPointX","PrincipalPointY",
                                "RadialDistortionSecondOrder",
                                "RadialDistortionFourthOrder",
                                "RadialDistortionSixthOrder"};  
                                
        CameraIntrinsics intrinsics = {};
        
        // call the class method
        Kin2_instance->getDepthIntrinsics(intrinsics);
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, 1};
        plhs[0] = mxCreateStructArray(2,dims,7,field_names);
        
        // Copy the intrinsic parameters to the the output variables
        
        // I AM HERE
        // output data
        mxArray *fx, *fy, *ppx, *ppy, *rd2, *rd4, *rd6;

        //Create mxArray data structures to hold the data
        //to be assigned for the structure.
        fx  = mxCreateDoubleScalar(intrinsics.FocalLengthX);
        fy  = mxCreateDoubleScalar(intrinsics.FocalLengthY);
        ppx  = mxCreateDoubleScalar(intrinsics.PrincipalPointX);
        ppy  = mxCreateDoubleScalar(intrinsics.PrincipalPointY);
        rd2  = mxCreateDoubleScalar(intrinsics.PrincipalPointY);
        rd4  = mxCreateDoubleScalar(intrinsics.RadialDistortionFourthOrder);
        rd6  = mxCreateDoubleScalar(intrinsics.RadialDistortionSixthOrder);

        //Assign the output matrices to the struct
        mxSetFieldByNumber(plhs[0],0,0, fx);
        mxSetFieldByNumber(plhs[0],0,1, fy);
        mxSetFieldByNumber(plhs[0],0,2, ppx);
        mxSetFieldByNumber(plhs[0],0,3, ppy);
        mxSetFieldByNumber(plhs[0],0,4, rd2);
        mxSetFieldByNumber(plhs[0],0,5, rd4);
        mxSetFieldByNumber(plhs[0],0,6, rd6);
                
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
    /*
    if (!strcmp("mapDepthFrame2Color", cmd)) 
    {                                          
        // Prepare output array
        UINT16 *colorCoordinates;   // pointer to output data
        int outDim[2]={424,512,2};        // two values (row vector)
        int invalidData[3] = {0,0,0};
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(3, outDim, mxUINT16_CLASS, mxREAL); 
        
        // Assign pointers to the output parameters
        colorCoordinates = (UINT16*)mxGetPr(plhs[0]);             
        
        // call the class method
        bool result = Kin2_instance->mapDepthFrame2Color(colorCoordinates);
               
        if(!result)
        {
            plhs[0] = mxCreateNumericArray(3, invalidData, mxUINT16_CLASS, mxREAL);
            return
        }
        else
        {
            
        }
                
        return;
    }
     */
    
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
    
    // alignColor2Depth method
    if (!strcmp("alignColor2Depth", cmd)) 
    {
        unsigned char *alignedImage;    // pointer to output data
        int colorDim[3]={424,512,3};
        int invalidData[3] = {0,0,0};        
        
        // Call the method
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        
        // Assign pointers to the output parameters
        alignedImage = (unsigned char*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validData;
        Kin2_instance->alignColor2Depth(alignedImage,validData);
        
        if(!validData)
            plhs[0] = mxCreateNumericArray(3, invalidData, mxUINT8_CLASS, mxREAL);
        
        return;
    }
    
    // getBodies method
    if (!strcmp("getBodies", cmd)) 
    {
        //Assign field names
        const char *field_names[] = {"Position", "Orientation", "TrackingState",
                                "LeftHandState","RightHandState"};                
        
        // Get input parameter: Quat or Euler angles for orientation
        // 0 = Quaternion
        // 1 = Euler angles
        int *orientationType;
        orientationType = (int*)mxGetData(prhs[2]);        

        
        // Call the class function
        std::vector<std::vector<Joint> > bodiesJoints;
        std::vector<std::vector<JointOrientation> > bodiesJointsOrientations;
        std::vector<HandState> lhs;
        std::vector<HandState> rhs;
        Vector4 floorClipPlane; 
        INT64 time = 0;
                
        Kin2_instance->getBodies(bodiesJoints,bodiesJointsOrientations,lhs,rhs,floorClipPlane,time);
        
        int bodiesSize = bodiesJoints.size();
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, bodiesSize};
        plhs[0] = mxCreateStructArray(2,dims,5,field_names);
        
        // Allocate memory for the Floor clip plane output
        mwSize fcpSize[2] = {1, 4};
        plhs[1] = mxCreateNumericArray(2,fcpSize, mxDOUBLE_CLASS, mxREAL);
        double *fcp = (double*)mxGetPr(plhs[1]);
        fcp[0] = floorClipPlane.x;
        fcp[1] = floorClipPlane.y;
        fcp[2] = floorClipPlane.z;
        fcp[3] = floorClipPlane.w;
        
        // Allocate memory for the timestamp
        mwSize timeDim[2] = {1, 1};
        plhs[2] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        INT64 *timeStamp = (INT64*)mxGetPr(plhs[2]);
        
        if(bodiesJoints.size() > 0)
            timeStamp[0] = time;
        else
             timeStamp[0] = 0;
        
        // Copy number of bodies to output variable
        //numBodies[0] = (int)bodiesJoints.size();
                
        // Copy the body data to the output matrices
        for(unsigned int i=0; i < bodiesJoints.size(); i++)
        {
            // output data
            mxArray *position, *orientation, *trackingState, *leftHandState, *rightHandState;
            
            //Create mxArray data structures to hold the data
            //to be assigned for the structure.
            position  = mxCreateDoubleMatrix(3,25,mxREAL);
            double* posptr = (double*)mxGetPr(position);
            if(*orientationType == 0) // if quaternion
                orientation  = mxCreateDoubleMatrix(4,25,mxREAL);
            else if(*orientationType == 1) // if Euler angles
                orientation  = mxCreateDoubleMatrix(3,25,mxREAL);
            
            double* orientationptr = (double*)mxGetPr(orientation);
            int trackStateDim[2]={1,25};
            trackingState  = mxCreateNumericArray(2, trackStateDim, mxINT32_CLASS, mxREAL);
            int* trackStatePtr = (int*)mxGetPr(trackingState);
            int handsDim[1] = {1};
            leftHandState = mxCreateNumericArray(1, handsDim, mxINT32_CLASS, mxREAL);
            rightHandState = mxCreateNumericArray(1, handsDim, mxINT32_CLASS, mxREAL);
            int *lhsptr = (int*)mxGetPr(leftHandState);
            int *rhsptr = (int*)mxGetPr(rightHandState);
        
            // Get next body joints
            std::vector<Joint> curBody = bodiesJoints[i];
            // Get the body orientations
            std::vector<JointOrientation> curOrientation = bodiesJointsOrientations[i];
            
            // For each joint
            for(int j=0; j<JointType_Count; j++)
            {
                // Copy joints position to output matrix 
                posptr[j*3] = curBody[j].Position.X;
                posptr[j*3 + 1] = curBody[j].Position.Y;
                posptr[j*3 + 2] = curBody[j].Position.Z; 
                
                // Copy joints orientations to output matrix
                if(*orientationType == 0) // if Quaternion
                {
                    orientationptr[j*4] = curOrientation[j].Orientation.x;
                    orientationptr[j*4 + 1] = curOrientation[j].Orientation.y;
                    orientationptr[j*4 + 2] = curOrientation[j].Orientation.z; 
                    orientationptr[j*4 + 3] = curOrientation[j].Orientation.w; 
                }
                if(*orientationType == 1) // if Euler Angles
                {
                    double pitch, yaw, roll;
                    Kin2_instance->extractRotationInDegrees(curOrientation[j].Orientation, pitch, yaw, roll);
                    orientationptr[j*3] = pitch;
                    orientationptr[j*3 + 1] = yaw;
                    orientationptr[j*3 + 2] = roll; 
                }
                // Copy joints tracking state to output matrix
                trackStatePtr[j] = curBody[j].TrackingState;
            }
            
            lhsptr[0] = (int)lhs[i];
            rhsptr[0] = (int)rhs[i];
               
            //Assign the output matrices to the struct
            mxSetFieldByNumber(plhs[0],i,0, position);
            mxSetFieldByNumber(plhs[0],i,1, orientation);
            mxSetFieldByNumber(plhs[0],i,2, trackingState);
            mxSetFieldByNumber(plhs[0],i,3, leftHandState);
            mxSetFieldByNumber(plhs[0],i,4, rightHandState);
        } // for each body joint
        
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
            Kin2_instance->extractRotationInDegrees(curFace.faceRotation, pitch, yaw, roll);
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
        // Get input parameters
        int wVertices;  
        wVertices = (int)mxGetScalar(prhs[2]); 
        
        //Assign field names
        const char *field_names[] = {"FaceBox", "FaceRotation",
                                "HeadPivot","AnimationUnits","FaceModel"};  
                                
        std::vector<k2::HDFaceData> facesData;
        
        // call the class method
        bool withVertices = false;                
        if(wVertices) withVertices = true;
        
        Kin2_instance->getHDFaces(withVertices, facesData);
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, facesData.size()};
        plhs[0] = mxCreateStructArray(2,dims,5,field_names);
        
        // Copy the faces data to the output matrices
        for(unsigned int i=0; i < facesData.size(); i++)
        {
            // output data
            mxArray *faceBox, *faceRotation, *headPivot, *animationUnits;
            mxArray *faceModel;
            
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
            
            int numVertices = facesData[0].faceModel.size();
            faceModel  = mxCreateDoubleMatrix(3,numVertices,mxREAL);
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
            Kin2_instance->extractRotationInDegrees(curFace.faceRotation, pitch, yaw, roll);
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
            
            // Get face points
            for(int j=0; j<numVertices; j++)
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
            mxSetFieldByNumber(plhs[0],i,4, faceModel);
        }
                
        return;
    }
    
    // buildHDFaceModels method
    if (!strcmp("buildHDFaceModels", cmd)) 
    {
        // Create a 1-by-1 real integer
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
        plhs[1] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    
        int collectionStatus, captureStatus;
        Kin2_instance->buildHDFaceModels(collectionStatus, captureStatus);
        
        //mexPrintf("Collection Status: %d\n",collectionStatus);
        //mexPrintf("Capture Status: %d",captureStatus);
        
        // fill the output parameters
        int *outCollStatus = (int *) mxGetData(plhs[0]);
        outCollStatus[0] = collectionStatus;
        
        int *outCapStatus = (int *) mxGetData(plhs[1]);
        outCapStatus[0] = captureStatus;       
        
        return;
    }
    
    // KF_init method
    if (!strcmp("KF_init", cmd)) 
    {       
        int voxelsPerMeter;
        int voxelsX, voxelsY, voxelsZ;
        bool processorType;
        
        //mexPrintf("nrhs:%d\n",nrhs);
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