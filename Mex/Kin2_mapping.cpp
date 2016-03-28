#include "Kin2.h"
#include "mex.h"
#include "class_handle.hpp"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <Kinect.Face.h>
#include <vector>

///////// Function: getPointCloud ///////////////////////////////////////////
// Get camera points from depth frame and copy them to Matlab matrix
// You must call updateData first and have depth activated
///////////////////////////////////////////////////////////////////////////

void Kin2::getPointCloud(double pointCloud[], unsigned char colors[], bool color, bool& validData)
{   
    // Create coordinate mapping from depth to camera
	HRESULT hr;
	const int numDepthPoints = cDepthWidth * cDepthHeight;

    if(m_newPointCloudData)
    {
        CameraSpacePoint cameraPoints[numDepthPoints];	
        hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(numDepthPoints,
            (UINT16*)m_pDepthArray16U, numDepthPoints, cameraPoints);

        // If successfull depth to camera space mapping
        if (SUCCEEDED(hr))
        {
            // if the user want color
            if(color)
            {
                // map camera points to color space
                ColorSpacePoint colorPoints[numDepthPoints];
                hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(numDepthPoints, 
                    cameraPoints, numDepthPoints, colorPoints);
    
                // fill up the colors matrix with R,G,B values from the current color image
                if (SUCCEEDED(hr))
                {		
                    int colorCoordX, colorCoordY;
                    for (int i = 0; i < numDepthPoints; i++)
                    {                               
                        colorCoordX = (UINT16)colorPoints[i].X;
                        colorCoordY = (UINT16)colorPoints[i].Y;
                        
                        // Sample the RGB components from the color image
                        unsigned char R, G, B;
                        
                        // first make sure the coordinates maps to a valid point in color space
                        int colorX = (int)(floor(colorCoordX + 0.5));
                        int colorY = (int)(floor(colorCoordY + 0.5));
                        if ((colorX >= 0) && (colorX < cColorWidth) && (colorY >= 0) && (colorY < cColorHeight))
                        {
                            // calculate index into color array
                            int colorIndex = (colorX + (colorY * cColorWidth)) * 4;
                            
                            R = m_pColor[colorIndex];
                            G = m_pColor[colorIndex + 1];
                            B = m_pColor[colorIndex + 2];
                        }
                        
                        colors[i] = R;
                        colors[i + numDepthPoints] = G;
                        colors[i + numDepthPoints + numDepthPoints] = B;
                    }
                }    
            } // if color
            
            // Fill-up the point cloud with x,y,z values from camera space
            for (int i = 0; i < numDepthPoints; i++)
            {
                float X, Y, Z;
                X = cameraPoints[i].X;
                Y = cameraPoints[i].Y;
                Z = cameraPoints[i].Z;

                pointCloud[i] = X;
                pointCloud[i + numDepthPoints] = Y;
                pointCloud[i + numDepthPoints + numDepthPoints] = Z;
            }
        } // If successfull depth to camera space mapping   
        else
        {
            pointCloud[0] = 0;
            pointCloud[1] = 0;
            pointCloud[2] = 0;
            mexPrintf("Error getting depth-to-camera mapping.\n");
        }
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

/*
bool Kin2::mapDepthFrame2Color(UINT16* depth2colorCoordinates)
{
    HRESULT hr;
    
    int numDepthPoints = cDepthWidth * cDepthHeight;
    
	// Create coordinate mapping from depth to color	
    ColorSpacePoint* depth2ColorMapping = new ColorSpacePoint[numDepthPoints];
    
	hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(numDepthPoints,
		(UINT16*)m_pDepthArray16U, numDepthPoints, depth2ColorMapping);
    
    if (SUCCEEDED(hr))
	{		
		for (int i = 0; i < numDepthPoints; i++)
		{       
            depth2colorCoordinates[i] = (int)depth2ColorMapping[i].X;
            depth2colorCoordinates[i + numDepthPoints] = (int)depth2ColorMapping[i].Y;
        }
    }    
	else
    {
        colorCoords[0] = 0;
		colorCoords[1] = 0;
		mexPrintf("Mapping error.\n");
    }

	if (SUCCEEDED(hr)) return true;
	else return false;
} // end mapDepthFrame2Color
*/

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

void Kin2::alignColor2Depth(unsigned char alignedImage[], bool& validData)
{
    HRESULT hr;    
    const int numDepthPoints = cDepthWidth * cDepthHeight;
   
	// Map from depth to color	
    ColorSpacePoint depth2ColorMapping[numDepthPoints];
    
	hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(numDepthPoints,
		(UINT16*)m_pDepthArray16U, numDepthPoints, depth2ColorMapping);

    // fill up the output matrix with R,G,B values from the current color image
    if (SUCCEEDED(hr))
    {		
        int colorCoordX, colorCoordY;
        for (int x=0, k=0; x < cDepthWidth; x++)
        {
            for (int y=0; y <cDepthHeight; y++,k++)
            {
                int idx = y * cDepthWidth + x;
                colorCoordX = (UINT16)depth2ColorMapping[idx].X;
                colorCoordY = (UINT16)depth2ColorMapping[idx].Y;

                // Sample the RGB components from the color image
                unsigned char R=0, G=0, B=0;

                // first make sure the coordinates maps to a valid point in color space
                int colorX = (int)(floor(colorCoordX + 0.5));
                int colorY = (int)(floor(colorCoordY + 0.5));
                if ((colorX >= 0) && (colorX < cColorWidth) && (colorY >= 0) && (colorY < cColorHeight))
                {
                    // calculate index into color array
                    int colorIndex = (colorX + (colorY * cColorWidth)) * 4;

                   R = m_pColor[colorIndex];
                   G = m_pColor[colorIndex + 1];
                   B = m_pColor[colorIndex + 2];
                }

                alignedImage[k] = R;
                alignedImage[k + numDepthPoints] = G;
                alignedImage[k + numDepthPoints + numDepthPoints] = B;
            }
        }        
        validData = true;
    }
    else
    {
        validData = false;
        mexPrintf("Depth to Color Mapping error.\n");
    }
} // end alignColor2Depth
