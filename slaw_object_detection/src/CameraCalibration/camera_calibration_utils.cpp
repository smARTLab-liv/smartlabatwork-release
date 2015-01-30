#include "CameraCalibration/camera_calibration_utils.h"
#include <boost/shared_ptr.hpp>

namespace CameraCalibrationUtils
{


class NoMatException: public exception
{
    virtual const char* what() const throw()
    {
        return "The Parameter you are passing is no matrix, or has different field-names than : data, rows, cold";
    }
};

Mat getMat64F_FromParam(ros::NodeHandle &nh, string param)
{
    Mat mat;
    XmlRpc::XmlRpcValue array;
    int rows  , cols  ;

    if(nh.getParam(param + "/"+ DATA_STR, array)
            && nh.getParam(param + "/" + COLS_STR, cols)
            && nh.getParam(param + "/" + ROWS_STR, rows))
    {
        mat = Mat(rows, cols, CV_64F);
        for (int y = 0; y < rows; y++)
        {
            for( int x = 0; x < cols; x++)
            {

                XmlRpc::XmlRpcValue rpcvalue = array[(y*cols) + x];

                if(rpcvalue.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    mat.at<double>(y,x) = rpcvalue;
                }
                if(rpcvalue.getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    mat.at<double>(y,x) = (int)rpcvalue;
                }
            }
        }
    }
    else
    {
        throw new NoMatException;
    }

    return mat;
}




vector<Point3d> generate3DChessboard(int boardWidth, int boardHeight, double fieldSize)
{
    vector<Point3d> boardPoints;
    for (int i=0; i<boardWidth; i++)
    {
        for (int j=0; j<boardHeight; j++)
        {
            boardPoints.push_back( Point3d( double(i) * fieldSize, double(j) * fieldSize, 0.0) );
        }
    }
    return boardPoints;
}

void mcvGetIPM(const CvMat* inImage, CvMat* outImage,
               IPMInfo *ipmInfo, const CameraInfo *cameraInfo,
               list<CvPoint> *outPoints)
{
    //check input images types
    //CvMat inMat, outMat;
    //cvGetMat(inImage, &inMat);
    //cvGetMat(outImage, &outMat);
    //cout << CV_MAT_TYPE(inImage->type) << " " << CV_MAT_TYPE(FLOAT_MAT_TYPE) <<  " " << CV_MAT_TYPE(INT_MAT_TYPE)<<"\n";
    if (!(CV_ARE_TYPES_EQ(inImage, outImage) &&
          (CV_MAT_TYPE(inImage->type)==CV_MAT_TYPE(FLOAT_MAT_TYPE) ||
           (CV_MAT_TYPE(inImage->type)==CV_MAT_TYPE(INT_MAT_TYPE)))))
    {
        if(CV_ARE_TYPES_EQ(inImage, outImage)){
            cerr << "Types are equal" << CV_MAT_TYPE(inImage->type);
        }else{
            cerr << "Types are NOT equal" << CV_MAT_TYPE(inImage->type);
        }


        cerr << "Unsupported image types in mcvGetIPM";
        exit(1);
    }

    //get size of input image
    FLOAT u, v;
    v = inImage->height;
    u = inImage->width;

    //get the vanishing point
    FLOAT_POINT2D vp;
    vp = mcvGetVanishingPoint(cameraInfo);
    vp.y = MAX(0, vp.y);
    //vp.y = 30;

    //get extent of the image in the xfyf plane
    FLOAT_MAT_ELEM_TYPE eps = ipmInfo->vpPortion * v;//VP_PORTION*v;
    ipmInfo->ipmLeft = MAX(0, ipmInfo->ipmLeft);
    ipmInfo->ipmRight = MIN(u-1, ipmInfo->ipmRight);
    ipmInfo->ipmTop = MAX(vp.y+eps, ipmInfo->ipmTop);
    ipmInfo->ipmBottom = MIN(v-1, ipmInfo->ipmBottom);
    FLOAT_MAT_ELEM_TYPE uvLimitsp[] = {vp.x,
                                       ipmInfo->ipmRight, ipmInfo->ipmLeft, vp.x,
                                       ipmInfo->ipmTop, ipmInfo->ipmTop,   ipmInfo->ipmTop,  ipmInfo->ipmBottom};
    //{vp.x, u, 0, vp.x,
    //vp.y+eps, vp.y+eps, vp.y+eps, v};
    CvMat uvLimits = cvMat(2, 4, FLOAT_MAT_TYPE, uvLimitsp);

    //get these points on the ground plane
    CvMat * xyLimitsp = cvCreateMat(2, 4, FLOAT_MAT_TYPE);
    CvMat xyLimits = *xyLimitsp;
    mcvTransformImage2Ground(&uvLimits, &xyLimits,cameraInfo);


    //SHOW_MAT(xyLimitsp, "xyLImits");

    //get extent on the ground plane
    CvMat row1, row2;
    cvGetRow(&xyLimits, &row1, 0);
    cvGetRow(&xyLimits, &row2, 1);
    double xfMax, xfMin, yfMax, yfMin;
    cvMinMaxLoc(&row1, (double*)&xfMin, (double*)&xfMax, 0, 0, 0);
    cvMinMaxLoc(&row2, (double*)&yfMin, (double*)&yfMax, 0, 0, 0);

    INT outRow = outImage->height;
    INT outCol = outImage->width;

    FLOAT_MAT_ELEM_TYPE stepRow = (yfMax-yfMin)/outRow;
    FLOAT_MAT_ELEM_TYPE stepCol = (xfMax-xfMin)/outCol;

    //construct the grid to sample
    CvMat *xyGrid = cvCreateMat(2, outRow*outCol, FLOAT_MAT_TYPE);
    INT i, j;
    FLOAT_MAT_ELEM_TYPE x, y;
    //fill it with x-y values on the ground plane in world frame
    for (i=0, y=yfMax-.5*stepRow; i<outRow; i++, y-=stepRow)
        for (j=0, x=xfMin+.5*stepCol; j<outCol; j++, x+=stepCol)
        {
            CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, i*outCol+j) = x;
            CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, i*outCol+j) = y;
        }
    //get their pixel values in image frame
    CvMat *uvGrid = cvCreateMat(2, outRow*outCol, FLOAT_MAT_TYPE);
    mcvTransformGround2Image(xyGrid, uvGrid, cameraInfo);
    //now loop and find the nearest pixel value for each position
    //that's inside the image, otherwise put it zero

    //generic loop to work for both float and int matrix types

    if (CV_MAT_TYPE(inImage->type)==FLOAT_MAT_TYPE)
    {
        //test<int>();
        //MCV_GET_IPM(FLOAT_MAT_ELEM_TYPE)
        interpolation<FLOAT_MAT_ELEM_TYPE>(inImage, outImage, uvGrid,outCol, outRow, ipmInfo);
    }
    else
    {
        //test<double>();
        //MCV_GET_IPM(INT_MAT_ELEM_TYPE)
        interpolation<INT_MAT_ELEM_TYPE>(inImage, outImage, uvGrid,outCol, outRow, ipmInfo);
    }
    //return the ipm info
    ipmInfo->xLimits[0] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, 0);
    ipmInfo->xLimits[1] =
            CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, (outRow-1)*outCol+outCol-1);
    ipmInfo->yLimits[1] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, 0);
    ipmInfo->yLimits[0] =
            CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, (outRow-1)*outCol+outCol-1);
    ipmInfo->xScale = 1/stepCol;
    ipmInfo->yScale = 1/stepRow;
    ipmInfo->width = outCol;
    ipmInfo->height = outRow;

    //clean
    cvReleaseMat(&xyLimitsp);
    cvReleaseMat(&xyGrid);
    cvReleaseMat(&uvGrid);
}

void mcvIplImageToCvMat(IplImage* im, CvMat **clrImage, CvMat** channelImage)
{
    // convert to mat and get first channel
    CvMat temp;
    cvGetMat(im, &temp);
    *clrImage = cvCloneMat(&temp);
    // convert to single channel
    CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
    cvSplit(*clrImage, tchannelImage, NULL, NULL, NULL);
    // convert to float
    *channelImage = cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
    cvConvertScale(tchannelImage, *channelImage, 1./255);
    // destroy
    cvReleaseMat(&tchannelImage);
    //cvReleaseImage(&im);

}



template<typename T>
void interpolation(const CvMat *inImage, CvMat *outImage, CvMat *uvGrid, int outCol, int outRow,const IPMInfo *ipmInfo, list<CvPoint>* outPoints){
    FLOAT_MAT_ELEM_TYPE ui, vi;
    //get mean of the input image
    CvScalar means = cvAvg(inImage);
    double mean = means.val[0];
    for (int i=0; i<outRow; i++)
        for (int j=0; j<outCol; j++)
        {
            /*get pixel coordiantes*/

            ui = CV_MAT_ELEM(*uvGrid, FLOAT_MAT_ELEM_TYPE, 0, i*outCol+j);
            vi = CV_MAT_ELEM(*uvGrid, FLOAT_MAT_ELEM_TYPE, 1, i*outCol+j);

            if(i*outCol+j >  uvGrid->cols || i*outCol+j < 0)
            {
                cout << "PROBLEM 1" <<endl;
            }


            if(isnan(ui)){
                ui = 0;
            }
            if(isnan(vi)){
                vi=0;
            }

            /*check if out-of-bounds*/ \
            /*if (ui<0 || ui>u-1 || vi<0 || vi>v-1) \*/ \
            if (ui < ipmInfo->ipmLeft || ui > ipmInfo->ipmRight ||
                    vi < ipmInfo->ipmTop || vi > ipmInfo->ipmBottom)
            {

                CV_MAT_ELEM(*outImage, T, i, j) = (T)mean;
            }
            /*not out of bounds, then get nearest neighbor*/
            else
            {

                /*Bilinear interpolation*/
                if (ipmInfo->ipmInterpolation == 0)
                {

                    int x1 = int(ui), x2 = int(ui+1);
                    int y1 = int(vi), y2 = int(vi+1);
                    float x = ui - x1, y = vi - y1;
                    float val = CV_MAT_ELEM(*inImage, T, y1, x1) * (1-x) * (1-y) +
                            CV_MAT_ELEM(*inImage, T, y1, x2) * x * (1-y) +
                            CV_MAT_ELEM(*inImage, T, y2, x1) * (1-x) * y +
                            CV_MAT_ELEM(*inImage, T, y2, x2) * x * y;
                    CV_MAT_ELEM(*outImage, T, i, j) =  (T)val;

                }
                /*nearest-neighbor interpolation*/
                else
                {
                    if(vi+.5 < inImage->rows && ui +.5 < inImage->cols)
                    {
                        CV_MAT_ELEM(*outImage, T, i, j) =
                                CV_MAT_ELEM(*inImage, T, int(vi+.5), int(ui+.5));
                    }
                    else
                        cout << "vi+.5 " << (vi+.5) << " inImage->rows " << inImage->rows << " ui +.5 "
                             << (ui +.5) << " inImage->cols " << inImage->cols <<endl;


                }
            }
            if (outPoints &&
                    (ui<ipmInfo->ipmLeft+10 || ui>ipmInfo->ipmRight-10 ||
                     vi<ipmInfo->ipmTop || vi>ipmInfo->ipmBottom-2) )
                outPoints->push_back(cvPoint(j, i));
        }
}





/**
 * Transforms points from the image frame (uv-coordinates)
 * into the real world frame on the ground plane (z=-height)
 *
 * \param inPoints input points in the image frame
 * \param outPoints output points in the world frame on the ground
 *          (z=-height)
 * \param cemaraInfo the input camera parameters
 *
 */
void mcvTransformImage2Ground(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo)
{

    //add two rows to the input points
    CvMat *inPoints4 = cvCreateMat(inPoints->rows+2, inPoints->cols,
                                   cvGetElemType(inPoints));

    //copy inPoints to first two rows
    CvMat inPoints2, inPoints3, inPointsr4, inPointsr3;
    cvGetRows(inPoints4, &inPoints2, 0, 2);
    cvGetRows(inPoints4, &inPoints3, 0, 3);
    cvGetRow(inPoints4, &inPointsr3, 2);
    cvGetRow(inPoints4, &inPointsr4, 3);
    cvSet(&inPointsr3, cvRealScalar(1));
    cvCopy(inPoints, &inPoints2);
    //create the transformation matrix
    float c1 = cos(cameraInfo->pitch);
    float s1 = sin(cameraInfo->pitch);
    float c2 = cos(cameraInfo->yaw);
    float s2 = sin(cameraInfo->yaw);
    float matp[] = {
        -cameraInfo->cameraHeight*c2/cameraInfo->focalLength.x,
        cameraInfo->cameraHeight*s1*s2/cameraInfo->focalLength.y,
        (cameraInfo->cameraHeight*c2*cameraInfo->opticalCenter.x/
        cameraInfo->focalLength.x)-
        (cameraInfo->cameraHeight *s1*s2* cameraInfo->opticalCenter.y/
        cameraInfo->focalLength.y) - cameraInfo->cameraHeight *c1*s2,

        cameraInfo->cameraHeight *s2 /cameraInfo->focalLength.x,
        cameraInfo->cameraHeight *s1*c2 /cameraInfo->focalLength.y,
        (-cameraInfo->cameraHeight *s2* cameraInfo->opticalCenter.x
        /cameraInfo->focalLength.x)-(cameraInfo->cameraHeight *s1*c2*
        cameraInfo->opticalCenter.y /cameraInfo->focalLength.y) -
        cameraInfo->cameraHeight *c1*c2,

        0,
        cameraInfo->cameraHeight *c1 /cameraInfo->focalLength.y,
        (-cameraInfo->cameraHeight *c1* cameraInfo->opticalCenter.y /
        cameraInfo->focalLength.y) + cameraInfo->cameraHeight *s1,

        0,
        -c1 /cameraInfo->focalLength.y,
        (c1* cameraInfo->opticalCenter.y /cameraInfo->focalLength.y) - s1,
    };

    CvMat mat = cvMat(4, 3, CV_32FC1, matp);
    //multiply
    cvMatMul(&mat, &inPoints3, inPoints4);
    //divide by last row of inPoints4
    for (int i=0; i<inPoints->cols; i++)
    {
        float div = CV_MAT_ELEM(inPointsr4, float, 0, i);
        CV_MAT_ELEM(*inPoints4, float, 0, i) =
                CV_MAT_ELEM(*inPoints4, float, 0, i) / div ;
        CV_MAT_ELEM(*inPoints4, float, 1, i) =
                CV_MAT_ELEM(*inPoints4, float, 1, i) / div;
    }
    //put back the result into outPoints
    cout << "cameraInfo->cameraHeight " << cameraInfo->cameraHeight << endl;
    cvCopy(&inPoints2, outPoints);
    //clear
    cvReleaseMat(&inPoints4);
}


/**
 * Transforms points from the ground plane (z=-h) in the world frame
 * into points on the image in image frame (uv-coordinates)
 *
 * \param inPoints 2xN array of input points on the ground in world coordinates
 * \param outPoints 2xN output points in on the image in image coordinates
 * \param cameraInfo the camera parameters
 *
 */
void mcvTransformGround2Image(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo)
{
    //add two rows to the input points
    CvMat *inPoints3 = cvCreateMat(inPoints->rows+1, inPoints->cols,
                                   cvGetElemType(inPoints));
    cvSet(inPoints3, cvScalar(0));

    //copy inPoints to first two rows
    CvMat inPoints2,  inPointsr3;
    cvGetRows(inPoints3, &inPoints2, 0, 2);
    cvGetRow(inPoints3, &inPointsr3, 2);
    cvSet(&inPointsr3, cvRealScalar(-cameraInfo->cameraHeight));
    cvCopy(inPoints, &inPoints2);
    //create the transformation matrix
    float c1 = cos(cameraInfo->pitch);
    float s1 = sin(cameraInfo->pitch);
    float c2 = cos(cameraInfo->yaw);
    float s2 = sin(cameraInfo->yaw);
    float matp[] = {
        cameraInfo->focalLength.x * c2 + c1*s2* cameraInfo->opticalCenter.x,
        -cameraInfo->focalLength.x * s2 + c1*c2* cameraInfo->opticalCenter.x,
        - s1 * cameraInfo->opticalCenter.x,

        s2 * (-cameraInfo->focalLength.y * s1 + c1* cameraInfo->opticalCenter.y),
        c2 * (-cameraInfo->focalLength.y * s1 + c1* cameraInfo->opticalCenter.y),
        -cameraInfo->focalLength.y * c1 - s1* cameraInfo->opticalCenter.y,

        c1*s2,
        c1*c2,
        -s1
    };
    CvMat mat = cvMat(3, 3, FLOAT_MAT_TYPE, matp);
    //multiply
    //std::cout << " inPoints3.rows " << inPoints3->rows << " inPoints3.cols "<< inPoints3->cols << "\n ";

    cvMatMul(&mat, inPoints3, inPoints3);
    //divide by last row of inPoints4
    for (int i=0; i<inPoints->cols; i++)
    {
        float div = CV_MAT_ELEM(inPointsr3, float, 0, i);

        CV_MAT_ELEM(*inPoints3, float, 0, i) =
                CV_MAT_ELEM(*inPoints3, float, 0, i) / div ;
        CV_MAT_ELEM(*inPoints3, float, 1, i) =
                CV_MAT_ELEM(*inPoints3, float, 1, i) / div;
        if(isnan(CV_MAT_ELEM(*inPoints3, float, 0, i) ) || isnan(CV_MAT_ELEM(*inPoints3, float, 1, i) ))
        {
            cout << "cameraInfo->pitch " << cameraInfo->pitch << endl;
            cout << "cameraInfo->yaw " << cameraInfo->yaw << endl;
            cout << "opticalCenter.x " << cameraInfo->opticalCenter.x << endl;
            cout << "opticalCenter.y " << cameraInfo->opticalCenter.y << endl;
            cout << "focalLength.x " << cameraInfo->focalLength.x << endl;
            cout << "focalLength.y " << cameraInfo->focalLength.y << endl;
        }
        //if(isnan(CV_MAT_ELEM(*inPoints3, float, 1, i) )) cout << "JAP!!!2 div " << div <<endl;
    }
    //put back the result into outPoints
    //std::cout << " inPoints2.rows " << inPoints2.rows << " inPoints2.cols "<< inPoints2.cols << "\n ";
    // std::cout << " outPoints.rows " << outPoints->rows << " outPoints.cols "<< outPoints->cols << "\n ";
    cvCopy(&inPoints2, outPoints);
    // std::cout << " outPoints.rows " << outPoints->rows << " outPoints.cols "<< outPoints->cols << "\n ";
    //clear
    cvReleaseMat(&inPoints3);
}

FLOAT_POINT2D mcvGetVanishingPoint(const CameraInfo *cameraInfo)
{
    //get the vp in world coordinates
    FLOAT_MAT_ELEM_TYPE vpp[] = {sin(cameraInfo->yaw)/cos(cameraInfo->pitch),
                                 cos(cameraInfo->yaw)/cos(cameraInfo->pitch), 0};
    CvMat vp = cvMat(3, 1, FLOAT_MAT_TYPE, vpp);

    //transform from world to camera coordinates
    //
    //rotation matrix for yaw
    FLOAT_MAT_ELEM_TYPE tyawp[] = {cos(cameraInfo->yaw), -sin(cameraInfo->yaw), 0,
                                   sin(cameraInfo->yaw), cos(cameraInfo->yaw), 0,
                                   0, 0, 1};
    CvMat tyaw = cvMat(3, 3, FLOAT_MAT_TYPE, tyawp);
    //rotation matrix for pitch
    FLOAT_MAT_ELEM_TYPE tpitchp[] = {1, 0, 0,
                                     0, -sin(cameraInfo->pitch), -cos(cameraInfo->pitch),
                                     0, cos(cameraInfo->pitch), -sin(cameraInfo->pitch)};
    CvMat transform = cvMat(3, 3, FLOAT_MAT_TYPE, tpitchp);
    //combined transform
    cvMatMul(&transform, &tyaw, &transform);

    //
    //transformation from (xc, yc) in camra coordinates
    // to (u,v) in image frame
    //
    //matrix to shift optical center and focal length
    FLOAT_MAT_ELEM_TYPE t1p[] = {
        cameraInfo->focalLength.x, 0,
        cameraInfo->opticalCenter.x,
        0, cameraInfo->focalLength.y,
        cameraInfo->opticalCenter.y,
        0, 0, 1};
    CvMat t1 = cvMat(3, 3, FLOAT_MAT_TYPE, t1p);
    //combine transform
    cvMatMul(&t1, &transform, &transform);
    //transform
    cvMatMul(&transform, &vp, &vp);

    //
    //clean and return
    //
    FLOAT_POINT2D ret;
    ret.x = cvGetReal1D(&vp, 0);
    ret.y = cvGetReal1D(&vp, 1);
    return ret;
}





}//end namespace
