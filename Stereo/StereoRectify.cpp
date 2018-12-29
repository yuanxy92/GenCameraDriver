/**
@brief class to rectify stereo images
@author Shane Yuan
@date Apr 26, 2018
*/

#include "StereoRectify.h"

#if (CV_MAJOR_VERSION >= 4)
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>

using namespace cv;

static const char* cvDistCoeffErr = "Distortion coefficients must be 1x4, 4x1, 1x5, 5x1, 1x8, 8x1, 1x12, 12x1, 1x14 or 14x1 floating-point vector";

int cvRodrigues2( const CvMat* src, CvMat* dst, CvMat* jacobian  = 0);

template <typename FLOAT>
void computeTiltProjectionMatrix(FLOAT tauX,
    FLOAT tauY,
    Matx<FLOAT, 3, 3>* matTilt = 0,
    Matx<FLOAT, 3, 3>* dMatTiltdTauX = 0,
    Matx<FLOAT, 3, 3>* dMatTiltdTauY = 0,
    Matx<FLOAT, 3, 3>* invMatTilt = 0)
{
    FLOAT cTauX = cos(tauX);
    FLOAT sTauX = sin(tauX);
    FLOAT cTauY = cos(tauY);
    FLOAT sTauY = sin(tauY);
    Matx<FLOAT, 3, 3> matRotX = Matx<FLOAT, 3, 3>(1,0,0,0,cTauX,sTauX,0,-sTauX,cTauX);
    Matx<FLOAT, 3, 3> matRotY = Matx<FLOAT, 3, 3>(cTauY,0,-sTauY,0,1,0,sTauY,0,cTauY);
    Matx<FLOAT, 3, 3> matRotXY = matRotY * matRotX;
    Matx<FLOAT, 3, 3> matProjZ = Matx<FLOAT, 3, 3>(matRotXY(2,2),0,-matRotXY(0,2),0,matRotXY(2,2),-matRotXY(1,2),0,0,1);
    if (matTilt)
    {
        // Matrix for trapezoidal distortion of tilted image sensor
        *matTilt = matProjZ * matRotXY;
    }
    if (dMatTiltdTauX)
    {
        // Derivative with respect to tauX
        Matx<FLOAT, 3, 3> dMatRotXYdTauX = matRotY * Matx<FLOAT, 3, 3>(0,0,0,0,-sTauX,cTauX,0,-cTauX,-sTauX);
        Matx<FLOAT, 3, 3> dMatProjZdTauX = Matx<FLOAT, 3, 3>(dMatRotXYdTauX(2,2),0,-dMatRotXYdTauX(0,2),
          0,dMatRotXYdTauX(2,2),-dMatRotXYdTauX(1,2),0,0,0);
        *dMatTiltdTauX = (matProjZ * dMatRotXYdTauX) + (dMatProjZdTauX * matRotXY);
    }
    if (dMatTiltdTauY)
    {
        // Derivative with respect to tauY
        Matx<FLOAT, 3, 3> dMatRotXYdTauY = Matx<FLOAT, 3, 3>(-sTauY,0,-cTauY,0,0,0,cTauY,0,-sTauY) * matRotX;
        Matx<FLOAT, 3, 3> dMatProjZdTauY = Matx<FLOAT, 3, 3>(dMatRotXYdTauY(2,2),0,-dMatRotXYdTauY(0,2),
          0,dMatRotXYdTauY(2,2),-dMatRotXYdTauY(1,2),0,0,0);
        *dMatTiltdTauY = (matProjZ * dMatRotXYdTauY) + (dMatProjZdTauY * matRotXY);
    }
    if (invMatTilt)
    {
        FLOAT inv = 1./matRotXY(2,2);
        Matx<FLOAT, 3, 3> invMatProjZ = Matx<FLOAT, 3, 3>(inv,0,inv*matRotXY(0,2),0,inv,inv*matRotXY(1,2),0,0,1);
        *invMatTilt = matRotXY.t()*invMatProjZ;
    }
}

void cvProjectPoints2( const CvMat* objectPoints,
                  const CvMat* r_vec,
                  const CvMat* t_vec,
                  const CvMat* A,
                  const CvMat* distCoeffs,
                  CvMat* imagePoints, CvMat* dpdr = 0,
                  CvMat* dpdt = 0, CvMat* dpdf = 0,
                  CvMat* dpdc = 0, CvMat* dpdk = 0,
                  double aspectRatio  = 0)
{
    Ptr<CvMat> matM, _m;
    Ptr<CvMat> _dpdr, _dpdt, _dpdc, _dpdf, _dpdk;

    int i, j, count;
    int calc_derivatives;
    const CvPoint3D64f* M;
    CvPoint2D64f* m;
    double r[3], R[9], dRdr[27], t[3], a[9], k[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}, fx, fy, cx, cy;
    Matx33d matTilt = Matx33d::eye();
    Matx33d dMatTiltdTauX(0,0,0,0,0,0,0,-1,0);
    Matx33d dMatTiltdTauY(0,0,0,0,0,0,1,0,0);
    CvMat _r, _t, _a = cvMat( 3, 3, CV_64F, a ), _k;
    CvMat matR = cvMat( 3, 3, CV_64F, R ), _dRdr = cvMat( 3, 9, CV_64F, dRdr );
    double *dpdr_p = 0, *dpdt_p = 0, *dpdk_p = 0, *dpdf_p = 0, *dpdc_p = 0;
    int dpdr_step = 0, dpdt_step = 0, dpdk_step = 0, dpdf_step = 0, dpdc_step = 0;
    bool fixedAspectRatio = aspectRatio > FLT_EPSILON;

    if( !CV_IS_MAT(objectPoints) || !CV_IS_MAT(r_vec) ||
        !CV_IS_MAT(t_vec) || !CV_IS_MAT(A) ||
        /*!CV_IS_MAT(distCoeffs) ||*/ !CV_IS_MAT(imagePoints) )
        CV_Error( CV_StsBadArg, "One of required arguments is not a valid matrix" );

    int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type);
    if(total % 3 != 0)
    {
        //we have stopped support of homogeneous coordinates because it cause ambiguity in interpretation of the input data
        CV_Error( CV_StsBadArg, "Homogeneous coordinates are not supported" );
    }
    count = total / 3;

    if( CV_IS_CONT_MAT(objectPoints->type) &&
        (CV_MAT_DEPTH(objectPoints->type) == CV_32F || CV_MAT_DEPTH(objectPoints->type) == CV_64F)&&
        ((objectPoints->rows == 1 && CV_MAT_CN(objectPoints->type) == 3) ||
        (objectPoints->rows == count && CV_MAT_CN(objectPoints->type)*objectPoints->cols == 3) ||
        (objectPoints->rows == 3 && CV_MAT_CN(objectPoints->type) == 1 && objectPoints->cols == count)))
    {
        matM.reset(cvCreateMat( objectPoints->rows, objectPoints->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(objectPoints->type)) ));
        cvConvert(objectPoints, matM);
    }
    else
    {
//        matM = cvCreateMat( 1, count, CV_64FC3 );
//        cvConvertPointsHomogeneous( objectPoints, matM );
        CV_Error( CV_StsBadArg, "Homogeneous coordinates are not supported" );
    }

    if( CV_IS_CONT_MAT(imagePoints->type) &&
        (CV_MAT_DEPTH(imagePoints->type) == CV_32F || CV_MAT_DEPTH(imagePoints->type) == CV_64F) &&
        ((imagePoints->rows == 1 && CV_MAT_CN(imagePoints->type) == 2) ||
        (imagePoints->rows == count && CV_MAT_CN(imagePoints->type)*imagePoints->cols == 2) ||
        (imagePoints->rows == 2 && CV_MAT_CN(imagePoints->type) == 1 && imagePoints->cols == count)))
    {
        _m.reset(cvCreateMat( imagePoints->rows, imagePoints->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(imagePoints->type)) ));
        cvConvert(imagePoints, _m);
    }
    else
    {
//        _m = cvCreateMat( 1, count, CV_64FC2 );
        CV_Error( CV_StsBadArg, "Homogeneous coordinates are not supported" );
    }

    M = (CvPoint3D64f*)matM->data.db;
    m = (CvPoint2D64f*)_m->data.db;

    if( (CV_MAT_DEPTH(r_vec->type) != CV_64F && CV_MAT_DEPTH(r_vec->type) != CV_32F) ||
        (((r_vec->rows != 1 && r_vec->cols != 1) ||
        r_vec->rows*r_vec->cols*CV_MAT_CN(r_vec->type) != 3) &&
        ((r_vec->rows != 3 && r_vec->cols != 3) || CV_MAT_CN(r_vec->type) != 1)))
        CV_Error( CV_StsBadArg, "Rotation must be represented by 1x3 or 3x1 "
                  "floating-point rotation vector, or 3x3 rotation matrix" );

    if( r_vec->rows == 3 && r_vec->cols == 3 )
    {
        _r = cvMat( 3, 1, CV_64FC1, r );
        cvRodrigues2( r_vec, &_r );
        cvRodrigues2( &_r, &matR, &_dRdr );
        cvCopy( r_vec, &matR );
    }
    else
    {
        _r = cvMat( r_vec->rows, r_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(r_vec->type)), r );
        cvConvert( r_vec, &_r );
        cvRodrigues2( &_r, &matR, &_dRdr );
    }

    if( (CV_MAT_DEPTH(t_vec->type) != CV_64F && CV_MAT_DEPTH(t_vec->type) != CV_32F) ||
        (t_vec->rows != 1 && t_vec->cols != 1) ||
        t_vec->rows*t_vec->cols*CV_MAT_CN(t_vec->type) != 3 )
        CV_Error( CV_StsBadArg,
            "Translation vector must be 1x3 or 3x1 floating-point vector" );

    _t = cvMat( t_vec->rows, t_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(t_vec->type)), t );
    cvConvert( t_vec, &_t );

    if( (CV_MAT_TYPE(A->type) != CV_64FC1 && CV_MAT_TYPE(A->type) != CV_32FC1) ||
        A->rows != 3 || A->cols != 3 )
        CV_Error( CV_StsBadArg, "Instrinsic parameters must be 3x3 floating-point matrix" );

    cvConvert( A, &_a );
    fx = a[0]; fy = a[4];
    cx = a[2]; cy = a[5];

    if( fixedAspectRatio )
        fx = fy*aspectRatio;

    if( distCoeffs )
    {
        if( !CV_IS_MAT(distCoeffs) ||
            (CV_MAT_DEPTH(distCoeffs->type) != CV_64F &&
            CV_MAT_DEPTH(distCoeffs->type) != CV_32F) ||
            (distCoeffs->rows != 1 && distCoeffs->cols != 1) ||
            (distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 4 &&
            distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 5 &&
            distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 8 &&
            distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 12 &&
            distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 14) )
            CV_Error( CV_StsBadArg, cvDistCoeffErr );

        _k = cvMat( distCoeffs->rows, distCoeffs->cols,
                    CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), k );
        cvConvert( distCoeffs, &_k );
        if(k[12] != 0 || k[13] != 0)
        {
          computeTiltProjectionMatrix(k[12], k[13],
            &matTilt, &dMatTiltdTauX, &dMatTiltdTauY);
        }
    }

    if( dpdr )
    {
        if( !CV_IS_MAT(dpdr) ||
            (CV_MAT_TYPE(dpdr->type) != CV_32FC1 &&
            CV_MAT_TYPE(dpdr->type) != CV_64FC1) ||
            dpdr->rows != count*2 || dpdr->cols != 3 )
            CV_Error( CV_StsBadArg, "dp/drot must be 2Nx3 floating-point matrix" );

        if( CV_MAT_TYPE(dpdr->type) == CV_64FC1 )
        {
            _dpdr.reset(cvCloneMat(dpdr));
        }
        else
            _dpdr.reset(cvCreateMat( 2*count, 3, CV_64FC1 ));
        dpdr_p = _dpdr->data.db;
        dpdr_step = _dpdr->step/sizeof(dpdr_p[0]);
    }

    if( dpdt )
    {
        if( !CV_IS_MAT(dpdt) ||
            (CV_MAT_TYPE(dpdt->type) != CV_32FC1 &&
            CV_MAT_TYPE(dpdt->type) != CV_64FC1) ||
            dpdt->rows != count*2 || dpdt->cols != 3 )
            CV_Error( CV_StsBadArg, "dp/dT must be 2Nx3 floating-point matrix" );

        if( CV_MAT_TYPE(dpdt->type) == CV_64FC1 )
        {
            _dpdt.reset(cvCloneMat(dpdt));
        }
        else
            _dpdt.reset(cvCreateMat( 2*count, 3, CV_64FC1 ));
        dpdt_p = _dpdt->data.db;
        dpdt_step = _dpdt->step/sizeof(dpdt_p[0]);
    }

    if( dpdf )
    {
        if( !CV_IS_MAT(dpdf) ||
            (CV_MAT_TYPE(dpdf->type) != CV_32FC1 && CV_MAT_TYPE(dpdf->type) != CV_64FC1) ||
            dpdf->rows != count*2 || dpdf->cols != 2 )
            CV_Error( CV_StsBadArg, "dp/df must be 2Nx2 floating-point matrix" );

        if( CV_MAT_TYPE(dpdf->type) == CV_64FC1 )
        {
            _dpdf.reset(cvCloneMat(dpdf));
        }
        else
            _dpdf.reset(cvCreateMat( 2*count, 2, CV_64FC1 ));
        dpdf_p = _dpdf->data.db;
        dpdf_step = _dpdf->step/sizeof(dpdf_p[0]);
    }

    if( dpdc )
    {
        if( !CV_IS_MAT(dpdc) ||
            (CV_MAT_TYPE(dpdc->type) != CV_32FC1 && CV_MAT_TYPE(dpdc->type) != CV_64FC1) ||
            dpdc->rows != count*2 || dpdc->cols != 2 )
            CV_Error( CV_StsBadArg, "dp/dc must be 2Nx2 floating-point matrix" );

        if( CV_MAT_TYPE(dpdc->type) == CV_64FC1 )
        {
            _dpdc.reset(cvCloneMat(dpdc));
        }
        else
            _dpdc.reset(cvCreateMat( 2*count, 2, CV_64FC1 ));
        dpdc_p = _dpdc->data.db;
        dpdc_step = _dpdc->step/sizeof(dpdc_p[0]);
    }

    if( dpdk )
    {
        if( !CV_IS_MAT(dpdk) ||
            (CV_MAT_TYPE(dpdk->type) != CV_32FC1 && CV_MAT_TYPE(dpdk->type) != CV_64FC1) ||
            dpdk->rows != count*2 || (dpdk->cols != 14 && dpdk->cols != 12 && dpdk->cols != 8 && dpdk->cols != 5 && dpdk->cols != 4 && dpdk->cols != 2) )
            CV_Error( CV_StsBadArg, "dp/df must be 2Nx14, 2Nx12, 2Nx8, 2Nx5, 2Nx4 or 2Nx2 floating-point matrix" );

        if( !distCoeffs )
            CV_Error( CV_StsNullPtr, "distCoeffs is NULL while dpdk is not" );

        if( CV_MAT_TYPE(dpdk->type) == CV_64FC1 )
        {
            _dpdk.reset(cvCloneMat(dpdk));
        }
        else
            _dpdk.reset(cvCreateMat( dpdk->rows, dpdk->cols, CV_64FC1 ));
        dpdk_p = _dpdk->data.db;
        dpdk_step = _dpdk->step/sizeof(dpdk_p[0]);
    }

    calc_derivatives = dpdr || dpdt || dpdf || dpdc || dpdk;

    for( i = 0; i < count; i++ )
    {
        double X = M[i].x, Y = M[i].y, Z = M[i].z;
        double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
        double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
        double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
        double r2, r4, r6, a1, a2, a3, cdist, icdist2;
        double xd, yd, xd0, yd0, invProj;
        Vec3d vecTilt;
        Vec3d dVecTilt;
        Matx22d dMatTilt;
        Vec2d dXdYd;

        z = z ? 1./z : 1;
        x *= z; y *= z;

        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
        icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
        xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
        yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

        // additional distortion by projecting onto a tilt plane
        vecTilt = matTilt*Vec3d(xd0, yd0, 1);
        invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
        xd = invProj * vecTilt(0);
        yd = invProj * vecTilt(1);

        m[i].x = xd*fx + cx;
        m[i].y = yd*fy + cy;

        if( calc_derivatives )
        {
            if( dpdc_p )
            {
                dpdc_p[0] = 1; dpdc_p[1] = 0; // dp_xdc_x; dp_xdc_y
                dpdc_p[dpdc_step] = 0;
                dpdc_p[dpdc_step+1] = 1;
                dpdc_p += dpdc_step*2;
            }

            if( dpdf_p )
            {
                if( fixedAspectRatio )
                {
                    dpdf_p[0] = 0; dpdf_p[1] = xd*aspectRatio; // dp_xdf_x; dp_xdf_y
                    dpdf_p[dpdf_step] = 0;
                    dpdf_p[dpdf_step+1] = yd;
                }
                else
                {
                    dpdf_p[0] = xd; dpdf_p[1] = 0;
                    dpdf_p[dpdf_step] = 0;
                    dpdf_p[dpdf_step+1] = yd;
                }
                dpdf_p += dpdf_step*2;
            }
            for (int row = 0; row < 2; ++row)
                for (int col = 0; col < 2; ++col)
                    dMatTilt(row,col) = matTilt(row,col)*vecTilt(2)
                      - matTilt(2,col)*vecTilt(row);
            double invProjSquare = (invProj*invProj);
            dMatTilt *= invProjSquare;
            if( dpdk_p )
            {
                dXdYd = dMatTilt*Vec2d(x*icdist2*r2, y*icdist2*r2);
                dpdk_p[0] = fx*dXdYd(0);
                dpdk_p[dpdk_step] = fy*dXdYd(1);
                dXdYd = dMatTilt*Vec2d(x*icdist2*r4, y*icdist2*r4);
                dpdk_p[1] = fx*dXdYd(0);
                dpdk_p[dpdk_step+1] = fy*dXdYd(1);
                if( _dpdk->cols > 2 )
                {
                    dXdYd = dMatTilt*Vec2d(a1, a3);
                    dpdk_p[2] = fx*dXdYd(0);
                    dpdk_p[dpdk_step+2] = fy*dXdYd(1);
                    dXdYd = dMatTilt*Vec2d(a2, a1);
                    dpdk_p[3] = fx*dXdYd(0);
                    dpdk_p[dpdk_step+3] = fy*dXdYd(1);
                    if( _dpdk->cols > 4 )
                    {
                        dXdYd = dMatTilt*Vec2d(x*icdist2*r6, y*icdist2*r6);
                        dpdk_p[4] = fx*dXdYd(0);
                        dpdk_p[dpdk_step+4] = fy*dXdYd(1);

                        if( _dpdk->cols > 5 )
                        {
                            dXdYd = dMatTilt*Vec2d(
                              x*cdist*(-icdist2)*icdist2*r2, y*cdist*(-icdist2)*icdist2*r2);
                            dpdk_p[5] = fx*dXdYd(0);
                            dpdk_p[dpdk_step+5] = fy*dXdYd(1);
                            dXdYd = dMatTilt*Vec2d(
                              x*cdist*(-icdist2)*icdist2*r4, y*cdist*(-icdist2)*icdist2*r4);
                            dpdk_p[6] = fx*dXdYd(0);
                            dpdk_p[dpdk_step+6] = fy*dXdYd(1);
                            dXdYd = dMatTilt*Vec2d(
                              x*cdist*(-icdist2)*icdist2*r6, y*cdist*(-icdist2)*icdist2*r6);
                            dpdk_p[7] = fx*dXdYd(0);
                            dpdk_p[dpdk_step+7] = fy*dXdYd(1);
                            if( _dpdk->cols > 8 )
                            {
                                dXdYd = dMatTilt*Vec2d(r2, 0);
                                dpdk_p[8] = fx*dXdYd(0); //s1
                                dpdk_p[dpdk_step+8] = fy*dXdYd(1); //s1
                                dXdYd = dMatTilt*Vec2d(r4, 0);
                                dpdk_p[9] = fx*dXdYd(0); //s2
                                dpdk_p[dpdk_step+9] = fy*dXdYd(1); //s2
                                dXdYd = dMatTilt*Vec2d(0, r2);
                                dpdk_p[10] = fx*dXdYd(0);//s3
                                dpdk_p[dpdk_step+10] = fy*dXdYd(1); //s3
                                dXdYd = dMatTilt*Vec2d(0, r4);
                                dpdk_p[11] = fx*dXdYd(0);//s4
                                dpdk_p[dpdk_step+11] = fy*dXdYd(1); //s4
                                if( _dpdk->cols > 12 )
                                {
                                    dVecTilt = dMatTiltdTauX * Vec3d(xd0, yd0, 1);
                                    dpdk_p[12] = fx * invProjSquare * (
                                      dVecTilt(0) * vecTilt(2) - dVecTilt(2) * vecTilt(0));
                                    dpdk_p[dpdk_step+12] = fy*invProjSquare * (
                                      dVecTilt(1) * vecTilt(2) - dVecTilt(2) * vecTilt(1));
                                    dVecTilt = dMatTiltdTauY * Vec3d(xd0, yd0, 1);
                                    dpdk_p[13] = fx * invProjSquare * (
                                      dVecTilt(0) * vecTilt(2) - dVecTilt(2) * vecTilt(0));
                                    dpdk_p[dpdk_step+13] = fy * invProjSquare * (
                                      dVecTilt(1) * vecTilt(2) - dVecTilt(2) * vecTilt(1));
                                }
                            }
                        }
                    }
                }
                dpdk_p += dpdk_step*2;
            }

            if( dpdt_p )
            {
                double dxdt[] = { z, 0, -x*z }, dydt[] = { 0, z, -y*z };
                for( j = 0; j < 3; j++ )
                {
                    double dr2dt = 2*x*dxdt[j] + 2*y*dydt[j];
                    double dcdist_dt = k[0]*dr2dt + 2*k[1]*r2*dr2dt + 3*k[4]*r4*dr2dt;
                    double dicdist2_dt = -icdist2*icdist2*(k[5]*dr2dt + 2*k[6]*r2*dr2dt + 3*k[7]*r4*dr2dt);
                    double da1dt = 2*(x*dydt[j] + y*dxdt[j]);
                    double dmxdt = (dxdt[j]*cdist*icdist2 + x*dcdist_dt*icdist2 + x*cdist*dicdist2_dt +
                                       k[2]*da1dt + k[3]*(dr2dt + 4*x*dxdt[j]) + k[8]*dr2dt + 2*r2*k[9]*dr2dt);
                    double dmydt = (dydt[j]*cdist*icdist2 + y*dcdist_dt*icdist2 + y*cdist*dicdist2_dt +
                                       k[2]*(dr2dt + 4*y*dydt[j]) + k[3]*da1dt + k[10]*dr2dt + 2*r2*k[11]*dr2dt);
                    dXdYd = dMatTilt*Vec2d(dmxdt, dmydt);
                    dpdt_p[j] = fx*dXdYd(0);
                    dpdt_p[dpdt_step+j] = fy*dXdYd(1);
                }
                dpdt_p += dpdt_step*2;
            }

            if( dpdr_p )
            {
                double dx0dr[] =
                {
                    X*dRdr[0] + Y*dRdr[1] + Z*dRdr[2],
                    X*dRdr[9] + Y*dRdr[10] + Z*dRdr[11],
                    X*dRdr[18] + Y*dRdr[19] + Z*dRdr[20]
                };
                double dy0dr[] =
                {
                    X*dRdr[3] + Y*dRdr[4] + Z*dRdr[5],
                    X*dRdr[12] + Y*dRdr[13] + Z*dRdr[14],
                    X*dRdr[21] + Y*dRdr[22] + Z*dRdr[23]
                };
                double dz0dr[] =
                {
                    X*dRdr[6] + Y*dRdr[7] + Z*dRdr[8],
                    X*dRdr[15] + Y*dRdr[16] + Z*dRdr[17],
                    X*dRdr[24] + Y*dRdr[25] + Z*dRdr[26]
                };
                for( j = 0; j < 3; j++ )
                {
                    double dxdr = z*(dx0dr[j] - x*dz0dr[j]);
                    double dydr = z*(dy0dr[j] - y*dz0dr[j]);
                    double dr2dr = 2*x*dxdr + 2*y*dydr;
                    double dcdist_dr = (k[0] + 2*k[1]*r2 + 3*k[4]*r4)*dr2dr;
                    double dicdist2_dr = -icdist2*icdist2*(k[5] + 2*k[6]*r2 + 3*k[7]*r4)*dr2dr;
                    double da1dr = 2*(x*dydr + y*dxdr);
                    double dmxdr = (dxdr*cdist*icdist2 + x*dcdist_dr*icdist2 + x*cdist*dicdist2_dr +
                                       k[2]*da1dr + k[3]*(dr2dr + 4*x*dxdr) + (k[8] + 2*r2*k[9])*dr2dr);
                    double dmydr = (dydr*cdist*icdist2 + y*dcdist_dr*icdist2 + y*cdist*dicdist2_dr +
                                       k[2]*(dr2dr + 4*y*dydr) + k[3]*da1dr + (k[10] + 2*r2*k[11])*dr2dr);
                    dXdYd = dMatTilt*Vec2d(dmxdr, dmydr);
                    dpdr_p[j] = fx*dXdYd(0);
                    dpdr_p[dpdr_step+j] = fy*dXdYd(1);
                }
                dpdr_p += dpdr_step*2;
            }
        }
    }

    if( _m != imagePoints )
        cvConvert( _m, imagePoints );

    if( _dpdr != dpdr )
        cvConvert( _dpdr, dpdr );

    if( _dpdt != dpdt )
        cvConvert( _dpdt, dpdt );

    if( _dpdf != dpdf )
        cvConvert( _dpdf, dpdf );

    if( _dpdc != dpdc )
        cvConvert( _dpdc, dpdc );

    if( _dpdk != dpdk )
        cvConvert( _dpdk, dpdk );
}

void cvConvertPointsHomogeneous( const CvMat* _src, CvMat* _dst )
{
    cv::Mat src = cv::cvarrToMat(_src), dst = cv::cvarrToMat(_dst);
    const cv::Mat dst0 = dst;

    int d0 = src.channels() > 1 ? src.channels() : MIN(src.cols, src.rows);

    if( src.channels() == 1 && src.cols > d0 )
        cv::transpose(src, src);

    int d1 = dst.channels() > 1 ? dst.channels() : MIN(dst.cols, dst.rows);

    if( d0 == d1 )
        src.copyTo(dst);
    else if( d0 < d1 )
        cv::convertPointsToHomogeneous(src, dst);
    else
        cv::convertPointsFromHomogeneous(src, dst);

    bool tflag = dst0.channels() == 1 && dst0.cols > d1;
    dst = dst.reshape(dst0.channels(), (tflag ? dst0.cols : dst0.rows));

    if( tflag )
    {
        CV_Assert( dst.rows == dst0.cols && dst.cols == dst0.rows );
        if( dst0.type() == dst.type() )
            transpose( dst, dst0 );
        else
        {
            transpose( dst, dst );
            dst.convertTo( dst0, dst0.type() );
        }
    }
    else
    {
        CV_Assert( dst.size() == dst0.size() );
        if( dst.data != dst0.data )
            dst.convertTo(dst0, dst0.type());
    }
}



void cvUndistortPointsInternal( const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
                   const CvMat* _distCoeffs,
                   const CvMat* matR, const CvMat* matP, cv::TermCriteria criteria)
{
    double A[3][3], RR[3][3], k[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    CvMat matA=cvMat(3, 3, CV_64F, A), _Dk;
    CvMat _RR=cvMat(3, 3, CV_64F, RR);
    cv::Matx33d invMatTilt = cv::Matx33d::eye();
    cv::Matx33d matTilt = cv::Matx33d::eye();

    CV_Assert( CV_IS_MAT(_src) && CV_IS_MAT(_dst) &&
        (_src->rows == 1 || _src->cols == 1) &&
        (_dst->rows == 1 || _dst->cols == 1) &&
        _src->cols + _src->rows - 1 == _dst->rows + _dst->cols - 1 &&
        (CV_MAT_TYPE(_src->type) == CV_32FC2 || CV_MAT_TYPE(_src->type) == CV_64FC2) &&
        (CV_MAT_TYPE(_dst->type) == CV_32FC2 || CV_MAT_TYPE(_dst->type) == CV_64FC2));

    CV_Assert( CV_IS_MAT(_cameraMatrix) &&
        _cameraMatrix->rows == 3 && _cameraMatrix->cols == 3 );

    cvConvert( _cameraMatrix, &matA );


    if( _distCoeffs )
    {
        CV_Assert( CV_IS_MAT(_distCoeffs) &&
            (_distCoeffs->rows == 1 || _distCoeffs->cols == 1) &&
            (_distCoeffs->rows*_distCoeffs->cols == 4 ||
             _distCoeffs->rows*_distCoeffs->cols == 5 ||
             _distCoeffs->rows*_distCoeffs->cols == 8 ||
             _distCoeffs->rows*_distCoeffs->cols == 12 ||
             _distCoeffs->rows*_distCoeffs->cols == 14));

        _Dk = cvMat( _distCoeffs->rows, _distCoeffs->cols,
            CV_MAKETYPE(CV_64F,CV_MAT_CN(_distCoeffs->type)), k);

        cvConvert( _distCoeffs, &_Dk );
        if (k[12] != 0 || k[13] != 0)
        {
            computeTiltProjectionMatrix<double>(k[12], k[13], NULL, NULL, NULL, &invMatTilt);
            computeTiltProjectionMatrix<double>(k[12], k[13], &matTilt, NULL, NULL);
        }
    }

    if( matR )
    {
        CV_Assert( CV_IS_MAT(matR) && matR->rows == 3 && matR->cols == 3 );
        cvConvert( matR, &_RR );
    }
    else
        cvSetIdentity(&_RR);

    if( matP )
    {
        double PP[3][3];
        CvMat _P3x3, _PP=cvMat(3, 3, CV_64F, PP);
        CV_Assert( CV_IS_MAT(matP) && matP->rows == 3 && (matP->cols == 3 || matP->cols == 4));
        cvConvert( cvGetCols(matP, &_P3x3, 0, 3), &_PP );
        cvMatMul( &_PP, &_RR, &_RR );
    }

    const CvPoint2D32f* srcf = (const CvPoint2D32f*)_src->data.ptr;
    const CvPoint2D64f* srcd = (const CvPoint2D64f*)_src->data.ptr;
    CvPoint2D32f* dstf = (CvPoint2D32f*)_dst->data.ptr;
    CvPoint2D64f* dstd = (CvPoint2D64f*)_dst->data.ptr;
    int stype = CV_MAT_TYPE(_src->type);
    int dtype = CV_MAT_TYPE(_dst->type);
    int sstep = _src->rows == 1 ? 1 : _src->step/CV_ELEM_SIZE(stype);
    int dstep = _dst->rows == 1 ? 1 : _dst->step/CV_ELEM_SIZE(dtype);

    double fx = A[0][0];
    double fy = A[1][1];
    double ifx = 1./fx;
    double ify = 1./fy;
    double cx = A[0][2];
    double cy = A[1][2];

    int n = _src->rows + _src->cols - 1;
    for( int i = 0; i < n; i++ )
    {
        double x, y, x0 = 0, y0 = 0, u, v;
        if( stype == CV_32FC2 )
        {
            x = srcf[i*sstep].x;
            y = srcf[i*sstep].y;
        }
        else
        {
            x = srcd[i*sstep].x;
            y = srcd[i*sstep].y;
        }
        u = x; v = y;
        x = (x - cx)*ifx;
        y = (y - cy)*ify;

        if( _distCoeffs ) {
            // compensate tilt distortion
            cv::Vec3d vecUntilt = invMatTilt * cv::Vec3d(x, y, 1);
            double invProj = vecUntilt(2) ? 1./vecUntilt(2) : 1;
            x0 = x = invProj * vecUntilt(0);
            y0 = y = invProj * vecUntilt(1);

            double error = std::numeric_limits<double>::max();
            // compensate distortion iteratively

            for( int j = 0; ; j++ )
            {
                if ((criteria.type & cv::TermCriteria::COUNT) && j >= criteria.maxCount)
                    break;
                if ((criteria.type & cv::TermCriteria::EPS) && error < criteria.epsilon)
                    break;
                double r2 = x*x + y*y;
                double icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
                double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x)+ k[8]*r2+k[9]*r2*r2;
                double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y+ k[10]*r2+k[11]*r2*r2;
                x = (x0 - deltaX)*icdist;
                y = (y0 - deltaY)*icdist;

                if(criteria.type & cv::TermCriteria::EPS)
                {
                    double r4, r6, a1, a2, a3, cdist, icdist2;
                    double xd, yd, xd0, yd0;
                    cv::Vec3d vecTilt;

                    r2 = x*x + y*y;
                    r4 = r2*r2;
                    r6 = r4*r2;
                    a1 = 2*x*y;
                    a2 = r2 + 2*x*x;
                    a3 = r2 + 2*y*y;
                    cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
                    icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
                    xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
                    yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

                    vecTilt = matTilt*cv::Vec3d(xd0, yd0, 1);
                    invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
                    xd = invProj * vecTilt(0);
                    yd = invProj * vecTilt(1);

                    double x_proj = xd*fx + cx;
                    double y_proj = yd*fy + cy;

                    error = sqrt( pow(x_proj - u, 2) + pow(y_proj - v, 2) );
                }
            }
        }

        double xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
        double yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
        double ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
        x = xx*ww;
        y = yy*ww;

        if( dtype == CV_32FC2 )
        {
            dstf[i*dstep].x = (float)x;
            dstf[i*dstep].y = (float)y;
        }
        else
        {
            dstd[i*dstep].x = x;
            dstd[i*dstep].y = y;
        }
    }
}

void cvUndistortPoints( const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
                   const CvMat* _distCoeffs,
                   const CvMat* matR = 0, const CvMat* matP = 0 )
{
    cvUndistortPointsInternal(_src, _dst, _cameraMatrix, _distCoeffs, matR, matP,
                              cv::TermCriteria(cv::TermCriteria::COUNT, 5, 0.01));
}

int cvRodrigues2( const CvMat* src, CvMat* dst, CvMat* jacobian)
{
    int depth, elem_size;
    int i, k;
    double J[27] = {0};
    CvMat matJ = cvMat( 3, 9, CV_64F, J );

    if( !CV_IS_MAT(src) )
        CV_Error( !src ? CV_StsNullPtr : CV_StsBadArg, "Input argument is not a valid matrix" );

    if( !CV_IS_MAT(dst) )
        CV_Error( !dst ? CV_StsNullPtr : CV_StsBadArg,
        "The first output argument is not a valid matrix" );

    depth = CV_MAT_DEPTH(src->type);
    elem_size = CV_ELEM_SIZE(depth);

    if( depth != CV_32F && depth != CV_64F )
        CV_Error( CV_StsUnsupportedFormat, "The matrices must have 32f or 64f data type" );

    if( !CV_ARE_DEPTHS_EQ(src, dst) )
        CV_Error( CV_StsUnmatchedFormats, "All the matrices must have the same data type" );

    if( jacobian )
    {
        if( !CV_IS_MAT(jacobian) )
            CV_Error( CV_StsBadArg, "Jacobian is not a valid matrix" );

        if( !CV_ARE_DEPTHS_EQ(src, jacobian) || CV_MAT_CN(jacobian->type) != 1 )
            CV_Error( CV_StsUnmatchedFormats, "Jacobian must have 32fC1 or 64fC1 datatype" );

        if( (jacobian->rows != 9 || jacobian->cols != 3) &&
            (jacobian->rows != 3 || jacobian->cols != 9))
            CV_Error( CV_StsBadSize, "Jacobian must be 3x9 or 9x3" );
    }

    if( src->cols == 1 || src->rows == 1 )
    {
        int step = src->rows > 1 ? src->step / elem_size : 1;

        if( src->rows + src->cols*CV_MAT_CN(src->type) - 1 != 3 )
            CV_Error( CV_StsBadSize, "Input matrix must be 1x3, 3x1 or 3x3" );

        if( dst->rows != 3 || dst->cols != 3 || CV_MAT_CN(dst->type) != 1 )
            CV_Error( CV_StsBadSize, "Output matrix must be 3x3, single-channel floating point matrix" );

        Point3d r;
        if( depth == CV_32F )
        {
            r.x = src->data.fl[0];
            r.y = src->data.fl[step];
            r.z = src->data.fl[step*2];
        }
        else
        {
            r.x = src->data.db[0];
            r.y = src->data.db[step];
            r.z = src->data.db[step*2];
        }

        double theta = norm(r);

        if( theta < DBL_EPSILON )
        {
            cvSetIdentity( dst );

            if( jacobian )
            {
                memset( J, 0, sizeof(J) );
                J[5] = J[15] = J[19] = -1;
                J[7] = J[11] = J[21] = 1;
            }
        }
        else
        {
            double c = cos(theta);
            double s = sin(theta);
            double c1 = 1. - c;
            double itheta = theta ? 1./theta : 0.;

            r *= itheta;

            Matx33d rrt( r.x*r.x, r.x*r.y, r.x*r.z, r.x*r.y, r.y*r.y, r.y*r.z, r.x*r.z, r.y*r.z, r.z*r.z );
            Matx33d r_x(    0, -r.z,  r.y,
                          r.z,    0, -r.x,
                         -r.y,  r.x,    0 );

            // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
            Matx33d R = c*Matx33d::eye() + c1*rrt + s*r_x;

            Mat(R).convertTo(cvarrToMat(dst), dst->type);

            if( jacobian )
            {
                const double I[] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
                double drrt[] = { r.x+r.x, r.y, r.z, r.y, 0, 0, r.z, 0, 0,
                                  0, r.x, 0, r.x, r.y+r.y, r.z, 0, r.z, 0,
                                  0, 0, r.x, 0, 0, r.y, r.x, r.y, r.z+r.z };
                double d_r_x_[] = { 0, 0, 0, 0, 0, -1, 0, 1, 0,
                                    0, 0, 1, 0, 0, 0, -1, 0, 0,
                                    0, -1, 0, 1, 0, 0, 0, 0, 0 };
                for( i = 0; i < 3; i++ )
                {
                    double ri = i == 0 ? r.x : i == 1 ? r.y : r.z;
                    double a0 = -s*ri, a1 = (s - 2*c1*itheta)*ri, a2 = c1*itheta;
                    double a3 = (c - s*itheta)*ri, a4 = s*itheta;
                    for( k = 0; k < 9; k++ )
                        J[i*9+k] = a0*I[k] + a1*rrt.val[k] + a2*drrt[i*9+k] +
                                   a3*r_x.val[k] + a4*d_r_x_[i*9+k];
                }
            }
        }
    }
    else if( src->cols == 3 && src->rows == 3 )
    {
        Matx33d U, Vt;
        Vec3d W;
        double theta, s, c;
        int step = dst->rows > 1 ? dst->step / elem_size : 1;

        if( (dst->rows != 1 || dst->cols*CV_MAT_CN(dst->type) != 3) &&
            (dst->rows != 3 || dst->cols != 1 || CV_MAT_CN(dst->type) != 1))
            CV_Error( CV_StsBadSize, "Output matrix must be 1x3 or 3x1" );

        Matx33d R = cvarrToMat(src);

        if( !checkRange(R, true, NULL, -100, 100) )
        {
            cvZero(dst);
            if( jacobian )
                cvZero(jacobian);
            return 0;
        }

        SVD::compute(R, W, U, Vt);
        R = U*Vt;

        Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

        s = std::sqrt((r.x*r.x + r.y*r.y + r.z*r.z)*0.25);
        c = (R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5;
        c = c > 1. ? 1. : c < -1. ? -1. : c;
        theta = acos(c);

        if( s < 1e-5 )
        {
            double t;

            if( c > 0 )
                r = Point3d(0, 0, 0);
            else
            {
                t = (R(0, 0) + 1)*0.5;
                r.x = std::sqrt(MAX(t,0.));
                t = (R(1, 1) + 1)*0.5;
                r.y = std::sqrt(MAX(t,0.))*(R(0, 1) < 0 ? -1. : 1.);
                t = (R(2, 2) + 1)*0.5;
                r.z = std::sqrt(MAX(t,0.))*(R(0, 2) < 0 ? -1. : 1.);
                if( fabs(r.x) < fabs(r.y) && fabs(r.x) < fabs(r.z) && (R(1, 2) > 0) != (r.y*r.z > 0) )
                    r.z = -r.z;
                theta /= norm(r);
                r *= theta;
            }

            if( jacobian )
            {
                memset( J, 0, sizeof(J) );
                if( c > 0 )
                {
                    J[5] = J[15] = J[19] = -0.5;
                    J[7] = J[11] = J[21] = 0.5;
                }
            }
        }
        else
        {
            double vth = 1/(2*s);

            if( jacobian )
            {
                double t, dtheta_dtr = -1./s;
                // var1 = [vth;theta]
                // var = [om1;var1] = [om1;vth;theta]
                double dvth_dtheta = -vth*c/s;
                double d1 = 0.5*dvth_dtheta*dtheta_dtr;
                double d2 = 0.5*dtheta_dtr;
                // dvar1/dR = dvar1/dtheta*dtheta/dR = [dvth/dtheta; 1] * dtheta/dtr * dtr/dR
                double dvardR[5*9] =
                {
                    0, 0, 0, 0, 0, 1, 0, -1, 0,
                    0, 0, -1, 0, 0, 0, 1, 0, 0,
                    0, 1, 0, -1, 0, 0, 0, 0, 0,
                    d1, 0, 0, 0, d1, 0, 0, 0, d1,
                    d2, 0, 0, 0, d2, 0, 0, 0, d2
                };
                // var2 = [om;theta]
                double dvar2dvar[] =
                {
                    vth, 0, 0, r.x, 0,
                    0, vth, 0, r.y, 0,
                    0, 0, vth, r.z, 0,
                    0, 0, 0, 0, 1
                };
                double domegadvar2[] =
                {
                    theta, 0, 0, r.x*vth,
                    0, theta, 0, r.y*vth,
                    0, 0, theta, r.z*vth
                };

                CvMat _dvardR = cvMat( 5, 9, CV_64FC1, dvardR );
                CvMat _dvar2dvar = cvMat( 4, 5, CV_64FC1, dvar2dvar );
                CvMat _domegadvar2 = cvMat( 3, 4, CV_64FC1, domegadvar2 );
                double t0[3*5];
                CvMat _t0 = cvMat( 3, 5, CV_64FC1, t0 );

                cvMatMul( &_domegadvar2, &_dvar2dvar, &_t0 );
                cvMatMul( &_t0, &_dvardR, &matJ );

                // transpose every row of matJ (treat the rows as 3x3 matrices)
                CV_SWAP(J[1], J[3], t); CV_SWAP(J[2], J[6], t); CV_SWAP(J[5], J[7], t);
                CV_SWAP(J[10], J[12], t); CV_SWAP(J[11], J[15], t); CV_SWAP(J[14], J[16], t);
                CV_SWAP(J[19], J[21], t); CV_SWAP(J[20], J[24], t); CV_SWAP(J[23], J[25], t);
            }

            vth *= theta;
            r *= vth;
        }

        if( depth == CV_32F )
        {
            dst->data.fl[0] = (float)r.x;
            dst->data.fl[step] = (float)r.y;
            dst->data.fl[step*2] = (float)r.z;
        }
        else
        {
            dst->data.db[0] = r.x;
            dst->data.db[step] = r.y;
            dst->data.db[step*2] = r.z;
        }
    }

    if( jacobian )
    {
        if( depth == CV_32F )
        {
            if( jacobian->rows == matJ.rows )
                cvConvert( &matJ, jacobian );
            else
            {
                float Jf[3*9];
                CvMat _Jf = cvMat( matJ.rows, matJ.cols, CV_32FC1, Jf );
                cvConvert( &matJ, &_Jf );
                cvTranspose( &_Jf, jacobian );
            }
        }
        else if( jacobian->rows == matJ.rows )
            cvCopy( &matJ, jacobian );
        else
            cvTranspose( &matJ, jacobian );
    }

    return 1;
}

void openCV3_4_0stereoRectify(cv::Mat _cameraMatrix1, cv::Mat _distCoeffs1,
		cv::Mat _cameraMatrix2, cv::Mat _distCoeffs2,
		cv::Size imageSize, cv::Mat _Rmat, cv::Mat _Tmat,
		cv::Mat &_Rmat1, cv::Mat & _Rmat2,
		cv::Mat & _Pmat1, cv::Mat & _Pmat2,
		cv::Mat & _Qmat, int flags,
		double alpha, cv::Size newImageSize,
		cv::Rect* validPixROI1, cv::Rect* validPixROI2);

void openCV3_4_0cvStereoRectify(const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
	const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
	CvSize imageSize, const CvMat* matR, const CvMat* matT,
	CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
	CvMat* matQ, int flags, double alpha, CvSize newImgSize,
	CvRect* roi1, CvRect* roi2);

void openCV3_4_0icvGetRectangles(const CvMat* cameraMatrix, const CvMat* distCoeffs,
	const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
	cv::Rect_<float>& inner, cv::Rect_<float>& outer);

void openCV3_4_0stereoRectify(
	cv::Mat _cameraMatrix1, cv::Mat _distCoeffs1,
	cv::Mat _cameraMatrix2, cv::Mat _distCoeffs2,
	cv::Size imageSize, cv::Mat _Rmat,
	cv::Mat _Tmat, cv::Mat & _Rmat1,
	cv::Mat & _Rmat2, cv::Mat & _Pmat1,
	cv::Mat & _Pmat2, cv::Mat & _Qmat,
	int flags, double alpha,
	cv::Size newImageSize, cv::Rect * validPixROI1,
	cv::Rect * validPixROI2)
{
	Mat cameraMatrix1 = _cameraMatrix1, cameraMatrix2 = _cameraMatrix2;
	Mat distCoeffs1 = _distCoeffs1, distCoeffs2 = _distCoeffs2;
	Mat Rmat = _Rmat, Tmat = _Tmat;
	CvMat c_cameraMatrix1 = cameraMatrix1;
	CvMat c_cameraMatrix2 = cameraMatrix2;
	CvMat c_distCoeffs1 = distCoeffs1;
	CvMat c_distCoeffs2 = distCoeffs2;
	CvMat c_R = Rmat, c_T = Tmat;

	int rtype = CV_64F;
	_Rmat1.create(3, 3, rtype);
	_Rmat2.create(3, 3, rtype);
	_Pmat1.create(3, 4, rtype);
	_Pmat2.create(3, 4, rtype);
	CvMat c_R1 = _Rmat1, c_R2 = _Rmat2, c_P1 = _Pmat1, c_P2 = _Pmat2;
	CvMat c_Q, *p_Q = 0;

	_Qmat.create(4, 4, rtype);
	p_Q = &(c_Q = _Qmat);

	CvMat *p_distCoeffs1 = distCoeffs1.empty() ? NULL : &c_distCoeffs1;
	CvMat *p_distCoeffs2 = distCoeffs2.empty() ? NULL : &c_distCoeffs2;
	openCV3_4_0cvStereoRectify(&c_cameraMatrix1, &c_cameraMatrix2, p_distCoeffs1, p_distCoeffs2,
		imageSize, &c_R, &c_T, &c_R1, &c_R2, &c_P1, &c_P2, p_Q, flags, alpha,
		newImageSize, (CvRect*)validPixROI1, (CvRect*)validPixROI2);
}


void openCV3_4_0cvStereoRectify(const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
	const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
	CvSize imageSize, const CvMat* matR, const CvMat* matT,
	CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
	CvMat* matQ, int flags, double alpha, CvSize newImgSize,
	CvRect* roi1, CvRect* roi2)
{
	double _om[3], _t[3] = { 0 }, _uu[3] = { 0,0,0 }, _r_r[3][3], _pp[3][4];
	double _ww[3], _wr[3][3], _z[3] = { 0,0,0 }, _ri[3][3];
	cv::Rect_<float> inner1, inner2, outer1, outer2;

	CvMat om = cvMat(3, 1, CV_64F, _om);
	CvMat t = cvMat(3, 1, CV_64F, _t);
	CvMat uu = cvMat(3, 1, CV_64F, _uu);
	CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
	CvMat pp = cvMat(3, 4, CV_64F, _pp);
	CvMat ww = cvMat(3, 1, CV_64F, _ww); // temps
	CvMat wR = cvMat(3, 3, CV_64F, _wr);
	CvMat Z = cvMat(3, 1, CV_64F, _z);
	CvMat Ri = cvMat(3, 3, CV_64F, _ri);
	double nx = imageSize.width, ny = imageSize.height;
	int i, k;

	if (matR->rows == 3 && matR->cols == 3)
		cvRodrigues2(matR, &om);          // get vector rotation
	else
		cvConvert(matR, &om); // it's already a rotation vector
	cvConvertScale(&om, &om, -0.5); // get average rotation
	cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging
	cvMatMul(&r_r, matT, &t);

	int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
	double c = _t[idx], nt = cvNorm(&t, 0, CV_L2);
	_uu[idx] = c > 0 ? 1 : -1;

	// calculate global Z rotation
	cvCrossProduct(&t, &uu, &ww);
	double nw = cvNorm(&ww, 0, CV_L2);
	if (nw > 0.0)
		cvConvertScale(&ww, &ww, acos(fabs(c) / nt) / nw);
	cvRodrigues2(&ww, &wR);

	// apply to both views
	cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
	cvConvert(&Ri, _R1);
	cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
	cvConvert(&Ri, _R2);
	cvMatMul(&Ri, matT, &t);

	// calculate projection/camera matrices
	// these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
	double fc_new = DBL_MAX;
	CvPoint2D64f cc_new[2] = { { 0,0 },{ 0,0 } };

	for (k = 0; k < 2; k++) {
		const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
		const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
		double dk1 = Dk && Dk->data.ptr ? cvmGet(Dk, 0, 0) : 0;
		double fc = cvmGet(A, idx ^ 1, idx ^ 1);
		if (dk1 < 0) {
			fc *= 1 + dk1*(nx*nx + ny*ny) / (4 * fc*fc);
		}
		fc_new = MIN(fc_new, fc);
	}

	for (k = 0; k < 2; k++)
	{
		const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
		const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
		CvPoint2D32f _pts[4];
		CvPoint3D32f _pts_3[4];
		CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
		CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);

		for (i = 0; i < 4; i++)
		{
			int j = (i<2) ? 0 : 1;
			_pts[i].x = (float)((i % 2)*(nx));
			_pts[i].y = (float)(j*(ny));
		}
		cvUndistortPoints(&pts, &pts, A, Dk, 0, 0);
		cvConvertPointsHomogeneous(&pts, &pts_3);

		//Change camera matrix to have cc=[0,0] and fc = fc_new
		double _a_tmp[3][3];
		CvMat A_tmp = cvMat(3, 3, CV_64F, _a_tmp);
		_a_tmp[0][0] = fc_new;
		_a_tmp[1][1] = fc_new;
		_a_tmp[0][2] = 0.0;
		_a_tmp[1][2] = 0.0;
		cvProjectPoints2(&pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts);
		CvScalar avg = cvAvg(&pts);
		cc_new[k].x = (nx) / 2 - avg.val[0];
		cc_new[k].y = (ny) / 2 - avg.val[1];
	}

	// vertical focal length must be the same for both images to keep the epipolar constraint
	// (for horizontal epipolar lines -- TBD: check for vertical epipolar lines)
	// use fy for fx also, for simplicity

	// For simplicity, set the principal points for both cameras to be the average
	// of the two principal points (either one of or both x- and y- coordinates)
	if (flags & CALIB_ZERO_DISPARITY)
	{
		cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
		cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
	}
	else if (idx == 0) // horizontal stereo
		cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
	else // vertical stereo
		cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

	cvZero(&pp);
	_pp[0][0] = _pp[1][1] = fc_new;
	_pp[0][2] = cc_new[0].x;
	_pp[1][2] = cc_new[0].y;
	_pp[2][2] = 1;
	cvConvert(&pp, _P1);

	_pp[0][2] = cc_new[1].x;
	_pp[1][2] = cc_new[1].y;
	_pp[idx][3] = _t[idx] * fc_new; // baseline * focal length
	cvConvert(&pp, _P2);

	alpha = MIN(alpha, 1.);

	openCV3_4_0icvGetRectangles(_cameraMatrix1, _distCoeffs1, _R1, _P1, imageSize, inner1, outer1);
	openCV3_4_0icvGetRectangles(_cameraMatrix2, _distCoeffs2, _R2, _P2, imageSize, inner2, outer2);

	{
		newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
		double cx1_0 = cc_new[0].x;
		double cy1_0 = cc_new[0].y;
		double cx2_0 = cc_new[1].x;
		double cy2_0 = cc_new[1].y;
		double cx1 = newImgSize.width*cx1_0 / imageSize.width;
		double cy1 = newImgSize.height*cy1_0 / imageSize.height;
		double cx2 = newImgSize.width*cx2_0 / imageSize.width;
		double cy2 = newImgSize.height*cy2_0 / imageSize.height;
		double s = 1.;

		if (alpha >= 0)
		{
			double s0 = std::max(std::max(std::max((double)cx1 / (cx1_0 - inner1.x), (double)cy1 / (cy1_0 - inner1.y)),
				(double)(newImgSize.width - cx1) / (inner1.x + inner1.width - cx1_0)),
				(double)(newImgSize.height - cy1) / (inner1.y + inner1.height - cy1_0));
			s0 = std::max(std::max(std::max(std::max((double)cx2 / (cx2_0 - inner2.x), (double)cy2 / (cy2_0 - inner2.y)),
				(double)(newImgSize.width - cx2) / (inner2.x + inner2.width - cx2_0)),
				(double)(newImgSize.height - cy2) / (inner2.y + inner2.height - cy2_0)),
				s0);

			double s1 = std::min(std::min(std::min((double)cx1 / (cx1_0 - outer1.x), (double)cy1 / (cy1_0 - outer1.y)),
				(double)(newImgSize.width - cx1) / (outer1.x + outer1.width - cx1_0)),
				(double)(newImgSize.height - cy1) / (outer1.y + outer1.height - cy1_0));
			s1 = std::min(std::min(std::min(std::min((double)cx2 / (cx2_0 - outer2.x), (double)cy2 / (cy2_0 - outer2.y)),
				(double)(newImgSize.width - cx2) / (outer2.x + outer2.width - cx2_0)),
				(double)(newImgSize.height - cy2) / (outer2.y + outer2.height - cy2_0)),
				s1);

			s = s0*(1 - alpha) + s1*alpha;
		}

		fc_new *= s;
		cc_new[0] = cvPoint2D64f(cx1, cy1);
		cc_new[1] = cvPoint2D64f(cx2, cy2);

		cvmSet(_P1, 0, 0, fc_new);
		cvmSet(_P1, 1, 1, fc_new);
		cvmSet(_P1, 0, 2, cx1);
		cvmSet(_P1, 1, 2, cy1);

		cvmSet(_P2, 0, 0, fc_new);
		cvmSet(_P2, 1, 1, fc_new);
		cvmSet(_P2, 0, 2, cx2);
		cvmSet(_P2, 1, 2, cy2);
		cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));

		if (roi1)
		{
			*roi1 = cv::Rect(cvCeil((inner1.x - cx1_0)*s + cx1),
				cvCeil((inner1.y - cy1_0)*s + cy1),
				cvFloor(inner1.width*s), cvFloor(inner1.height*s))
				& cv::Rect(0, 0, newImgSize.width, newImgSize.height);
		}

		if (roi2)
		{
			*roi2 = cv::Rect(cvCeil((inner2.x - cx2_0)*s + cx2),
				cvCeil((inner2.y - cy2_0)*s + cy2),
				cvFloor(inner2.width*s), cvFloor(inner2.height*s))
				& cv::Rect(0, 0, newImgSize.width, newImgSize.height);
		}
	}

	if (matQ)
	{
		double q[] =
		{
			1, 0, 0, -cc_new[0].x,
			0, 1, 0, -cc_new[0].y,
			0, 0, 0, fc_new,
			0, 0, -1. / _t[idx],
			(idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y) / _t[idx]
		};
		CvMat Q = cvMat(4, 4, CV_64F, q);
		cvConvert(&Q, matQ);
	}
}

void openCV3_4_0icvGetRectangles(const CvMat* cameraMatrix, const CvMat* distCoeffs,
	const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
	cv::Rect_<float>& inner, cv::Rect_<float>& outer)
{
	const int N = 9;
	int x, y, k;
	cv::Ptr<CvMat> _pts(cvCreateMat(1, N*N, CV_32FC2));
	CvPoint2D32f* pts = (CvPoint2D32f*)(_pts->data.ptr);

	for (y = k = 0; y < N; y++)
		for (x = 0; x < N; x++)
			pts[k++] = cvPoint2D32f((float)x*imgSize.width / (N - 1),
			(float)y*imgSize.height / (N - 1));

	cvUndistortPoints(_pts, _pts, cameraMatrix, distCoeffs, R, newCameraMatrix);

	float iX0 = -FLT_MAX, iX1 = FLT_MAX, iY0 = -FLT_MAX, iY1 = FLT_MAX;
	float oX0 = FLT_MAX, oX1 = -FLT_MAX, oY0 = FLT_MAX, oY1 = -FLT_MAX;
	// find the inscribed rectangle.
	// the code will likely not work with extreme rotation matrices (R) (>45%)
	for (y = k = 0; y < N; y++)
		for (x = 0; x < N; x++)
		{
			CvPoint2D32f p = pts[k++];
			oX0 = MIN(oX0, p.x);
			oX1 = MAX(oX1, p.x);
			oY0 = MIN(oY0, p.y);
			oY1 = MAX(oY1, p.y);

			if (x == 0)
				iX0 = MAX(iX0, p.x);
			if (x == N - 1)
				iX1 = MIN(iX1, p.x);
			if (y == 0)
				iY0 = MAX(iY0, p.y);
			if (y == N - 1)
				iY1 = MIN(iY1, p.y);
		}
	inner = cv::Rect_<float>(iX0, iY0, iX1 - iX0, iY1 - iY0);
	outer = cv::Rect_<float>(oX0, oY0, oX1 - oX0, oY1 - oY0);
}

#endif

StereoRectify::StereoRectify() {}
StereoRectify::~StereoRectify() {}

/**
@brief function to init stero rectify class
@param std::string intfile: file contain intrinsic parameter
@param std::string extfile: file contain extrinsic parameter
@param cv::Size imgsize: input image size
@return int
*/
int StereoRectify::init(std::string intfile, std::string extfile, cv::Size imgsize) {
	this->intfile = intfile;
	this->extfile = extfile;

	std::cout << "int File: " <<this->intfile <<std::endl;
	cv::FileStorage fs1(this->intfile, cv::FileStorage::READ);
	fs1["M1"] >> M1;
	fs1["D1"] >> D1;
	fs1["M2"] >> M2;
	fs1["D2"] >> D2;
	fs1.release();
	std::cout << "ext File: " <<this->extfile <<std::endl;
	cv::FileStorage fs2(this->extfile, cv::FileStorage::READ);
	fs2["R"] >> R;
	fs2["T"] >> T;
	fs2["R1"] >> R1;
	fs2["R2"] >> R2;
	fs2["P1"] >> P1;
	fs2["P2"] >> P2;
	fs2["Q"] >> Q;
	fs2.release();
	std::cout << "R : " << R <<std::endl;
	std::cout << "T : " << T <<std::endl;

	this->imgsize = imgsize;
#if (CV_MAJOR_VERSION >= 4)
	openCV3_4_0stereoRectify(M1, D1, M2, D2,
		imgsize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
#else
	cv::stereoRectify(M1, D1, M2, D2,
		imgsize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
#endif
	rect = validRoi[0] & validRoi[1];
	std::cout << "Rect : " << rect <<std::endl;
	//Precompute maps for cv::remap()
	initUndistortRectifyMap(M1, D1, R1, P1, imgsize, CV_32FC1, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imgsize, CV_32FC1, rmap[1][0], rmap[1][1]);

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			gpu_rmap[i][j].upload(rmap[i][j]);

	return 0;
}


/**
@brief function to rectify images
@param cv::Mat & leftImg: image of left view
@param cv::Mat & rightImg: image of right view
@return int
*/
int StereoRectify::rectify(cv::Mat & leftImg, cv::Mat & rightImg) {
	//TODO: change to GPU version
	cv::remap(leftImg, leftImg, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(rightImg, rightImg, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
	leftImg(rect).copyTo(leftImg);
	rightImg(rect).copyTo(rightImg);
	return 0;
}

int StereoRectify::rectify(cv::cuda::GpuMat & srcImg0, cv::cuda::GpuMat & dstImg0, cv::cuda::GpuMat & srcImg1, cv::cuda::GpuMat & dstImg1)
{
	cv::cuda::remap(srcImg0, dstImg0, gpu_rmap[0][0], gpu_rmap[0][1], cv::INTER_LINEAR);
	std::cout << "cv::cuda::remap done" << std::endl;
	cv::cuda::GpuMat tmp_dst0;
	dstImg0(rect).copyTo(tmp_dst0);
	std::cout << "cv::cuda::GpuMat::copyTo done" << std::endl;
	std::cout << "size = " << tmp_dst0.cols << " " << tmp_dst0.rows << std::endl;
	cv::cuda::resize(tmp_dst0, dstImg0, cv::Size(srcImg0.cols, srcImg0.rows));
	std::cout << "cv::cuda::resize done" << std::endl;

	cv::cuda::remap(srcImg1, dstImg1, gpu_rmap[1][0], gpu_rmap[1][1], cv::INTER_LINEAR);
	cv::cuda::GpuMat tmp_dst1;
	dstImg1(rect).copyTo(tmp_dst1);
	cv::cuda::resize(tmp_dst1, dstImg1, cv::Size(srcImg1.cols, srcImg1.rows));
	return 0;
}
