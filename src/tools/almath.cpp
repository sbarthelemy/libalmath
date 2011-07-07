/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved \n
*
*/

#include <almath/tools/almath.h>

#include <cmath>

namespace AL
{
  namespace Math
  {

    bool clipData(
      const float& pMin,
      const float& pMax,
      float& pData)
    {
      if (pData < pMin)
      {
        pData = pMin;
        return true;
      }
      else if (pData > pMax)
      {
        pData = pMax;
        return true;
      }
      else
      {
        return false;
      }
    }


    void diffLog(
      const AL::Math::Velocity6D&  pM,
      const AL::Math::Velocity6D&  pIn,
      AL::Math::Velocity6D&        pOut)
    {
      // pOut = diffLog(pM)*pIn;

      float nw;
      //w.norm_Frobenius(); // square root of sum of squares of the elements
      nw =(float) sqrt(pM.wxd*pM.wxd + pM.wyd*pM.wyd + pM.wzd*pM.wzd);

      float a, b;

      if (nw>=0.001f) // float epsilon = 0.001f;
      {
        float alpha, beta;
        float nw_2 = 0.5f*nw;
        alpha = nw_2/tanf(nw_2); //cot
        beta  = powf( (nw_2/sinf(nw_2)) ,2); //motionSin

        a = 2.0f*(1.0f - alpha) + 0.5f*(alpha-beta);
        b = 1.0f - alpha        + 0.5f*(alpha-beta);
      }
      else
      {
        a = +0.08333333333f; // 1/12
        b = -0.00138888889f; // -1/720
      }

      // tM = Idd + 0.5f*ad_x + a*ad_x2 + b*ad_x4;

      float wxd_2 = powf(pM.wxd,2);
      float wyd_2 = powf(pM.wyd,2);
      float wzd_2 = powf(pM.wzd,2);

      // this could probably be factorised some more.
      float a11 = a*( - wzd_2 - wyd_2 ) +
        b*( pM.wyd * ( wxd_2*pM.wyd - pM.wyd*( - wzd_2 - wyd_2)) - pM.wzd*( pM.wzd*( - wzd_2 - wyd_2 ) - wxd_2*pM.wzd )) +
        1.0f;
      float a12 = b*pM.wyd*(pM.wxd*( - wzd_2 - wxd_2 ) - pM.wxd*wyd_2 ) - 0.5f*pM.wzd + a*pM.wxd*pM.wyd;
      float a13 = -b*pM.wzd*( pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2 )) + a*pM.wxd*pM.wzd + 0.5f*pM.wyd;
      float a21 = -b*pM.wxd*( wxd_2*pM.wyd - pM.wyd*( - wzd_2 - wyd_2 )) + 0.5f*pM.wzd + a*pM.wxd*pM.wyd;
      float a22 = a*( - wzd_2 - wxd_2 ) +
        b*( pM.wzd*( wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2 )) - pM.wxd*( pM.wxd*( - wzd_2 - wxd_2 ) - pM.wxd*wyd_2 )) +
        1.0f;

      float a23 = b*pM.wzd*( pM.wyd*( - wyd_2 - wxd_2 ) - pM.wyd*wzd_2 ) + a*pM.wyd*pM.wzd - 0.5f*pM.wxd;
      float a31 = b*pM.wxd*( pM.wzd*( - wzd_2 - wyd_2 ) - wxd_2*pM.wzd ) + a*pM.wxd*pM.wzd - 0.5f*pM.wyd;
      float a32 = -b*pM.wyd*( wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2 )) + a*pM.wyd*pM.wzd + 0.5f*pM.wxd;
      float a33 = b*( pM.wxd*( pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2 )) - pM.wyd*( pM.wyd*( - wyd_2 - wxd_2 ) - pM.wyd*wzd_2 )) +
        a*( - wyd_2 - wxd_2 ) + 1.0f;

      float b11 = b*(-pM.wzd*(pM.wzd*(-2.0f*pM.wzd*pM.zd-2.0f*pM.wyd*pM.yd) - pM.wxd*(pM.wxd*pM.zd+pM.wzd*pM.xd) + (-wzd_2-wyd_2)*pM.zd-pM.wxd*pM.wzd*pM.xd) + pM.wyd*(-pM.wyd*(-2.0f*pM.wzd*pM.zd-2.0f*pM.wyd*pM.yd) + pM.wxd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - (-wzd_2-wyd_2)*pM.yd + pM.wxd*pM.wyd*pM.xd) - (pM.wzd*(-wzd_2-wyd_2)-wxd_2*pM.wzd)*pM.zd + (wxd_2*pM.wyd - pM.wyd*(-wzd_2-wyd_2))*pM.yd) + a*(-2.0f*pM.wzd*pM.zd - 2.0f*pM.wyd*pM.yd);

      float b12 = b*(pM.wyd*(pM.wxd*(-2.0f*pM.wzd*pM.zd-2.0f*pM.wxd*pM.xd) - pM.wyd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wxd*pM.wyd*pM.yd + (-wzd_2-wxd_2)*pM.xd) - pM.wzd*(-pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) + pM.wxd*pM.wyd*pM.zd + pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wyd*pM.wzd*pM.xd) + (pM.wxd*( - wzd_2 - wxd_2) - pM.wxd*wyd_2)*pM.yd) - 0.5f*pM.zd + a*(pM.wxd*pM.yd + pM.wyd*pM.xd);

      float b13 = b*(pM.wyd*(pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wzd*pM.yd + pM.wyd*pM.wzd*pM.xd) - pM.wzd*(pM.wzd*(pM.wxd*pM.zd + pM.wzd*pM.xd) + pM.wxd*pM.wzd*pM.zd - pM.wxd*(-2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) - (-wyd_2-wxd_2)*pM.xd) - (pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2 ))*pM.zd) + a*(pM.wxd*pM.zd + pM.wzd*pM.xd) + 0.5f*pM.yd;

      float b21 = b*(-pM.wxd*(-pM.wyd*( -2.0f*pM.wzd*pM.zd - 2.0f*pM.wyd*pM.yd) + pM.wxd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - ( - wzd_2 - wyd_2)*pM.yd + pM.wxd*pM.wyd*pM.xd) + pM.wzd*(pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wyd*pM.zd - pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) + pM.wxd*pM.wzd*pM.yd) - (wxd_2*pM.wyd - pM.wyd*( - wzd_2 - wyd_2))*pM.xd) + 0.5f*pM.zd + a*(pM.wxd*pM.yd + pM.wyd*pM.xd);

      float b22 = b*(pM.wzd*(-pM.wzd*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd) + pM.wyd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - ( - wzd_2 - wxd_2)*pM.zd + pM.wyd*pM.wzd*pM.yd) - pM.wxd*(pM.wxd*( -2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd) - pM.wyd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wxd*pM.wyd*pM.yd + ( - wzd_2 - wxd_2)*pM.xd) + (wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2))*pM.zd - (pM.wxd*( - wzd_2 - wxd_2) - pM.wxd*wyd_2)*pM.xd) + a*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd);

      float b23 = b*(pM.wzd*( - pM.wzd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*pM.wzd*pM.zd + pM.wyd*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) + ( - wyd_2 - wxd_2)*pM.yd) - pM.wxd*(pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wzd*pM.yd + pM.wyd*pM.wzd*pM.xd) + (pM.wyd*( - wyd_2 - wxd_2) - pM.wyd*wzd_2)*pM.zd) + a*(pM.wyd*pM.zd + pM.wzd*pM.yd) - 0.5f*pM.xd;

      float b31 = b*(pM.wxd*(pM.wzd*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wyd*pM.yd) - pM.wxd*(pM.wxd*pM.zd + pM.wzd*pM.xd) + ( - wzd_2 - wyd_2)*pM.zd - pM.wxd*pM.wzd*pM.xd) - pM.wyd*(pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wyd*pM.zd - pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) + pM.wxd*pM.wzd*pM.yd) + (pM.wzd*( - wzd_2 - wyd_2) - wxd_2*pM.wzd)*pM.xd) + a*(pM.wxd*pM.zd + pM.wzd*pM.xd) - 0.5f*pM.yd;

      float b32 = b*( - pM.wyd*( - pM.wzd*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd) + pM.wyd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - ( - wzd_2 - wxd_2)*pM.zd + pM.wyd*pM.wzd*pM.yd) + pM.wxd*( - pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) + pM.wxd*pM.wyd*pM.zd + pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wyd*pM.wzd*pM.xd) - (wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2))*pM.yd) + a*(pM.wyd*pM.zd + pM.wzd*pM.yd) + 0.5f*pM.xd;

      float b33 = b*( - pM.wyd*( - pM.wzd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*pM.wzd*pM.zd + pM.wyd*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) + ( - wyd_2 - wxd_2)*pM.yd) + pM.wxd*(pM.wzd*(pM.wxd*pM.zd + pM.wzd*pM.xd) + pM.wxd*pM.wzd*pM.zd - pM.wxd*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) - ( - wyd_2 - wxd_2)*pM.xd) - (pM.wyd*( - wyd_2 - wxd_2) - pM.wyd*wzd_2)*pM.yd + (pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2))*pM.xd) + a*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd);

      pOut.xd  = a11*pIn.xd + a12*pIn.yd + a13*pIn.zd + b11*pIn.wxd + b12*pIn.wyd + b13*pIn.wzd;
      pOut.yd  = a21*pIn.xd + a22*pIn.yd + a23*pIn.zd + b21*pIn.wxd + b22*pIn.wyd + b23*pIn.wzd;
      pOut.zd  = a31*pIn.xd + a32*pIn.yd + a33*pIn.zd + b31*pIn.wxd + b32*pIn.wyd + b33*pIn.wzd;
      pOut.wxd =                                        a11*pIn.wxd + a12*pIn.wyd + a13*pIn.wzd;
      pOut.wyd =                                        a21*pIn.wxd + a22*pIn.wyd + a23*pIn.wzd;
      pOut.wzd =                                        a31*pIn.wxd + a32*pIn.wyd + a33*pIn.wzd;
    } // end diffLog


    void invDiffLog(
      const AL::Math::Velocity6D&  pM,
      const AL::Math::Velocity6D&  pIn,
      AL::Math::Velocity6D&        pOut)
    {
      // pOut = diffLog(pM)*pIn;
      float nw;
      //w.norm_Frobenius(); // square root of sum of squares of the elements
      nw =(float) sqrt(pM.wxd*pM.wxd + pM.wyd*pM.wyd + pM.wzd*pM.wzd);

      float a, b;

      if (nw>=0.001f) // float epsilon = 0.001f;
      {
        float alpha, beta;
        float nw_2 = 0.5f*nw;
        alpha = nw_2/tanf(nw_2); //cot
        beta  = powf( (nw_2/sinf(nw_2)) ,2); //motionSin

        a = 2.0f*(1.0f - alpha) + 0.5f*(alpha-beta);
        b = 1.0f - alpha        + 0.5f*(alpha-beta);
      }
      else
      {
        a = +0.08333333333f; // 1/12
        b = -0.00138888889f; // -1/720
      }

      // tM = Idd + 0.5f*ad_x + a*ad_x2 + b*ad_x4;

      float wxd_2 = powf(pM.wxd,2);
      float wyd_2 = powf(pM.wyd,2);
      float wzd_2 = powf(pM.wzd,2);

      // this could probably be factorised some more.
      float a11 = a*( - wzd_2 - wyd_2 ) + b*( pM.wyd * ( wxd_2*pM.wyd - pM.wyd*( - wzd_2 - wyd_2)) - pM.wzd*( pM.wzd*( - wzd_2 - wyd_2 ) - wxd_2*pM.wzd )) + 1.0f;
      float a12 = b*pM.wyd*(pM.wxd*( - wzd_2 - wxd_2 ) - pM.wxd*wyd_2 ) - 0.5f*pM.wzd + a*pM.wxd*pM.wyd;
      float a13 = -b*pM.wzd*( pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2 )) + a*pM.wxd*pM.wzd + 0.5f*pM.wyd;
      float a21 = -b*pM.wxd*( wxd_2*pM.wyd - pM.wyd*( - wzd_2 - wyd_2 )) + 0.5f*pM.wzd + a*pM.wxd*pM.wyd;
      float a22 = a*( - wzd_2 - wxd_2 ) + b*( pM.wzd*( wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2 )) - pM.wxd*( pM.wxd*( - wzd_2 - wxd_2 ) - pM.wxd*wyd_2 ))+1.0f;
      float a23 = b*pM.wzd*( pM.wyd*( - wyd_2 - wxd_2 ) - pM.wyd*wzd_2 ) + a*pM.wyd*pM.wzd - 0.5f*pM.wxd;
      float a31 = b*pM.wxd*( pM.wzd*( - wzd_2 - wyd_2 ) - wxd_2*pM.wzd ) + a*pM.wxd*pM.wzd - 0.5f*pM.wyd;
      float a32 = -b*pM.wyd*( wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2 )) + a*pM.wyd*pM.wzd + 0.5f*pM.wxd;
      float a33 = b*( pM.wxd*( pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2 )) - pM.wyd*( pM.wyd*( - wyd_2 - wxd_2 ) - pM.wyd*wzd_2 )) + a*( - wyd_2 - wxd_2 ) + 1.0f;

      float b11 = b*(-pM.wzd*(pM.wzd*(-2.0f*pM.wzd*pM.zd-2.0f*pM.wyd*pM.yd) - pM.wxd*(pM.wxd*pM.zd+pM.wzd*pM.xd) + (-wzd_2-wyd_2)*pM.zd-pM.wxd*pM.wzd*pM.xd) + pM.wyd*(-pM.wyd*(-2.0f*pM.wzd*pM.zd-2.0f*pM.wyd*pM.yd) + pM.wxd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - (-wzd_2-wyd_2)*pM.yd + pM.wxd*pM.wyd*pM.xd) - (pM.wzd*(-wzd_2-wyd_2)-wxd_2*pM.wzd)*pM.zd + (wxd_2*pM.wyd - pM.wyd*(-wzd_2-wyd_2))*pM.yd) + a*(-2.0f*pM.wzd*pM.zd - 2.0f*pM.wyd*pM.yd);

      float b12 = b*(pM.wyd*(pM.wxd*(-2.0f*pM.wzd*pM.zd-2.0f*pM.wxd*pM.xd) - pM.wyd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wxd*pM.wyd*pM.yd + (-wzd_2-wxd_2)*pM.xd) - pM.wzd*(-pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) + pM.wxd*pM.wyd*pM.zd + pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wyd*pM.wzd*pM.xd) + (pM.wxd*( - wzd_2 - wxd_2) - pM.wxd*wyd_2)*pM.yd) - 0.5f*pM.zd + a*(pM.wxd*pM.yd + pM.wyd*pM.xd);

      float b13 = b*(pM.wyd*(pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wzd*pM.yd + pM.wyd*pM.wzd*pM.xd) - pM.wzd*(pM.wzd*(pM.wxd*pM.zd + pM.wzd*pM.xd) + pM.wxd*pM.wzd*pM.zd - pM.wxd*(-2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) - (-wyd_2-wxd_2)*pM.xd) - (pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2 ))*pM.zd) + a*(pM.wxd*pM.zd + pM.wzd*pM.xd) + 0.5f*pM.yd;

      float b21 = b*(-pM.wxd*(-pM.wyd*( -2.0f*pM.wzd*pM.zd - 2.0f*pM.wyd*pM.yd) + pM.wxd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - ( - wzd_2 - wyd_2)*pM.yd + pM.wxd*pM.wyd*pM.xd) + pM.wzd*(pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wyd*pM.zd - pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) + pM.wxd*pM.wzd*pM.yd) - (wxd_2*pM.wyd - pM.wyd*( - wzd_2 - wyd_2))*pM.xd) + 0.5f*pM.zd + a*(pM.wxd*pM.yd + pM.wyd*pM.xd);

      float b22 = b*(pM.wzd*(-pM.wzd*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd) + pM.wyd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - ( - wzd_2 - wxd_2)*pM.zd + pM.wyd*pM.wzd*pM.yd) - pM.wxd*(pM.wxd*( -2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd) - pM.wyd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wxd*pM.wyd*pM.yd + ( - wzd_2 - wxd_2)*pM.xd) + (wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2))*pM.zd - (pM.wxd*( - wzd_2 - wxd_2) - pM.wxd*wyd_2)*pM.xd) + a*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd);

      float b23 = b*(pM.wzd*( - pM.wzd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*pM.wzd*pM.zd + pM.wyd*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) + ( - wyd_2 - wxd_2)*pM.yd) - pM.wxd*(pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wzd*pM.yd + pM.wyd*pM.wzd*pM.xd) + (pM.wyd*( - wyd_2 - wxd_2) - pM.wyd*wzd_2)*pM.zd) + a*(pM.wyd*pM.zd + pM.wzd*pM.yd) - 0.5f*pM.xd;

      float b31 = b*(pM.wxd*(pM.wzd*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wyd*pM.yd) - pM.wxd*(pM.wxd*pM.zd + pM.wzd*pM.xd) + ( - wzd_2 - wyd_2)*pM.zd - pM.wxd*pM.wzd*pM.xd) - pM.wyd*(pM.wyd*(pM.wxd*pM.zd + pM.wzd*pM.xd) - pM.wxd*pM.wyd*pM.zd - pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) + pM.wxd*pM.wzd*pM.yd) + (pM.wzd*( - wzd_2 - wyd_2) - wxd_2*pM.wzd)*pM.xd) + a*(pM.wxd*pM.zd + pM.wzd*pM.xd) - 0.5f*pM.yd;

      float b32 = b*( - pM.wyd*( - pM.wzd*( - 2.0f*pM.wzd*pM.zd - 2.0f*pM.wxd*pM.xd) + pM.wyd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - ( - wzd_2 - wxd_2)*pM.zd + pM.wyd*pM.wzd*pM.yd) + pM.wxd*( - pM.wxd*(pM.wyd*pM.zd + pM.wzd*pM.yd) + pM.wxd*pM.wyd*pM.zd + pM.wzd*(pM.wxd*pM.yd + pM.wyd*pM.xd) - pM.wyd*pM.wzd*pM.xd) - (wyd_2*pM.wzd - pM.wzd*( - wzd_2 - wxd_2))*pM.yd) + a*(pM.wyd*pM.zd + pM.wzd*pM.yd) + 0.5f*pM.xd;

      float b33 = b*( - pM.wyd*( - pM.wzd*(pM.wyd*pM.zd + pM.wzd*pM.yd) - pM.wyd*pM.wzd*pM.zd + pM.wyd*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) + ( - wyd_2 - wxd_2)*pM.yd) + pM.wxd*(pM.wzd*(pM.wxd*pM.zd + pM.wzd*pM.xd) + pM.wxd*pM.wzd*pM.zd - pM.wxd*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd) - ( - wyd_2 - wxd_2)*pM.xd) - (pM.wyd*( - wyd_2 - wxd_2) - pM.wyd*wzd_2)*pM.yd + (pM.wxd*wzd_2 - pM.wxd*( - wyd_2 - wxd_2))*pM.xd) + a*( - 2.0f*pM.wyd*pM.yd - 2.0f*pM.wxd*pM.xd);

      float iaConst = 1.0f / ( a11*( a22*a33 - a23*a32 ) + a12 * ( a23*a31 - a21*a33 ) + a13*( a21*a32 - a22*a31 ));
      float ia11 = iaConst*( a22*a33 - a23*a32 );
      float ia12 = iaConst*( a13*a32 - a12*a33 );
      float ia13 = iaConst*( a12*a23 - a13*a22 );
      float ia21 = iaConst*( a23*a31 - a21*a33 );
      float ia22 = iaConst*( a11*a33 - a13*a31 );
      float ia23 = iaConst*( a13*a21 - a11*a23 );
      float ia31 = iaConst*( a21*a32 - a22*a31 );
      float ia32 = iaConst*( a12*a31 - a11*a32 );
      float ia33 = iaConst*( a11*a22 - a12*a21 );

      float ib11 = - ia13*( b33*ia31 + b32*ia21 + b31*ia11 ) - ia12*( b23*ia31 + b22*ia21 + b21*ia11 ) - ia11*( b13*ia31 + b12*ia21 + b11*ia11 );
      float ib12 = - ia13*( b33*ia32 + b32*ia22 + b31*ia12 ) - ia12*( b23*ia32 + b22*ia22 + b21*ia12 ) - ia11*( b13*ia32 + b12*ia22 + b11*ia12 );
      float ib13 = - ia13*( b33*ia33 + b32*ia23 + b31*ia13 ) - ia12*( b23*ia33 + b22*ia23 + b21*ia13 ) - ia11*( b13*ia33 + b12*ia23 + b11*ia13 );
      float ib21 = - ia23*( b33*ia31 + b32*ia21 + b31*ia11 ) - ia22*( b23*ia31 + b22*ia21 + b21*ia11 ) - ia21*( b13*ia31 + b12*ia21 + b11*ia11 );
      float ib22 = - ia23*( b33*ia32 + b32*ia22 + b31*ia12 ) - ia22*( b23*ia32 + b22*ia22 + b21*ia12 ) - ia21*( b13*ia32 + b12*ia22 + b11*ia12 );
      float ib23 = - ia23*( b33*ia33 + b32*ia23 + b31*ia13 ) - ia22*( b23*ia33 + b22*ia23 + b21*ia13 ) - ia21*( b13*ia33 + b12*ia23 + b11*ia13 );
      float ib31 = - ia33*( b33*ia31 + b32*ia21 + b31*ia11 ) - ia32*( b23*ia31 + b22*ia21 + b21*ia11 ) - ia31*( b13*ia31 + b12*ia21 + b11*ia11 );
      float ib32 = - ia33*( b33*ia32 + b32*ia22 + b31*ia12 ) - ia32*( b23*ia32 + b22*ia22 + b21*ia12 ) - ia31*( b13*ia32 + b12*ia22 + b11*ia12 );
      float ib33 = - ia33*( b33*ia33 + b32*ia23 + b31*ia13 ) - ia32*( b23*ia33 + b22*ia23 + b21*ia13 ) - ia31*( b13*ia33 + b12*ia23 + b11*ia13 );

      pOut.xd  = ia11*pIn.xd + ia12*pIn.yd + ia13*pIn.zd + ib11*pIn.wxd + ib12*pIn.wyd + ib13*pIn.wzd;
      pOut.yd  = ia21*pIn.xd + ia22*pIn.yd + ia23*pIn.zd + ib21*pIn.wxd + ib22*pIn.wyd + ib23*pIn.wzd;
      pOut.zd  = ia31*pIn.xd + ia32*pIn.yd + ia33*pIn.zd + ib31*pIn.wxd + ib32*pIn.wyd + ib33*pIn.wzd;
      pOut.wxd =                                           ia11*pIn.wxd + ia12*pIn.wyd + ia13*pIn.wzd;
      pOut.wyd =                                           ia21*pIn.wxd + ia22*pIn.wyd + ia23*pIn.wzd;
      pOut.wzd =                                           ia31*pIn.wxd + ia32*pIn.wyd + ia33*pIn.wzd;
    } // end invDiffLog


    Transform transformFromPosition3DAndRotation(
      const float x,
      const float y,
      const float z,
      const Rotation& pRotation)
    {
      Transform T = Transform();

      T.r1_c4 = x;
      T.r2_c4 = y;
      T.r3_c4 = z;

      T.r1_c1 = pRotation.r1_c1;
      T.r1_c2 = pRotation.r1_c2;
      T.r1_c3 = pRotation.r1_c3;

      T.r2_c1 = pRotation.r2_c1;
      T.r2_c2 = pRotation.r2_c2;
      T.r2_c3 = pRotation.r2_c3;

      T.r3_c1 = pRotation.r3_c1;
      T.r3_c2 = pRotation.r3_c2;
      T.r3_c3 = pRotation.r3_c3;

      return T;
    }


    Transform transformFromPosition3DAndRotation(
      const Position3D& pPosition,
      const Rotation&   pRotation)
    {
      Transform T = Transform();

      T.r1_c4 = pPosition.x;
      T.r2_c4 = pPosition.y;
      T.r3_c4 = pPosition.z;

      T.r1_c1 = pRotation.r1_c1;
      T.r1_c2 = pRotation.r1_c2;
      T.r1_c3 = pRotation.r1_c3;

      T.r2_c1 = pRotation.r2_c1;
      T.r2_c2 = pRotation.r2_c2;
      T.r2_c3 = pRotation.r2_c3;

      T.r3_c1 = pRotation.r3_c1;
      T.r3_c2 = pRotation.r3_c2;
      T.r3_c3 = pRotation.r3_c3;

      return T;
    }


    Position6D position6DFromVelocity6D(const Velocity6D& pIn)
    {
      Position6D pOut;
      pOut.x  = pIn.xd;
      pOut.y  = pIn.yd;
      pOut.z  = pIn.zd;
      pOut.wx = pIn.wxd;
      pOut.wy = pIn.wyd;
      pOut.wz = pIn.wzd;
      return pOut;
    }

    Position3D operator*(
      const Rotation&   pRot,
      const Position3D& pPos)
    {
      Position3D result;
      result.x = (pRot.r1_c1 * pPos.x) + (pRot.r1_c2 * pPos.y) + (pRot.r1_c3 * pPos.z);
      result.y = (pRot.r2_c1 * pPos.x) + (pRot.r2_c2 * pPos.y) + (pRot.r2_c3 * pPos.z);
      result.z = (pRot.r3_c1 * pPos.x) + (pRot.r3_c2 * pPos.y) + (pRot.r3_c3 * pPos.z);
      return result;
    }

    Velocity6D operator*(
      const float       pVal,
      const Position6D& pPos)
    {
      /** cyrille 27/04/2009 static ? **/

      Velocity6D pVel;
      pVel.xd  = pVal * pPos.x;
      pVel.yd  = pVal * pPos.y;
      pVel.zd  = pVal * pPos.z;
      pVel.wxd = pVal * pPos.wx;
      pVel.wyd = pVal * pPos.wy;
      pVel.wzd = pVal * pPos.wz;
      return pVel;
    }


    AL::Math::Rotation rotationFromAngleDirection(
      const float&                pTheta,
      const AL::Math::Position3D& pPos)
    {
      return AL::Math::rotationFromAngleDirection(
            pTheta,
            pPos.x,
            pPos.y,
            pPos.z);
    }

    void applyRotation(
      const AL::Math::Rotation& pRotation,
      AL::Math::Position3D&     pVector)
    {
      float x = pVector.x;
      float y = pVector.y;
      float z = pVector.z;
      pVector.x = x*pRotation.r1_c1 + y*pRotation.r1_c2 + z*pRotation.r1_c3;
      pVector.y = x*pRotation.r2_c1 + y*pRotation.r2_c2 + z*pRotation.r2_c3;
      pVector.z = x*pRotation.r3_c1 + y*pRotation.r3_c2 + z*pRotation.r3_c3;
    }


  } // namespace Math
} // namespace AL

