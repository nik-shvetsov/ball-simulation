/**********************************************************************************
**
** Copyright (C) 1994 Narvik University College
** Contact: GMlib Online Portal at http://episteme.hin.no
**
** This file is part of the Geometric Modeling Library, GMlib.
**
** GMlib is free software: you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** GMlib is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with GMlib.  If not, see <http://www.gnu.org/licenses/>.
**
**********************************************************************************/

/*! \file gmpcurplane.c
 *
 *  Implementation of the PCurPlane template class.
 */


// for syntaxhighlighting
#include "../gmlib/modules/parametrics/src/surfaces/gmpplane.h"
#include "gmpcurplane.h"



  template <typename T>
  inline
  PCurPlane<T>::PCurPlane( const GMlib::DMatrix<GMlib::Vector<T,3>>& m) {

    _m = m;

    this->_dm = GMlib::GM_DERIVATION_EXPLICIT;
  }


  template <typename T>
  inline
  PCurPlane<T>::PCurPlane( const PCurPlane<T>& copy ) : GMlib::PSurf<T,3>( copy ) {

    _m = copy._m;
  }


  template <typename T>
  PCurPlane<T>::~PCurPlane() {}


  template <typename T>
  void PCurPlane<T>::eval(T u, T v, int d1, int d2, bool /*lu*/, bool /*lv*/ ) {

    this->_p.setDim( d1+1, d2+1 );

//    this->(_p[0][0] = ((1-u)*(1-u))

      GMlib::DVector<T> u1(3);
      u1[0]=(1-u)*(1-u);
      u1[1]=2*u*(1-u);
      u1[2]=u*u;

      GMlib::DVector<T> v1(3);
      v1[0]=(1-v)*(1-v);
      v1[1]=2*v*(1-v);
      v1[2]=v*v;

      this->_p[0][0] = u1*(_m^v1);

    if( this->_dm == GMlib::GM_DERIVATION_EXPLICIT ) {


      //u=((1-u)*(1-u),2*u*(1-u),u*u);
      //v=((1-v)*(1-v),2*v*(1-v),v*v);
      // 1st
        GMlib::DVector<T> uu1(3);
        uu1[0] = (-2*(1-u));
        uu1[1] = 2 - 4*u;
        uu1[2] = 2*u;

        GMlib::DVector<T> vv1(3);
        vv1[0] = (-2*(1-v));
        vv1[1] = 2 - 4*v;
        vv1[2] = 2*v;

      if(d1)            this->_p[1][0] = uu1*(_m^v1); // S_u
      if(d2)            this->_p[0][1] = u1*(_m^vv1); // S_v
      if(d1>1 && d2>1)  this->_p[1][1] = uu1*(_m^vv1); // S_uv

      // 2nd
      if(d1>1)          this->_p[2][0] = GMlib::Vector<T,3>(T(0)); // S_uu
      if(d2>1)          this->_p[0][2] = GMlib::Vector<T,3>(T(0)); // S_vv
      if(d1>1 && d2)    this->_p[2][1] = GMlib::Vector<T,3>(T(0)); // S_uuv
      if(d1   && d2>1)  this->_p[1][2] = GMlib::Vector<T,3>(T(0)); // S_uvv
      if(d1>1 && d2>1)  this->_p[2][2] = GMlib::Vector<T,3>(T(0)); // S_uuvv
	  
	  // 3rd
      if(d1>2)          this->_p[3][0] = GMlib::Vector<T,3>(T(0)); // S_uuu
      if(d2>2)          this->_p[0][3] = GMlib::Vector<T,3>(T(0)); // S_vvv
      if(d1>2 && d2)    this->_p[3][1] = GMlib::Vector<T,3>(T(0)); // S_uuuv
      if(d1   && d2>2)  this->_p[1][3] = GMlib::Vector<T,3>(T(0)); // S_uvvv
      if(d1>2 && d2>1)  this->_p[3][2] = GMlib::Vector<T,3>(T(0)); // S_uuuvv
      if(d1>1 && d2>2)  this->_p[2][3] = GMlib::Vector<T,3>(T(0)); // S_uuvvv
      if(d1>2 && d2>2)  this->_p[3][3] = GMlib::Vector<T,3>(T(0)); // S_uuuvvv

    }
  }

  template <typename T>
  T PCurPlane<T>::getEndPU()	{

    return T(1);
  }

  template <typename T>
  T PCurPlane<T>::getEndPV()	{

    return T(1);
  }


  template <typename T>
  T PCurPlane<T>::getStartPU() {

    return T(0);
  }


  template <typename T>
  T PCurPlane<T>::getStartPV() {

    return T(0);
  }

