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

/*! \file gmpwall.c
 *
 *  Implementation of the PWall template class.
 */


// for syntaxhighlighting
#include "../gmlib/modules/parametrics/src/surfaces/gmpplane.h"
#include "gmpwall.h"


  template <typename T>
  inline
  PWall<T>::PWall( const GMlib::Point<T,3>& p1, const GMlib::Point<T,3>& p2, const GMlib::Point<T,3>& p3, const GMlib::Point<T,3>& p4 ) {

    _p1 = p1;
    _p2 = p2;
    _p3 = p3;
    _p4 = p4;

    this->_dm = GMlib::GM_DERIVATION_EXPLICIT;
  }


  template <typename T>
  inline
  PWall<T>::PWall( const PWall<T>& copy ) : GMlib::PSurf<T,3>( copy ) {

    _p1    = copy._p1;
    _p2    = copy._p2;
    _p3    = copy._p3;
    _p4    = copy._p4;
  }


  template <typename T>
 PWall<T>::~PWall() {}

  template <typename T>
  void PWall<T>::eval(T u, T v, int d1, int d2, bool /*lu*/, bool /*lv*/ ) {

    this->_p.setDim( d1+1, d2+1 );

    this->_p[0][0] = _p1 + u * (_p2 - _p1) + v * (_p4 - _p1 + u *
                    (_p1 - _p2 + _p3 - _p4)) ;

    if( this->_dm == GMlib::GM_DERIVATION_EXPLICIT ) {

      // 1st
      if(d1)            this->_p[1][0] = _p2 - _p1 + v *
                                        (_p1 - _p2 + _p3 - _p4); // S_u
      if(d2)            this->_p[0][1] = _p4 - _p1 + u *
                                        (_p1 - _p2 + _p3 - _p4); // S_v
      if(d1>1 && d2>1)  this->_p[1][1] = _p1 - _p2 + _p3 - _p4; // S_uv

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
  T PWall<T>::getEndPU()	{

    return T(1);
  }

  template <typename T>
  T PWall<T>::getEndPV()	{

    return T(1);
  }


  template <typename T>
  T PWall<T>::getStartPU() {

    return T(0);
  }


  template <typename T>
  T PWall<T>::getStartPV() {

    return T(0);
  }

