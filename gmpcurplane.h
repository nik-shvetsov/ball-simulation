#ifndef GMPCURPLANE
#define GMPCURPLANE

#include "../gmlib/modules/parametrics/src/gmpsurf.h"

  template <typename T>
  class PCurPlane : public GMlib::PSurf<T,3> {
    GM_SCENEOBJECT(PCurPlane)
  public:
    PCurPlane( const GMlib::DMatrix<GMlib::Vector<T,3>>& m);
    PCurPlane( const PCurPlane<T>& copy );
    virtual ~PCurPlane();

  protected:
    GMlib::DMatrix<GMlib::Vector<T,3>> _m;


    void                    eval(T u, T v, int d1, int d2, bool lu = true, bool lv = true );
    T                       getEndPU();
    T                       getEndPV();
    T                       getStartPU();
    T                       getStartPV();
  }; // END class PPlane

 // END namespace GMlib

// Include PPlane class function implementations
#include "gmpcurplane.c"

#endif // GMPCURPLANE
