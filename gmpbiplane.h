#ifndef GMPBIPLANE
#define GMPBIPLANE

#include "../gmlib/modules/parametrics/src/gmpsurf.h"

  template <typename T>
  class PBiPlane : public GMlib::PSurf<T,3> {
    GM_SCENEOBJECT(PBiPlane)
  public:
    PBiPlane( const GMlib::Point<T,3>& p1, const GMlib::Point<T,3>& p2, const GMlib::Point<T,3>& p3, const GMlib::Point<T,3>& p4 );
    PBiPlane( const PBiPlane<T>& copy );
    virtual ~PBiPlane();

  protected:
    GMlib::Point<T,3>       _p1;
    GMlib::Point<T,3>		_p2;
    GMlib::Point<T,3>       _p3;
    GMlib::Point<T,3>       _p4;

    void                    eval(T u, T v, int d1, int d2, bool lu = true, bool lv = true );
    T                       getEndPU();
    T                       getEndPV();
    T                       getStartPU();
    T                       getStartPV();
  }; // END class PPlane

 // END namespace GMlib

// Include PPlane class function implementations
#include "gmpbiplane.c"

#endif // GMPBIPLANE
