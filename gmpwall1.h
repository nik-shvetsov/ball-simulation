#ifndef GMPWALL
#define GMPWALL

#include "../gmlib/modules/parametrics/src/gmpsurf.h"
#include <parametrics/gmpplane>

  template <typename T>
  class PWall : public GMlib::PSurf<T,3> {
    GM_SCENEOBJECT(PWall)
  public:
    PWall( const GMlib::Point<T,3>& p1, const GMlib::Point<T,3>& p2, const GMlib::Point<T,3>& p3, const GMlib::Point<T,3>& p4 );
    PWall( const PWall<T>& copy );
    virtual ~PWall();

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

  }; // END class PWall

 // END namespace GMlib

// Include PPlane class function implementations
#include "gmpwall.c"

#endif // GMPWALL
