#ifndef BALL_H
#define BALL_H

#include <parametrics/gmpsphere>
#include "gmpbiplane.h"
#include "gmpcurplane.h"
#include <gmParametricsModule>

#include <QDebug>

class Ball : public GMlib::PSphere<float> {
    GM_SCENEOBJECT(Ball)

public:
  Ball(double radius, double mass, GMlib::Vector<float,3> velocity, GMlib::PBezierSurf<float>* surface);
  ~Ball();

//methods for ball properties

    void setVelocity(const GMlib::Vector<float,3> velocity);
    GMlib::Vector<float,3> getVelocity();
    double getMass();
    GMlib::Vector<float,3> getDs();
//  void setLocation(GMlib::Point<float,3> &location);

    void updateX(double x);
    double getX();

    GMlib::Vector<float,3> getSurfNormal();
    void setUV(GMlib::PBezierSurf<float>* surface);

    void computeStep(double dt);

    void moveUp();
    void moveDown();
    void moveRight();
    void moveLeft();

protected:
  void localSimulate(double dt);

private:
  GMlib::Vector<float,3> _velocity;
  double _mass;

  GMlib::Vector<float,3> _dS;
  float _u;
  float _v;

  double _x;

  GMlib::PBezierSurf<float>* _surface;
  //std::shared_ptr<PBiPlane<float>> _surface;

  //GMlib::Point<float,3> _p;

}; // END class ball


#endif // BALL_H

//TODO:
// location variable in ball and estimateclppar there (arne notion)
// get rid of ball-wall velocityvect=0 when getlength>0.1
// corrS when ball-ball intersecting must be *0.50000001 CHECK IT! (arne notion)
// modify controls of player ball, add HUD
// fix problem with divpos when collide > 0.1
// check the algorithm of controller localsimulate
// when colliding ball-wall translate ball to wall before recalculaing(reversing) velocity vector (arne notion)
// add totation force into consideration when colliding (arne notion)
// fix high velocity (or sometimes low) problems and balls disappearing (see report for info)

//CMAKE options
//-DGMLIB_DIR=D:\Data\Projects\project_fem\gmlib\build
//-DCMAKE_BUILD_TYPE=Release //-DBUILD_TYPE=Release
//-DCMAKE_BUILD_TYPE=debug
//-DGM_DEBUG_OPENGL=on

