#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <parametrics/gmpsphere>
#include "collision.h"
//#include "surface type"

#include <QDebug>

class Controller:public GMlib::PSphere<float> {
    GM_SCENEOBJECT(PSphere)

public:

  Controller(GMlib::PBezierSurf<float>* surf);
  ~Controller();

    void insertBall(Ball* ball);
    void insertWall(PWall* wall);

    void findBBCol(Ball* ball1, Ball* ball2, GMlib::Array<Collision>& cols, double prevX);
    void findBWCol(Ball* ball, PWall* wall, GMlib::Array<Collision>& cols, double prevX);
    void handleBBCol(Ball* ball1, Ball* ball2, double dt_part);
    void handleBWCol(Ball* ball, PWall* wall, double dt_part);


protected:

    void localSimulate (double dt);

private:

    GMlib::Array<Collision> _arrCols;
    GMlib::Array<Ball*> _arrBalls;
    GMlib::Array<PWall*> _arrWalls;
    GMlib::PBezierSurf<float>* _surf;

}; // END class controller

#endif // CONTROLLER_H
