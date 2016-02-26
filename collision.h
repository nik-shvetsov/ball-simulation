#ifndef COLLISION_H
#define COLLISION_H

#include "gmpwall.h"
#include "ball.h"


class Collision {
public:
    Collision(){}

    Collision(Ball* ball1, Ball* ball2, double x)
    {
        _balls[0] = ball1;
        _balls[1] = ball2;
        this->_x = x;
        _colBW = false;

    }

    Collision(Ball* ball, PWall* wall, double x)
    {
        _balls[0] = ball;
        this->_wall = wall;
        this->_x = x;
        _colBW = true;
    }

    Ball* getBall(int i) const
    {
        return _balls[i];
    }

    PWall* getWall() const
    {
        return _wall;
    }

    double getX() const
    {
        return _x;
    }

    bool isColBW() const
    {
        return _colBW;
    }

    void updateX(double x)
    {
      _x = x;
    }

    //operators
    bool operator < (const  Collision& other)const
    {
        return _x < other._x;
    }

    bool operator == (const  Collision& other)const
    {
        if (_balls[0] == other._balls[0]) return true;
        if (!other._colBW && _balls[0] == other._balls[1]) return true;
        if (!(_colBW) && _balls[1] == other._balls[0]) return true;
        if (!(_colBW) && !(other._colBW) && _balls[1] == other._balls[1]) return true;
        return false;
    }

  ~Collision() {}

private:
    Ball* _balls[2];
    PWall* _wall;
    double _x;
    bool _colBW;

}; // END class collision



#endif // COLLISION_H
