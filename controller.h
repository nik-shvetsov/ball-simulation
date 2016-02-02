#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <parametrics/gmpsphere>
#include "gmpwall.h"
#include "ball.h"
#include "gmpbiplane.h"
#include "collision.h"

//#include "colobject.h"


class Controller:public GMlib::PSphere<float> {
    GM_SCENEOBJECT(PSphere)

public:

  Controller(PBiPlane<float>* surf)
    {
        this->toggleDefaultVisualizer();
        this->replot(30,30,1,1);
        this->setVisible(false);
        this->_surf = surf;
        this->insert(_surf);
    }

    void insertBall(Ball* ball)
    {
        this->insert(ball);
        _arrBalls += ball;
    }

    void insertWall(PWall* wall)
    {
        this->insert(wall);
        _arrWalls += wall;
    }

  ~Controller() {
  }


protected:

    void localSimulate (double dt)
    {
        for (int i=0; i<_arrBalls.size();i++)
        {
            _arrBalls[i]->computeStep(dt);
        }

        for (int i=0; i<_arrBalls.size();i++)
        {
            for (int j=i+1; j<_arrBalls.size();j++)
            {
                findBBCol(_arrBalls[i],_arrBalls[j], _arrCols, 0);
            }
        }

        for (int i=0; i<_arrBalls.size();i++)
        {
            for (int j=0; j<_arrWalls.size();j++)
            {
                findBWCol(_arrBalls[i],_arrWalls[j], _arrCols, 0);
            }
        }

        while (_arrCols.getSize()>0)
        {
            _arrCols.sort();
            _arrCols.makeUnique();

            Collision col = _arrCols[0];
            _arrCols.removeFront();

            //checks!!

            if (col.isColBW())
            {
                handleBWCol(col.getBall(0),col.getWall(), (1-col.getX())*dt);
                col.getBall(0)->updateX(col.getX());

                for (int i = 0; i < _arrBalls.size(); i++)
                {
                    if (_arrBalls[i] != col.getBall(0))
                    {
                        findBBCol(_arrBalls[i], col.getBall(0), _arrCols, col.getX());
                    }
                }

                for (int i = 0; i < _arrWalls.size(); i++)
                {
                    if (_arrWalls[i] != col.getWall())
                    {
                        findBWCol(col.getBall(0), _arrWalls[i], _arrCols, col.getX());
                    }
                }
            }
            else
            {
                handleBBCol(col.getBall(0), col.getBall(1), (1-col.getX())*dt);
                col.getBall(0)->updateX(col.getX());
                col.getBall(1)->updateX(col.getX());

                for (int i = 0; i < _arrBalls.size(); i++)
                {
                    if (_arrBalls[i] != col.getBall(0) && _arrBalls[i] != col.getBall(1))
                    {
                        findBBCol(_arrBalls[i], col.getBall(0), _arrCols, col.getX());
                        findBBCol(_arrBalls[i], col.getBall(1), _arrCols, col.getX());
                    }
                }

                for (int i = 0; i < _arrWalls.size(); i++)
                {
                    if (_arrWalls[i] != col.getWall())
                    {
                        findBWCol(col.getBall(0), _arrWalls[i], _arrCols, col.getX());
                        findBWCol(col.getBall(1), _arrWalls[i], _arrCols, col.getX());
                    }
                }
            }
        }
    }

    void findBBCol(Ball* ball1, Ball* ball2, GMlib::Array<Collision>& cols, double prX)
    {
        GMlib::Vector<float,3> divDs = ball1->getDs() - ball2->getDs(); //DS
        GMlib::Point<float,3> divPos = ball1->getCenterPos() - ball2->getCenterPos(); //q
        double sumRad = ball1->getRadius() + ball2->getRadius(); //r

        //a(x^2)+ bx + c = 0
        double a = divDs*divDs;
        double b = 2*(divPos*divDs);
        double c = (divPos*divPos) - (sumRad*sumRad);

        double alterDskr =  b*b-a*c;

        if (alterDskr>0)
        {
            if (std::abs(a)<0.0000001f)
            {
                return;
            }
            else
            {
                double x = (-b - std::sqrt(alterDskr))/a;
                if (prX < x && x <= 1) //&& x>0
                {
                    cols+= Collision(ball1,ball2,x);
                }
            }
        }
    }

    void findBWCol(Ball* ball, PWall* wall, GMlib::Array<Collision>& cols, double prX)
    {
        GMlib::Point<float,3> p = ball->getPos();
        double r = ball->getRadius();

        GMlib::Vector<float,3> n = wall->getNormal();
        GMlib::Vector<float,3> d = wall->getPoint() - p; //

        GMlib::Vector<float,3> dS = ball->getDs();

        double x = (r+d*n)/(dS*n); //double x = (r-d*n)/(dS*n);

        if (prX < x && x <= 1)
        {
            cols.insertAlways(Collision(ball,wall,x));
        }

    }

    void handleBBCol(Ball* ball1, Ball* ball2, double dt_con) //check!
    {
        GMlib::Vector<float,3> vel_upd1 = ball1->getVelocity();
        GMlib::Vector<float,3> vel_upd2 = ball2->getVelocity();

        GMlib::UnitVector<float,3> d = ball2->getPos() - ball1->getPos();

        float dd = d*d;

        GMlib::Vector<float,3> v1 = ((ball1->getVelocity() * d)/dd)*d;
        GMlib::Vector<float,3> v1n = ball1->getVelocity() - v1;
        GMlib::Vector<float,3> v2 = ((ball2->getVelocity() * d)/dd)*d;
        GMlib::Vector<float,3> v2n = ball2->getVelocity() - v2;

        float mass1 = ball1->getMass();
        float mass2 = ball2->getMass();

        GMlib::Vector<float,3> v11 = ((mass1-mass2)/(mass1+mass2))*v1+((2*mass2)/(mass1+mass2))*v2;
        GMlib::Vector<float,3> v22 = ((mass2-mass1)/(mass2+mass1))*v2+((2*mass1)/(mass2+mass1))*v1;

        vel_upd1 = v11 + v1n;
        vel_upd2 = v22 + v2n;

        ball1->setVelocity(vel_upd1);
        ball1->computeStep(dt_con);


        ball2->setVelocity(vel_upd2);
        ball2->computeStep(dt_con);


    }

    void handleBWCol(Ball* ball, PWall* wall, double dt_con)
    {
        GMlib::Vector<float,3> velocity_upd = ball->getVelocity();
        GMlib::Vector<float,3> norm = wall->getNormal();
        velocity_upd -= (2 * (velocity_upd*norm)) * norm;
        ball->setVelocity(velocity_upd);
        ball->computeStep(dt_con);
    }



private:
    GMlib::Array<Collision> _arrCols;
    GMlib::Array<Ball*> _arrBalls;
    GMlib::Array<PWall*> _arrWalls;
    PBiPlane<float>* _surf;
    //GMlib::Array<Ball*> _arrX;




}; // END class controller



#endif // CONTROLLER_H
