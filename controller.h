#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <parametrics/gmpsphere>
#include "collision.h"
//#include "surface type"

#include <QDebug>

class Controller:public GMlib::PSphere<float> {
    GM_SCENEOBJECT(PSphere)

public:

  Controller(GMlib::PBezierSurf<float>* surf)
    {
        this->toggleDefaultVisualizer();
        this->replot(30,30,1,1);
        this->setVisible(false); //hiding controller sphere

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

    void findBBCol(Ball* ball1, Ball* ball2, GMlib::Array<Collision>& cols, double prevX)
    {
        GMlib::Vector<float,3> divDs = ball1->getDs() - ball2->getDs(); //DS = k
        //GMlib::Point<float,3> divPos = ball1->getCenterPos() - ball2->getCenterPos(); //q
        GMlib::Point<float,3> divPos = ball1->getPos() - ball2->getPos(); //q
        double sumRad = (ball1->getRadius()) + (ball2->getRadius()); //r

        //a(x^2)+ bx + c = 0
        double a = divDs*divDs;
        double b = 2*divPos*divDs;
        double c = (divPos*divPos) - (sumRad*sumRad);

        double alterDskr =  b*b-4*a*c;

        //if (divPos < (sumRad + 0.1))

        if (c<0) //check if balls get intersected
        {
            float corrS = 0.51*(sumRad - divPos.getLength())/divPos.getLength();
            ball1->translate(corrS*divPos);
            ball2->translate(-corrS*divPos);

            divPos *= 1+(2*corrS);
            b = (divPos*divDs);
            c = (divPos*divPos) - (sumRad*sumRad);
        }

        if (alterDskr>0)
        {
            double x = (-b - std::sqrt(alterDskr))/(2.0*a);
            if (prevX < x && x <= 1.0)
            {
                cols.insertAlways(Collision(ball1,ball2,x));
                //cols+= Collision(ball1,ball2,x);
                qDebug() << (ball1->getPos() - ball2->getPos()).getLength();
                //qDebug() << x;
            }
        }
    }

    void findBWCol(Ball* ball, PWall* wall, GMlib::Array<Collision>& cols, double prevX)
    {
        GMlib::Point<float,3> p = ball->getPos();
        double r = ball->getRadius();
        GMlib::Vector<float,3> n = wall->getNormal();

        wall->getClosestPoint(this->getPos(),_u,_v);
        GMlib::DMatrix<GMlib::Vector<float,3>> sMatrix = wall->evaluate(_u,_v, 1,1);

        GMlib::Vector<float,3> d = sMatrix[0][0] - p;
        double dn = d * n;
        GMlib::Vector<float,3> dS = ball->getDs();

        if (dn + r > 0.0) //if ball and wall intersected
        {
            ball->translate(2.0*(dn + r) * wall->getNormal());
            dn -= 2.0 * (dn + r);
        }

        if ((dS * n) < -0.00000001)
        {
            double x = (r + dn)/(dS*n); //double x = (r-d*n)/(dS*n);
            if (prevX < x && x <= 1.0)
            {
                cols.insertAlways(Collision(ball,wall,x));
                //cols+=(Collision(ball,wall,x));
            }
        }
    }

    void handleBBCol(Ball* ball1, Ball* ball2, double dt_part)
    {

        GMlib::Vector<float,3> vel_upd1 = ball1->getVelocity();
        GMlib::Vector<float,3> vel_upd2 = ball2->getVelocity();

        GMlib::UnitVector<float,3> d = ball2->getPos() - ball1->getPos();

        double dd = d*d;

        GMlib::Vector<float,3> v1 = ((ball1->getVelocity() * d)/dd)*d;
        GMlib::Vector<float,3> v1n = ball1->getVelocity() - v1;
        GMlib::Vector<float,3> v2 = ((ball2->getVelocity() * d)/dd)*d;
        GMlib::Vector<float,3> v2n = ball2->getVelocity() - v2;

        double mass1 = ball1->getMass();
        double mass2 = ball2->getMass();

        GMlib::Vector<float,3> v11 = ((mass1-mass2)/(mass1+mass2))*v1+((2*mass2)/(mass1+mass2))*v2;
        GMlib::Vector<float,3> v22 = ((mass2-mass1)/(mass2+mass1))*v2+((2*mass1)/(mass2+mass1))*v1;

        vel_upd1 = v11 + v1n;
        vel_upd2 = v22 + v2n;

        ball1->setVelocity(vel_upd1);
        ball1->computeStep(dt_part);

        ball2->setVelocity(vel_upd2);
        ball2->computeStep(dt_part);

        //qDebug() << (ball1->getPos() - ball2->getPos()).getLength();
    }

    void handleBWCol(Ball* ball, PWall* wall, double dt_part)
    {
        GMlib::Vector<float,3> velocity_upd = ball->getVelocity();
        GMlib::Vector<float,3> norm = wall->getNormal();
        velocity_upd -= (2.0*(velocity_upd*norm)) * norm; //reflection of velocity vector

        if (ball->getVelocity().getLength() > 0.1) //check if the ball have non-zero(close to zero) velocity vector
        {
            ball->setVelocity(velocity_upd);
            ball->computeStep(dt_part);
        }
        else
        {
            ball->setVelocity(GMlib::Vector<float,3>(0,0,0));
        }
    }


protected:

    void localSimulate (double dt)
    {
        for (int i=0; i<_arrBalls.size();i++)
        {
            _arrBalls[i]->computeStep(dt); //compute step for all balls
        }

        for (int i=0; i<_arrBalls.size();i++)
        {
            for (int j=i+1; j<_arrBalls.size();j++)
            {
                findBBCol(_arrBalls[i],_arrBalls[j], _arrCols, 0); //find all ball-ball collisions
            }
        }

        for (int i=0; i<_arrBalls.size();i++)
        {
            for (int j=0; j<_arrWalls.size();j++)
            {
                findBWCol(_arrBalls[i],_arrWalls[j], _arrCols, 0); //find all ball-wall collisions
            }
        }

        while (_arrCols.getSize()>0)
        {
            _arrCols.sort();
            _arrCols.makeUnique();

            Collision col = _arrCols[0];
            _arrCols.removeFront();

            //checks

            if (col.isColBW()) //if collision is between ball and wall
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
            else //if collision is between ball and ball
            {
                col.getBall(0)->updateX(col.getX());
                col.getBall(1)->updateX(col.getX());

                handleBBCol(col.getBall(0), col.getBall(1), (1-col.getX())*dt);

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

            //
        }
    }

private:

    GMlib::Array<Collision> _arrCols;
    GMlib::Array<Ball*> _arrBalls;
    GMlib::Array<PWall*> _arrWalls;
    GMlib::PBezierSurf<float>* _surf;

}; // END class controller



#endif // CONTROLLER_H
