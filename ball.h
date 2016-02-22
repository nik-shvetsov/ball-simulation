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
  using PSphere::PSphere;

  Ball(double radius, double mass, GMlib::Vector<float,3> velocity, GMlib::PBezierSurf<float>* surface)
      :GMlib::PSphere<float>(radius)
  {
      this->_radius = radius;
      this->_mass = mass;
      this->_velocity = velocity;
      this->_surface = surface;
      this->_x=0;

      this->_surface->estimateClpPar(this->getPos(),_u,_v); //evaluating _u, _v
  }

  ~Ball() {
  }

//methods for setting ball properties

    void setVelocity(const GMlib::Vector<float,3> velocity)
    {
        _velocity = velocity;
    }

    GMlib::Vector<float,3> getVelocity()
    {
        return _velocity;
    }

    double getMass()
    {
        return _mass;
    }

    GMlib::Vector<float,3> getDs()
    {
        return _dS;
    }

//    void setLocation(GMlib::Point<float,3> &location)
//    {
//        _location = location;
//        translate(location+_velocity); //point1 = vector + point2
//    }

    void updateX(double x)
    {
        _x=x;
    }

    double getX()
    {
        return _x;
    }

    void setUV(GMlib::PBezierSurf<float>* surface)
    {
        surface->estimateClpPar(this->getPos(), _u, _v);
    }

    GMlib::Vector<float,3> getSurfNormal()
    {
        _surface->getClosestPoint(this->getPos(),_u,_v);
        GMlib::DMatrix<GMlib::Vector<float,3>> sMatrix = _surface->evaluate(_u,_v, 1,1);
        GMlib::UnitVector<float,3> norm = sMatrix[0][1] ^ sMatrix[1][0];
        return norm;
    }

    void computeStep(double dt)
    {
        static auto g = GMlib::Vector<float,3>(0,0,-9.8);
        _dS = dt * _velocity + 0.5 * dt * dt * g;

        _surface->getClosestPoint(this->getPos()+_dS,_u,_v);// _p = this->getPos()+_dS

        GMlib::DMatrix<GMlib::Vector<float,3>> sMatrix =_surface->evaluate(_u,_v,1,1);
        GMlib::UnitVector<float,3> norm = sMatrix[0][1] ^ sMatrix[1][0];  //norm.normalize();

        _dS = sMatrix[0][0]+(_radius*norm)-this->getPos();

        double v1 = _velocity*_velocity + 2.0*(g*_dS); //

        _velocity+=dt*g;
        _velocity-=(_velocity*norm)*norm; //should be: _velocity+=(norm*_velocity)*norm; ?

        double v2 = _velocity*_velocity; //

        if(v2 > 0.0001) //
        { //
            if(v1 > 0.0001) //
            _velocity *= std::sqrt(v1/v2); //
        } //
    }

    void moveUp()
    {
        GMlib::Vector<float,3> newVelVect = this->getVelocity();
        if (newVelVect[1] < 8.0 && newVelVect[1] > -8.0)
        {
            if (newVelVect[1] < 0.0)
            {
                newVelVect[1] = 0.0;
            }

            newVelVect[1] += 0.5;
            newVelVect[0] *= 0.5;
            //newVelVect[2] *= 0.5;
            this->setVelocity(newVelVect);

//        qDebug() << newVelVect[0];
//        qDebug() << "";
//        qDebug() << newVelVect[1];
//        qDebug() << "";
//        qDebug() << newVelVect[2];
//        qDebug() << "--------------";
        }

    }
    void moveDown()
    {
        GMlib::Vector<float,3> newVelVect = this->getVelocity();
        if (newVelVect[1] < 8.0 && newVelVect[1] > -8.0)
        {
            if (newVelVect[1] > 0.0)
            {
                newVelVect[1] = 0.0;
            }

            newVelVect[1] -= 0.5;
            newVelVect[0] *= 0.5;
            //newVelVect[2] *= 0.5;
            this->setVelocity(newVelVect);
        }
    }
    void moveRight()
    {
        GMlib::Vector<float,3> newVelVect = this->getVelocity();
        if (newVelVect[0] < 8.0 && newVelVect[0] > -8.0)
        {
            if (newVelVect[0] < 0.0)
            {
                newVelVect[0] = 0.0;
            }

            newVelVect[0] += 0.5;
            newVelVect[1] *= 0.5;
            //newVelVect[2] *= 0.5;
            this->setVelocity(newVelVect);
        }
    }
    void moveLeft()
    {
        GMlib::Vector<float,3> newVelVect = this->getVelocity();
        if (newVelVect[0] < 8.0 && newVelVect[0] > -8.0)
        {
            if (newVelVect[0] > 0.0)
            {
                newVelVect[0] = 0.0;
            }

            newVelVect[0] -= 0.5;
            newVelVect[1] *= 0.5;
            //newVelVect[2] *= 0.5;
            this->setVelocity(newVelVect);
        }
    }



protected:
  void localSimulate(double dt)
  {
    rotateGlobal(GMlib::Angle(_dS.getLength()/this->getRadius()), this->getSurfNormal()^_dS);
    //rotateParent(_dS.getLength(), this->getGlobalPos(), this->getSurfNormal()^_dS);
    this->translateParent(_dS);

    //computeStep(dt);
    //qDebug() << "test";
  }

private:
  GMlib::Vector<float,3> _velocity;
  double _mass;

  GMlib::Vector<float,3> _dS;
  float _u;
  float _v;

  double _x;

  GMlib::PBezierSurf<float>* _surface;
  //std::shared_ptr<PBiPlane<float>> _surface;

  GMlib::Point<float,3> _p; //

}; // END class ball


#endif // BALL_H
