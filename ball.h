#ifndef BALL_H
#define BALL_H


#include <parametrics/gmpsphere>
#include "gmpbiplane.h"


class Ball : public GMlib::PSphere<float> {
    GM_SCENEOBJECT(Ball)
public:
  using PSphere::PSphere;

  Ball(double radius, double mass, GMlib::Vector<float,3> velocity, GMlib::Point<float,3> location, PBiPlane<float>* surface)
      :GMlib::PSphere<float>(radius)
  {
      this->_radius = radius;
      this->_mass = mass;
      this->_velocity = velocity;
      this->_location = location;
      this->_surface = surface;
      _x=0;

      _surface->estimateClpPar(this->getPos(),_u,_v); //evaluating _u, _v

      this->setLocation(_location);
  }


  ~Ball() {
  }

//setting ball properties

    void setVelocity(const GMlib::Vector<float,3> &velocity)
    {
        _velocity = velocity;
    }

    GMlib::Vector<float,3> getVelocity()
    {
        return _velocity;
    }

    void setMass(double mass)
    {
        _mass = mass;
    }

    float getMass()
    {
        return _mass;

    }

    GMlib::Vector<float,3> getDs()
    {
        return _dS;
    }

    void setLocation(GMlib::Point<float,3> location)
    {
        _location = location;
        translate(location+_velocity); //point1 = vector + point2
    }

    GMlib::Point<float,3> getLocation()
    {
        return _location; //equal to getPos()
    }

    void updateX(double x)
    {
        this->_x=x;
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

        _surface->getClosestPoint(getPos()+_dS,_u,_v);
        GMlib::DMatrix<GMlib::Vector<float,3>> sMatrix =_surface->evaluate(_u,_v,1,1);
        GMlib::UnitVector<float,3> norm = sMatrix[0][1] ^ sMatrix[1][0];
        _dS = sMatrix[0][0]+(_radius*norm)-getPos();
        _velocity+=dt*g;
        _velocity-=(norm*_velocity)*norm; //should be: _velocity+=(norm*_velocity)*norm; ?

        //deal with cout once case
    }



protected:
  void localSimulate(double dt) override {

    computeStep(dt);
    rotate(_dS.getLength(), this->getSurfNormal()^_dS);
    translateGlobal(_dS);

    //rotateParent(_dS.getLength(), this->getGlobalPos(), this->getSurfNormal()^_dS);
    //translate(3*dt * GMlib::Vector<float,3>( 1.0f, 1.0f, 0.0f ));
    //rotate( GMlib::Angle(90) * dt, GMlib::Vector<float,3>( 0.0f, 0.0f, 1.0f ) );
    //rotate( GMlib::Angle(180) * dt, GMlib::Vector<float,3>( 1.0f, 1.0f, 0.0f ) );

  }

private:
  GMlib::Point<float,3> _location;
  GMlib::Vector<float,3> _velocity;
  double _mass;

  GMlib::Vector<float,3> _dS;
  float _u;
  float _v;

  double _x;

  PBiPlane<float>* _surface;
  //std::shared_ptr<PBiPlane<float>> _surface;

}; // END class ball


#endif // BALL_H
