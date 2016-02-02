#ifndef GMPWALL_H
#define GMPWALL_H


#include <parametrics/gmpplane>


class PWall : public GMlib::PPlane<float> {
public:
    using PPlane::PPlane;

const GMlib::Point<float,3>& getPoint();


};

const GMlib::Point<float, 3> &PWall::getPoint()
    {
      return this->_pt;
    }


#endif
