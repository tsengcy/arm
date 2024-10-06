#ifndef __DYNFRAME_HPP_
#define __DYNFRAME_HPP_

#include <robotics/frame.hpp>
#include <iostream>
#include <vector>
#include <memory>


template<typename T>
class dynFrame : public Frame<T>
{
public:
    dynFrame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype,
             T _upperLimit, T _lowerLimit, std::shared_ptr<Frame<T>> _parent,
             T _mass, Eigen::Matrix<T, 3, 3> _inertia, Eigen::Matrix<T, 4, 4> _TCOM);

    int get_Id(){return mnDynId;}

    void InverseDynamicBackward();

    Eigen::Matrix<T, -1, 1> get_Force(){return mforce;}
private:

    int mnDynId;

    T mMass;

    Eigen::Matrix<T, 3, 3> mInertia;

    Eigen::Matrix<T, -1, -1> mSpatialInertia;

    Eigen::Matrix<T, 4, 4> mTCOM; // transformation matrix from axis to COM

    int visited{0};

    Eigen::Matrix<T, -1, 1> mforce{Eigen::Matrix<T, -1, 1>::Zero(6)};

    T mTorque{0};
};

template class dynFrame<float>;
// template class dynFrame<double>;

typedef dynFrame<float> dynFramef;
// typedef dynFrame<double> dynFramed;

#endif //__DYNFRAME_HPP_