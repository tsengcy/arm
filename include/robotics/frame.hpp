#ifndef __FRAME_HPP_
#define __FRAME_HPP_

#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>


int nId = 0;

template<typename T>
class Frame : public std::enable_shared_from_this<Frame<T>>
{
public:
    /** constructor of frame by using modified dh parameter */
    Frame(T _a, T _alpha, T _d, T _theta, std::shared_ptr<Frame<T>> _parent = nullptr);
    
    /** set angle*/
    void set_Angle(float _angle);

    /** get Pose */
    Eigen::Matrix<T, 4, 4> get_LocalPose();

    Eigen::Matrix<T, 4, 4> get_globalPose();

    void property();

    bool isRoot();

    void set_Child(std::shared_ptr<Frame<T>> child);

    int get_Id(){return mnid;}

    /** setter for DH parameter*/
    void set_a(T _a){ma = _a;}
    void set_d(T _d){md = _d;}
    void set_theta(T _theta){mtheta = _theta;}
    void set_alpha(T _alpha){malpha = _alpha;}

    /** getter for DH parameter*/
    T get_a(){return ma;}
    T get_d(){return md;}
    T get_alpha(){return malpha;}
    T get_theta(){return mtheta;}


private:
    Eigen::Matrix<T, 4, 4> mlocal;
    Eigen::Matrix<T, 4, 4> mglobal;
    Eigen::Matrix<T, 4, 4> mbase;
    
    /** @brief DH parameter */
    T ma;
    T malpha;
    T md;
    T mtheta;

    /** @brief current angle */
    T mangle{0};

    /** @brief id of frame */
    int mnid;

    /** @brief pointer to children*/
    std::vector<std::shared_ptr<Frame<T>>> mvpChildren;

    /** @brief pointer to parent */
    std::weak_ptr<Frame<T>> mpParent;
};

template class Frame<float>;
template class Frame<double>;

typedef Frame<float> Framef;
typedef Frame<double> Framed;

#endif // __FRAME_HPP_