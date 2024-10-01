#include <robotics/frame.hpp>

extern int nId;

template<typename T>
Frame<T>::Frame(T _a, T _alpha, T _d, T _theta, std::shared_ptr<Frame<T>> _parent) : ma(_a), malpha(_alpha), md(_d), mtheta(_theta)
{
    mnid = nId++;

    if(_parent != nullptr)
    {
#ifdef DEBUG
        std::cout << mnid << ": Debug" << std::endl;
#endif 
        mpParent = _parent;
        // _parent->set_Child(this->shared_from_this());
    }
    else
    {
        mbase = Eigen::Matrix<T, 4, 4>::Identity();
    }
    set_Angle(0);
}

template<typename T>
void Frame<T>::set_Angle(float _angle)
{
    mangle = _angle;
    mlocal << cos(mtheta+mangle)            ,            -sin(mtheta+mangle),            0,              ma,
             sin(mtheta+mangle)*cos(malpha), cos(mtheta+mangle)*cos(malpha), -sin(malpha), -md*sin(malpha),
             sin(mtheta+mangle)*sin(malpha), cos(mtheta+mangle)*sin(malpha),  cos(malpha),  md*cos(malpha),
                                          0,                              0,            0,               1;
    if(!isRoot())
    {
        mbase = mpParent.lock()->get_globalPose();
    }

    mglobal = mbase * mlocal;
}

template<typename T>
Eigen::Matrix<T, 4, 4> Frame<T>::get_LocalPose()
{
    return mlocal;
}

template<typename T>
Eigen::Matrix<T, 4, 4> Frame<T>::get_globalPose()
{
    return mglobal;
}

template<typename T>
void Frame<T>::property()
{
#ifdef DEBUG
    std::cout << "+----------------------------------+\n"
              << "id: " << mnid << std::endl
              << mglobal << std::endl;
              
    if(!isRoot())
        std::cout << "parent: " << mpParent.lock()->get_Id() << std::endl;
    else
        std::cout << "parent: NULL\n";
    std::cout << "child: ";
    for(int i=0; i<mvpChildren.size(); i++)
    {
        std::cout << mvpChildren.at(i)->get_Id() << "\t";
    }
    std::cout << "\n+----------------------------------+\n";
#else
    std::cout << "this is the function defined for Debug mode" << std::endl;
#endif
}

template<typename T>
bool Frame<T>::isRoot()
{
    return mpParent.lock() == nullptr;
}

template<typename T>
void Frame<T>::set_Child(std::shared_ptr<Frame<T>> child)
{
    mvpChildren.push_back(child);
}

