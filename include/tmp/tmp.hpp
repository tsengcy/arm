#ifndef __TMP_HPP__
#define __TMP_HPP__

#include <iostream>
#include <type_traits>

extern int id;
int id = 0;

class tmp
{
public:
    tmp()
    {
        id++;
        std::cout << id << std::endl;
    }

};

enum COLOR
{
    RED,
    BLUE
};

template<typename T, COLOR c>
class tmp1
{
public:
    tmp1();
    void func();
};

template class tmp1<float, RED>;
template class tmp1<float, BLUE>;
// template class tmp1<float, BLUE>;
// template class tmp1<float, GREEN>;
// using tmp1f = tmp1<float>;

#endif // __TMP_HPP__