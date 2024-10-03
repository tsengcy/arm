#include <tmp/tmp.hpp>
#include <iostream>
#include <type_traits>

template<typename T, COLOR c>
tmp1<T, c>::tmp1()
{
    std::cout << "tmp1" << std::endl;
}

template<typename T, COLOR c>
void tmp1<T, c>::func()
{   
    if constexpr(c == COLOR::RED)
    {
        std::cout << "RED" << std::endl;
    }
    else if constexpr(c == COLOR::BLUE)
    {
        std::cout << "BLUE" << std::endl;
    }
    
}

// template<typename T, COLOR c>
// template<typename std::enable_if<c == COLOR::BLUE>::type*>
// void tmp1<T, c>::func()
// {
//     std::cout << "BLUE" << std::endl;
// }

// template<typename T>
// tmp1<T, COLOR::BLUE>::tmp1()
// {
//     std::cout << "tmp1 BLUE" << std::endl;
// }

// template<typename T>
// tmp1<T, COLOR::GREEN>::tmp1()
// {
//     std::cout << "tmp1 GREEN" << std::endl;
// }



