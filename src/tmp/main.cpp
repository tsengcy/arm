// #include <Eigen/Core>
// #include <iostream>
// #include <tmp/tmp.hpp>

// int main()
// {
//     tmp1<float, RED>* mt1 = new tmp1<float, RED>();
//     tmp1<float, BLUE>* mt2 = new tmp1<float, BLUE>();

//     mt1->func();
//     mt2->func();
//     delete(mt1);
//     delete(mt2);
// }


#include <iostream>
#include <type_traits>

template<typename T>
constexpr bool is_int_or_double_v = 
    std::is_same<T, int>::value || std::is_same<T, double>::value;

template<typename T>
void func(T t,
    std::enable_if_t<!is_int_or_double_v<T>>* = nullptr)
{
    std::cout << "gerenal" << std::endl;
}

template<typename T>
void func(T t, 
    std::enable_if_t<is_int_or_double_v<T>>* = nullptr)
{
    std::cout << "hello" << std::endl;
}

template<typename T>
void cal(T x)
{
    std::cout << x << "\t";
}

template<typename T, typename... Ts>
void cal(T x, Ts... s)
{
    std::cout << x << "\t";
    cal(s...);
}

void cals(int i)
{
    std::cout << i << "\t";
}

template<typename T, typename... Ts>
typename std::enable_if_t<std::is_same<T, int>::value> cals(T i, Ts... is)
{
    std::cout << i << "\t";
    cals(is...);
}

#include <vector>

int main()
{
    // func(1);
    // func(2.0);
    // func('c');

    // cal(1, 2.2, "a");
    // std::cout << std::endl;

    // cals(1, 2, 3);
    // std::cout << std::endl;

    // cals(1, 'a');
    // std::cout << std::endl;

    std::vector<int> a{1, 0, 1};

    std::cout << a.size() << std::endl;    

    return 0;
}