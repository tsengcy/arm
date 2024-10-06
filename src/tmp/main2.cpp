#include <iostream>

template<typename T>
class A
{
public:
    A()
    {
        a = 10;
        b = 10;
        c = 10;
    }
public:
    int a;
protected:
    int b;
private:
    int c;
};

template<typename T>
class B : public A<T>
{
public:
    B(){}

    void func()
    {
        // std::cout << a << std::endl;
    }
};

int main()
{
    B<float>* cb = new B<float>();

    cb->func();

}