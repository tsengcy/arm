#include <iostream>
#include <memory>

class mclass
{
public:
    mclass()
    {
        std::cout << "constructor" << std::endl;
    }

    void func()
    {
        std::cout << "function" << std::endl;
    }
};

int main()
{
    std::shared_ptr<mclass> mc(new mclass());

    mc->func();
}
