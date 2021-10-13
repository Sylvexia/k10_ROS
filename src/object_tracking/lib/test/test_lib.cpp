#include "test/test.h"

Test::Test()
{
    printf("class test has been constructed");
}

Test::~Test()
{
    printf("class test has been destructed");
}

void Test::print(std::string str)
{
    std::cout<<str<<"\n";
}