#include "iostream"

int main(int argc, char * argv[])
{
    std::cout << "number of arguments: " << argc << std::endl;
    std::cout << "arguments:" << argv[0] << std::endl;
    std::string arg1 = argv[1];
    if (arg1 == "--help")
    {
        std::cout << "help" << std::endl;
    }
    return 0;
}
