#include "iostream"

int main() {
    auto x = 42; // 'x' is inferred to be of type int
    auto y = 3.14f; // 'y' is inferred to be of type
    auto z = "Hello, World!"; // 'z' is inferred to be of type const char*

    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << z << std::endl;
    return 0;
}
