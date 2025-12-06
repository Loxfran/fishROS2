#include <iostream>
#include <algorithm>

int main() {
    // [capture_list](parameters) -> {function_body}  lambda 表达式的基本语法
    auto add = [](int a, int b) -> int {
        return a + b;
    };
    int sum = add(3, 5);
    auto print_sum = [sum](){
        std::cout << "The sum is: " << sum << std::endl;
    };
    print_sum();

    return 0;
}