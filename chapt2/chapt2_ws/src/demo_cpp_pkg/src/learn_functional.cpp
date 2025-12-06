#include <iostream>
#include <functional>


// 自由函数
void free_fun(const std::string & filename) {
    std::cout << "自由函数： " << filename << std::endl;
}

// 成员函数
class FileSave {
private:

public:
    void member_fun(const std::string & filename) {
        std::cout << "成员函数： " << filename << std::endl;
    }
};

int main() {
    FileSave file_save;

    auto lambda_func = [](const std::string & filename) -> void{
        std::cout << "Lambda函数: " << filename << std::endl;
    };
    // free_fun("data_free.txt");
    // file_save.member_fun("data_member.txt");
    // lambda_func("data_lambda.txt");

    std::function<void(const std::string &)> save1 = free_fun;
    std::function<void(const std::string &)> save2 = lambda_func;
    std::function<void(const std::string &)> save3 = std::bind(&FileSave::member_fun,
         &file_save, std::placeholders::_1);
    save1("data_free.txt");
    save2("data_lambda.txt");
    save3("data_member.txt");

    return 0;
}