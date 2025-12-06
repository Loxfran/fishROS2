#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include "cpp-httplib/httplib.h"

class Download
{
public:
    void download(const std::string &url, const std::string &path,
        const std::function<void(const std::string &, const std::string &)> &callback)
    {
        std::cout << "线程: " << std::this_thread::get_id() << " 开始下载文件: " << url.c_str() << std::endl;
        httplib::Client client(url);    
        auto res = client.Get(path);
        std::cout << res->status << " 结束下载文件: " << url.c_str() << std::endl;
        if (res && res->status == 200) {
            // std::cout << "线程: " << std::this_thread::get_id() << " 下载文件成功: " << url << std::endl;
            callback(path, res->body);
        } 
    };
    void start_download(const std::string &url, const std::string &path,
        const std::function<void(const std::string &, const std::string &)> &callback)
    {
        auto dl = std::bind(&Download::download, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        std::thread t(dl, url, path, callback);
        t.detach();
    };
};

int main() 
{
    Download d;
    auto word_count = [](const std::string &path, const std::string &content) -> void
    {
        std::cout << "下载完成: " << path << ", 内容长度: " << content.length() << "->" << content.substr(0,15)
        << std::endl;
    };
    d.start_download("http://0.0.0.0:8000", "/novel1.txt",word_count);
    d.start_download("http://0.0.0.0:8000", "/novel2.txt",word_count);
    d.start_download("http://0.0.0.0:8000", "/novel3.txt",word_count);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}
