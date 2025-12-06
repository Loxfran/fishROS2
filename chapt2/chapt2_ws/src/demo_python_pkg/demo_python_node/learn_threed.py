import threading
import requests


class Download():
    def download_file(self, url, callback):
        print(f"线程:{threading.get_ident()} ,下载: {url}")
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback(url, response.text)
    
    def start_download(self, url, callback):
        thread = threading.Thread(target=self.download_file, args=(url, callback))
        thread.start()

def word_count(url, content):
    print(f"Downloaded from {url}: {len(content)} {content[0:5]}...")

def main():
    download = Download()
    download.start_download("http://0.0.0.0:8000/novel1.txt", word_count)
    download.start_download("http://0.0.0.0:8000/novel2.txt", word_count)
    download.start_download("http://0.0.0.0:8000/novel3.txt", word_count)

