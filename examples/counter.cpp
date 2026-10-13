#include <mutex>

class Counter {
public:
    void increment() {
        std::lock_guard<std::mutex> guard(mutex_);
        ++value_;
    }

private:
    std::mutex mutex_;
    int value_{0};
};

int main() {
    Counter counter;
    counter.increment();
}
