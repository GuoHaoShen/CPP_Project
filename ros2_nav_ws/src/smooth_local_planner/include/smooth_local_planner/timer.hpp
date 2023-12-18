#include <iostream>
#include <chrono>
#include <vector>

class Timer {
public:
    Timer() : totalDuration(0), runs(0) {}

    void start() {
        startTime = std::chrono::high_resolution_clock::now();
    }

    void stop() {
        endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = endTime - startTime;
        std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;
        totalDuration += duration.count();
        runs++;
    }

    void reset() {
        totalDuration = 0;
        runs = 0;
    }

    double averageTime() const {
        if (runs == 0) {
            return 0;
        }
        return totalDuration / runs;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime, endTime;
    double totalDuration;
    int runs;
};