#ifndef MY_TIMER_H
#define MY_TIMER_H

#include <chrono>


class ScopedTimer {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    std::string name;
public:
    ScopedTimer(std::string name) : name(name){
        start_time = std::chrono::high_resolution_clock::now();
    }

    ~ScopedTimer(){
        end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = end_time - start_time;
        // end_time - start_time;
        std::cout << "Elapsed time: "  << name << " " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_ms).count() << "s\n";
    }
};

// non thread safe static timer helper functions
using TimerVar = std::chrono::time_point<std::chrono::high_resolution_clock>;
struct TimerHelper {
    static TimerVar start_time;
    static TimerVar end_time;
    static std::string name;
public:
    static void start_timer(std::string name){
        start_time = std::chrono::high_resolution_clock::now();
    }

    static void end_and_print(){
        end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = end_time - start_time;
        // end_time - start_time;
        std::cout << "Elapsed time: "  << name << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed_ms).count() << "ns\n";
    }
};

TimerVar TimerHelper::start_time;
TimerVar TimerHelper::end_time;
std::string TimerHelper::name;

#endif // MY_TIMER_H
