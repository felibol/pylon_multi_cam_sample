/**
 * @file PatternGenerator.hpp
 * @author Furkan Elibol (elibolfurkan@gmail.com)
 * @brief This class and some other helper functions 
 * are required for geenrating hw triggering signals at jetson paltforms.
 * This implementation taken from basler command line sample application implementation
 * @version 0.1
 * @date 2022-01-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>

#include <vector>
#include <string>
#include <iostream>
#include <regex>
#include <exception>
#include <thread>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>
#include <errno.h>
#include <numeric>
#include <functional>
#include <atomic>

namespace Scheduling {
enum Policy
{
    RoundRobin = SCHED_RR
};
void setPriority(double priority = 1.0, Policy policy = RoundRobin);
}  // namespace Scheduling

class GpioChip
{
   private:
    int m_fd = -1;
    int m_requested_gpio_count = 0;
    struct gpiochip_info m_chip_info;
    struct gpiohandle_request m_request;
    struct gpiohandle_data m_data;

   public:
    ~GpioChip() { close(); }
    bool open(const std::string& t_gpio_chip_name);
    void close();
    bool isOpen() const { return m_fd != -1; }
    int count() const;
    std::string name() const { return std::string(m_chip_info.name); }
    std::string label() const { return std::string(m_chip_info.label); }
    bool requestOutputs(const std::vector<int>& t_offsets);
    int requestedOutputs() const { return m_requested_gpio_count; }
    void setOutputs(const std::vector<bool>& t_values);
};

class PatternGenerator
{
   public:
    // PatternGenerator() : m_is_stop(false) {};
    bool open(int t_gpio_pin, double t_period_duration, double t_duty_cycle);
    void generateSquareWave();
    void appendPattern(double durationSeconds, const std::vector<bool>& values);
    void appendPattern(double durationSeconds, const unsigned long values);
    int patternCount() const { return patterns.size(); }
    bool validPatternIndex(const int index) const { return index < patternCount(); }
    void executePattern(const int index);
    double patternDuration(const int index) const;
    void loopAllPatternsRealtime();
    void stopAllPatterns() { m_is_stop = true; };

   private:
    void openGpioController();
    bool openGpioByLabel(const std::string& name);
    bool openGpioByIndex(unsigned int index);

   private:
    struct Pattern
    {
        double duration;
        std::vector<bool> values;
    };

    GpioChip gpio;
    std::vector<Pattern> patterns;

    std::string m_gpioControllerName = "tegra-gpio";
    double m_periodDuration = 0.04;  // ms for 25 fps (40ms)
    double m_dutyCycle = 0.25;       // ms high signal level part of pwm (10ms)
    std::vector<int> m_gpios;
    std::atomic<bool> m_is_stop;
};
