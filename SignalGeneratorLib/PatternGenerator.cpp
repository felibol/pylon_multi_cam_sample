
#include "PatternGenerator.hpp"

/* Alternative to `chrt -r -p 99 <pid>`. */
void Scheduling::setPriority(double priority, Policy policy)
{
    if (priority < 0 || priority > 1.0)
        throw std::invalid_argument(std::string("setPriority: priority out of range"));

    /* Lock all current and future pages from preventing of being paged to swap */
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        throw std::runtime_error(std::string("failed to lock memory for realtime: ") +
                                 strerror(errno));

    int min_priority = sched_get_priority_min(policy),
        max_priority = sched_get_priority_max(policy);
    int adjusted_priority = int((max_priority - min_priority) * priority) + min_priority;
    sched_param param = {adjusted_priority};
    if (sched_setscheduler(0, policy, &param))
        throw std::runtime_error(std::string("failed to set thread priority."));
}

bool GpioChip::open(const std::string& t_gpio_chip_name)
{
    close();

    m_fd = ::open(t_gpio_chip_name.c_str(), 0);
    if (m_fd < 0)
        throw std::runtime_error(std::string("failed to open gpio controller ") + t_gpio_chip_name +
                                 " : " + std::string(strerror(errno)));

    const int ret = ioctl(m_fd, GPIO_GET_CHIPINFO_IOCTL, &m_chip_info);
    if (ret < 0)
        throw std::runtime_error(std::string("Failed to get gpiochip info: ") +
                                 std::string(strerror(errno)));

    return true;
}

void GpioChip::close()
{
    if (isOpen()) {
        ::close(m_fd);
        m_fd = -1;
    }
}

int GpioChip::count() const
{
    if (!isOpen()) return 0;
    return m_chip_info.lines;
}

bool GpioChip::requestOutputs(const std::vector<int>& t_offsets)
{
    m_request.lines = t_offsets.size();
    for (auto i = 0; i < m_request.lines; ++i) m_request.lineoffsets[i] = t_offsets.at(i);
    m_request.flags = GPIOHANDLE_REQUEST_OUTPUT;
    strcpy(m_request.consumer_label, "siggen");
    int lhfd = ioctl(m_fd, GPIO_GET_LINEHANDLE_IOCTL, &m_request);
    if (lhfd < 0)
        throw std::runtime_error(std::string("failed to request outputs: (") +
                                 std::to_string(lhfd) + ") :" + std::string(strerror(errno)));
    m_requested_gpio_count = t_offsets.size();
    return true;
}

void GpioChip::setOutputs(const std::vector<bool>& t_values)
{
    if (m_requested_gpio_count == 0) return;
    for (auto i = 0; (i < t_values.size()) && (i < m_requested_gpio_count); ++i)
        m_data.values[i] = t_values[i] ? 1 : 0;
    const int ret = ioctl(m_request.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &m_data);
    if (ret < 0)
        throw std::runtime_error(std::string("Failed to set output: ") +
                                 std::string(strerror(errno)));
}

bool PatternGenerator::open(int t_gpio_pin, double t_period_duration, double t_duty_cycle)
{
    m_gpios.push_back(t_gpio_pin);
    m_periodDuration = t_period_duration;
    m_dutyCycle = t_duty_cycle;
    m_is_stop = false;
    openGpioController();
    return gpio.requestOutputs(m_gpios);
}

void PatternGenerator::generateSquareWave()
{
    const double periodDuration = m_periodDuration;

    //-- Generate pattern
    patterns.resize(2);
    for (auto i = 0; i < patterns.size(); ++i) {
        if (patterns.size() == 2) {
            if (i == 0) patterns[i].duration = periodDuration * m_dutyCycle;
            else
                patterns[i].duration = periodDuration * (1.0 - m_dutyCycle);
        } else
            patterns[i].duration = periodDuration / patterns.size();

        patterns[i].values.resize(gpio.requestedOutputs());
        for (auto j = 0; j < gpio.requestedOutputs(); ++j)
            patterns[i].values[j] = (i == 0) ? true : false;
    }
}

void PatternGenerator::appendPattern(double durationSeconds, const std::vector<bool>& values)
{
    patterns.push_back({durationSeconds, values});
}

void PatternGenerator::appendPattern(double durationSeconds, const unsigned long values)
{
    std::vector<bool> v;
    v.resize(gpio.requestedOutputs());
    for (auto i = 0; i < gpio.requestedOutputs(); ++i) v[i] = ((values >> i) & 1) ? true : false;
    appendPattern(durationSeconds, v);
}

void PatternGenerator::executePattern(const int index)
{
    if (!validPatternIndex(index)) return;
    gpio.setOutputs(patterns[index].values);
}

double PatternGenerator::patternDuration(const int index) const
{
    if (!validPatternIndex(index)) return 0;
    return patterns[index].duration;
}

void PatternGenerator::loopAllPatternsRealtime()
{
    Scheduling::setPriority(1.0, Scheduling::RoundRobin);
    std::cout << "Trigger signal generator is starting..." << std::endl;
    std::chrono::system_clock::time_point nextPatternStart = std::chrono::system_clock::now();
    while (!m_is_stop) {
        for (auto i = 0; i < patterns.size(); ++i) {
            executePattern(i);
            nextPatternStart += std::chrono::microseconds(
                                                      long(patternDuration(i) * 1000.0 * 1000.0));
            std::this_thread::sleep_until(nextPatternStart);
        }
    }
}

void PatternGenerator::openGpioController()
{
    if (!m_gpioControllerName.empty()) {
        if (!openGpioByLabel(m_gpioControllerName))
            throw std::runtime_error(std::string("gpio controller not found: " +
                                                 m_gpioControllerName));
    } else
        throw std::runtime_error(std::string("no valid gpio controller provided"));
}

bool PatternGenerator::openGpioByLabel(const std::string& name)
{
    for (auto i = 0; i < 10; ++i) {
        if (!openGpioByIndex(i)) return false;
        if (gpio.label() == name)  // this is the main gpio controller on nvidia.
            return true;
    }
    return false;
}

bool PatternGenerator::openGpioByIndex(unsigned int index)
{
    return gpio.open(std::string("/dev/gpiochip") + std::to_string(index));
}
