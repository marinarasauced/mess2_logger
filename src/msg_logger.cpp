
#include "mess2_logger/msg_logger.hpp"


Logger::Logger(
    const std::string &log_file_path,
    bool overwrite
) {
    auto mode = std::ios::out | (overwrite ? std::ios::trunc : std::ios::app);
    log_file.open(log_file_path, mode);
    if (!log_file.is_open()) {
        throw std::runtime_error("Unable to open log file : " + log_file_path);
    }
}


Logger::~Logger() {
    if (log_file.is_open()) {
        log_file.close();
    }
}


void Logger::log(
    const std::vector<std::string> &data
) {
    for (size_t i = 0; i < data.size(); ++i) {
        log_file << data[i];
        if (i < data.size() - 1) {
            log_file << ";";
        }
    }
    log_file << "\n";
    log_file.flush();
}


PYBIND11_MODULE(msg_logger, m) {
    m.doc() = "ros2 message logger";
    pybind11::class_<Logger>(m, "Logger")
        .def(pybind11::init<const std::string&>())
        .def("log", &Logger::log);
}
