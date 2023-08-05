#include <string>
#include <fstream>
#include <chrono>
#include <string>
#include <ctime>
#include "Utils/Logger.h"

// All logging is done in JSON format. This in order to work with formal format
// and allow simple connection to other frameworks/languages

std::string insert_tabs_on_newline(std::string str, size_t tabs_amount) {
    std::string replace_target = "\n";
    while (tabs_amount != 0) {
        tabs_amount -= 1;
        replace_target += "\t";
    }

    size_t pos = 0;
    while ((pos = str.find("\n", pos)) != std::string::npos) {
        str.replace(pos, 1, replace_target);
        pos += replace_target.length();
    }
    return str;
}


Logger::Logger(std::string filename)
    : log_filestream(filename.c_str(), std::ofstream::out | std::ofstream::app), first_entry(true) {
    this->run_start_time = Clock::now();
    this->log_filestream << "[" << std::endl; // Start list of logs
}


Logger::~Logger() {
    this->log_filestream << "]" << std::endl; // End list of logs
    this->log_filestream.close();
}


void Logger::start_search(size_t source, size_t target, std::string search_info_json) {
    std::time_t start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    this->run_start_time = Clock::now();
    this->total_run_duration_ms = 0;
    this->run_time_since_previous_log_ms = 0;
    std::string string_start_time = std::ctime(&start_time);
    string_start_time.pop_back();

    if (first_entry) {
        first_entry = false;
    } else {
        this->log_filestream << ",\n";
    }

    this->log_filestream << "{\n"
                         <<     "\t\"start_time\": \"" << string_start_time << "\",\n"
                         <<     "\t\"source\": " << source << ",\n"
                         <<     "\t\"target\": " << target << ",\n"
                         <<     "\t\"search_info\": " << insert_tabs_on_newline(search_info_json,2) << "," << std::endl;
}


void Logger::finish_search(std::string finish_info_json) {
    long int total_runtime_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - this->run_start_time).count();
    this->log_filestream <<     "\t\"finish_info\": " << insert_tabs_on_newline(finish_info_json,2) << ",\n"
                         <<     "\t\"total_runtime(ms)\": " << total_runtime_ms << "\n"
                         << "}" << std::endl;
}
