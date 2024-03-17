#include "LowLevel/AbstractSolver.h"


std::ostream& operator <<(std::ostream &stream, const std::vector<double> &vec){
    stream << "[";
    for (size_t i = 0 ;  i < vec.size(); i ++){
        stream << vec[i];
        if (i + 1 <vec.size()){
            stream << ", ";
        }
    }
    stream << "]";
    return stream;
}


void AbstractSolver::start_logging(size_t source, size_t target) {
    // All logging is done in JSON format
    std::stringstream start_info_json;
    start_info_json
        << "{\n"
        <<      "\t\"name\": \"" << get_solver_name() << "\",\n"
        <<      "\t\"eps_merge\": " << this->eps_merge << "\n"
        <<      "\t\"eps_merge\": " << this->eps_prune << "\n"
        << "}";

    if (this->logger != nullptr) {
        LOG_START_SEARCH(*this->logger, source, target, start_info_json.str());
    }
}

void AbstractSolver::end_logging(SolutionSet &solutions, bool succ) {
    // All logging is done in JSON format
    std::stringstream finish_info_json;
    finish_info_json
        << "{\n"
        <<      "\t\"solutions\": [";

    size_t solutions_count = 0;
    for (auto solution = solutions.begin(); solution != solutions.end(); ++solution) {
        if (solution != solutions.begin()) {
            finish_info_json << ",";
        }
        finish_info_json << "\n\t\t" << **solution;
        solutions_count++;
    }

    finish_info_json
        <<      "\n\t],\n"
        <<      "\t\"amount_of_solutions\": " << solutions_count << ",\n"
        <<      "\t\"status\": " << ( succ ? "\"Success\"": "\"Failed\"" )<< "\n"
        << "}" <<std::endl;

    if (this->logger != nullptr) {
        LOG_FINISH_SEARCH(*(this->logger), finish_info_json.str());
    }
}
