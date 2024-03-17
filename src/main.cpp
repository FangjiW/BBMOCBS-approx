#include <iostream>

#include <memory>
#include <time.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <stdlib.h>
#include <dirent.h>
#include <string>

#include "LowLevel/Utils/Logger.h"
#include "HighLevel/HighLevelSolver.h"
#include "HighLevel/epsSolver.h"
#include "HighLevel/pexSolver.h"
#include "HighLevel/kSolver.h"

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

// using namespace std;

std::unordered_map<size_t, std::vector<int>> id2coord;  // node_id to coordinate
std::string map_name;
std::ofstream output;

int main(int argc, char** argv)
{

namespace po = boost::program_options;
// Declare the supported options.
po::options_description desc("Allowed options");
desc.add_options()
    ("help", "produce help message")
    ("map_file,m", po::value<std::string>()->required(), "Map File")
    ("agent_num,n", po::value<int>()->default_value(-1), "number of agents")
    ("dim,d", po::value<int>()->required(), "cost dimension")
    ("scenario_file,s", po::value<std::string>()->required(), "Scenario file")
    ("eps,e", po::value<double>()->default_value(0), "Approximation factor")
    // ("eps_ratio,r", po::value<double>()->default_value(0.8), "eps_HighLevelMerge divided by eps")
    ("algorithm,a", po::value<std::string>()->default_value("BBMOCBS_pex"), "algorithm")
    ("c1", po::value<std::string>()->required(), "cost map 1")
    ("c2", po::value<std::string>()->default_value(""), "cost map 2")
    ("c3", po::value<std::string>()->default_value(""), "cost map 3")
    ("CB", po::value<std::string>()->default_value("true"), "if conflict-based")
    ("eager", po::value<std::string>()->default_value("true"), "if eager")

    ("turn_dim", po::value<int>()->default_value(-1), "dimension with turn cost")
    ("turn_cost", po::value<int>()->default_value(5), "extra cost for turn action")
    
    ("solution_num,k", po::value<int>()->default_value(INT64_MAX), "number of solution")
    ("time_limit,t", po::value<int>()->default_value(120), "cutoff time (seconds)")
    
    ("output_file,o", po::value<std::string>()->required(), "Name of the output file")
    ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
    ;
po::variables_map vm;
po::store(po::parse_command_line(argc, argv, desc), vm);

if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
}

po::notify(vm);

LoggerPtr logger = nullptr;
if (vm["logging_file"].as<std::string>() != ""){
    logger = new Logger(vm["logging_file"].as<std::string>());
}

output.open(vm["output_file"].as<std::string>(), std::ios::app);
output.seekp(0, std::ios::end);
/********************  INPUT CONFIG  *********************/
// init preprocessor
PreProcessor p;

// import configures
int dim = vm["dim"].as<int>();
double eps = vm["eps"].as<double>();
MergingStrategy ms = vm["CB"].as<std::string>() == "true" ? MergingStrategy::CONFLICT_BASED : MergingStrategy::REVERSE_LEX;
bool if_eager = vm["eager"].as<std::string>() == "true" ? true : false;
int turn_dim = vm["turn_dim"].as<int>();
int turn_cost = vm["turn_cost"].as<int>();

// import map
Map map;
map_name = vm["map_file"].as<std::string>();
p.read_map(map_name, map, id2coord);

// import scenario file
std::vector<std::pair<size_t, size_t>>  start_goal;
p.read_scenario(vm["scenario_file"].as<std::string>(), map , vm["agent_num"].as<int>(), start_goal);

// import cost file
std::vector<Edge>  edges;
p.cost_init(map, edges, dim);
for(int i = 0; i < dim; i ++){
    std::string temp = "c" + std::to_string(i+1);
    p.read_cost(vm[temp].as<std::string>(), map, edges, i);
}

// initialize solver
std::unique_ptr<HighLevelSolver> h_solver;
if(vm["algorithm"].as<std::string>() == "BBMOCBS-eps"){
    h_solver = std::make_unique<epsSolver>(map.graph_size, vm["agent_num"].as<int>(), Algorithm::BBMOCBS_EPS, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["time_limit"].as<int>());
    ((epsSolver*)h_solver.get())->set_eps(eps);
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-pex"){
    h_solver = std::make_unique<pexSolver>(map.graph_size, vm["agent_num"].as<int>(), Algorithm::BBMOCBS_PEX, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["time_limit"].as<int>());
    ((pexSolver*)h_solver.get())->set_eps(eps);
    ((pexSolver*)h_solver.get())->set_merging_strategy(ms);
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-k"){
    h_solver = std::make_unique<kSolver>(map.graph_size, vm["agent_num"].as<int>(), Algorithm::BBMOCBS_K, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["time_limit"].as<int>());
    ((kSolver*)h_solver.get())->set_merging_strategy(ms);
    ((kSolver*)h_solver.get())->set_solution_num(ms);
}else{
    output << std::endl << std::endl << vm["algorithm"].as<std::string>() + " is not an allowed algorithm";
    exit(1);
}



HSolutionID        hsolution_ids;
std::vector<CostVector>    hsolution_costs;

auto result = h_solver->run(edges, start_goal, hsolution_ids, hsolution_costs, logger);

double HLMergingTime = 0, LowLevelTime = 0, TotalTime = 0;
int ConflictSolvingNum = 0, SolutionNum = 0;

// success_num ++;
HLMergingTime = std::get<0>(result);
LowLevelTime = std::get<1>(result);
TotalTime = std::get<2>(result);
ConflictSolvingNum = std::get<3>(result);
SolutionNum = std::get<4>(result);

// output << std::endl;
output << std::endl;
output << "HLMergingTime = " << HLMergingTime << std::endl;
output << "LowLevelTime = " << LowLevelTime << std::endl;
output << "Total Time = " << TotalTime << std::endl;
output << "ConflictSolvingNum = " << ConflictSolvingNum << std::endl;
output << "SolutionNum = " << SolutionNum << std::endl;

std::cout << "FINISH ONCE" << std::endl;

output.close();
}