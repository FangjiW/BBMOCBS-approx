#include <iostream>

#include <memory>
#include <time.h>
#include <fstream>
#include <chrono>
#include<ctime>
#include <stdlib.h>
#include <dirent.h>
#include <string>

#include "Utils/Logger.h"
#include "Utils/Solver.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

using namespace std;

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
    ("eps,e", po::value<double>()->default_value(0), "Approximate factor")
    ("eps_ratio,r", po::value<double>()->default_value(0.8), "eps_HighLevelMerge divided by eps")
    ("algorithm,a", po::value<std::string>()->default_value("BBMOCBS_pex"), "algorithm")
    ("c1", po::value<std::string>()->required(), "cost map 1")
    ("c2", po::value<std::string>()->default_value(""), "cost map 2")
    ("c3", po::value<std::string>()->default_value(""), "cost map 3")
    // ("metric", po::value<std::string>()->required(), "cost metric")
    // ("base_cost", po::value<int>()->default_value(2), "cost for no turn action")
    ("CB", po::value<std::string>()->default_value("true"), "if conflict-based")
    ("eager", po::value<std::string>()->default_value("true"), "if eager")

    ("turn_dim", po::value<int>()->default_value(-1), "dimension with turn cost")
    ("turn_cost", po::value<int>()->default_value(5), "extra cost for turn action")
    
    ("solution_num,k", po::value<int>()->default_value(INT64_MAX), "number of solution")
    // ("start_scene", po::value<int>()->default_value(0), "start scene number")
    ("time_limit,t", po::value<int>()->default_value(120), "cutoff time (seconds)")
    
    ("output_file,o", po::value<std::string>()->required(), "Name of the output file")
    ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
    // ("cost,c", po::value<std::string>()->default_value("cost"), "Cost Directory")
    // ("config", po::value<std::string>()->default_value(""), "Configure File")
    // ("dim,d", po::value<int>()->default_value(2), "dimension of cost function")
    // ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK, LESS_CONFLICT")
    // ("algorithm,a", po::value<std::string>()->default_value("Apex"), "low-level solvers (BOA, PPA or Apex search)")
    // ("unique_file", po::value<std::string>()->default_value("2"), "unique cost file")
    ;
po::variables_map vm;
po::store(po::parse_command_line(argc, argv, desc), vm);
po::notify(vm);

if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
}

LoggerPtr logger = nullptr;
if (vm["logging_file"].as<std::string>() != ""){
    logger = new Logger(vm["logging_file"].as<std::string>());
}

output.open(vm["output_file"].as<std::string>(), std::ios::app);
output.seekp(0, std::ios::end);
/********************  INPUT CONFIG  *********************/
// init preprocessor
PreProcessor p;

// input dimension
int dim = vm["dim"].as<int>();

// input map
Map map;
map_name = vm["map_file"].as<std::string>();
p.read_map(map_name, map, id2coord);

// input start and end
std::vector<std::pair<size_t, size_t>>  start_end;
p.read_config(vm["scenario_file"].as<std::string>(), map , vm["agent_num"].as<int>(), start_end);

// input cost
int turn_dim = vm["turn_dim"].as<int>();
int turn_cost = vm["turn_cost"].as<int>();
std::vector<Edge>  edges;
p.cost_init(map, edges, dim);
for(int i = 0; i < dim; i ++){
    std::string temp = "c" + std::to_string(i+1);
    p.read_cost(vm[temp].as<std::string>(), map, edges, i);
}

// input approximate factor
double eps = vm["eps"].as<double>(), eps_hm = vm["eps_ratio"].as<double>() * eps;


// input algorithm
Algorithm algorithm;
if(vm["algorithm"].as<std::string>() == "BBMOCBS-eps"){
    algorithm = Algorithm::BBMOCBS_EPS;
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-pex"){
    algorithm = Algorithm::BBMOCBS_PEX;
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-k"){
    algorithm = Algorithm::BBMOCBS_K;
}else{
    output << std::endl << std::endl << vm["algorithm"].as<std::string>() + " is not an allowed algorithm";
    exit(1);
}

// input merge strategy
MergeStrategy ms = vm["CB"].as<std::string>() == "true" ? MergeStrategy::LESS_CONFLICT : MergeStrategy::REVERSE_LEX;

// input if eager
bool if_eager = vm["eager"].as<std::string>() == "true" ? true : false;


Solver  solver;
solver.init(map.graph_size, vm["agent_num"].as<int>(), algorithm, ms, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["solution_num"].as<int>(), eps, eps_hm, vm["time_limit"].as<int>());

HSolutionID        hsolution_ids;
std::vector<CostVector>    hsolution_costs;

auto result = solver.search(edges, start_end, hsolution_ids, hsolution_costs, logger);

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