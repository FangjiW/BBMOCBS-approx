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
// time_t current_time;
// time(&current_time);
// std::string cur_time(std::ctime(&current_time));

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
// std::string if_eager_temp = vm["eager"].as<std::string>() == "true" ? "-E" : "";
// std::string ms_temp;

// if(ms == MergeStrategy::MORE_SLACK){
//     ms_temp = "-SLK";
// }else if(ms == MergeStrategy::LESS_CONFLICT){
//     ms_temp = "-CB";
// }
if(vm["algorithm"].as<std::string>() == "BBMOCBS-eps"){
    algorithm = Algorithm::BBMOCBS_EPS;
    // output.open("../" + std::to_string(dim) + "d.out/" + vm["map"].as<std::string>() + "/n=" + std::to_string(vm["agent_num"].as<int>()) + "/EPS-"
    //     + std::to_string(eps).substr(0, 4) + if_eager_temp + vm["metric"].as<string>() + "-" + cur_time);
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-pex"){
    algorithm = Algorithm::BBMOCBS_PEX;
    // output.open("../" + std::to_string(dim) + "d.out/" + vm["map"].as<std::string>() + "/n=" + std::to_string(vm["agent_num"].as<int>()) + "/PEX-" 
    //     + std::to_string(eps).substr(0, 4) + "-" + std::to_string(vm["eps_ratio"].as<double>()).substr(0, 3)
    //     + ms_temp + if_eager_temp + vm["metric"].as<string>() + "-" + cur_time);
}else if(vm["algorithm"].as<std::string>() == "BBMOCBS-k"){
    algorithm = Algorithm::BBMOCBS_K;
    // output.open("../" + std::to_string(dim) + "d.out/" + vm["map"].as<std::string>() + "/n=" + std::to_string(vm["agent_num"].as<int>()) + "/K-" 
    //     + std::to_string(vm["solution_num"].as<int>()) + ms_temp + if_eager_temp + vm["metric"].as<string>() + "-" + cur_time);
}else{
    output << std::endl << std::endl << vm["algorithm"].as<std::string>() + " is not an allowed algorithm";
    exit(1);
}

// input merge strategy
MergeStrategy ms = vm["CB"].as<std::string>() == "true" ? MergeStrategy::LESS_CONFLICT : MergeStrategy::MORE_SLACK;

// input if eager
bool if_eager = vm["eager"].as<std::string>() == "true" ? true : false;

// std::vector<CostMetric>  cm(dim);



// //input cost
// for(int i = 0; i < dim; i++){
//     if(vm["cost_metric"].as<std::string>().at(i) == 'r'){
//         cm.at(i) = CostMetric::R;
//     }else if(vm["cost_metric"].as<std::string>().at(i) == 't'){
//         cm.at(i) = CostMetric::T;
//     }else if(vm["cost_metric"].as<std::string>().at(i) == 'd'){
//         cm.at(i) = CostMetric::D;
//     }else{
//         output << vm["cost_metric"].as<std::string>().at(i) + " is not an allowed metric";
//     }
// }


// input 


// p.read_cost(vm["cost_map"].as<std::string>(), map, edges, i);



// output << vm["algorithm"].as<std::string>() << ",   " << "eps = " << vm["eps"].as<double>() << ", "
//     << "eps_ratio = " << vm["eps_ratio"].as<double>() << std::endl << "Agent Number = " << vm["agent_num"].as<int>() << std::endl
//     << "Cost Metric:  " << vm["metric"].as<std::string>() << std::endl << "Turn Cost:  " << vm["turn_cost"].as<int>() << std::endl << std::endl;


/**************************  Search  *****************************/
// std::string ScenarioDirectory = "../dataset/" + vm["map"].as<std::string>() + "/config";
// DIR *dir;
// dir = opendir(ScenarioDirectory.c_str());
// for(int i = 0; i < dim; i++){
//     if(cm.at(i) == CostMetric::D){
//         p.read_cost("../dataset/" + vm["map"].as<std::string>() + "/" + vm["cost"].as<string>() + "/distance.cost", map, edges, i);
//     }else if(cm.at(i) == CostMetric::T){
//         p.read_cost(vm["base_cost"].as<int>(), map, edges, i);
//     }
// }

Solver  solver;
solver.init(map.graph_size, vm["agent_num"].as<int>(), algorithm, ms, if_eager, dim, turn_dim, vm["turn_cost"].as<int>(), vm["solution_num"].as<int>(), eps, eps_hm, vm["time_limit"].as<int>());

// int total_num = 0, success_num = 0;
// double t_HLMergingTime = 0, t_LowLevelTime = 0, t_TotalTime = 0;
// int t_ConflictSolvingNum = 0, t_SolutionNum = 0;

// struct dirent *entry;
// while ((entry = readdir(dir)) != NULL) {
//     if (entry->d_type == DT_REG) {
// scene_num ++;
// total_num ++;

// if(scene_num < vm["start_scene"].as<int>()){
//     continue;
// }
// std::vector<std::pair<size_t, size_t>>  start_end;
// std::string filePath = ScenarioDirectory + "/" + (std::string)entry->d_name;
// p.read_config(filePath, map, vm["agent_num"].as<int>(), start_end);
// output << "************************************************************************" << std::endl;
// output << "Scenario Number: " << scene_num << std::endl;
// output << "Scenario File: " << filePath << std::endl << std::endl;
// std::vector<std::tuple<double, double, double, double, int, int, int, int>> temp_result;

// bool have_r = false;
// for(int i = 0; i < dim; i++){
//     if(cm.at(i) == 'r'){
//         p.read_cost("../dataset/" + vm["map"].as<std::string>() + "/" + vm["cost"].as<string>() + "/random-" + to_string(scene_num + 100*i) + ".cost", map, edges, i);
//         output << "COST MAP:  " << std::to_string(dim) << "/" << std::to_string(scene_num + 100*i) + ".cost";
//         have_r = true;
//     }
// }
// if(have_r){
//     output << std::endl << std::endl;
// }
HSolutionID        hsolution_ids;
std::vector<CostVector>    hsolution_costs;

auto result = solver.search(edges, start_end, hsolution_ids, hsolution_costs, logger);

// if (std::get<2>(one_result) > vm["time_limit"].as<int>()){
//     break;
// }

double HLMergingTime = 0, LowLevelTime = 0, TotalTime = 0;
int ConflictSolvingNum = 0, SolutionNum = 0;

// success_num ++;
HLMergingTime = std::get<0>(result);      //t_HLMergingTime += HLMergingTime;
LowLevelTime = std::get<1>(result);     //t_LowLevelTime += LowLevelTime;
TotalTime = std::get<2>(result);      //  t_TotalTime += TotalTime;
ConflictSolvingNum = std::get<3>(result);  //  t_ConflictSolvingNum += ConflictSolvingNum;
SolutionNum = std::get<4>(result);    //  t_SolutionNum += SolutionNum;

// output << std::endl;
output << std::endl;
output << "HLMergingTime = " << HLMergingTime << std::endl;
output << "LowLevelTime = " << LowLevelTime << std::endl;
output << "Total Time = " << TotalTime << std::endl;
output << "ConflictSolvingNum = " << ConflictSolvingNum << std::endl;
output << "SolutionNum = " << SolutionNum << std::endl;
// output << std::endl << std::endl;
std::cout << "FINISH ONCE" << std::endl;
    // }
// }
// output << "AVERAGE :" << std::endl;
// output << "NonDomTime = " << t_HLMergingTime/(double)success_num << std::endl;
// output << "LowLevelTime = " << t_LowLevelTime/(double)success_num << std::endl;
// output << "Total Time = " << t_TotalTime/(double)success_num << std::endl;
// output << "ConflictSolvingNum = " << t_ConflictSolvingNum/(double)success_num << std::endl;
// output << "SolutionNum = " << t_SolutionNum/(double)success_num << std::endl;
// output << "Success Rate = " << success_num << "/" << total_num << " = " << (double)success_num/total_num;

output.close();
    // Solver      solver;
    // HSolutionID        hsolutions;
    // std::vector<CostVector>    hsolution_costs;

    // auto start_time = std::chrono::high_resolution_clock::now();    // record time
    // solver.search(map.graph_size, edges, vm, start_end, ms, logger, hsolutions, hsolution_costs, output);
    // auto end_time = std::chrono::high_resolution_clock::now();
    
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // delete(logger);


// /********************  Print  Path  Info  ************************/
// getchar();
//     std::cout << endl << endl;
//     for(size_t num = 0; num < hsolutions.size(); num ++){
//         std::cout << "SOLUTION   " << num+1 << endl;
//         std::cout << "COST   " << "{";
//         if(dim == 2){
//             std::cout << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1);
//         }else{
//             std::cout << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1) << ", " << hsolution_costs.at(num).at(2);
//         }
//         std::cout << "}" << endl;
//         for(size_t i = 0; i < hsolutions.at(num).size(); i++){
//             std::cout << "agent " << i+1 << ":" << std::endl;
//             for(size_t id : hsolutions.at(num).at(i)){
//                 std::cout << "{" << id2coord[id].at(0) << ", " << id2coord[id].at(1) << "}, ";
//             }
//             std::cout << endl;
//         }
//         std::cout << endl << endl;
//     }
//     std::cout << "Total constraint number = " << constraint_num << std::endl;
//     std::cout << "RUN TIME: " << ((double)duration.count())/1000000.0 << " seconds" << std::endl;
}