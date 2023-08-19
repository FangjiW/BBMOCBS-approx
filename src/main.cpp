#include <iostream>

#include <memory>
#include <time.h>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <dirent.h>
#include <string>

#include "Utils/Logger.h"
#include "Utils/Solver.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>


const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;

using namespace std;

std::unordered_map<size_t, std::vector<int>> id2coord;  // node_id to coordinate

int main(int argc, char** argv)
{

namespace po = boost::program_options;
// Declare the supported options.
po::options_description desc("Allowed options");
desc.add_options()
    ("help", "produce help message")
    ("map,m", po::value<std::string>()->required(), "Map File")
    ("cost,c", po::value<std::string>()->default_value(""), "Cost File")
    ("config", po::value<std::string>()->default_value(""), "Configure File")
    ("dim,d", po::value<int>()->default_value(2), "dimension of cost function")
    ("eps,e", po::value<double>()->default_value(0), "Approximate factor")
    ("hem", po::value<double>()->default_value(0), "High Level merge approximate factor")
    ("lem", po::value<double>()->default_value(0), "Low Level merge approximate factor")
    ("hep", po::value<double>()->default_value(0), "High Level prune approximate factor")
    ("lep", po::value<double>()->default_value(0), "Low Level prune approximate factor")
    ("agent_num,n", po::value<int>()->default_value(-1), "number of agents")
    ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
    ("algorithm,a", po::value<std::string>()->default_value("Apex"), "low-level solvers (BOA, PPA or Apex search)")
    ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
    ("r1", po::value<int>()->default_value(3), "total runing times per scen")
    ("r2", po::value<int>()->default_value(8), "total scen")

    ("output,o", po::value<std::string>()->default_value("output.txt"), "Name of the output file")
    ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
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

/********************  Check  Input  Format  *********************/
if(vm["algorithm"].as<std::string>() == "BOA" && vm["dim"].as<int>() > 2){
    std::cout << std::endl << "BOA* only works for cost dimension = 2";
    exit(1);
}
// MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
auto ms = MergeStrategy::RANDOM;


/*************************  Build  Map  ****************************/
Map map;
PreProcessor p;
p.read_map("../dataset/" + vm["map"].as<std::string>() + "/" + vm["map"].as<std::string>() + ".map", map, id2coord);
// p.read_config(vm["config"].as<std::string>(), map, vm["agent_num"].as<int>(), start_end);
// p.read_cost(vm["cost"].as<std::string>(), map, edges);
// p.generate_cost(map, edges, vm["dim"].as<int>());

std::ofstream output;
if(vm["algorithm"].as<std::string>() == "Apex"){
    output.open("../" + std::to_string(vm["dim"].as<int>()) + ".out/" + vm["map"].as<std::string>() + "/n=" 
    + std::to_string(vm["agent_num"].as<int>()) + "/" + std::to_string(vm["hem"].as<double>()).substr(0, 4)
    + ", " + std::to_string(vm["hep"].as<double>()).substr(0, 4) + ", " + std::to_string(vm["lem"].as<double>()).substr(0, 4) 
    + ", " + std::to_string(vm["lep"].as<double>()).substr(0, 4));

    auto current_time = std::time(nullptr);
    output << std::ctime(&current_time) << std::endl;
    output << "Map: " << vm["map"].as<std::string>() << std::endl;

    output << vm["algorithm"].as<std::string>() << ", " << "hem = " << vm["hem"].as<double>() << ", "
    << "hep = " << vm["hep"].as<double>() << ", lem = " << vm["lem"].as<double>()
    << ", lep = " << vm["lep"].as<double>() << std::endl << "agent num = " << vm["agent_num"].as<int>() 
    << std::endl << std::endl;
}else{
    output.open("../" + std::to_string(vm["dim"].as<int>()) + ".out/" + vm["map"].as<std::string>() + "/n=" 
    + std::to_string(vm["agent_num"].as<int>()) + "/" 
    + vm["algorithm"].as<std::string>() + "-" + std::to_string(vm["eps"].as<double>()).substr(0, 4));
    auto current_time = std::time(nullptr);
    output << std::ctime(&current_time) << std::endl;
    output << "Map: " << vm["map"].as<std::string>() << std::endl << vm["algorithm"].as<std::string>() << std::endl
    << "agent num = " << vm["agent_num"].as<int>() 
    << std::endl << std::endl;
}

/**************************  Search  *****************************/
std::string directoryPath = "../dataset/" + vm["map"].as<std::string>() + "/config";
DIR *dir;
struct dirent *entry;
dir = opendir(directoryPath.c_str());
int scene_num = 0;
int total_num = 0;
int total_success_num = 0;
double t_NonDomTime = 0, t_LowLevelTime = 0, t_TotalTime = 0, t_CATTime = 0, t_DomPruneNum = 0, t_constraint_num = 0;
while ((entry = readdir(dir)) != NULL) {
    if (entry->d_type == DT_REG) {
        scene_num ++;
        if(scene_num > vm["r2"].as<int>()){
            break;
        }
        std::vector<std::pair<size_t, size_t>>  start_end;
        std::string filePath = directoryPath + "/" + (std::string)entry->d_name;
        p.read_config(filePath, map, vm["agent_num"].as<int>(), start_end);
        output << "************************************************************************" << std::endl;
        output << "Config File:  " << filePath << std::endl << std::endl;
        std::vector<std::tuple<double, double, double, double, int, int>> temp;
        int success_num = 0;
        for(int i = 1; i < vm["r1"].as<int>()+1; i++){
            std::cout << "iteration: " << scene_num << "      run number: " << i << std::endl;
            total_num ++;
            Solver      solver;
            HSolutionID        hsolutions;
            std::vector<CostVector>    hsolution_costs;
            std::vector<Edge>  edges;
            p.read_cost("../dataset/" + vm["map"].as<std::string>() + "/cost/" + std::to_string(vm["dim"].as<int>()) + "/" + std::to_string(i) + ".cost", map, edges, vm["dim"].as<int>());
            output << "COST MAP:" <<  std::to_string(vm["dim"].as<int>()) + "/" + std::to_string(i) + ".cost" << std::endl << std::endl;

            auto one_data = solver.search(map.graph_size, edges, vm, start_end, ms, logger, hsolutions, hsolution_costs, output);
            
            if (std::get<2>(one_data) > vm["cutoffTime"].as<int>()){
                continue;
            }
            success_num ++;
            total_success_num ++;
            temp.push_back(one_data);
        }
        double NonDomTime = 0, LowLevelTime = 0, TotalTime = 0, CATTime = 0, DomPruneNum = 0, constraint_num = 0;
        for(int i = 0; i < temp.size(); i++){
            NonDomTime += std::get<0>(temp.at(i))/(double)success_num;
            LowLevelTime += std::get<1>(temp.at(i))/(double)success_num;
            TotalTime += std::get<2>(temp.at(i))/(double)success_num;
            CATTime += std::get<3>(temp.at(i))/(double)success_num;
            DomPruneNum += std::get<4>(temp.at(i))/(double)success_num;
            constraint_num += std::get<5>(temp.at(i))/(double)success_num;

            t_NonDomTime += std::get<0>(temp.at(i));
            t_LowLevelTime += std::get<1>(temp.at(i));
            t_TotalTime += std::get<2>(temp.at(i));
            t_CATTime += std::get<3>(temp.at(i));
            t_DomPruneNum += std::get<4>(temp.at(i));
            t_constraint_num += std::get<5>(temp.at(i));
        }
        output << std::endl;
        output << "NonDomTime = " << NonDomTime << std::endl;
        output << "LowLevelTime = " << LowLevelTime << std::endl;
        output << "Total Time = " << TotalTime << std::endl;
        output << "CAT Time = " << CATTime << std::endl;
        output << "DomPruneNum/NodeExpandNum = " << DomPruneNum << "/" << constraint_num << std::endl;
        output << std::endl << std::endl;
        std::cout << "FINISH ONCE" << std::endl;
    }
}
output << "AVERAGE :" << std::endl;
output << "NonDomTime = " << t_NonDomTime/(double)total_success_num << std::endl;
output << "LowLevelTime = " << t_LowLevelTime/(double)total_success_num << std::endl;
output << "Total Time = " << t_TotalTime/(double)total_success_num << std::endl;
output << "CAT Time = " << t_CATTime/(double)total_success_num << std::endl;
output << "DomPruneNum/NodeExpandNum = " << t_DomPruneNum/(double)total_success_num << "/" << t_constraint_num/(double)total_success_num << std::endl;
output << "Success Rate = " << total_success_num << "/" << total_num << " = " << (double)total_success_num/total_num;
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
//         if(vm["dim"].as<int>() == 2){
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