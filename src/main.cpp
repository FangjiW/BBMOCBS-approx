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


const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;

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
    ("map,m", po::value<std::string>()->required(), "Map File")
    ("cost,c", po::value<std::string>()->default_value("cost"), "Cost Directory")
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
    ("r1", po::value<int>()->default_value(1), "total runing times per scen")
    ("r2", po::value<int>()->default_value(25), "total scen")
    ("cost_mode", po::value<std::string>()->required(), "cost mode")
    ("turn_mode", po::value<int>()->default_value(-1), "turn mode id")
    ("turn_cost", po::value<int>()->default_value(0), "turn cost")
    ("solution_num,s", po::value<int>()->default_value(0), "number of solution")
    ("mode", po::value<std::string>()->default_value("given_eps"), "solve mode")
    ("CAT", po::value<std::string>()->default_value("true"), "if CAT")
    ("eager", po::value<std::string>()->default_value("true"), "if eager")
    ("unique_file", po::value<std::string>()->default_value("2"), "unique cost file")
    ("start_scene", po::value<int>()->default_value(0), "start scene number")

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
if(vm["merge"].as<std::string>() == "slack"){
    ms = MergeStrategy::MORE_SLACK;
}
if(vm["merge"].as<std::string>() == "NONE"){
    ms = MergeStrategy::NONE;
}
if(vm["mode"].as<string>() == "smallest_eps" || vm["mode"].as<string>() == "diversity"){
    ms = MergeStrategy::MORE_SLACK;
}


/*************************  Build  Map  ****************************/
Map map;
PreProcessor p;
p.read_map("../dataset/" + vm["map"].as<std::string>() + "/" + vm["map"].as<std::string>() + ".map", map, id2coord);
// p.read_config(vm["config"].as<std::string>(), map, vm["agent_num"].as<int>(), start_end);
// p.read_cost(vm["cost"].as<std::string>(), map, edges);
// p.generate_cost(map, edges, vm["dim"].as<int>());
std::vector<Edge>  edges;
p.cost_init(map, edges, vm["dim"].as<int>());

map_name = vm["map"].as<std::string>();

std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
char* time_str = std::ctime(&now_time_t);
std::string cur_time(time_str);
std::string if_CAT = vm["CAT"].as<std::string>() == "true" ? "" : "noCAT-";
std::string if_eager = vm["eager"].as<std::string>() == "true" ? "" : "no_eager-";
if(vm["algorithm"].as<std::string>() == "Apex"){
    output.open("../new_" + std::to_string(vm["dim"].as<int>()) + ".out/" + vm["map"].as<std::string>() + "/n=" 
    + std::to_string(vm["agent_num"].as<int>()) + "/" + if_CAT + if_eager + std::to_string(vm["hem"].as<double>()).substr(0, 5)
    + ", " + std::to_string(vm["hep"].as<double>()).substr(0, 5) + ", " + std::to_string(vm["lem"].as<double>()).substr(0, 5) 
    + ", " + std::to_string(vm["lep"].as<double>()).substr(0, 5) + "-" + vm["cost_mode"].as<string>() + "-" + cur_time);

    auto current_time = std::time(nullptr);
    output << std::ctime(&current_time) << std::endl;
    output << "Map: " << vm["map"].as<std::string>() << std::endl;

    output << vm["algorithm"].as<std::string>() << ", " << "hem = " << vm["hem"].as<double>() << ", "
    << "hep = " << vm["hep"].as<double>() << ", lem = " << vm["lem"].as<double>()
    << ", lep = " << vm["lep"].as<double>() << std::endl << "agent num = " << vm["agent_num"].as<int>() << std::endl;
}else{
    output.open("../new_" + std::to_string(vm["dim"].as<int>()) + ".out/" + vm["map"].as<std::string>() + "/n=" 
    + std::to_string(vm["agent_num"].as<int>()) + "/" 
    + if_eager + vm["algorithm"].as<std::string>() + "-" + std::to_string(vm["eps"].as<double>()).substr(0, 5) + "-" + vm["cost_mode"].as<string>() + "-" + cur_time);
    auto current_time = std::time(nullptr);
    output << std::ctime(&current_time) << std::endl;
    output << "Map: " << vm["map"].as<std::string>() << std::endl << vm["algorithm"].as<std::string>() << std::endl
    << "agent num = " << vm["agent_num"].as<int>() << std::endl;
}
std::string cost_mode = vm["cost_mode"].as<std::string>();
output << "Cost Mode:  " << cost_mode << std::endl;
output << "Turn Mode:  " << vm["turn_mode"].as<int>() << std::endl;
output << "Turn Cost:  " << vm["turn_cost"].as<int>() << std::endl << std::endl;

/**************************  Search  *****************************/
std::string directoryPath = "../dataset/" + vm["map"].as<std::string>() + "/config";
DIR *dir;
struct dirent *entry;
dir = opendir(directoryPath.c_str());
int scene_num = 0;
int total_num = 0;
int total_success_num = 0;
double t_NonDomTime = 0, t_LowLevelTime = 0, t_TotalTime = 0, t_EagerTime = 0, t_DomPruneNum = 0, t_constraint_num = 0, t_SolutionNum_before = 0, t_SolutionNum_after = 0;
for(int i = 0; i < vm["dim"].as<int>(); i++){
    if(cost_mode.at(i) == 'd'){
        p.read_cost("../dataset/" + vm["map"].as<std::string>() + "/" + vm["cost"].as<string>() + "/distance.cost", map, edges, i);
    }else if(cost_mode.at(i) == 'u'){
        p.read_cost("../dataset/" + vm["map"].as<std::string>() + "/" + vm["cost"].as<string>() + "/unique" + vm["unique_file"].as<string>() + ".cost", map, edges, i);
    }
}

while ((entry = readdir(dir)) != NULL) {
    if (entry->d_type == DT_REG) {
        scene_num ++;
        if(scene_num < vm["start_scene"].as<int>()){
            continue;
        }
        if(scene_num > vm["r2"].as<int>()){
            break;
        }
        std::vector<std::pair<size_t, size_t>>  start_end;
        std::string filePath = directoryPath + "/" + (std::string)entry->d_name;
        p.read_config(filePath, map, vm["agent_num"].as<int>(), start_end);
        output << "************************************************************************" << std::endl;
        output << "Config Num  : " << scene_num << std::endl;
        output << "Config File:  " << filePath << std::endl << std::endl;
        int success_num = 0;
        std::vector<std::tuple<double, double, double, double, int, int, int, int>> temp;
        for(int i = vm["r1"].as<int>()*(scene_num-1) + 1; i < vm["r1"].as<int>()*scene_num + 1; i++){
            std::cout << "iteration: " << scene_num << "      run number: " << i << std::endl;
            total_num ++;
            Solver      solver;
            HSolutionID        hsolutions;
            std::vector<CostVector>    hsolution_costs;

            bool have_r = false;
            for(int j = 0; j < vm["dim"].as<int>(); j++){
                if(cost_mode.at(j) == 'r'){
                    p.read_cost("../dataset/" + vm["map"].as<std::string>() + "/" + vm["cost"].as<string>() + "/random-" + to_string(i+100*j) + ".cost", map, edges, j);
                    have_r = true;
                }
            }
            if(have_r){
                output << "COST MAP:";
                for(int j = 0; j < vm["dim"].as<int>(); j++){
                    if(cost_mode.at(j) == 'r'){
                        output << "  " << std::to_string(vm["dim"].as<int>()) << "/" << std::to_string(i+100*j) + ".cost";
                    }
                }
                output << std::endl << std::endl;
            }

            auto one_data = solver.search(map.graph_size, edges, vm, start_end, ms, logger, hsolutions, hsolution_costs);
            
            if (std::get<2>(one_data) > vm["cutoffTime"].as<int>()){
                if(!have_r){
                    break;
                }
                continue;
            }
            success_num ++;
            total_success_num ++;
            temp.push_back(one_data);
            if(!have_r){
                break;
            }
        }
        double NonDomTime = 0, LowLevelTime = 0, TotalTime = 0, EagerTime = 0, DomPruneNum = 0, constraint_num = 0, SolutionNum_before = 0, SolutionNum_after = 0;
        for(int i = 0; i < temp.size(); i++){
            NonDomTime += std::get<0>(temp.at(i))/(double)success_num;
            LowLevelTime += std::get<1>(temp.at(i))/(double)success_num;
            TotalTime += std::get<2>(temp.at(i))/(double)success_num;
            EagerTime += std::get<3>(temp.at(i))/(double)success_num;
            DomPruneNum += std::get<4>(temp.at(i))/(double)success_num;
            constraint_num += std::get<5>(temp.at(i))/(double)success_num;
            SolutionNum_before += std::get<6>(temp.at(i))/(double)success_num;
            SolutionNum_after += std::get<7>(temp.at(i))/(double)success_num;

            t_NonDomTime += std::get<0>(temp.at(i));
            t_LowLevelTime += std::get<1>(temp.at(i));
            t_TotalTime += std::get<2>(temp.at(i));
            t_EagerTime += std::get<3>(temp.at(i));
            t_DomPruneNum += std::get<4>(temp.at(i));
            t_constraint_num += std::get<5>(temp.at(i));
            t_SolutionNum_before += std::get<6>(temp.at(i));
            t_SolutionNum_after  += std::get<7>(temp.at(i));
        }
        output << std::endl;
        output << "NonDomTime = " << NonDomTime << std::endl;
        output << "LowLevelTime = " << LowLevelTime << std::endl;
        output << "Total Time = " << TotalTime << std::endl;
        output << "Eager Time = " << EagerTime << std::endl;
        output << "DomPruneNum/NodeExpandNum = " << DomPruneNum << "/" << constraint_num << std::endl;
        output << "SolutionNum = " << SolutionNum_before << "/" << SolutionNum_after << std::endl;
        output << std::endl << std::endl;
        std::cout << "FINISH ONCE" << std::endl;
    }
}
output << "AVERAGE :" << std::endl;
output << "NonDomTime = " << t_NonDomTime/(double)total_success_num << std::endl;
output << "LowLevelTime = " << t_LowLevelTime/(double)total_success_num << std::endl;
output << "Total Time = " << t_TotalTime/(double)total_success_num << std::endl;
output << "Eager Time = " << t_EagerTime/(double)total_success_num << std::endl;
output << "DomPruneNum/NodeExpandNum = " << t_DomPruneNum/(double)total_success_num << "/" << t_constraint_num/(double)total_success_num << std::endl;
output << "SolutionNum = " << t_SolutionNum_before/(double)total_success_num << "/" << t_SolutionNum_after/(double)total_success_num << std::endl;
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