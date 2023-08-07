#include <iostream>

#include <memory>
#include <time.h>
#include <fstream>
#include <chrono>

#include "Utils/Logger.h"
#include "Utils/Solver.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>


const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;

using namespace std;

int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("map,m", po::value<std::string>()->required(), "Map File")
        ("cost,c", po::value<std::string>()->default_value(""), "Cost File")
        ("config", po::value<std::string>()->required(), "Configure File")
        ("dim,d", po::value<int>()->default_value(2), "dimension of cost function")
        ("hem", po::value<double>()->default_value(0), "High Level merge approximate factor")
        ("lem", po::value<double>()->default_value(0), "Low Level merge approximate factor")
        ("hep", po::value<double>()->default_value(0), "High Level prune approximate factor")
        ("lep", po::value<double>()->default_value(0), "Low Level prune approximate factor")
        ("agent_num,n", po::value<int>()->default_value(-1), "number of agents")
        ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "low-level solvers (BOA, PPA or Apex search)")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")

        ("output,o", po::value<std::string>()->required(), "Name of the output file")
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
    MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
    if(vm["dim"].as<int>() == 3){
        ms = MergeStrategy::RANDOM;
    }


/*************************  Build  Map  ****************************/
    Map map;
    PreProcessor p;
    std::unordered_map<size_t, std::vector<int>> id2coord;
    std::vector<std::pair<size_t, size_t>>  start_end;
    std::vector<Edge>  edges;
    p.read_map(vm["map"].as<std::string>(), map, id2coord);
    p.read_config(vm["config"].as<std::string>(), map, vm["agent_num"].as<int>(), start_end);
    // p.read_cost(vm["cost"].as<std::string>(), map, edges);
    p.generate_cost(map, edges, vm["dim"].as<int>());
    map.ddelete();


/**************************  Search  *****************************/
    Solver      solver;
    HSolutionID        hsolutions;
    std::vector<CostVector>    hsolution_costs;

    auto start_time = std::chrono::high_resolution_clock::now();    // record time
    size_t constraint_num = solver.search(map.graph_size, edges, vm, start_end, ms, logger, hsolutions, hsolution_costs);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    delete(logger);


/********************  Print  Path  Info  ************************/
getchar();
    std::cout << endl << endl;
    for(size_t num = 0; num < hsolutions.size(); num ++){
        std::cout << "SOLUTION   " << num+1 << endl;
        std::cout << "COST   " << "{";
        if(vm["dim"].as<int>() == 2){
            std::cout << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1);
        }else{
            std::cout << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1) << ", " << hsolution_costs.at(num).at(2);
        }
        std::cout << "}" << endl;
        for(size_t i = 0; i < hsolutions.at(num).size(); i++){
            std::cout << "agent " << i+1 << ":" << std::endl;
            for(size_t id : hsolutions.at(num).at(i)){
                std::cout << "{" << id2coord[id].at(0) << ", " << id2coord[id].at(1) << "}, ";
            }
            std::cout << endl;
        }
        std::cout << endl << endl;
    }
    std::cout << "Total constraint number = " << constraint_num << std::endl;
    std::cout << "RUN TIME: " << ((double)duration.count())/1000000.0 << " seconds" << std::endl;
}