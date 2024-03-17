#include "HighLevel/kSolver.h"
#include "Utils.h"
#include "HighLevel/ConflictResolver.h"

#include <cstdlib>
#include <list>
#include <algorithm>
#include <set>
#include <random>
#include <iostream>
#include <vector>

extern std::unordered_map<size_t, std::vector<int>> id2coord;
extern std::string map_name;
extern std::ofstream output;


// Number of Solution
void kSolver::MergeJointPaths(HighLevelNodePtr node, int solution_num, double max_eps)
{
    for(auto e : node->sop_waypoints){
        std::cout << e.size() << " ";
    }
    auto _t1 = std::chrono::high_resolution_clock::now();
    int dim = node->sop_apex.at(0)[0].size();

    std::pair<CostVector, int>  temp_apex_id;
    CostVector     temp_real_cost;
    std::vector<size_t>    temp_id;  // of one joint path

    std::list<std::pair<CostVector, int>>   apex_idx_combos;
    std::vector<CostVector>     real_costs_vector;
    std::vector<int>            conflict_nums_vector;
    std::vector<std::vector<size_t>>    ids_vector;

    node->all_jps.clear();    

    apex_idx_combos.push_back(std::make_pair(CostVector(dim, 0), 0));
    real_costs_vector.push_back(CostVector(dim, 0));
    ids_vector.push_back(std::vector<size_t>());

    for(int i = 0; i < node->sop_waypoints.size(); i++){
        if(difftime(time(NULL), start_time) > TIME_LIMIT){
            return;
        }
        std::list<std::pair<CostVector, int>> _apex_idx_combos(apex_idx_combos);
        std::vector<CostVector>     _real_costs_vector(real_costs_vector);
        std::vector<std::vector<size_t>>    _ids_vector(ids_vector);
        apex_idx_combos.clear();
        real_costs_vector.clear(); real_costs_vector.shrink_to_fit();
        ids_vector.clear(); ids_vector.shrink_to_fit();
        int new_id = 0;
        for(const auto& apex_id_combo : _apex_idx_combos){
            for(const auto& indiv_path : node->sop_waypoints.at(i)){
                temp_apex_id = apex_id_combo;
                temp_real_cost = _real_costs_vector.at(apex_id_combo.second);
                temp_id = _ids_vector.at(apex_id_combo.second);
                add_cost(temp_apex_id.first, node->sop_apex.at(i)[indiv_path.first]);
                add_cost(temp_real_cost, node->sop_cost.at(i)[indiv_path.first]);
                temp_id.push_back(indiv_path.first);
                apex_idx_combos.push_back(std::make_pair(temp_apex_id.first, new_id++));
                real_costs_vector.push_back(temp_real_cost);
                ids_vector.push_back(temp_id);
            }
        }
        apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
            return less_than_pair<std::pair<CostVector, int>>(a, b);});        
        NonDomVec(apex_idx_combos, real_costs_vector, ids_vector, conflict_nums_vector, MergingStrategy::NONE, 0);
        MergeUntil(apex_idx_combos, real_costs_vector, solution_num, max_eps);
    }

    apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
        return less_than_pair<std::pair<CostVector, int>>(a, b);
    });     

    for(auto& element : apex_idx_combos){
        node->all_jps.push_back(std::make_pair(element.first, ids_vector.at(element.second)));
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    double duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1).count())/1000000.0;
    std::cout << " HLMergingTime = " << duration << std::endl;
    std::cout << node->all_jps.size() << std::endl;
    // getchar();
}


// A*pex version
void kSolver::NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
    std::vector<int>& conflict_nums_vector, MergingStrategy ms, double eps)
{
    if(ms != CONFLICT_BASED){
        if (apex_idx_combos.front().first.size() == 2) {
            size_t g_min = UINT64_MAX;
            for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ) {
                if(difftime(time(NULL), start_time) > TIME_LIMIT){
                    return;
                }
                if (iter->first.at(1) < g_min) {
                    g_min = iter->first.at(1);
                    if(ms == MergingStrategy::NONE || eps == 0){
                        iter ++;
                        continue;
                    }
                }else{
                    iter = apex_idx_combos.erase(iter);
                }
            }
        }else if(apex_idx_combos.front().first.size() == 3){
        // dominance check
            std::set<CostVector, less_than_vector>  domination_set;
            for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ){
                if(difftime(time(NULL), start_time) > TIME_LIMIT){
                    return;
                }
            //  dominance check: if apex be strictly dominated(not eps-), then prune; 
            //  else erase the behind strictly dominated ones in RB-tree
                std::vector<size_t>  trun_vec({iter->first.at(1), iter->first.at(2)});
                if(domination_set.empty()){
                    domination_set.insert(trun_vec);
                    iter ++;
                    continue;
                }
                auto it = domination_set.upper_bound(trun_vec);
                if(it != domination_set.begin() && (*std::prev(it)).at(1) <= trun_vec.at(1)){
                    iter = apex_idx_combos.erase(iter);
                }else{  
                    if(ms == MergingStrategy::NONE || eps == 0){
                        iter ++;
                        continue;
                    }
                }
            }
        }else{
            output << std::endl << "(A*pex) ERROR: non-LEAST_CONFLICT strategy only for g size = 2 or 3" << std::endl;
            output << std::endl << "d = " << apex_idx_combos.front().first.size();
            exit(1);
        }
    }else{
        // for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ){
        //     if(difftime(time(NULL), start_time) > TIME_LIMIT){
        //         return;
        //     }
        //     bool is_merge = false;
        //     if(iter == apex_idx_combos.begin()){
        //         iter ++;
        //         continue;
        //     }
        //     for(auto iter1 = std::prev(iter); ; iter1--){
        //         if(HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second),
        //         conflict_nums_vector.at(iter1->second), conflict_nums_vector.at(iter->second), ms, eps)){
        //             iter = apex_idx_combos.erase(iter);
        //             is_merge = true;
        //             break;
        //         }
        //         if(iter1 == apex_idx_combos.begin()){
        //             break;
        //         }
        //     }
        //     // for (auto iter1 = apex_idx_combos.begin(); iter1 != iter; iter1++) {
        //     //     if(HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second),
        //     //     conflict_nums_vector.at(iter1->second), conflict_nums_vector.at(iter->second), ms, eps)){
        //     //         iter = apex_idx_combos.erase(iter);
        //     //         is_merge = true;
        //     //         break;
        //     //     }
        //     // }
        //     if (!is_merge) {
        //         iter ++;
        //     }
        // }
    }
}


void kSolver::MergeUntil(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps)
{
    int total_num = apex_idx_combos.size();

    using mutual_eps = std::tuple<double, size_t, size_t>; // eps,apex_idx,real_idx,if_valid
    std::list<mutual_eps>               eps_list;
    std::vector<CostVector>             apex_vector;
    std::vector<bool>                   valid_vector(total_num, true);
    std::vector<int>                    id_vector;

    for(auto ele: apex_idx_combos){
        apex_vector.push_back(ele.first);
        id_vector.push_back(ele.second);
    }

    for(int i = 0; i < total_num; i++){
        for(int j = 0; j < total_num; j++){
            if(j == i){
                continue;
            }
            double eps = calculate_BF(apex_vector.at(i), real_costs_vector.at(id_vector.at(j)));
            eps_list.push_back(std::make_tuple(eps, i, j));
        }
    }

    int  erase_num = 0;
    while(total_num - erase_num > solution_num){
        if(difftime(time(NULL), start_time) > TIME_LIMIT){
            return;
        }
        mutual_eps element;
        double min_eps = INT_MAX;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); iter++){
            if(std::get<0>(*iter) < min_eps){
                element = *iter;
                min_eps = std::get<0>(*iter);
            }
        }
        if(min_eps > max_eps){
            break;
        }
        erase_num ++;
        size_t invalid_id = std::get<1>(element);
        size_t valid_id = std::get<2>(element);
        apex_vector.at(valid_id) = vector_min(apex_vector.at(valid_id), apex_vector.at(invalid_id));
        valid_vector.at(invalid_id) = false;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); ){
            if(std::get<1>(*iter) == invalid_id || std::get<2>(*iter) == invalid_id){
                iter = eps_list.erase(iter);
                continue;
            }
            if(std::get<1>(*iter) == valid_id){
                std::get<0>(*iter) = calculate_BF(apex_vector.at(valid_id), real_costs_vector.at(id_vector.at(std::get<2>(*iter))));
            }
            iter ++;
        }
    }

    apex_idx_combos.clear();
    int j = 0;
    for(int i = 0; i < total_num; i++){
        if(!valid_vector.at(i)){
            continue;
        }
        apex_idx_combos.push_back(std::make_pair(apex_vector.at(i), id_vector.at(i)));
    }
}


void kSolver::MergeUntil(std::vector<CostVector>& apex_vectors, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps)
{
    int total_num = apex_vectors.size();

    using mutual_eps = std::tuple<double, size_t, size_t>; // eps,apex_idx,real_idx,if_valid
    std::list<mutual_eps>               eps_list;
    std::vector<CostVector>             apex_vector = apex_vectors;
    std::vector<CostVector>             cost_vector = real_costs_vector;
    std::vector<bool>                   valid_vector(total_num, true);

    for(int i = 0; i < total_num; i++){
        for(int j = 0; j < total_num; j++){
            if(j == i){
                continue;
            }
            double eps = calculate_BF(apex_vector.at(i), cost_vector.at(j));
            eps_list.push_back(std::make_tuple(eps, i, j));
        }
    }

    int  erase_num = 0;
    while(total_num - erase_num > solution_num){
        mutual_eps element;
        double min_eps = INT_MAX;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); iter++){
            if(std::get<0>(*iter) < min_eps){
                element = *iter;
                min_eps = std::get<0>(*iter);
            }
        }
        if(min_eps > max_eps){
            break;
        }
        erase_num ++;
        size_t invalid_id = std::get<1>(element);
        size_t valid_id = std::get<2>(element);
        apex_vector.at(valid_id) = vector_min(apex_vector.at(valid_id), apex_vector.at(invalid_id));
        valid_vector.at(invalid_id) = false;
        for(auto iter = eps_list.begin(); iter != eps_list.end(); ){
            if(std::get<1>(*iter) == invalid_id || std::get<2>(*iter) == invalid_id){
                iter = eps_list.erase(iter);
                continue;
            }
            if(std::get<1>(*iter) == valid_id){
                std::get<0>(*iter) = calculate_BF(apex_vector.at(valid_id), cost_vector.at(std::get<2>(*iter)));
            }
            iter ++;
        }
    }

    apex_vectors.clear();
    real_costs_vector.clear();
    
    for(int i = 0; i < total_num; i++){
        if(!valid_vector.at(i)){
            continue;
        }
        apex_vectors.push_back(apex_vector.at(i));
        real_costs_vector.push_back(cost_vector.at(i));
    }
}



void kSolver::calculate_CAT(HighLevelNodePtr node, VertexCAT& vertex_cat, EdgeCAT& edge_cat, int agent_id)
{
    vertex_cat.clear(); edge_cat.clear();
    for(int i = 0; i < node->cur_ids.size(); i++){
        if(i == agent_id){
            continue;
        }
        auto path_nodes = node->sop_waypoints.at(i)[node->cur_ids.at(i)];
        for(int t = 0; t < path_nodes.size(); t++){
            size_t node_id = path_nodes.at(t);
            if(!vertex_cat.count(t)){
                vertex_cat.insert(std::make_pair(t, std::vector<int>(GRAPH_SIZE, 0)));
            }
            vertex_cat.at(t).at(node_id) ++;
        }
        for(int t = 0; t < path_nodes.size() - 1; t++){
            size_t source_id = path_nodes.at(t);
            size_t target_id = path_nodes.at(t+1);
            if(!edge_cat.count(t)){
                edge_cat.insert(std::make_pair(t, std::vector<std::unordered_map<size_t, int>>(GRAPH_SIZE)));
            }
            if(!edge_cat.at(t).at(source_id).count(target_id)){
                edge_cat.at(t).at(source_id).insert(std::make_pair(target_id, 0));
            }
            edge_cat.at(t).at(source_id).at(target_id) ++;
        }
    }
}


OutputTuple kSolver::run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_goal, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger)
{
    time_t end_time;
    std::chrono::_V2::system_clock::time_point t1, t2;
    double duration;

    std::vector<CostVector> hsolution_apex_costs;
    bool is_success = true;
    
    double HLMergingTime = 0, LowLevelTime = 0, TotalTime;
    int ConflictSolvingNum = 0, SolutionNum = 0;

    ConflictResolver conflict_resolver;

    start_time = time(NULL);
    HLQueue open_list;

    VertexCAT  vertex_cat;
    EdgeCAT    edge_cat;

// calculate heuristic
    std::vector<Heuristic> heuristics(AGENT_NUM);
    AdjacencyMatrix graph(GRAPH_SIZE, edges);   // can run outside and only once
    AdjacencyMatrix inv_graph(GRAPH_SIZE, edges, true);
    for(int i = 0; i < AGENT_NUM; i++){
        ShortestPathHeuristic sp_heuristic(start_goal.at(i).second, GRAPH_SIZE, inv_graph, TURN_DIM, TURN_COST);
        heuristics.at(i) = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }

//  initialize open_list
    HighLevelNodePtr root_node = std::make_shared<HighLevelNode>(AGENT_NUM);

    for(size_t i = 0; i < AGENT_NUM; i ++){
        t1 = std::chrono::high_resolution_clock::now();

        if(MS == MergingStrategy::CONFLICT_BASED){
            single_run_map(GRAPH_SIZE, graph, heuristics.at(i), start_goal.at(i).first, start_goal.at(i).second, 
                LSOLVER, EPS, DEFAULT_MS, logger, TIME_LIMIT, root_node->sop_waypoints.at(i), root_node->sop_apex.at(i), root_node->sop_cost.at(i), 
                root_node->vertex_constraints.at(i), root_node->edge_constraints.at(i), vertex_cat, edge_cat, root_node->conflict_num, TURN_DIM, TURN_COST);
        }else{
            single_run_map(GRAPH_SIZE, graph, heuristics.at(i), start_goal.at(i).first, start_goal.at(i).second, 
                LSOLVER, EPS, MS, logger, TIME_LIMIT, root_node->sop_waypoints.at(i), root_node->sop_apex.at(i), root_node->sop_cost.at(i), 
                root_node->vertex_constraints.at(i), root_node->edge_constraints.at(i), vertex_cat, edge_cat, root_node->conflict_num, TURN_DIM, TURN_COST);
        }

        std::list<std::pair<CostVector, int>>   apex_idx_combos;
        std::vector<CostVector>     real_costs_vector;

        for(auto ele: root_node->sop_waypoints.at(i)){
            apex_idx_combos.push_back(std::make_pair(root_node->sop_apex.at(i)[ele.first], ele.first));
            real_costs_vector.push_back(root_node->sop_cost.at(i)[ele.first]);
        }

        MergeUntil(apex_idx_combos, real_costs_vector, SOLUTION_NUM);

        auto apex_map = root_node->sop_apex.at(i);
        auto cost_map = root_node->sop_cost.at(i);
        auto path_map = root_node->sop_waypoints.at(i);
        root_node->sop_apex.at(i).clear(); 
        root_node->sop_cost.at(i).clear();
        root_node->sop_waypoints.at(i).clear();
        
        int idx = 0;
        for (auto ele: apex_idx_combos){
            root_node->sop_apex.at(i).insert(std::make_pair(idx, apex_map[ele.second]));
            root_node->sop_cost.at(i).insert(std::make_pair(idx, cost_map[ele.second]));
            root_node->sop_waypoints.at(i).insert(std::make_pair(idx, path_map[ele.second]));
            idx ++;
        }
        
        t2 = std::chrono::high_resolution_clock::now();
        duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
        LowLevelTime += duration;
        std::cout << "Agent ID: " << i << " Low Level Time = " << duration << "  size = " << root_node->sop_waypoints.at(i).size() << std::endl;
    }

    t1 = std::chrono::high_resolution_clock::now();

    MergeJointPaths(root_node, SOLUTION_NUM);

    t2 = std::chrono::high_resolution_clock::now();
    duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
    HLMergingTime += duration;

    if(difftime(time(NULL), start_time) > TIME_LIMIT){
        output << "FAIL" << std::endl 
            << "apex cost:" << std::endl 
            << std::endl 
            << "real cost:" << std::endl 
            << std::endl << std::endl << std::endl;

        return std::make_tuple(HLMergingTime, LowLevelTime, difftime(time(NULL), start_time), 0, 0);
    }

    root_node->cur_ids = root_node->all_jps.front().second;
    root_node->cur_apex = root_node->all_jps.front().first;
    open_list.insert(root_node);

    std::tuple<int, int, CostVector, size_t> cft;
//  main loop    
    while(!open_list.empty())
    {
        if(difftime(time(NULL), start_time) > TIME_LIMIT){
            is_success = false;
            break;
        }
        auto node = open_list.pop();

        auto current_path = node->all_jps.front();

    // eager solution update
        if(EAGER){
            //  exhaust all joint path in high-level node, if no collision, then add to solution set
            //  remark: this cannot ensure non-dominated set
            for(auto iter = node->all_jps.begin(); iter != node->all_jps.end(); ){
                cft = conflict_resolver.DetectConflict(*iter, node->sop_waypoints);
                if(std::get<2>(cft).empty()){   // collision-free, use weakly dominate to prune
                    CostVector  real_cost(iter->first.size(), 0);
                    for(int i = 0; i < AGENT_NUM; i++){
                        add_cost(real_cost, node->sop_cost.at(i)[iter->second.at(i)]);
                    }
                    //  dominance check
                    bool if_dominated = false;
                    for(int i = 0; i < hsolution_costs.size(); i++){
                        if(is_dominated(iter->first, hsolution_costs.at(i), EPS)){
                            hsolution_apex_costs.at(i) = vector_min(hsolution_apex_costs.at(i), iter->first);
                            if_dominated = true;
                            break;
                        }
                    }

                    if(!if_dominated){   
                        //  add to solution
                        int flag = 0;
                        for(auto iter1 = hsolution_apex_costs.begin(); iter1 != hsolution_apex_costs.end(); ){
                            if(is_dominated(*iter1, real_cost, EPS)){
                                iter->first = vector_min(iter->first, *iter1);
                                iter1 = hsolution_apex_costs.erase(iter1);
                                hsolution_costs.erase(hsolution_costs.begin()+flag);
                                hsolution_ids.erase(hsolution_ids.begin()+flag);
                            }else{
                                flag ++;
                                iter1 ++;
                            }
                        }

                        std::vector<std::vector<size_t>> new_hsolution;
                        for(int i = 0; i < AGENT_NUM; i ++){
                            new_hsolution.push_back(node->sop_waypoints.at(i)[iter->second.at(i)]);
                        }
                        hsolution_ids.push_back(new_hsolution);
                        hsolution_costs.push_back(real_cost);
                        hsolution_apex_costs.push_back(iter->first);
                        std::cout << "there is a solution" << std::endl;
                        
                        MergeUntil(hsolution_apex_costs, hsolution_costs, SOLUTION_NUM);
                        for(int i = 0; i < hsolution_apex_costs.size(); i++){
                            double _eps = calculate_BF(hsolution_apex_costs.at(i), hsolution_costs.at(i));
                            EPS = EPS > _eps ? EPS : _eps;
                        }
                        // if(solve_mode == SolveMode::SMALLEST_eps || solve_mode == SolveMode::DIVERSITY){
                        //     double _eps = calculate_BF(iter->first, real_cost);
                        //     hEPS_apex = hEPS_apex > _eps ? hEPS_apex : _eps;
                        // }
                    }

                    iter = node->all_jps.erase(iter);
                    continue;
                }else{
                    iter ++;
                }
            }

            if(node->all_jps.empty()){
                continue;
            }
        }

    //  DomPrune
        while(!node->all_jps.empty()){
            bool is_pruned = false;
            for(int i = 0; i < hsolution_costs.size(); i++){
                if(is_dominated(node->all_jps.front().first, hsolution_costs.at(i), EPS)){
                    hsolution_apex_costs.at(i) = vector_min(hsolution_apex_costs.at(i), node->all_jps.front().first);
                    is_pruned = true;
                    break;
                }
            }
            if(is_pruned){
                node->all_jps.pop_front();
            }else{
                break;
            }
        }

        if(node->all_jps.empty()){
            continue;
        }

        node->cur_ids = node->all_jps.front().second;
        node->cur_apex = node->all_jps.front().first;

        if(node->all_jps.front() != current_path){
            open_list.insert(node);
            continue;
        }

        cft = conflict_resolver.DetectConflict(node->all_jps.front(), node->sop_waypoints);

        if(std::get<2>(cft).empty()){
            if(EAGER){
                output << "error : cft not empty with eager solution update";
                exit(1);
            }else{
                //  solution update
                CostVector  real_cost(node->cur_apex.size(), 0);
                for(int i = 0; i < AGENT_NUM; i++){
                    add_cost(real_cost, node->sop_cost.at(i)[node->cur_ids.at(i)]);
                }
                int flag = 0;
                for(auto iter1 = hsolution_apex_costs.begin(); iter1 != hsolution_apex_costs.end(); ){
                    if(is_dominated(*iter1, real_cost, EPS)){
                        node->cur_apex = vector_min(node->cur_apex, *iter1);
                        iter1 = hsolution_apex_costs.erase(iter1);
                        hsolution_costs.erase(hsolution_costs.begin()+flag);
                        hsolution_ids.erase(hsolution_ids.begin()+flag);
                    }else{
                        flag ++;
                        iter1 ++;
                    }
                }

                std::vector<std::vector<size_t>> new_hsolution;
                new_hsolution.push_back(node->cur_ids);
                hsolution_ids.push_back(new_hsolution);
                hsolution_costs.push_back(real_cost);
                hsolution_apex_costs.push_back(node->cur_apex);
                std::cout << "there is a solution" << std::endl;

                node->all_jps.pop_front();

                if(node->all_jps.empty()){
                    continue;
                }

                node->cur_ids = node->all_jps.front().second;
                node->cur_apex = node->all_jps.front().first;

                if(ALGORITHM == Algorithm::BBMOCBS_K){
                    MergeUntil(hsolution_apex_costs, hsolution_costs, SOLUTION_NUM);
                    double _eps = calculate_BF(node->cur_apex, real_cost);
                    EPS = EPS > _eps ? EPS : _eps;
                }

                open_list.insert(node);
                continue;
            }
        }
        
        // print constraint info
        ConflictSolvingNum ++;
        if (ConflictSolvingNum % (500/AGENT_NUM) == 0) {
            std::cout << "[INFO] * Solver::Search, after " << ConflictSolvingNum << " conflict splits " 
            << "       time = " << difftime(time(NULL), start_time) << std::endl
            << map_name << std::endl;
        }


        if(std::get<2>(cft).size() == 1){   // vertex confliction, split 1 or 2 constraints
            for(int i = 0; i < 2; i ++){
                int agent_id = i == 0 ? std::get<0>(cft) : std::get<1>(cft);
                if(agent_id < 0){
                    continue;
                }
                auto new_node = std::make_shared<HighLevelNode>(*node);
                new_node->sop_waypoints.at(agent_id).clear();
                new_node->sop_apex.at(agent_id).clear();
                new_node->sop_cost.at(agent_id).clear();
                new_node->conflict_num.clear();
                new_node->all_jps.clear();


                conflict_resolver.AddConstraint(new_node->vertex_constraints, agent_id, std::get<2>(cft).front(), std::get<3>(cft));

            //  Low Level Search
                if(MS == MergingStrategy::CONFLICT_BASED){
                    calculate_CAT(node, vertex_cat, edge_cat, agent_id);
                }

                t1 = std::chrono::high_resolution_clock::now();
                single_run_map(GRAPH_SIZE, graph, heuristics.at(agent_id), start_goal.at(agent_id).first, start_goal.at(agent_id).second, 
                    LSOLVER, EPS, MS, logger, TIME_LIMIT, new_node->sop_waypoints.at(agent_id),  
                    new_node->sop_apex.at(agent_id), new_node->sop_cost.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id], vertex_cat, edge_cat, new_node->conflict_num, TURN_DIM, TURN_COST); 


                std::list<std::pair<CostVector, int>>   apex_idx_combos;
                std::vector<CostVector>     real_costs_vector;

                for(auto ele: new_node->sop_waypoints.at(agent_id)){
                    apex_idx_combos.push_back(std::make_pair(new_node->sop_apex.at(agent_id)[ele.first], ele.first));
                    real_costs_vector.push_back(new_node->sop_cost.at(agent_id)[ele.first]);
                }

                MergeUntil(apex_idx_combos, real_costs_vector, SOLUTION_NUM);

                auto apex_map = new_node->sop_apex.at(agent_id);
                auto cost_map = new_node->sop_cost.at(agent_id);
                auto path_map = new_node->sop_waypoints.at(agent_id);
                new_node->sop_apex.at(agent_id).clear(); 
                new_node->sop_cost.at(agent_id).clear();
                new_node->sop_waypoints.at(agent_id).clear();
                
                int idx = 0;
                for (auto ele: apex_idx_combos){
                    new_node->sop_apex.at(agent_id).insert(std::make_pair(idx, apex_map[ele.second]));
                    new_node->sop_cost.at(agent_id).insert(std::make_pair(idx, cost_map[ele.second]));
                    new_node->sop_waypoints.at(agent_id).insert(std::make_pair(idx, path_map[ele.second]));
                    idx ++;
                }


                t2 = std::chrono::high_resolution_clock::now();
                duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
                LowLevelTime += duration;
                std::cout << "LowLevelTime = " << duration << std::endl;

                if(new_node->sop_waypoints.at(agent_id).empty()){
                    continue;
                }

            //  Calculate NonDomSet
                t1 = std::chrono::high_resolution_clock::now();

                
                MergeJointPaths(new_node, SOLUTION_NUM);

                if(difftime(time(NULL), start_time) > TIME_LIMIT){
                    is_success = false;
                    continue;
                }
                t2 = std::chrono::high_resolution_clock::now();
                duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
                HLMergingTime += duration;
                
                new_node->cur_ids = new_node->all_jps.front().second;
                new_node->cur_apex = new_node->all_jps.front().first;
                open_list.insert(new_node);
            }

        }else{  //  edge confliction
            for(int i = 0; i < 2; i++){
                int agent_id = i == 0 ? std::get<0>(cft) : std::get<1>(cft);
                auto new_node = std::make_shared<HighLevelNode>(*node);
                new_node->sop_waypoints.at(agent_id).clear();
                new_node->sop_apex.at(agent_id).clear();
                new_node->sop_cost.at(agent_id).clear();
                new_node->conflict_num.clear();
                new_node->all_jps.clear();


                conflict_resolver.AddConstraint(new_node->edge_constraints, agent_id, std::get<2>(cft).at(i), std::get<2>(cft).at(1-i), std::get<3>(cft));

                if(MS == MergingStrategy::CONFLICT_BASED){
                    calculate_CAT(node, vertex_cat, edge_cat, agent_id);
                }

                t1 = std::chrono::high_resolution_clock::now();
                single_run_map(GRAPH_SIZE, graph, heuristics.at(agent_id), start_goal.at(agent_id).first, start_goal.at(agent_id).second, 
                    LSOLVER, EPS, MS, logger, TIME_LIMIT, new_node->sop_waypoints.at(agent_id), 
                    new_node->sop_apex.at(agent_id), new_node->sop_cost.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id], vertex_cat, edge_cat, new_node->conflict_num, TURN_DIM, TURN_COST);


                std::list<std::pair<CostVector, int>>   apex_idx_combos;
                std::vector<CostVector>     real_costs_vector;

                for(auto ele: new_node->sop_waypoints.at(agent_id)){
                    apex_idx_combos.push_back(std::make_pair(new_node->sop_apex.at(agent_id)[ele.first], ele.first));
                    real_costs_vector.push_back(new_node->sop_cost.at(agent_id)[ele.first]);
                }

                MergeUntil(apex_idx_combos, real_costs_vector, SOLUTION_NUM);

                auto apex_map = new_node->sop_apex.at(agent_id);
                auto cost_map = new_node->sop_cost.at(agent_id);
                auto path_map = new_node->sop_waypoints.at(agent_id);
                new_node->sop_apex.at(agent_id).clear(); 
                new_node->sop_cost.at(agent_id).clear();
                new_node->sop_waypoints.at(agent_id).clear();
                
                int idx = 0;
                for (auto ele: apex_idx_combos){
                    new_node->sop_apex.at(agent_id).insert(std::make_pair(idx, apex_map[ele.second]));
                    new_node->sop_cost.at(agent_id).insert(std::make_pair(idx, cost_map[ele.second]));
                    new_node->sop_waypoints.at(agent_id).insert(std::make_pair(idx, path_map[ele.second]));
                    idx ++;
                }

                t2 = std::chrono::high_resolution_clock::now();
                duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
                std::cout << "LowLevelTime = " << duration << std::endl;
                LowLevelTime += duration;

                if(new_node->sop_waypoints.at(agent_id).empty()){
                    continue;
                }

            //  Calculate NonDomSet
                t1 = std::chrono::high_resolution_clock::now();

                
                MergeJointPaths(new_node, SOLUTION_NUM);

                if(difftime(time(NULL), start_time) > TIME_LIMIT){
                    is_success = false;
                    continue;
                }
                t2 = std::chrono::high_resolution_clock::now();
                duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
                HLMergingTime += duration;
                
                new_node->cur_ids = new_node->all_jps.front().second;
                new_node->cur_apex = new_node->all_jps.front().first;
                open_list.insert(new_node);
            }
        }
    }
    end_time = time(NULL); // for timing.
    TotalTime = difftime(end_time, start_time);


    
    std::vector<std::pair<CostVector, int>> apex_id_list(hsolution_costs.size());
    auto _hsolution_costs = hsolution_costs;
    for(int i = 0; i < hsolution_costs.size(); i++){
        apex_id_list.at(i) = std::make_pair(hsolution_apex_costs.at(i), i);
    }
    hsolution_costs.clear(); hsolution_costs.shrink_to_fit();
    hsolution_apex_costs.clear(); hsolution_apex_costs.shrink_to_fit();    
    std::sort(apex_id_list.begin(), apex_id_list.end(), [](const std::pair<CostVector, int>& a, const std::pair<CostVector, int>& b){
        for(int i = 0; i < a.first.size(); i++){
            if(a.first.at(i) != b.first.at(i)){
                return a.first.at(i) < b.first.at(i);
            }
        }
        return true;
    });
    for(auto ele: apex_id_list){
        hsolution_apex_costs.push_back(ele.first);
        hsolution_costs.push_back(_hsolution_costs.at(ele.second));
    }
    
    // if(ALGORITHM == Algorithm::BBMOCBS_K){
    //     for(int i = 0; i < hsolution_apex_costs.size(); i ++){
    //         double _eps = calculate_BF(hsolution_apex_costs.at(i), hsolution_costs.at(i));
    //         EPS = EPS > _eps ? EPS : _eps;
    //     }
    // }

    if(is_success){
        output << "SUCCESS" << std::endl;
    }else{
        output << "FAIL" << std::endl;
    }
    output << "apex cost: " << std::endl;
    int i = 0;
    for(size_t num = 0; num < hsolution_apex_costs.size(); num ++){
        if(i++ == 7){
            output << std::endl;
            i = 1;
        }
        if(DIM == 2){
            output << "{" << hsolution_apex_costs.at(num).at(0) << ", " << hsolution_apex_costs.at(num).at(1);
        }else{
            output << "{" << hsolution_apex_costs.at(num).at(0) << ", " << hsolution_apex_costs.at(num).at(1) << ", " << hsolution_apex_costs.at(num).at(2);
        }
        output << "}, ";
    }
    output << std::endl;
    output << "real cost: " << std::endl;
    int j = 0;
    for(size_t num = 0; num < hsolution_costs.size(); num ++){
        if(j++ == 7){
            output << std::endl;
            j = 1;
        }
        if(DIM == 2){
            output << "{" << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1);
        }else{
            output << "{" << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1) << ", " << hsolution_costs.at(num).at(2);
        }
        output << "}, ";
    }
    output << std::endl << std::endl;
    if(ALGORITHM == Algorithm::BBMOCBS_K){
        output << "EPSilon = " << EPS << std::endl << std::endl;
    }

    return std::make_tuple(HLMergingTime, LowLevelTime, TotalTime, ConflictSolvingNum, hsolution_costs.size());
}