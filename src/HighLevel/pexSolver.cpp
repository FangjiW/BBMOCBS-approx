#include "HighLevel/pexSolver.h"
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



void pexSolver::MergeJointPaths(HighLevelNodePtr node, MergingStrategy ms, double eps, int agent_id)
{
    for(auto e : node->sop_waypoints){
        std::cout << e.size() << " ";
    }
    auto _t1 = std::chrono::high_resolution_clock::now();

    std::list<std::pair<CostVector, int>>   apex_idx_combos;
    std::vector<CostVector>     real_costs_vector;
    std::vector<int>            conflict_nums_vector;
    std::vector<std::vector<size_t>>    ids_vector;

    if(ms != MergingStrategy::CONFLICT_BASED){
        std::pair<CostVector, int>  temp_apex_id;
        CostVector     temp_real_cost;
        std::vector<size_t>    temp_id;  // of one joint path


        apex_idx_combos.push_back(std::make_pair(CostVector(DIM, 0), 0));
        real_costs_vector.push_back(CostVector(DIM, 0));
        ids_vector.push_back(std::vector<size_t>());

        for(int i = 0; i < AGENT_NUM; i++){
            if(difftime(time(NULL), start_time) > TIME_LIMIT){
                return;
            }
            if(i == agent_id){
                for(auto& id_vector: ids_vector){
                    id_vector.push_back(0);
                }
                continue;
            }
            std::list<std::pair<CostVector, int>> _apex_idx_combos(apex_idx_combos);
            std::vector<CostVector>     _real_costs_vector(real_costs_vector);
            std::vector<std::vector<size_t>>    _ids_vector(ids_vector);
            
            apex_idx_combos = std::list<std::pair<CostVector, int>>();
            real_costs_vector = std::vector<CostVector>();
            ids_vector = std::vector<std::vector<size_t>>(); 

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
            NonDomVec(apex_idx_combos, real_costs_vector, ids_vector, conflict_nums_vector, ms, eps);
        }
    }else{
        if((time(NULL) - start_time)/CLOCKS_PER_SEC > TIME_LIMIT){
            return;
        }

        int new_id = 0;
        for(auto& joint_path: node->all_jps){
            CostVector apex = joint_path.first;
            CostVector cost(DIM, 0);
            
            for(int i = 0; i < AGENT_NUM; i ++){
                if(i == agent_id){
                    continue;
                }
                add_cost(cost, node->sop_cost.at(i).at(joint_path.second.at(i)));
            }

            for(auto& path_id: node->sop_waypoints.at(agent_id)){
                auto _apex = apex, _cost = cost;
                // std::cout << _apex.at(0) << " " << _apex.at(1); getchar();
                add_cost(_apex, node->sop_apex.at(agent_id).at(path_id.first));
                apex_idx_combos.push_back(std::make_pair(_apex, new_id++));

                add_cost(_cost, node->sop_cost.at(agent_id).at(path_id.first));
                real_costs_vector.push_back(_cost);

                auto id_vector = joint_path.second;
                id_vector.at(agent_id) = path_id.first;
                ids_vector.push_back(id_vector);

                conflict_nums_vector.push_back(node->conflict_num.at(path_id.first));
            }
        }
        apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
            return less_than_pair<std::pair<CostVector, int>>(a, b);});        
        NonDomVec(apex_idx_combos, real_costs_vector, ids_vector, conflict_nums_vector, MergingStrategy::CONFLICT_BASED, eps);
    }
    apex_idx_combos.sort([](const std::pair<CostVector, int>& a, std::pair<CostVector, int>& b){
        return less_than_pair<std::pair<CostVector, int>>(a, b);});

    node->all_jps.clear();
    for(auto element : apex_idx_combos){
        node->all_jps.push_back(std::make_pair(element.first, ids_vector.at(element.second)));
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    std::cout << " HLMergingTime = " << ((double)duration.count()) / 1000000.0 << std::endl;
    std::cout << node->all_jps.size() << std::endl;
    // getchar();
}



void pexSolver::NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
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
                    bool is_merge = false;
                    if(iter == apex_idx_combos.begin()){
                        iter ++;
                        continue;
                    }
                    for(auto iter1 = std::prev(iter); ; iter1--){
                        if (HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second), -1, -1, ms, eps)){
                            iter = apex_idx_combos.erase(std::next(iter1), std::next(iter));
                            // std::cout << apex_idx_combos.size();
                            // getchar();
                            is_merge = true;
                            break;
                        }
                        if(iter1 == apex_idx_combos.begin()){
                            break;
                        }
                    }
                    if (!is_merge) {
                        iter ++;
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
                //  merge
                    bool is_merge = false;
                    if(iter == apex_idx_combos.begin()){
                        iter ++;
                        continue;
                    }
                    for(auto iter1 = std::prev(iter); ; iter1--){
                        if (HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second), -1, -1, ms, eps)){
                            trun_vec = std::vector<size_t>({iter1->first.at(1), iter1->first.at(2)});
                            iter = apex_idx_combos.erase(std::next(iter1), std::next(iter));
                            is_merge = true;
                            break;
                        }
                        if(iter1 == apex_idx_combos.begin()){
                            break;
                        }
                    }
                    if (!is_merge) {
                        iter ++;
                        continue;
                    }
                //  erase the behind weakly dominated ones
                    it = domination_set.lower_bound(trun_vec);
                    for(auto iter_ = it; iter_ != domination_set.end(); ){
                        if(trun_vec.at(1) <= iter_->at(1)){
                            iter_ = domination_set.erase(iter_);
                        }else{
                            break;
                        }
                    }
                    domination_set.insert(trun_vec);
                }
            }
        }else{
            output << std::endl << "(A*pex) ERROR: non-LEAST_CONFLICT strategy only for g size = 2 or 3" << std::endl;
            output << std::endl << "d = " << apex_idx_combos.front().first.size();
            exit(1);
        }
    }else{
        for(auto iter = apex_idx_combos.begin(); iter != apex_idx_combos.end(); ){
            if(difftime(time(NULL), start_time) > TIME_LIMIT){
                return;
            }
            bool is_merge = false;
            if(iter == apex_idx_combos.begin()){
                iter ++;
                continue;
            }
            for(auto iter1 = std::prev(iter); ; iter1--){
                if(HighLevelMerge(*iter1, *iter, real_costs_vector.at(iter1->second), real_costs_vector.at(iter->second),
                conflict_nums_vector.at(iter1->second), conflict_nums_vector.at(iter->second), ms, eps)){
                    iter = apex_idx_combos.erase(iter);
                    is_merge = true;
                    break;
                }
                if(iter1 == apex_idx_combos.begin()){
                    break;
                }
            }
            if (!is_merge) {
                iter ++;
            }
        }
    }
}


bool pexSolver::HighLevelMerge(std::pair<CostVector, int>& existing_path, std::pair<CostVector, int>& new_path, CostVector& real_cost1, 
    CostVector& real_cost2, int conflict_num1, int conflict_num2, MergingStrategy ms, double eps)
{
    CostVector apex = vector_min(std::get<0>(existing_path), std::get<0>(new_path));
    if(ms == MergingStrategy::REVERSE_LEX){
        for(int i = DIM-1; i >= 0; i --){
            if(real_cost1.at(i) < real_cost2.at(i)){
                if(is_dominated(apex, real_cost1, eps)){
                    existing_path.first = apex;
                    return true;
                }else{
                    return false;
                }
            }else if(real_cost1.at(i) > real_cost2.at(i)){
                if(is_dominated(apex, real_cost2, eps)){
                    existing_path.first = apex;
                    existing_path.second = new_path.second;
                    return true;
                }else{
                    return false;
                }
            }
        }
        if(is_dominated(apex, real_cost1, eps)){
            existing_path.first = apex;
            return true;
        }else{
            return false;
        }
    }else if(ms == MergingStrategy::RANDOM){
        if(is_dominated(apex, real_cost1, eps)){
            existing_path.first = apex;
            return true;
        }
        if(is_dominated(apex, real_cost2, eps)){
            existing_path.first = apex;
            existing_path.second = new_path.second;
            return true;
        }
        return false;
    }else if(ms == MergingStrategy::CONFLICT_BASED){
        if(conflict_num1 < conflict_num2){
            if(is_dominated(apex, real_cost1, eps)){
                existing_path.first = apex;
                return true;
            }
            return false;
        }else if(conflict_num2 < conflict_num1){
            if(is_dominated(apex, real_cost2, eps)){
                existing_path.first = apex;
                existing_path.second = new_path.second;
                return true;
            }
            return false;
        }else{
            if(is_dominated(apex, real_cost1, eps)){
                existing_path.first = apex;
                return true;
            }
            if(is_dominated(apex, real_cost2, eps)){
                existing_path.first = apex;
                existing_path.second = new_path.second;
                return true;
            }
            return false;
        }
    }else if(ms == MergingStrategy::MORE_SLACK){
        double eps1 = calculate_BF(apex, real_cost1);
        double eps2 = calculate_BF(apex, real_cost2);
        if(eps1 <= eps2 && eps1 <= eps){
            existing_path.first = apex;
            return true;
        }else if(eps2 < eps1 && eps2 <= eps){
            existing_path.first = apex;
            existing_path.second = new_path.second;
            return true;
        }else{
            return false;
        }
    }
    else{
        output << std::endl << "merge strategy only SMALLER_G2 and RANDOM";
        exit(1);
    }
}

void pexSolver::PruneApproxDom(std::list<JointPathPair>& jp_list, std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost, double eps)
{
    for(auto iter = jp_list.begin(); iter != jp_list.end(); ){
        bool pruned = false;
        for(int i = 0; i < SOLUTIONS_apex.size(); i++)
        {
            if(is_dominated(iter->first, SOLUTIONS_cost.at(i), eps)){
                pruned = true;
                SOLUTIONS_apex.at(i) = vector_min(SOLUTIONS_apex.at(i), iter->first);
                break;
            }
        }
        if(pruned){
            iter = jp_list.erase(iter);
        }else{
            iter ++;
        }
    }
}



void pexSolver::AddSolution(CostVector& apex, CostVector& cost, std::vector<std::vector<size_t>>& waypoints, std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost, HSolutionID& SOLUTIONS_waypoints)
{
    for(int i = 0; i < SOLUTIONS_cost.size(); ){
        if(is_dominated(cost, SOLUTIONS_cost.at(i))){
            SOLUTIONS_apex.at(i) = vector_min(SOLUTIONS_apex.at(i), apex);
            return;
        }else if(is_dominated(SOLUTIONS_cost.at(i), cost)){
            apex = vector_min(apex, SOLUTIONS_apex.at(i));
            SOLUTIONS_apex.erase(SOLUTIONS_apex.begin() + i);
            SOLUTIONS_cost.erase(SOLUTIONS_cost.begin() + i);
            SOLUTIONS_waypoints.erase(SOLUTIONS_waypoints.begin() + i);
        }else{
            i ++;
        }
    }

    SOLUTIONS_waypoints.push_back(waypoints);
    SOLUTIONS_cost.push_back(cost);
    SOLUTIONS_apex.push_back(apex);
}


void pexSolver::EagerSolutionUpdate(HighLevelNodePtr node, std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost, HSolutionID& SOLUTIONS_waypoints)
{
    static ConflictResolver conflict_resolver;
    static std::tuple<int, int, std::vector<size_t>, size_t> cft;
    
    for(auto iter = node->all_jps.begin(); iter != node->all_jps.end(); ){
        cft = conflict_resolver.DetectConflict(*iter, node->sop_waypoints);
        if(std::get<2>(cft).empty()){
            std::cout << "find a new solution" << std::endl;
            
            CostVector  cost(iter->first.size(), 0);
            std::vector<std::vector<size_t>> waypoints;
            for(int i = 0; i < AGENT_NUM; i++){
                add_cost(cost, node->sop_cost.at(i)[iter->second.at(i)]);
                waypoints.push_back(node->sop_waypoints.at(i)[iter->second.at(i)]);
            }

            AddSolution(iter->first, cost, waypoints, SOLUTIONS_apex, SOLUTIONS_cost, SOLUTIONS_waypoints);

            iter = node->all_jps.erase(iter);
            continue;
        }else{
            iter ++;
        }
    }
}

void pexSolver::calculate_CAT(HighLevelNodePtr node, VertexCAT& vertex_cat, EdgeCAT& edge_cat, int agent_id)
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


void pexSolver::post_process(std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost){
    auto _SOLUTIONS_apex = SOLUTIONS_apex;
    auto _SOLUTIONS_cost = SOLUTIONS_cost;
    
    SOLUTIONS_cost = std::vector<CostVector>();
    SOLUTIONS_apex = std::vector<CostVector>();

    for(int i = 0; i < _SOLUTIONS_apex.size(); i++){
        bool dominated = false;
        for(int j = 0; j < SOLUTIONS_apex.size(); j++){
            if(is_dominated(_SOLUTIONS_apex.at(i), SOLUTIONS_cost.at(j), EPS)){
                SOLUTIONS_apex.at(j) = vector_min(SOLUTIONS_apex.at(j), _SOLUTIONS_apex.at(i));
                dominated = true;
                break;
            }
        }
        if(!dominated){
            SOLUTIONS_apex.push_back(_SOLUTIONS_apex.at(i));
            SOLUTIONS_cost.push_back(_SOLUTIONS_cost.at(i));
        }
    }
    
    
    std::vector<std::pair<CostVector, int>> apex_id_list(SOLUTIONS_apex.size());
    for(int i = 0; i < SOLUTIONS_apex.size(); i++){
        apex_id_list.at(i) = std::make_pair(SOLUTIONS_apex.at(i), i);
    }
    
    SOLUTIONS_cost = std::vector<CostVector>();
    SOLUTIONS_apex = std::vector<CostVector>();
    std::sort(apex_id_list.begin(), apex_id_list.end(), [](const std::pair<CostVector, int>& a, const std::pair<CostVector, int>& b){
        for(int i = 0; i < a.first.size(); i++){
            if(a.first.at(i) != b.first.at(i)){
                return a.first.at(i) < b.first.at(i);
            }
        }
        return true;
    });
    for(auto ele: apex_id_list){
        SOLUTIONS_apex.push_back(ele.first);
        SOLUTIONS_cost.push_back(_SOLUTIONS_cost.at(ele.second));
    }
}

OutputTuple pexSolver::run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_goal, HSolutionID& SOLUTIONS_waypoints, std::vector<CostVector>& SOLUTIONS_cost, LoggerPtr& logger)
{
// variables
    std::chrono::_V2::system_clock::time_point   precise_start_time, precise_end_time;
    precise_start_time = std::chrono::high_resolution_clock::now();
    std::chrono::_V2::system_clock::time_point t1, t2;
    double duration;

    std::vector<CostVector> SOLUTIONS_apex;
    bool is_success = true;
    
    double HLMergingTime = 0, LowLevelTime = 0, TotalTime;
    int conflict_splits = 0, SolutionNum = 0;

    ConflictResolver conflict_resolver;
    std::tuple<int, int, std::vector<size_t>, size_t> cft;

    start_time = time(NULL);
    HLQueue open_list;

    VertexCAT  vertex_cat;
    EdgeCAT    edge_cat;

    HighLevelNodePtr node, new_node;

// calculate heuristic
    std::vector<Heuristic> heuristics(AGENT_NUM);
    AdjacencyMatrix graph(GRAPH_SIZE, edges);
    AdjacencyMatrix inv_graph(GRAPH_SIZE, edges, true);
    for(int i = 0; i < AGENT_NUM; i++){
        ShortestPathHeuristic sp_heuristic(start_goal.at(i).second, GRAPH_SIZE, inv_graph, TURN_DIM, TURN_COST);
        heuristics.at(i) = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }

//  initialize open_list
    node = std::make_shared<HighLevelNode>(AGENT_NUM);

    for(size_t i = 0; i < AGENT_NUM; i ++){
        t1 = std::chrono::high_resolution_clock::now();
        if(MS == MergingStrategy::CONFLICT_BASED){
            single_run_map(GRAPH_SIZE, graph, heuristics.at(i), start_goal.at(i).first, start_goal.at(i).second, 
                LSOLVER, EPS, DEFAULT_MS, logger, TIME_LIMIT, node->sop_waypoints.at(i), node->sop_apex.at(i), node->sop_cost.at(i), 
                node->vertex_constraints.at(i), node->edge_constraints.at(i), vertex_cat, edge_cat, node->conflict_num, TURN_DIM, TURN_COST);
        }else{
            single_run_map(GRAPH_SIZE, graph, heuristics.at(i), start_goal.at(i).first, start_goal.at(i).second, 
                LSOLVER, EPS, MS, logger, TIME_LIMIT, node->sop_waypoints.at(i), node->sop_apex.at(i), node->sop_cost.at(i), 
                node->vertex_constraints.at(i), node->edge_constraints.at(i), vertex_cat, edge_cat, node->conflict_num, TURN_DIM, TURN_COST);
        }
        t2 = std::chrono::high_resolution_clock::now();
        duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
        LowLevelTime += duration;
        std::cout << "Agent ID: " << i << " Low Level Time = " << duration << "  size = " << node->sop_waypoints.at(i).size() << std::endl;
    }

    if(MS == MergingStrategy::CONFLICT_BASED){
        MergeJointPaths(node, DEFAULT_MS, EPS);
    }else{
        MergeJointPaths(node, MS, EPS);
    }

    if(difftime(time(NULL), start_time) > TIME_LIMIT){
        output << "FAIL" << std::endl 
            << "apex:" << std::endl 
            << std::endl 
            << "cost:" << std::endl 
            << std::endl << std::endl << std::endl;

        return std::make_tuple(HLMergingTime, LowLevelTime, difftime(time(NULL), start_time), 0, 0);
    }

    node->cur_ids = node->all_jps.front().second;
    node->cur_apex = node->all_jps.front().first;
    open_list.insert(node);

//  main loop    
    while(!open_list.empty())
    {
        if(difftime(time(NULL), start_time) > TIME_LIMIT){
            is_success = false;
            break;
        }
        node = open_list.pop();

        auto current_path = node->all_jps.front();

    // Eager Solution Update
        if(EAGER){
            EagerSolutionUpdate(node, SOLUTIONS_apex, SOLUTIONS_cost, SOLUTIONS_waypoints);
        }

        if(node->all_jps.empty()){
            continue;
        }

    //  DomPrune
        PruneApproxDom(node->all_jps, SOLUTIONS_apex, SOLUTIONS_cost, EPS);

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

    // add to SOLUTION
        if(std::get<2>(cft).empty()){  assert(EAGER == false);
            std::cout << "find a new solution" << std::endl;

            CostVector  cost(node->cur_apex.size(), 0);
            std::vector<std::vector<size_t>> waypoints;
            for(int i = 0; i < AGENT_NUM; i++){
                add_cost(cost, node->sop_cost.at(i)[node->cur_ids.at(i)]);
                waypoints.push_back(node->sop_waypoints.at(i)[node->cur_ids.at(i)]);
            }

            AddSolution(node->cur_apex, cost, waypoints, SOLUTIONS_apex, SOLUTIONS_cost, SOLUTIONS_waypoints);

            node->all_jps.pop_front();

            if(node->all_jps.empty()){
                continue;
            }

            node->cur_ids = node->all_jps.front().second;
            node->cur_apex = node->all_jps.front().first;

            open_list.insert(node);
            continue;
        }
        
    // print conflict info
        conflict_splits ++;
        if (conflict_splits % (500/AGENT_NUM) == 0) {
            std::cout << "[INFO] * Solver::Search, after " << conflict_splits << " conflict splits " 
            << "       time = " << difftime(time(NULL), start_time) << std::endl
            << map_name << std::endl;
        }


    // Split Node
        for(int i = 0; i < 2; i++){
            int agent_id = i == 0 ? std::get<0>(cft) : std::get<1>(cft);

            if(agent_id < 0){
                continue;
            }
            
            new_node = std::make_shared<HighLevelNode>(*node);

            new_node->sop_waypoints.at(agent_id).clear();
            new_node->sop_apex.at(agent_id).clear();
            new_node->sop_cost.at(agent_id).clear();
            new_node->conflict_num.clear();
            new_node->all_jps.clear();

            t1 = std::chrono::high_resolution_clock::now();
            if(MS == MergingStrategy::CONFLICT_BASED){
                MergeJointPaths(new_node, DEFAULT_MS, EPS, agent_id);
            }
            t2 = std::chrono::high_resolution_clock::now();
            duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
            HLMergingTime += duration;

            if(std::get<2>(cft).size() == 1){
                conflict_resolver.AddConstraint(new_node->vertex_constraints, agent_id, std::get<2>(cft).front(), std::get<3>(cft));
            }else{
                conflict_resolver.AddConstraint(new_node->edge_constraints, agent_id, std::get<2>(cft).at(i), std::get<2>(cft).at(1-i), std::get<3>(cft));
            }

            if(MS == MergingStrategy::CONFLICT_BASED){
                calculate_CAT(node, vertex_cat, edge_cat, agent_id);
            }

            t1 = std::chrono::high_resolution_clock::now();
            single_run_map(GRAPH_SIZE, graph, heuristics.at(agent_id), start_goal.at(agent_id).first, start_goal.at(agent_id).second, 
                LSOLVER, EPS, MS, logger, TIME_LIMIT, new_node->sop_waypoints.at(agent_id), 
                new_node->sop_apex.at(agent_id), new_node->sop_cost.at(agent_id), 
                new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id], vertex_cat, edge_cat, new_node->conflict_num, TURN_DIM, TURN_COST);
            t2 = std::chrono::high_resolution_clock::now();
            duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
            std::cout << "LowLevelTime = " << duration << std::endl;
            LowLevelTime += duration;

            if(new_node->sop_waypoints.at(agent_id).empty()){
                continue;
            }

        //  Calculate NonDomSet
            t1 = std::chrono::high_resolution_clock::now();

            if(MS == MergingStrategy::CONFLICT_BASED){
                MergeJointPaths(new_node, MergingStrategy::CONFLICT_BASED, EPS, agent_id);
            }else{
                MergeJointPaths(new_node, MS, EPS);
            }

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

    precise_end_time = std::chrono::high_resolution_clock::now();
    TotalTime = (double)(std::chrono::duration_cast<std::chrono::microseconds>(precise_end_time - precise_start_time).count())/1000000.0;
    
    post_process(SOLUTIONS_apex, SOLUTIONS_cost);

    if(is_success){
        output << "SUCCESS" << std::endl;
    }else{
        output << "FAIL" << std::endl;
    }
    output << "apex: " << std::endl;
    int i = 0;
    for(size_t num = 0; num < SOLUTIONS_apex.size(); num ++){
        if(i++ == 7){
            output << std::endl;
            i = 1;
        }
        if(DIM == 2){
            output << "{" << SOLUTIONS_apex.at(num).at(0) << ", " << SOLUTIONS_apex.at(num).at(1);
        }else{
            output << "{" << SOLUTIONS_apex.at(num).at(0) << ", " << SOLUTIONS_apex.at(num).at(1) << ", " << SOLUTIONS_apex.at(num).at(2);
        }
        output << "}, ";
    }
    output << std::endl;
    output << "cost: " << std::endl;
    int j = 0;
    for(size_t num = 0; num < SOLUTIONS_cost.size(); num ++){
        if(j++ == 7){
            output << std::endl;
            j = 1;
        }
        if(DIM == 2){
            output << "{" << SOLUTIONS_cost.at(num).at(0) << ", " << SOLUTIONS_cost.at(num).at(1);
        }else{
            output << "{" << SOLUTIONS_cost.at(num).at(0) << ", " << SOLUTIONS_cost.at(num).at(1) << ", " << SOLUTIONS_cost.at(num).at(2);
        }
        output << "}, ";
    }
    output << std::endl << std::endl;

    return std::make_tuple(HLMergingTime, LowLevelTime, TotalTime, conflict_splits, SOLUTIONS_cost.size());
}