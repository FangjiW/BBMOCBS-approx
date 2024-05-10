#include "HighLevel/epsSolver.h"
#include "Utils.h"

#include <cstdlib>
#include <list>
#include <algorithm>
#include <set>
#include <random>

extern std::unordered_map<size_t, std::vector<int>> id2coord;
extern std::string map_name;
extern std::ofstream output;


//  BOA* and NAMOA*dr version
void epsSolver::NonDomJointPath(HighLevelNodePtr node)
{
    for(auto e : node->sop_waypoints){
        std::cout << e.size() << " ";
    }
    auto _t1 = std::chrono::high_resolution_clock::now();
    JointPathPair temp;
    node->all_jps.clear();
    std::list<JointPathPair>& joint_path_list = node->all_jps;

    for(size_t id = 0; id < node->sop_waypoints.at(0).size(); id++){
        joint_path_list.push_back(std::make_pair(node->sop_cost.at(0)[id], std::vector<size_t>({id})));
    }
    
    for(int i = 1; i < node->sop_waypoints.size(); i++){
        if(difftime(time(NULL), start_time) > TIME_LIMIT){
            return;
        }
        std::list<JointPathPair> _joint_path_list(joint_path_list);
        joint_path_list.clear();
        for(auto joint_path = _joint_path_list.begin(); joint_path != _joint_path_list.end(); joint_path++){
            for(auto iter = node->sop_waypoints.at(i).begin(); iter != node->sop_waypoints.at(i).end(); iter++){
                temp = *joint_path;
                temp.second.push_back(iter->first);
                add_cost(temp.first, node->sop_cost.at(i)[iter->first]);
                // joint_path.second.push_back(indiv_path.first);
                joint_path_list.push_back(std::make_pair(std::move(temp.first), std::move(temp.second)));
                // std::push_heap(_joint_path_vector.begin(), _joint_path_vector.end(), less_than_);
            }
        }
        joint_path_list.sort([](const JointPathPair& a, JointPathPair& b){
            return less_than_pair<JointPathPair>(a, b);});
        NonDomVec(joint_path_list);
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - _t1);
    std::cout << " HLMergingTime = " << ((double)duration.count()) / 1000000.0 << std::endl;
    std::cout << node->all_jps.size();
    std::cout <<std::endl;
}



//  BOA* and NAMOA* version
void epsSolver::NonDomVec(std::list<JointPathPair>& joint_path_list)
{
    if (joint_path_list.front().first.size() == 2) {
        size_t g_min = UINT64_MAX;
        for(auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ) {
            if(difftime(time(NULL), start_time) > TIME_LIMIT){
                return;
            }
            if (iter->first.at(1) < g_min) {
                g_min = iter->first.at(1);
                iter ++;
            }else{
                iter = joint_path_list.erase(iter);
            }
        }
    }else if(joint_path_list.front().first.size() == 3){
    // dominance check: if be strictly dominated, then prune; else erase the behind strictly dominated ones in RB-tree
		std::set<CostVector, less_than_vector>  domination_set;
		for(auto iter = joint_path_list.begin(); iter != joint_path_list.end(); ){
            if(difftime(time(NULL), start_time) > TIME_LIMIT){
                return;
            }
            bool is_dominated = false;
			std::vector<size_t>  trun_vec({iter->first.at(1), iter->first.at(2)});
            if(domination_set.empty()){
                domination_set.insert(trun_vec);
                iter ++;
                continue;
            }
			auto it = domination_set.upper_bound(trun_vec);
            if(it != domination_set.begin() && std::prev(it)->at(1) <= trun_vec.at(1)){
                iter = joint_path_list.erase(iter);
            }else{
                //  delete the behind dominated ones (this operation can be removed)
                for(auto iter_ = it; iter_ != domination_set.end(); ){
                    if(trun_vec.at(1) <= (*iter_).at(1)){
                        iter_ = domination_set.erase(iter_);
                    }else{
                        break;
                    }
                }
                domination_set.insert(trun_vec);
                iter ++;
            }
		}
    }
    else {
        output << std::endl << "BOA*/NAMOA* ERROR: only g size = 2 or 3" << std::endl;
        exit(1);
    }
}



OutputTuple epsSolver::run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger)
{
    std::chrono::_V2::system_clock::time_point   precise_start_time, precise_end_time;
    precise_start_time = std::chrono::high_resolution_clock::now();
    std::chrono::_V2::system_clock::time_point t1, t2;
    double duration;

    std::vector<CostVector> hsolution_apex_costs;
    bool is_success = true;
    
    double HLMergingTime = 0, LowLevelTime = 0, TotalTime;
    int ConflictSolvingNum = 0, SolutionNum = 0;

    ConflictResolver conflict_resolver;

    start_time = time(NULL);
    HLQueue open_list;

// calculate heuristic
    std::vector<Heuristic> heuristics(AGENT_NUM);
    AdjacencyMatrix graph(GRAPH_SIZE, edges);   // can run outside and only once
    AdjacencyMatrix inv_graph(GRAPH_SIZE, edges, true);
    for(int i = 0; i < AGENT_NUM; i++){
        ShortestPathHeuristic sp_heuristic(start_end.at(i).second, GRAPH_SIZE, inv_graph, TURN_DIM, TURN_COST);
        heuristics.at(i) = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }

//  initialize open_list
    HighLevelNodePtr root_node = std::make_shared<HighLevelNode>(AGENT_NUM);
    VertexCAT  vertex_cat;
    EdgeCAT    edge_cat;

    for(size_t i = 0; i < AGENT_NUM; i ++){
        t1 = std::chrono::high_resolution_clock::now();
        single_run_map(GRAPH_SIZE, graph, heuristics.at(i), start_end.at(i).first, start_end.at(i).second, 
            LSOLVER, EPS, MergingStrategy::NONE, logger, TIME_LIMIT, root_node->sop_waypoints.at(i), root_node->sop_apex.at(i), root_node->sop_cost.at(i), 
            root_node->vertex_constraints.at(i), root_node->edge_constraints.at(i), vertex_cat, edge_cat, root_node->conflict_num, TURN_DIM, TURN_COST);
        
        t2 = std::chrono::high_resolution_clock::now();
        duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
        LowLevelTime += duration;
        std::cout << "Agent ID: " << i << " Low Level Time = " << duration << "  size = " << root_node->sop_waypoints.at(i).size() << std::endl;
    }

    t1 = std::chrono::high_resolution_clock::now();
    NonDomJointPath(root_node);
    t2 = std::chrono::high_resolution_clock::now();
    duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
    HLMergingTime += duration;

    if(difftime(time(NULL), start_time) > TIME_LIMIT){
        output << "FAIL" << std::endl 
            << "cost:" << std::endl 
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
                            if(is_dominated(*iter1, real_cost, 0)){
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
                        std::cout << "find a new solution" << std::endl;
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
                    if(is_dominated(*iter1, real_cost, 0)){
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
                std::cout << "find a new solution" << std::endl;

                node->all_jps.pop_front();

                if(node->all_jps.empty()){
                    continue;
                }

                node->cur_ids = node->all_jps.front().second;
                node->cur_apex = node->all_jps.front().first;


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
                t1 = std::chrono::high_resolution_clock::now();
                single_run_map(GRAPH_SIZE, graph, heuristics.at(agent_id), start_end.at(agent_id).first, start_end.at(agent_id).second, 
                    LSOLVER, EPS, MergingStrategy::NONE, logger, TIME_LIMIT, new_node->sop_waypoints.at(agent_id),  
                    new_node->sop_apex.at(agent_id), new_node->sop_cost.at(agent_id), 
                    new_node->vertex_constraints[agent_id], new_node->edge_constraints[agent_id], vertex_cat, edge_cat, new_node->conflict_num, TURN_DIM, TURN_COST); 

                t2 = std::chrono::high_resolution_clock::now();
                duration = (double)(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count())/1000000.0;
                LowLevelTime += duration;
                std::cout << "LowLevelTime = " << duration << std::endl;

                if(new_node->sop_waypoints.at(agent_id).empty()){
                    continue;
                }

            //  Calculate NonDomSet
                t1 = std::chrono::high_resolution_clock::now();

                NonDomJointPath(new_node);

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

                t1 = std::chrono::high_resolution_clock::now();
                single_run_map(GRAPH_SIZE, graph, heuristics.at(agent_id), start_end.at(agent_id).first, start_end.at(agent_id).second, 
                    LSOLVER, EPS, MergingStrategy::NONE, logger, TIME_LIMIT, new_node->sop_waypoints.at(agent_id), 
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

                NonDomJointPath(new_node);

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
    precise_end_time = std::chrono::high_resolution_clock::now();
    TotalTime = (double)(std::chrono::duration_cast<std::chrono::microseconds>(precise_end_time - precise_start_time).count())/1000000.0;


// // Merge Joint paths
//     if(ALGORITHM == Algorithm::BBMOCBS_PEX){
//         for(int i = 0; i < hsolution_apex_costs.size(); ){
//             bool is_merged = false;
//             for(int j = 0; j < i; j++){
//                 if(is_dominated(hsolution_apex_costs.at(i), hsolution_costs.at(j), EPS)){
//                     hsolution_apex_costs.at(j) = vector_min(hsolution_apex_costs.at(i), hsolution_apex_costs.at(j));
//                     hsolution_apex_costs.erase(hsolution_apex_costs.begin()+i);
//                     hsolution_costs.erase(hsolution_costs.begin()+i);
                    
//                     is_merged = true;
//                     break;
//                 } 
//             }
//             if(!is_merged){
//                 i ++;
//             }
//         }

//         MergeBySmallestEps(hsolution_apex_costs, hsolution_costs, 0, EPS);
//     }
//  post process
    // if(ALGORITHM == Algorithm::BBMOCBS_K){
    //     auto _hsolution_costs = hsolution_costs;
    //     auto _hsolution_apex_costs = hsolution_apex_costs;
    //     hsolution_costs.clear(); hsolution_apex_costs.clear();
    //     hsolution_costs.shrink_to_fit(); hsolution_apex_costs.shrink_to_fit();
    //     for(int i = 0; i < _hsolution_apex_costs.size(); i++){
    //         bool if_merged = false;
    //         for(int j = 0; j < hsolution_apex_costs.size(); j++){
    //             if(calculate_BF(_hsolution_apex_costs.at(i), hsolution_costs.at(j)) < EPS){
    //                 hsolution_apex_costs.at(j) = vector_min(hsolution_apex_costs.at(j), _hsolution_apex_costs.at(i));
    //                 if_merged = true;
    //                 break;
    //             }
    //             if(calculate_BF(hsolution_apex_costs.at(j), _hsolution_costs.at(i)) < EPS){
    //                 hsolution_apex_costs.at(j) = vector_min(hsolution_apex_costs.at(j), _hsolution_apex_costs.at(i));
    //                 hsolution_costs.at(j) = _hsolution_costs.at(i);
    //                 if_merged = true;
    //                 break;
    //             }
    //         }
    //         if(!if_merged){
    //             hsolution_costs.push_back(_hsolution_costs.at(i));
    //             hsolution_apex_costs.push_back(_hsolution_apex_costs.at(i));
    //         }
    //     }
    // }
    
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
    output << "cost: " << std::endl;
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
    // output << std::endl;
    // output << "real cost: " << std::endl;
    // int j = 0;
    // for(size_t num = 0; num < hsolution_costs.size(); num ++){
    //     if(j++ == 7){
    //         output << std::endl;
    //         j = 1;
    //     }
    //     if(DIM == 2){
    //         output << "{" << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1);
    //     }else{
    //         output << "{" << hsolution_costs.at(num).at(0) << ", " << hsolution_costs.at(num).at(1) << ", " << hsolution_costs.at(num).at(2);
    //     }
    //     output << "}, ";
    // }
    output << std::endl << std::endl;

    return std::make_tuple(HLMergingTime, LowLevelTime, TotalTime, ConflictSolvingNum, hsolution_costs.size());
}