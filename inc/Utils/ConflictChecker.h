#include <iostream>

#include "Utils/Definitions.h"

class ConflictChecker
{
public:
    ConflictChecker(){};
    std::tuple<int, int, std::vector<size_t>, size_t> is_conflict(HighLevelNodePtr node); // 0 or (i, j, node_id, t)

    bool is_conflict(JointPathPair &joint_path, std::vector<PathSet>& indiv_paths_list, int agent_num);
};