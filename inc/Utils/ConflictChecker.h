#include <iostream>

#include "Utils/Definitions.h"

class ConflictChecker
{
public:
    ConflictChecker(){};
    std::vector<int> is_conflict(HighLevelNodePtr node); // 0 or (i, j, node_id, t)
};