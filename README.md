# find the Pareto-optimal frontier from node 20002 to node 164983 in the BAY map.
./bin/multiobj -m dataset/room-32-32-4/room-32-32-4.map -c dataset/room-32-32-4/room-32-32-4.cost --config dataset/room-32-32-4/room-32-32-4.config -n 2 -o output.txt

# d > 2 情况还差NonDomVec

# DominanceChecker.cpp  line 53为什么需要做这个dominance check