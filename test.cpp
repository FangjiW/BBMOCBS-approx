#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <list>
using namespace std;

bool compare(int a, int b)
{
   
    return (a > b);
}

void f(std::vector<int>& a){
    
}

int main()
{
    // std::list<int> a;
    // a.push_back(5); a.push_back(3); a.push_back(4); a.push_back(1);
    // std::push_heap(a.begin(), a.end(), compare);
    // std::cout << a.front();
    std::vector<std::pair<int, double>> a;
    a.push_back(std::make_pair(1,2.0));
    
}