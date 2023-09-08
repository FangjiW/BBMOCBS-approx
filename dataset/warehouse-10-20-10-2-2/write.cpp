#include <fstream>
#include <chrono> 
#include <random>
#include <iostream>
using namespace std;

int main(){
	ifstream input("warehouse-10-20-10-2-2.map");
	std::string temp;
    int height, width;
    std::getline(input, temp);
    input >> temp >> height >> temp >> width;
    std::getline(input, temp);
    std::getline(input, temp);

    // Create a Map object to store the map data
    int** map = new int*[height];
	for(int i = 0; i < height; i++){
		map[i] = new int[width];
	}

	// Read the map data from the file and store it in the Map object
    for (int x = 0; x < height; ++x) {
        std::string line;
        std::getline(input, line);

        for (int y = 0; y < width; ++y) {
            if (line[y] == '@' || line[y] == 'T') {
                map[x][y] = -1;
            } else if (line[y] == '.') {
                map[x][y] = 0;
            }
        }
    }
	input.close();
	
	// ofstream output("cost/unique3.cost");
	// for(int j = 0; j < height; j++){
	// 	for(int k = 0; k < width; k++){
	// 		output << 3 << " ";
	// 	}
	// 	output << endl;
	// }

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(1, 5);
	for(int i = 0; i < 200; i++){
		ofstream output("cost/random-" + to_string(i+1) + ".cost");
		for(int j = 0; j < height; j++){
				for(int k = 0; k < width; k++){
					output << distribution(generator) << " ";
				}
				output << endl;
		}			
	} 

	ofstream output("cost/distance.cost");
	for(int j = 0; j < height; j++){
		for(int k = 0; k < width; k ++){
			if(map[j][k] == -1){
				output << 0 << " ";
				continue;
			}
			bool have_write = false;
			for(int l = 0; l < 5; l++){
				for(int t = 0; t <= l; t ++){
					if(j-t >= 0 && k-(l-t) >= 0 && j-k < height && k-(l-t) < width && map[j-t][k-(l-t)] == -1){
						output << 6-l << " ";
						have_write = true;
						break;
					}
					if(j-t >= 0 && k+(l-t) >= 0 && j-k < height && k+(l-t) < width && map[j-t][k+(l-t)] == -1){
						output << 6-l << " ";
						have_write = true;
						break;
					}
					if(j+t >= 0 && k+(l-t) >= 0 && j+k < height && k+(l-t) < width && map[j+t][k+(l-t)] == -1){
						output << 6-l << " ";
						have_write = true;
						break;
					}
					if(j+t >= 0 && k-(l-t) >= 0 && j+k < height && k-(l-t) < width && map[j+t][k-(l-t)] == -1){
						output << 6-l << " ";
						have_write = true;
						break;
					}
				}
				if(have_write){
					break;
				}
			}
			if(!have_write){
				output << 1 << " ";
			}
		}
		output << std::endl;
	}
}
