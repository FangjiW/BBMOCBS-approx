#include <fstream>
#include <chrono> 
#include <random>
#include <iostream>
using namespace std;

int main(){
// unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
// 		std::default_random_engine generator(seed);
//     std::uniform_int_distribution<int> distribution(1, 5);
// 				ofstream output("cost/2/" + to_string(i+101) + ".cost");
// 				for(int j = 0; j < 48; j++){
// 						for(int k = 0; k < 48; k++){
// 							output << 1 << " ";
// 						}
// 						output << endl;
// 				}
// 				output << endl;
// 				for(int j = 0; j < 48; j++){
// 						for(int k = 0; k < 48; k++){
// 							output << distribution(generator) << " ";
// 						}
// 						output << endl;
// 				}
			
// 		} 
		// for(int i = 0; i < 100; i++){
		// 		ofstream output("cost/3/" + to_string(i+301) + ".cost");
		// 		for(int j = 0; j < 48; j++){
		// 				for(int k = 0; k < 48; k++){
		// 					output << distribution(generator) << " ";
		// 				}
		// 				output << endl;
		// 		}
		// 		output << endl;
		// 		for(int j = 0; j < 48; j++){
		// 				for(int k = 0; k < 48; k++){
		// 					output << distribution(generator) << " ";
		// 				}
		// 				output << endl;
		// 		}
		// 		output << endl;
		// 		for(int j = 0; j < 48; j++){
		// 				for(int k = 0; k < 48; k++){
		// 					output << distribution(generator) << " ";
		// 				}
		// 				output << endl;
		// 		}	
		// } for(int i = 0; i < 100; i++){

	ifstream input("random-32-32-20.map");
	std::string temp;
    int height, width;
    std::getline(input, temp);
    input >> temp >> height >> temp >> width;
    std::getline(input, temp);
    std::getline(input, temp);

    // Create a Map object to store the map data
    int map[32][32];
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);
std::uniform_int_distribution<int> distribution(1, 5);

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

	for(int i = 0; i < 100; i++){
		ofstream output("cost/3/" + to_string(i+101) + ".cost");
		for(int j = 0; j < 32; j++){
				for(int k = 0; k < 32; k++){
					output << distribution(generator) << " ";
				}
				output << endl;
		}
		output << endl;

		for(int j = 0; j < 32; j++){
			for(int k = 0; k < 32; k ++){
				if(map[j][k] == -1){
					output << 0 << " ";
					continue;
				}
				bool have_write = false;
				for(int l = 0; l < 5; l++){
					for(int t = 0; t <= l; t ++){
						if(j-t >= 0 && k-(l-t) >= 0 && j-k < 32 && k-(l-t) < 32 && map[j-t][k-(l-t)] == -1){
							output << 6-l << " ";
							have_write = true;
							break;
						}
						if(j-t >= 0 && k+(l-t) >= 0 && j-k < 32 && k+(l-t) < 32 && map[j-t][k+(l-t)] == -1){
							output << 6-l << " ";
							have_write = true;
							break;
						}
						if(j+t >= 0 && k+(l-t) >= 0 && j+k < 32 && k+(l-t) < 32 && map[j+t][k+(l-t)] == -1){
							output << 6-l << " ";
							have_write = true;
							break;
						}
						if(j+t >= 0 && k-(l-t) >= 0 && j+k < 32 && k-(l-t) < 32 && map[j+t][k-(l-t)] == -1){
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
		output << endl;

		for(int j = 0; j < 32; j++){
				for(int k = 0; k < 32; k++){
					output << 3 << " ";
				}
				output << endl;
		}
	}
}
