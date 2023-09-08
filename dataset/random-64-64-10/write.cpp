#include <fstream>
#include <chrono> 
#include <random>
using namespace std;

int main(){
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(1, 5);
		for(int i = 0; i < 100; i++){
				ofstream output("cost/2/" + to_string(i+1) + ".cost");
				for(int j = 0; j < 64; j++){
						for(int k = 0; k < 64; k++){
							output << distribution(generator) << " ";
						}
						output << endl;
				}
				output << endl;
				for(int j = 0; j < 64; j++){
						for(int k = 0; k < 64; k++){
							output << distribution(generator) << " ";
						}
						output << endl;
				}
			
		} 
		for(int i = 0; i < 100; i++){
				ofstream output("cost/3/" + to_string(i+1) + ".cost");
				for(int j = 0; j < 64; j++){
						for(int k = 0; k < 64; k++){
							output << distribution(generator) << " ";
						}
						output << endl;
				}
				output << endl;
				for(int j = 0; j < 64; j++){
						for(int k = 0; k < 64; k++){
							output << distribution(generator) << " ";
						}
						output << endl;
				}
				output << endl;
				for(int j = 0; j < 64; j++){
						for(int k = 0; k < 64; k++){
							output << distribution(generator) << " ";
						}
						output << endl;
				}	
		} 
		
}