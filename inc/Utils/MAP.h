#include <iostream>

class Map 
{
public:
    size_t graph_size = 0;
    int height, width;

    Map(){};

    Map(int width, int height) : height(height), width(width), graph_size(0){
        map_data = new size_t*[width];
        ids = new size_t*[width];
        for(int i = 0; i < width; i ++){
            map_data[i] = new size_t[height];
            ids[i] = new size_t[height];
        }
    }

    void setVal(int x, int y, size_t val){
        map_data[x][y] = val;
    }

    size_t getVal(int x, int y){
        return map_data[x][y];
    }

    void setID(int x, int y, size_t id){
        ids[x][y] = id;
    }

    size_t getID(int x, int y){
        return ids[x][y];
    }

    void ddelete(){
        for (int i = 0; i < width; i++) {
            delete[] map_data[i];
            delete[] ids[i];
        }
        delete[] map_data;
        delete[] ids;
    }

private:
    size_t ** map_data;
    size_t ** ids;
};