#include <vector>
#include <iostream>
#include <cstdlib> 
#include <ctime>

  // Remember the obstacles should be able to turn into goal
 //  As obstacles actually just represnet 'houses'
char obstacle = '#';
char emptySpace = '.';
char goal = 'G';

int mapDimensionX = 100; // change this to change the size of the map crazy
int mapDimensionY = 100;




// getting techy
std::vector<std::vector<char>> map(mapDimensionX, std::vector<char>(mapDimensionY, '.'));

int main(){
  srand(time(0));
  for (int i = 0; i < mapDimensionX; i++){
    for(int j = 0; j < mapDimensionY; j ++){
      if(i % 2 == 1 && j % 2 == 0){   
        int r = rand() % 100;
        if (r % 10 == 0){
          map[i][j] = 'G';
        }
        else{
          map[i][j] = '#';
        }
      }
      else{
      map[i][j] = '.';
      }
    }
  }    
  for (int i = 0; i < mapDimensionX; i++){
    for(int j = 0; j < mapDimensionY; j ++){
      std::cout << map[i][j];
    }
    std::cout << std::endl;
  }    

  std::cin.get();
}