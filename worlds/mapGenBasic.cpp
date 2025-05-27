#include <vector>
#include <iostream>
#include <cstdlib> 
#include <ctime>
#include "sdfGen.hpp"

   //use g++ mapGenBasic.cpp sdfGen.cpp -o mapGenBasic.exe
  // Remember the obstacles should be able to turn into goal
 //  As obstacles actually just represnet 'houses'
char obstacle = '#';
char emptySpace = ' ';
char goal = 'G';

int mapDimensionX = 100; // change this to change the size of the map crazy
int mapDimensionY = 100;




// getting techy
std::vector<std::vector<char>> map(mapDimensionX, std::vector<char>(mapDimensionY, ' '));

int main(){
  int mapDimensionX;
  int mapDimensionY;
  char preview = 'n';

  std::cout << "enter value for X (integer): ";
  std::cin >> mapDimensionX;
  std::cout << std::endl << "enter the value for y (Also int): ";
  std::cin >> mapDimensionY;
  std::cout << "preview grid y/n: ";
  std::cin >> preview;

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
      map[i][j] = ' ';
      }
    }
  }    
  if(preview = 'y'){
    for (int i = 0; i < mapDimensionX; i++){
      for(int j = 0; j < mapDimensionY; j ++){
        std::cout << map[i][j];
      }
      std::cout << std::endl;
    }    
  }

  std::string sdfContent = generateSDF(map);
  std::cout << "Please name the file, no spaces no . just the name:" << std::endl;
  std::string worldName;
  std::cin >> worldName;
  std::ofstream outFile(worldName +".world");
  outFile << "<sdf version='1.6'>\n";
  outFile << "  <world name='default'>\n";
  outFile << sdfContent;
  outFile << "  </world>\n</sdf>\n";
  outFile.close();
  std::cin.get();
}