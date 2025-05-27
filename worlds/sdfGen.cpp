#include "sdfGen.hpp"
#include <sstream>

std::string generateSDF(const std::vector<std::vector<char>>& grid, char wall,
    char goal, char openSpace){
    int size = grid.size();
    char cell;
    std::ostringstream SDF;

   // so we are going to loop through each item in out grid
  //  for each item determine what it is
 //   and correctly return it to a sdf file that gazebo 
//    can read
    for(int i = 0; i < size; i++){
        for(int j = 0; j < size; j++){
            cell = grid[i][j];
            if (cell == wall){
                SDF << "    <model name='block_|" << i << "-" << j << "|'>\n";
                SDF << "      <static>true</static>\n";
                SDF << "      <link name='link'>\n";
                SDF << "        <pose>" << i << " " << j << " 0.5 0 0 0</pose>\n";
                SDF << "        <collision name='collision'>\n";
                SDF << "          <geometry><box><size>1 1 1</size></box></geometry>\n";
                SDF << "        </collision>\n";
                SDF << "        <visual name='visual'>\n";
                SDF << "          <geometry><box><size>1 1 1</size></box></geometry>\n";
                SDF << "          <material><ambient>0.5 0.5 0.5 1</ambient></material>\n";
                SDF << "        </visual>\n";
                SDF << "      </link>\n";
                SDF << "    </model>\n";
            }
            else if (cell == goal){
                SDF << "    <model name='goal_|" << i << "-" << j << "|'>\n";
                SDF << "      <static>true</static>\n";
                SDF << "      <link name='link'>\n";
                SDF << "        <pose>" << i << " " << j << " 0.5 0 0 0</pose>\n";
                SDF << "        <collision name='collision'>\n";
                SDF << "          <geometry><box><size>1 1 1</size></box></geometry>\n";
                SDF << "        </collision>\n";
                SDF << "        <visual name='visual'>\n";
                SDF << "          <geometry><box><size>1 1 1</size></box></geometry>\n";
                SDF << "          <material><ambient>0 1 0 1</ambient></material>\n";
                SDF << "        </visual>\n";
                SDF << "      </link>\n";
                SDF << "    </model>\n";
            }
            else if (cell != openSpace) {
                // can add stuff here if needed if we wanna incorporate more types
               //  of obstacles
            }
            // if it is an open space we can literally just skip it 
        }
    }
    return SDF.str();
}