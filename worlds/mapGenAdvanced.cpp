#include <vector>
#include <iostream>
#include <cstdlib> 
#include <ctime>

 // This did not turn out like i expected but i guess it could be used to train
//  perhaps using the 'walls' as the goals ??? maybe 

char obstacle = '#';
char emptySpace = '.';
char goal = 'G';

std::vector<std::vector<char>> mapGenFunction(int mapDimensionX, int mapDimensionY, int fullNess) {
    srand(time(0));

    // Create the map filled with obstacles
    std::vector<std::vector<char>> map(mapDimensionX, std::vector<char>(mapDimensionY, obstacle));
    int totalToCarve = static_cast<float>(fullNess) / 100 * (mapDimensionX * mapDimensionY);

    struct CarvedCell {
        int x;
        int y;
    };

    std::vector<CarvedCell> carvedCells;

    // Choose a random starting point and mark it as carved
    int randomX = rand() % mapDimensionX;
    int randomY = rand() % mapDimensionY;
    map[randomX][randomY] = emptySpace;
    carvedCells.push_back({randomX, randomY});

    int carvedCount = 1;

    while (carvedCount < totalToCarve) {
        int direction = rand() % 4; // 0 = up, 1 = right, 2 = down, 3 = left

        std::vector<CarvedCell> candidates;

        for (const auto& cell : carvedCells) {
            int dx = 0, dy = 0;
            if (direction == 0) dx = -1;        // up
            else if (direction == 1) dy = 1;    // right
            else if (direction == 2) dx = 1;    // down
            else if (direction == 3) dy = -1;   // left

            int nx = cell.x + dx;
            int ny = cell.y + dy;

            // Check if the neighbor is in bounds and uncarved
            if (nx >= 0 && ny >= 0 && nx < mapDimensionX && ny < mapDimensionY && map[nx][ny] == obstacle) {
                candidates.push_back(cell);
            }
        }

        if (!candidates.empty()) {
            // Pick a random candidate to carve from
            CarvedCell chosen = candidates[rand() % candidates.size()];

            int dx = 0, dy = 0;
            if (direction == 0) dx = -1;
            else if (direction == 1) dy = 1;
            else if (direction == 2) dx = 1;
            else if (direction == 3) dy = -1;

            int nx = chosen.x + dx;
            int ny = chosen.y + dy;

            map[nx][ny] = emptySpace;
            carvedCells.push_back({nx, ny});
            carvedCount++;
        }
        // Else: no valid cells in that direction, retry next loop
    }

    return map;
}


int main(){
  std::cout << "Welcome to my thing how cool" << std::endl;
  int sizeX = 30;
  int sizeY = 30;
  std::vector<std::vector<char>> mapToPrint = mapGenFunction(sizeX, sizeY, 30);

  for (int i = 0; i < sizeX; i++){
    for (int j = 0; j < sizeY; j++){
      std::cout << mapToPrint[i][j] << ' ';
    }
    std::cout << std::endl;
  }
}