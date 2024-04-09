#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

using namespace std;

int euclideanDistance(const pair<int, int> &pointA, const pair<int, int> &pointB);

struct robotNode {
  pair<int, int> position;
  int cost;
  int heuristic;
  robotNode *parent;

  // Constructor 
  robotNode(const pair<int, int>&start, const pair<int, int>&goal){
    position = start;
    cost = 0;
    heuristic = euclideanDistance(start, goal);
    parent = nullptr;
  }

};

int euclideanDistance(const pair<int, int> &pointA,
                         const pair<int, int> &pointB) {
  return sqrt(pow(pointA.first - pointB.first, 2) +
              pow(pointA.second - pointB.second, 2));
}

// Function check to determine if we ecounter an obstacle
bool isObstacleCoordinate(const pair<int, int> &point,
                          const vector<vector<int>> &grid) {
  return grid[point.first][point.second] == 1;
}

// Get neighbors of robot's current position
vector<pair<int, int>> getNeighbors(const pair<int, int> &point,
                                    const vector<vector<int>> &grid) {

  vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Horizontal and vertical movements
  vector<pair<int, int>> neighbors;

  for (const auto &direction : directions) {
    int newX = point.first + direction.first;
    int newY = point.second + direction.second;
    if (newX >= 0 && newX < grid.size() && newY >= 0 && newY < grid[0].size()) {
      neighbors.push_back({newX, newY});
    }
  }

  return neighbors;
}

// A* search for finding the shortest path to the goal position
vector<pair<int, int>> aStarSearch(const vector<vector<int>> &grid,const pair<int, int> &start,const pair<int, int> &goal) {
  
  // Custom comparator for priority queue 
  auto comparefunc = [](const robotNode *left, const robotNode *right) {
    return left->cost + left->heuristic > right->cost + right->heuristic;
  };
  // Min heap storing pointers to the robot node
  priority_queue<robotNode *, vector<robotNode *>, decltype(comparefunc)>
      unvisited(comparefunc);
  set<pair<int, int>> visited; /// Stores the coordinates of visited positions in the grid 
  robotNode *startNode = new robotNode(start, goal); //Initializes start node
  
  unvisited.push(startNode); //Adds the start node to the priority queue

  while (!unvisited.empty()) {
    robotNode *currentNode = unvisited.top();
    unvisited.pop();
    
    // Goal check to determine if we are at the second point 
    if (currentNode->position == goal) {
      vector<pair<int, int>> path;
      while (currentNode != nullptr) {
        path.push_back(currentNode->position);
        currentNode = currentNode->parent;
      }
      reverse(path.begin(), path.end()); //Reconstucts the path to the goal
    
      return path;
    }

    visited.insert(currentNode->position);

    for (const auto &neighborPos : getNeighbors(currentNode->position, grid)) {
      if (isObstacleCoordinate(neighborPos, grid)) {
        continue; // Skip this position if it is located inside of the obstacle coordinates
      }

      int newCost = currentNode->cost + 1; // Adds a cost of 1 to each movement
      int newHeuristic = euclideanDistance(neighborPos, goal); // Returns the distance to the goal from the current neighbor position 
      robotNode *neighborNode = new robotNode(neighborPos, goal); // Creates new node object for neighboring position  
      
      // Intiliazes new node position      
      neighborNode->position = neighborPos; 
      neighborNode->cost = newCost;
      neighborNode->heuristic = newHeuristic;
      neighborNode->parent = currentNode;

      // Iterates through visited position set and updates neighbor node when a better position is found  
      if (visited.find(neighborPos) != visited.end()) {
        if (newCost < neighborNode->cost) {
          neighborNode->cost = newCost;
          neighborNode->parent = currentNode;
          unvisited.push(neighborNode);
        }
        continue; // Skips this position if path is not shorter
      }
      unvisited.push(neighborNode); // Adds new position to the unvisited queue for further exploration of its neighbor coordinates
    }
  }

  return {};
}

// File reading function 
void readFile(const string &filename, vector<vector<int>> &grid, int &rows,
              int &columns) {
  ifstream inputFile(filename);

  if (!inputFile.is_open()) {
    cerr << "Error opening input file." << endl;
    return;
  }

  inputFile >> rows >> columns;

  grid.resize(rows, vector<int>(columns));

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < columns; ++j) {
      inputFile >> grid[i][j];
    }
  }
  inputFile.close();
}

int main() {

  int rows, columns;
  vector<vector<int>> grid;
  pair<int, int> start = {0, 0};
  pair<int, int> goal = {1, 5};

  readFile("grid-input.txt", grid, rows, columns);

  // Returns the shortest path from start coodinate to end coordinate 
  vector<pair<int, int>> path = aStarSearch(grid, start, goal);

  // Prints the matrix containing a hexagon obstacle
  cout << "Grid:" << endl;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < columns; ++j) {
      cout << grid[i][j] << " ";
    }
    cout << endl;
  }


  // Prints the shortest path coordinates 
  cout << "Shortest path: ";
  for (const auto &point : path) {
    cout << "(" << point.first << ", " << point.second << ") ";
  }
  cout << endl;

  return 0;
}