// SmartParkingBikes.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <functional>
#include <time.h>


struct SimpleGraph {
    std::unordered_map<char, std::vector<char> > edges;

    std::vector<char> neighbors(char id) {
        return edges[id];
    }
};


struct GridLocation {
    int x, y;
    bool hasBicycle = false;
};

namespace std {
    /* implement hash function so we can put GridLocation into an unordered_set */
    template <> struct hash<GridLocation> {
        typedef GridLocation argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const GridLocation& id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };
}


struct SquareGrid {
    static std::array<GridLocation, 4> DIRS;

    int width, height;
    std::unordered_set<GridLocation> walls;

    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < width
            && 0 <= id.y && id.y < height;
    }

    bool passable(GridLocation id) const {
        return walls.find(id) == walls.end();
    }

    std::vector<GridLocation> neighbors(GridLocation id) const {
        std::vector<GridLocation> results;

        for (GridLocation dir : DIRS) {
            GridLocation next{ id.x + dir.x, id.y + dir.y };
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0) {
            // aesthetic improvement on square grids
            std::reverse(results.begin(), results.end());
        }

        return results;
    }
};

std::array<GridLocation, 4> SquareGrid::DIRS =
{ GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1} };

// Helpers for GridLocation

bool operator == (GridLocation a, GridLocation b) {
    return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) {
    return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, const GridLocation& loc) {
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}

// This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.

GridLocation goal;

double cost_in_time = 0;
template<class Graph>
void draw_grid(const Graph& graph, int field_width,
    std::unordered_map<GridLocation, double>* distances = nullptr,
    std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
    std::vector<GridLocation>* path = nullptr) {
	GridLocation previous = goal;
    
    for (int y = 0; y != graph.height; ++y) {
        for (int x = 0; x != graph.width; ++x) {
            GridLocation id{ x, y };
            std::cout << std::left << std::setw(field_width);
            if (graph.walls.find(id) != graph.walls.end()) {
                std::cout << std::string(field_width, '#');
            }
            else if (point_to != nullptr && point_to->count(id)) {
                GridLocation next = (*point_to)[id];
                if (next.x == x + 1) { std::cout << "> "; }
				else if (next.x == x - 1) { std::cout << "< "; }
				else if (next.y == y + 1) { std::cout << "v "; }
                else if (next.y == y - 1) { std::cout << "^ "; }
                else { std::cout << "* "; }
            }
            else if (distances != nullptr && distances->count(id)) {
                std::cout << (*distances)[id];
            }
            else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
               if (previous.x == id.x - 1 || previous.x == id.x + 1) {
                    cost_in_time += 1.875;
                    previous.x = id.x;
                    std::cout << 'X';
                }
                if (previous.y == id.y - 1 || previous.y == id.y + 1) {
                    cost_in_time += 1.14;
                    previous.y = id.y;
                    std::cout << 'Y';
                }

            }
            else {
                 std::cout << '.';
            }
        }
        std::cout << '\n';
    }
}

void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2) {
    for (int x = x1; x < x2; ++x) {
        for (int y = y1; y < y2; ++y) {
            grid.walls.insert(GridLocation{ x, y });
        }
    }
}

struct GridWithWeights : SquareGrid {
    std::unordered_set<GridLocation> map;
    GridWithWeights(int w, int h) : SquareGrid(w, h) {}
    double cost(GridLocation from_node, GridLocation to_node) const {
        return map.find(to_node) != map.end() ? 1 :  1;
    }
};

GridWithWeights make_diagram() {
    GridWithWeights grid(18, 15);
    //cabinas
    add_rect(grid, 0, 0, 1, 2); 
    add_rect(grid, 17, 13, 18, 15);
    //rect grande
    add_rect(grid, 13, 7, 18, 9);
    //powerbanks
    add_rect(grid, 7, 7, 8, 9);
    add_rect(grid, 10, 7, 11, 9);

    return grid;
}

template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
        std::greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

template<typename Location>
std::vector<Location> reconstruct_path(
    Location start, Location goal,
    std::unordered_map<Location, Location> came_from
) {
    std::vector<Location> path;
    Location current = goal;
    while (current != start) {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

//costo estimado para llegar al goal

inline double heuristic(GridLocation a, GridLocation b) {
    return  (abs(a.x - b.x) + abs(a.y - b.y));
}

template<typename Location, typename Graph>
void a_star_search
(Graph graph,
    Location start,
    Location goal,
    std::unordered_map<Location, Location>& came_from,
    std::unordered_map<Location, double>& cost_so_far)
{
    PriorityQueue<Location, double> frontier;
    std::vector<Location> neighbors;
    frontier.put(start, heuristic(start, goal));

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();
        if (current == goal) {
            break;
        }

        //graph.neighbors(current)
        neighbors = graph.neighbors(current);
        for (Location next : neighbors) {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
}

//swap casillas por espacio en blanco

inline void swap(GridLocation& whiteSpace, GridLocation& casilla) {
    GridLocation temp;
    temp = whiteSpace;
    whiteSpace = casilla;
    casilla = temp;
}

inline void calculateRoute(GridWithWeights grid, GridLocation start, GridLocation goal, std::unordered_map<GridLocation, GridLocation>& came_from, std::unordered_map<GridLocation, double>& cost_so_far) {
    a_star_search(grid, start, goal, came_from, cost_so_far);
}

inline void moveWhiteSpace(GridLocation& whiteSpace, GridLocation& goal, std::vector<GridLocation> path) {
    int penultimo = path.size() - 2;
    //while( path[penultimo] != whiteSpace){

    for(int i = 1; i < path.size() ; i++ ) {
         swap(whiteSpace, path[i]);

         if (whiteSpace.x != path[i].x)
         {
             cost_in_time += 1.875;
         }
         if (whiteSpace.y != path[i].y)
         {
             cost_in_time += 1.14;
         }
         // Imprimir la posicion del whitespace
         std::cout << "x: " << whiteSpace.x << ", y: " << whiteSpace.y << std::endl;
	}
	//}
    std::cout << whiteSpace.x << " " << whiteSpace.y << '\n';
}


int main()
{
    int cabinNumber;
    GridWithWeights grid = make_diagram();
    std::cout << "Which Cabin, 1 or 2?"  << std::endl;
    std::cin >> cabinNumber;

    GridLocation whiteSpace1{ 1, 0 };
    GridLocation whiteSpace2{ 16, 13 };


    //Cabina uno borde esta en x = 9
    if (cabinNumber == 1) {
        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, GridLocation> came_from_whitespace;
        std::unordered_map<GridLocation, double> cost_so_far;
        std::unordered_map<GridLocation, double> cost_so_far_whitespace;

        //Medir el tiempo
        clock_t tStart = clock();
        GridLocation start{ 9,12 };
        goal = whiteSpace1;

        //Calcular ruta del whitespace a la bicicleta
        std::cout << "Camino espacio en blanco a bicicleta";
        calculateRoute(grid, whiteSpace1, start, came_from_whitespace, cost_so_far_whitespace);
        draw_grid(grid, 3, nullptr, &came_from_whitespace);
        std::cout << '\n';
        std::cout << '\n';
        //Buscar la salida para la bicicleta
        std::vector<GridLocation> pathWhiteSpace = reconstruct_path(whiteSpace1, start, came_from_whitespace);
        draw_grid(grid, 3, nullptr, nullptr, &pathWhiteSpace);

        std::cout << '\n';
        std::cout << '\n';
        std::cout << '\n';
        std::cout << '\n';

        moveWhiteSpace(whiteSpace1, start, pathWhiteSpace);

        /*
        //Calcular ruta de bici a cabina
        std::cout << "Camino bici a cabina";
        a_star_search(grid, start, goal, came_from, cost_so_far);
        draw_grid(grid, 3, nullptr, &came_from);
        std::cout << '\n';
        std::cout << '\n';
        draw_grid(grid, 3, &cost_so_far, nullptr);
        std::cout << '\n';
        std::cout << '\n';
        std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
        draw_grid(grid, 3, nullptr, nullptr, &path);
        printf("Time taken: %.4fs \n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
        */

    }
    //Cabina dos borde esta en x = 17
    else if(cabinNumber == 2){
        clock_t tStart = clock();
		GridLocation start{ 16, 14 };
		goal = { 10, 9 };
        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, GridLocation> came_from_whitespace;
        std::unordered_map<GridLocation, double> cost_so_far;
        std::unordered_map<GridLocation, double> cost_so_far_whitespace;
		draw_grid(grid, 3, nullptr, &came_from);
		//std::cout << '\n';
		//std::cout << '\n';
		//draw_grid(grid, 3, &cost_so_far, nullptr);
		std::cout << '\n';
		std::cout << '\n';
		std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
		draw_grid(grid, 3, nullptr, nullptr, &path);
        printf("Time taken: %.4fs \n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
    }

	std::cout <<  "The time it took: "<< cost_in_time << " sec." <<std::endl;

    system("PAUSE");
    return 0;
}


/*  IDEAS >:( c:< 
- Hacer la funcion para mover la bici recursiva hasta que se encuentre con una GridLocation que tenga el booleano isExit
*/