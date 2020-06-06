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
//derecha, abajo, izquierda, arriba
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
                for (int i = 0; i < path->size() - 1; i++)
                {
                    if (id.x == path->at(i).x && id.y == path->at(i).y)
                    {
                        if (i != 0)
                        {
                            previous.x = path->at(i - 1).x;
                            previous.y = path->at(i - 1).y;
                            break;
                        }
                    }
                }

                if (previous.x == id.x - 1 || previous.x == id.x + 1) {
                    previous.x = id.x;
                    std::cout << 'X';
                }
                if (previous.y == id.y - 1 || previous.y == id.y + 1) {
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
        return map.find(to_node) != map.end() ? 1 : 1;
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
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();

        if (current == goal) {
            break;
        }

        for (Location next : graph.neighbors(current)) {
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

//arreglar swap
inline void swap(GridLocation& whiteSpace, GridLocation& casilla) {
    GridLocation temp;
    temp = whiteSpace;
    /*if (casilla.hasBicycle) {
        temp.hasBicycle = true;
        casilla.hasBicycle = false;
    }*/
    whiteSpace = casilla;
    casilla = temp;
}

std::vector<GridLocation> calculateRoute(GridWithWeights grid, GridLocation start, int cabin) {

    GridLocation goal1;
    GridLocation goal2;
    GridLocation goal3;

    if (cabin == 1)
    {
        goal1.x = 1;
        goal1.y = 0;
        goal2.x = 1;
        goal2.y = 0;
        goal3.x = 1;
        goal3.y = 0;
    }
    else {
        goal1.x = 17;
        goal1.y = 12;
        goal2.x = 16;
        goal2.y = 13;
        goal3.x = 16;
        goal3.y = 14;
    }

    //Calcular ruta de bici a cabina (1,0)
    std::unordered_map<GridLocation, GridLocation> came_from1;
    std::unordered_map<GridLocation, double> cost_so_far1;
    a_star_search(grid, start, goal1, came_from1, cost_so_far1);
    std::vector<GridLocation> path1 = reconstruct_path(start, goal1, came_from1);

    //Calcular ruta de bici a cabina (1,0)
    std::unordered_map<GridLocation, GridLocation> came_from2;
    std::unordered_map<GridLocation, double> cost_so_far2;
    a_star_search(grid, start, goal2, came_from2, cost_so_far2);
    std::vector<GridLocation> path2 = reconstruct_path(start, goal2, came_from2);

    //Calcular ruta de bici a cabina (1,0)
    std::unordered_map<GridLocation, GridLocation> came_from3;
    std::unordered_map<GridLocation, double> cost_so_far3;
    a_star_search(grid, start, goal3, came_from3, cost_so_far3);
    std::vector<GridLocation> path3 = reconstruct_path(start, goal3, came_from3);

    //Comparar los 3 caminos y elegir el mas corto
    if (path1.size() <= path2.size() && path1.size() <= path3.size())
    {
        // draw_grid(grid, 3, nullptr, &came_from1);
        return path1;
    }

    if (path2.size() <= path1.size() && path2.size() <= path3.size())
    {
        //draw_grid(grid, 3, nullptr, &came_from2);
        return path2;
    }

    if (path3.size() <= path1.size() && path3.size() <= path2.size())
    {
        //draw_grid(grid, 3, nullptr, &came_from3);
        return path3;
    }
}

void moveWhiteSpaceToBicycle(GridLocation& whiteSpace, GridLocation& goal, std::vector<GridLocation> path) {
    //while( path[penultimo] != whiteSpace){
    //std::cout << cost_in_time << std::endl;
    for (int i = 1; i < path.size() - 1; i++) {
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
        //std::cout << "Whitespace position x: " << whiteSpace.x << ", y: " << whiteSpace.y << ", time: " << cost_in_time << '\n';
    }
    //}
    //std::cout << "After movewhitespaceToBicycle: " << whiteSpace.x << " " << whiteSpace.y << '\n';
}

void moveWhiteSpaceToFront(GridLocation& whiteSpace, GridLocation& goal, std::vector<GridLocation> path) {
    //while( path[penultimo] != whiteSpace){
    //std::cout << cost_in_time << std::endl;
    for (int i = 1; i < path.size(); i++) {
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
        //std::cout << "Whitespace position x: " << whiteSpace.x << ", y: " << whiteSpace.y << ", time: " << cost_in_time << '\n';
    }
    //}
    //std::cout << "After movewhitespaceToFront: " << whiteSpace.x << " " << whiteSpace.y << '\n';
}

void BicycleSpaceMovement(GridLocation& whiteSpace, GridLocation& bicycle, GridWithWeights& grid, std::vector<GridLocation> path) {


    for (int i = 1; i < path.size(); i++) {

        GridWithWeights tempGrid = grid;

        if ((bicycle.x == 1 && bicycle.y == 0) || (bicycle.x == 1 && bicycle.y == 1) || (bicycle.x == 0 && bicycle.y == 2))
            break;

        if ((bicycle.x == 17 && bicycle.y == 12) || (bicycle.x == 16 && bicycle.y == 13) || (bicycle.x == 16 && bicycle.y == 14))
            break;

        std::unordered_map < GridLocation, GridLocation> came_from;
        std::unordered_map <GridLocation, double> cost_so_far;
        GridLocation bicycle_front{ path[i].x, path[i].y };
        /*std::cout <<"Bicycle_front position: " << bicycle_front.x << " " << bicycle_front.y << '\n';
        std::cout << "Bicycle position: " << bicycle.x << " " << bicycle.y << '\n';*/

        if (whiteSpace == path[i]) {
            swap(bicycle, whiteSpace);
            continue;
        }
        add_rect(tempGrid, bicycle.x, bicycle.y, bicycle.x + 1, bicycle.y + 1);
        a_star_search(tempGrid, whiteSpace, bicycle_front, came_from, cost_so_far);

        //draw_grid(grid, 3, nullptr, &came_from);

        std::vector<GridLocation> pathWhiteSpace = reconstruct_path(whiteSpace, bicycle_front, came_from);
        //draw_grid(grid, 3, nullptr, nullptr, &pathWhiteSpace);
        moveWhiteSpaceToFront(whiteSpace, bicycle_front, pathWhiteSpace);
        swap(bicycle, whiteSpace);
    }
}
// SI SE PUEDE CABRONES

int main()
{
    GridWithWeights grid = make_diagram();
    GridLocation start;
    clock_t time_req;

    char again;
    bool repeat = true;
    while (repeat)
    {
        bool pass = false;
        while (!pass) {
            std::cout << "Input start x: ";
            std::cin >> start.x;
            std::cout << "Input start y: ";
            std::cin >> start.y;
            std::cout << '\n';
            if (start.x >= 0 && start.x <= 17) {
                if (start.y >= 0 && start.y <= 14) {

                    if (grid.walls.find(start) == grid.walls.end())
                    {
                        pass = true;
                    }
                    else {
                        std::cout << "Invalid coordinates. Try AGAIN." << '\n';
                    }
                }
                else
                    std::cout << "Invalid coordinates. Try AGAIN." << '\n';
            }
            else {
                std::cout << "Invalid coordinates. Try AGAIN." << '\n';
            }
        }

        //Cabina uno borde esta en x = 9
        if (start.x <= 8) {
            time_req = clock();
            GridLocation whiteSpace{ 1, 0 };
            int cabin = 1;
            int i;

            //Medir el tiempo
            clock_t tStart = clock();

            ////Calcular ruta del whitespace a la bicicleta
            //std::cout << "Camino espacio en blanco a bicicleta \n";

            ////draw_grid(grid, 3, nullptr, &came_from_whitespace);
            //std::cout << '\n';
            //std::cout << '\n';

            //Buscar la ruta del espacio en blanco a la bicicleta
            std::unordered_map<GridLocation, GridLocation> came_from_whitespace;
            std::unordered_map<GridLocation, double> cost_so_far_whitespace;
            a_star_search(grid, whiteSpace, start, came_from_whitespace, cost_so_far_whitespace);
            std::vector<GridLocation> pathWhiteSpace = reconstruct_path(whiteSpace, start, came_from_whitespace);
            //draw_grid(grid, 3, nullptr, nullptr, &pathWhiteSpace);

            //std::cin >> i;

            //std::cout << '\n';
            //std::cout << '\n';

            moveWhiteSpaceToBicycle(whiteSpace, start, pathWhiteSpace);

            //std::cin >> i;

            std::vector<GridLocation> path = calculateRoute(grid, start, cabin);


            /*std::cout << '\n';
            std::cout << '\n';*/
            BicycleSpaceMovement(whiteSpace, start, grid, path);
            //draw_grid(grid, 3, nullptr, &came_from);


            time_req = clock() - time_req;

            std::cout << "Processor time taken : "
                << (float)time_req / CLOCKS_PER_SEC << " seconds" << std::endl;
        }

        //Cabina dos borde esta en x = 17
        else if (start.x >= 9) {
            //Medir el tiempo
            time_req = clock();

            GridLocation whiteSpace{ 16, 13 };
            int cabin = 2;
            int i;

            //Calcular ruta del whitespace a la bicicleta
          /*  std::cout << "Camino espacio en blanco a bicicleta \n";*/

            //draw_grid(grid, 3, nullptr, &came_from_whitespace);
            /*std::cout << '\n';
            std::cout << '\n';*/

            //Buscar la ruta del espacio en blanco a la bicicleta
            std::unordered_map<GridLocation, GridLocation> came_from_whitespace;
            std::unordered_map<GridLocation, double> cost_so_far_whitespace;
            a_star_search(grid, whiteSpace, start, came_from_whitespace, cost_so_far_whitespace);
            std::vector<GridLocation> pathWhiteSpace = reconstruct_path(whiteSpace, start, came_from_whitespace);
            //draw_grid(grid, 3, nullptr, nullptr, &pathWhiteSpace);

            //std::cin >> i;

            /*std::cout << '\n';
            std::cout << '\n';*/

            moveWhiteSpaceToBicycle(whiteSpace, start, pathWhiteSpace);

            //std::cin >> i;

            std::vector<GridLocation> path = calculateRoute(grid, start, cabin);


            /*std::cout << '\n';
            std::cout << '\n';*/
            BicycleSpaceMovement(whiteSpace, start, grid, path);

            time_req = clock() - time_req;

            std::cout << "Processor time taken : "
                << (float)time_req / CLOCKS_PER_SEC << " seconds" << std::endl;
        }

        /*std::cout << "The time it took: " << cost_in_time << " sec." << std::endl;
        std::cout << "Position of bicycle: " << start.x << " " << start.y << std::endl;*/



        std::cout << "Again? (y) yes : (n) no\n";
        std::cin >> again;

        if (again == 'n' || again == 'N') {
            repeat = false;
        }
        cost_in_time = 0;
    }

    std::cout << '\n \n';
    std::cout << "Se logró cabrones";
    system("PAUSE");
    return 0;
}
