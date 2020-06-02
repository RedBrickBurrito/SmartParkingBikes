// SmartParkingBikes.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
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
template<class Graph>
void draw_grid(const Graph& graph, int field_width,
    std::unordered_map<GridLocation, double>* distances = nullptr,
    std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
    std::vector<GridLocation>* path = nullptr) {
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
                std::cout << '@';
            }
            else {
                std::cout << 'O';
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

GridWithWeights make_diagram4() {
    GridWithWeights grid(18, 15);
    //add_rect(grid, 1, 7, 4, 9);
    typedef GridLocation L;
    grid.map = std::unordered_set<GridLocation>{
      //L{0,0}, L{1,0}, L{2,0}, L{3,0}, L{4,0}, L{5,0}, L{6,0}, L{7,0}, L{8,0}, L{9,0}, L{10,0}, L{11,0}, L{2,0}, L{13,0}, L{14,0}, L{15,0}, L{16,0}, L{17,0},
      //L{0,2}, L{1,2}, L{2,2}, L{3,2}, L{4,2}, L{5,2}, L{6,2}, L{7,2}, L{8,2}, L{9,2}, L{10,2}, L{11,2}, L{12,2}, L{13,2}, L{14,2}, L{15,2}, L{16,2}, L{17,2},
      //L{0,4}, L{1,4}, L{2,4}, L{3,4}, L{4,4}, L{5,4}, L{6,4}, L{7,4}, L{8,4}, L{9,4}, L{10,4}, L{11,4}, L{12,4}, L{13,4}, L{14,4}, L{15,4}, L{16,4}, L{17,4},
      //L{0,6}, L{1,6}, L{2,6}, L{3,6}, L{4,6}, L{5,6}, L{6,6}, L{7,6}, L{8,6}, L{9,6}, L{10,6}, L{11,6},	L{12,6}, L{13,6}, L{14,6}, L{15,6},	L{16,6}, L{17,6},
      //L{0,8}, L{1,8}, L{2,8}, L{3,8}, L{4,8}, L{5,8}, L{6,8}, L{7,8}, L{8,8}, L{9,8}, L{10,8}, L{11,8}, L{12,8}, L{13,8}, L{14,8}, L{15,8}, L{16,8}, L{17,8},
      //L{0,10}, L{1,10},	L{2,10}, L{3,10}, L{4,10}, L{5,10},	L{6,10}, L{7,10}, L{8,10}, L{9,10},	L{10,10}, L{11,10},	L{12,10}, L{13,10},	L{14,10}, L{15,10},	L{16,10}, L{17,10},
      //L{0,12}, L{1,12},	L{2,12}, L{3,12}, L{4,12}, L{5,12},	L{6,12}, L{7,12}, L{8,12}, L{9,12},	L{10,12}, L{11,12},	L{12,12}, L{13,12},	L{14,12}, L{15,12},	L{16,12}, L{17,12},
      //L{0,14}, L{1,14},	L{2,14}, L{3,14}, L{4,14}, L{5,14},	L{6,14}, L{7,14}, L{8,14}, L{9,14},	L{10,14}, L{11,14},	L{12,14}, L{13,14},	L{14,14}, L{15,14},	L{16,14}, L{17,14},
      //L{0,16}, L{1,16},	L{2,16}, L{3,16}, L{4,16}, L{5,16},	L{6,16}, L{7,16}, L{8,16}, L{9,16},	L{10,16}, L{11,16},	L{12,16}, L{13,16},	L{14,16}, L{15,16},	L{16,16}, L{17,16},
      //L{0,18}, L{1,18},	L{2,18}, L{3,18}, L{4,18}, L{5,18},	L{6,18}, L{7,18}, L{8,18}, L{9,18},	L{10,18}, L{11,18},	L{12,18}, L{13,18},	L{14,18}, L{15,18},	L{16,18}, L{17,18},
      //L{0,20}, L{1,20},	L{2,20}, L{3,20}, L{4,20}, L{5,20},	L{6,20}, L{7,20}, L{8,20}, L{9,20},	L{10,20}, L{11,20},	L{12,20}, L{13,20},	L{14,20}, L{15,20},	L{16,20}, L{17,20},
      //L{0,22}, L{1,22},	L{2,22}, L{3,22}, L{4,22}, L{5,22},	L{6,22}, L{7,22}, L{8,22}, L{9,22},	L{10,22}, L{11,22},	L{12,22}, L{13,22},	L{14,22}, L{15,22},	L{16,22}, L{17,22},
      //L{0,24}, L{1,24},	L{2,24}, L{3,24}, L{4,24}, L{5,24},	L{6,24}, L{7,24}, L{8,24}, L{9,24},	L{10,24}, L{11,24},	L{12,24}, L{13,24},	L{14,24}, L{15,24},	L{16,24}, L{17,24},
      //L{0,26}, L{1,26},	L{2,26}, L{3,26}, L{4,26}, L{5,26},	L{6,26}, L{7,26}, L{8,26}, L{9,26},	L{10,26}, L{11,26},	L{12,26}, L{13,26},	L{14,26}, L{15,26},	L{16,26}, L{17,26},
      //L{0,28}, L{1,28},	L{2,28}, L{3,28}, L{4,28}, L{5,28},	L{6,28}, L{7,28}, L{8,28}, L{9,28},	L{10,28}, L{11,28},	L{12,28}, L{13,28},	L{14,28}, L{15,28},	L{16,28}, L{17,28},

    };
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

template<typename Location, typename Graph>
void dijkstra_search
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
                came_from[next] = current;
                frontier.put(next, new_cost);
            }
        }
    }
}

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


int main()
{
    int cabinNumber;
    GridWithWeights grid = make_diagram4();
    std::cout << "Which Cabin, 1 or 2?"  << std::endl;
    std::cin >> cabinNumber;

    if (cabinNumber == 1) {
        GridLocation start{ 0,0 };
        GridLocation goal{ 8, 6 };

        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, double> cost_so_far;
        a_star_search(grid, start, goal, came_from, cost_so_far);
        draw_grid(grid, 3, nullptr, &came_from);
        std::cout << '\n';
        std::cout << '\n';
        draw_grid(grid, 3, &cost_so_far, nullptr);
        std::cout << '\n';
        std::cout << '\n';
        std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
        draw_grid(grid, 3, nullptr, nullptr, &path);

    }
    else {
        GridLocation start{ 18, 28 };
        GridLocation goal{ 13, 22 };

        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, double> cost_so_far;
        a_star_search(grid, start, goal, came_from, cost_so_far);
        draw_grid(grid, 10, nullptr, &came_from);
        std::cout << '\n';
        std::cout << '\n';
        draw_grid(grid, 10, &cost_so_far, nullptr);
        std::cout << '\n';
        std::cout << '\n';
        std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
        draw_grid(grid, 10, nullptr, nullptr, &path);
    }

    system("PAUSE");
    return 0;
}
