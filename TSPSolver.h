#ifndef EIGHTPUZZLE_TSPSOLVER_H
#define EIGHTPUZZLE_TSPSOLVER_H

#include <vector>
#include <utility>
#include <iostream>
#include <queue>
#include <random>
#include <algorithm>

using point_t = std::pair<float, float>;
using vwpair_t = std::pair<int, float>;

class State {
public:
  explicit State(const std::vector<point_t> *map, std::vector<int> visited) noexcept
          : map(map), visited(std::move(visited)) {}

  State(const State &other) = default;

  State(State &&other) = default;

  State &operator=(const State &other) = default;

  State &operator=(State &&other) = default;

  ~State() = default;

  void print() {
    for (const auto i: visited) {
      std::cout << i << '\t';
    }

    std::cout << std::endl;
  }

  State &update(int vertex) {
    visited.push_back(vertex);

    return *this;
  }

  inline static float L2norm(const point_t x, const point_t y) noexcept {
    return sqrt((x.first - y.first) * (x.first - y.first) + (x.second - y.second) * (x.second - y.second));
  }

  inline bool operator==(const State &other) const noexcept { return visited == other.visited; }

  const std::vector<point_t> *map;
  std::vector<int> visited;
};

struct CompareWVPairs {
  constexpr bool operator()(const vwpair_t &lhs, const vwpair_t &rhs) const noexcept { return lhs.second > rhs.second; }
};

class Node {
public:
  explicit Node(State &state, int action = -1, const Node *parent = nullptr, float g = 0) noexcept
          : state(std::move(state)), action(action), parent(parent), gValue(g), fValue(0) {}

  Node(const Node &other) = default;

  Node(Node &&other) = default;

  Node &operator=(const Node &other) = default;

  Node &operator=(Node &&other) = default;

  ~Node() = default;

  float heuristic() noexcept;

  State state;
  int action;
  const Node *parent;
  float gValue;
  float fValue;
private:
  float _MSTEstimate_(const std::vector<std::vector<vwpair_t>> &adjacencyMatrix) const noexcept;
};

struct CompareNodePointers {
  constexpr bool operator()(const Node *lhs, const Node *rhs) const noexcept { return lhs->fValue > rhs->fValue; }
};

struct SolveResult {
  bool status;
  std::vector<int> solution;
  size_t visitedNodes;
};

class TSPSolver {
public:
  explicit TSPSolver(const State &initialState) noexcept
          : _initialState_(initialState), _startPoint_(initialState.visited[0]), _visitedNodes_(0), _status_(false),
            _solution_() {}

  TSPSolver(const TSPSolver &other) = delete;

  TSPSolver(TSPSolver &&other) = delete;

  TSPSolver &operator=(const TSPSolver &other) = delete;

  TSPSolver &operator=(TSPSolver &&other) = delete;

  ~TSPSolver() = default;

  void solve(const std::string &method) noexcept {
    if (method == "A*") _AStar_();
    else if (method == "RBFS") _RBFS_();
    else
      assert(false);
  }

  inline SolveResult report() const noexcept {
    return {_status_, _status_ ? _solution_ : std::vector<int>(), _visitedNodes_};
  }

private:
  constexpr bool _goalTest_(const Node *node) const noexcept {
    return node->state.visited.size() == node->state.map->size();
  }

  std::vector<int> _actions_(const Node *node) noexcept;

  void _extractSolution_(const Node *node) noexcept;

  void _AStar_() noexcept;

  void _RBFS_() noexcept;

  std::pair<bool, float> _RBFSHelper_(Node *node, float limit) noexcept;

  State _initialState_;
  int _startPoint_;
  size_t _visitedNodes_;
  bool _status_;
  std::vector<int> _solution_;
};

class MapMaker {
public:
  MapMaker() = default;

  MapMaker(const MapMaker &other) = delete;

  MapMaker(MapMaker &&other) = delete;

  MapMaker &operator=(const MapMaker &other) = delete;

  MapMaker &operator=(MapMaker &&other) = delete;

  ~MapMaker() = default;

  State make(int pointCount) const noexcept;
};

struct TestResult {
  int iteration, pointCount;
  float AStarMeanVisitedNodes, RBFSMeanVisitedNodes;
  float AStarSDVisitedNodes, RBFSSDVisitedNodes;
  std::vector<int> AStarVisitedNodes, RBFSVisitedNodes;
};

class Tester {
public:
  Tester() = default;

  Tester(const Tester &other) = delete;

  Tester(Tester &&other) = delete;

  Tester &operator=(const Tester &other) = delete;

  Tester &operator=(Tester &&other) = delete;

  ~Tester() = default;

  void test(int pointCount, int iteration) noexcept;

  TestResult report() const noexcept;

private:
  MapMaker _maker_;
  std::vector<SolveResult> _AStarResults_, _RBFSResults_;
};

#endif
