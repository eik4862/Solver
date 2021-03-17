#ifndef EIGHTPUZZLE_PUZZLESOLVER_H
#define EIGHTPUZZLE_PUZZLESOLVER_H

#include <iostream>
#include <utility>
#include <vector>
#include <utility>
#include <string>
#include <queue>
#include <algorithm>
#include <random>

enum class Direction {
  UP,
  DOWN,
  LEFT,
  RIGHT,
  NA
};

using puzzle_t = std::vector<std::vector<int>>;

class State {
public:
  explicit State(std::vector<std::vector<int>> puzzle, std::pair<int, int> blank, int size) noexcept
          : puzzle(std::move(puzzle)), blank(std::move(blank)), size(size) {}

  State(const State &other) = default;

  State(State &&other) = default;

  State &operator=(const State &other) = default;

  State &operator=(State &&other) = default;

  ~State() = default;

  int L1norm(const State &goal) const noexcept;

  State &update(Direction direction);

  inline bool operator==(const State &other) const noexcept { return puzzle == other.puzzle; }

  puzzle_t puzzle;
  std::pair<int, int> blank;
  int size;
};

class Node {
public:
  explicit Node(State &state, Direction action = Direction::NA, const Node *parent = nullptr, int g = 0) noexcept
          : state(std::move(state)), action(action), parent(parent), gValue(g), fValue(0) {}

  Node(const Node &other) = default;

  Node(Node &&other) = default;

  Node &operator=(const Node &other) = default;

  Node &operator=(Node &&other) = default;

  ~Node() = default;

  int heuristic(const State &goal) const noexcept { return gValue + state.L1norm(goal); }

  State state;
  Direction action;
  const Node *parent;
  int gValue;
  float fValue;
};

struct CompareNodePointers {
  constexpr bool operator()(const Node *lhs, const Node *rhs) const noexcept { return lhs->fValue > rhs->fValue; }
};

struct SolveResult {
  bool status;
  std::vector<Direction> solution;
  size_t visitedNodes;
};

class PuzzleSolver {
public:
  explicit PuzzleSolver(const State &puzzle, const State &goal) noexcept
          : _puzzle_(puzzle), _goal_(goal), _visitedNodes_(0), _status_(false), _solution_() {}

  PuzzleSolver(const PuzzleSolver &other) = delete;

  PuzzleSolver(PuzzleSolver &&other) = delete;

  PuzzleSolver &operator=(const PuzzleSolver &other) = delete;

  PuzzleSolver &operator=(PuzzleSolver &&other) = delete;

  ~PuzzleSolver() = default;

  void solve(const std::string &method) noexcept {
    if (method == "A*") _AStar_();
    else if (method == "RBFS") _RBFS_(false);
    else if (method == "RBFS_TieBreak") _RBFS_(true);
    else
      assert(false);
  }

  inline SolveResult report() const noexcept {
    return {_status_, _status_ ? _solution_ : std::vector<Direction>(), _visitedNodes_};
  }

private:
  constexpr bool _goalTest_(const Node *node) const noexcept { return node->state == _goal_; }

  std::vector<Direction> _actions_(const Node *node) noexcept;

  void _extractSolution_(const Node *node) noexcept;

  void _AStar_() noexcept;

  void _RBFS_(bool tieBreak = false) noexcept;

  std::pair<bool, float> _RBFSHelper_(Node *node, float limit, bool tieBreak = false) noexcept;

  State _puzzle_;
  State _goal_;
  size_t _visitedNodes_;
  bool _status_;
  std::vector<Direction> _solution_;
};

class PuzzleMaker {
public:
  PuzzleMaker() = default;

  PuzzleMaker(const PuzzleMaker &other) = delete;

  PuzzleMaker(PuzzleMaker &&other) = delete;

  PuzzleMaker &operator=(const PuzzleMaker &other) = delete;

  PuzzleMaker &operator=(PuzzleMaker &&other) = delete;

  ~PuzzleMaker() = default;

  State make(const State &goal, int depth) const noexcept;

private:
  void _shuffle_(State &state, int times) const noexcept;
};

struct TestResult {
  int iteration, depth;
  float AStarMeanVisitedNodes, RBFSMeanVisitedNodes, RBFSTieBreakMeanVisitedNodes;
  float AStarSDVisitedNodes, RBFSSDVisitedNodes, RBFSTieBreakSDVisitedNodes;
  std::vector<int> AStarVisitedNodes, RBFSVisitedNodes, RBFSTiebreakVisitedNodes;
};

class Tester {
public:
  Tester() = default;

  Tester(const Tester &other) = delete;

  Tester(Tester &&other) = delete;

  Tester &operator=(const Tester &other) = delete;

  Tester &operator=(Tester &&other) = delete;

  ~Tester() = default;

  void test(const State &goal, int depth, int iteration) noexcept;

  TestResult report() const noexcept;

private:
  PuzzleMaker _maker_;
  std::vector<SolveResult> _AStarResults_, _RBFSResults_, _RBFSTieBreakResults_;
};

#endif
