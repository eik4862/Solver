#include "PuzzleSolver.h"

std::mt19937 generator(0);

int State::L1norm(const State &goal) const noexcept {
  int norm = 0;

  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      if (puzzle[i][j] == 0) continue;

      for (int k = 0; k < size; k++) {
        for (int l = 0; l < size; l++) {
          if (puzzle[i][j] == goal.puzzle[k][l]) {
            norm += abs(i - k) + abs(j - l);
            k = l = size;
          }
        }
      }
    }
  }

  return norm;
}

State &State::update(Direction direction) {
  switch (direction) {
    case Direction::UP:
      puzzle[blank.first][blank.second] = puzzle[blank.first - 1][blank.second];
      puzzle[blank.first - 1][blank.second] = 0;
      blank.first--;
      break;
    case Direction::DOWN:
      puzzle[blank.first][blank.second] = puzzle[blank.first + 1][blank.second];
      puzzle[blank.first + 1][blank.second] = 0;
      blank.first++;
      break;
    case Direction::LEFT:
      puzzle[blank.first][blank.second] = puzzle[blank.first][blank.second - 1];
      puzzle[blank.first][blank.second - 1] = 0;
      blank.second--;
      break;
    case Direction::RIGHT:
      puzzle[blank.first][blank.second] = puzzle[blank.first][blank.second + 1];
      puzzle[blank.first][blank.second + 1] = 0;
      blank.second++;
      break;
    default:
      break;
  }

  return *this;
}

std::vector<Direction> PuzzleSolver::_actions_(const Node *node) noexcept {
  std::vector<Direction> validActions;

  if (node->state.blank.first > 0) validActions.push_back(Direction::UP);
  if (node->state.blank.first < node->state.size - 1) validActions.push_back(Direction::DOWN);
  if (node->state.blank.second > 0) validActions.push_back(Direction::LEFT);
  if (node->state.blank.second < node->state.size - 1) validActions.push_back(Direction::RIGHT);

  return validActions;
}

void PuzzleSolver::_extractSolution_(const Node *node) noexcept {
  if (node->parent == nullptr) return;

  _extractSolution_(node->parent);
  _solution_.push_back(node->action);
}

void PuzzleSolver::_AStar_() noexcept {
  std::priority_queue<Node *, std::vector<Node *>, CompareNodePointers> frontier;
  std::vector<Node *> explored;
  Node *child, *node = new Node(_puzzle_);
  bool visited;
  _visitedNodes_++;

  if (_goalTest_(node)) {
    _status_ = true;
    _extractSolution_(node);

    return;
  }

  node->fValue = node->heuristic(State(_goal_));
  frontier.push(node);

  while (true) {
    if (frontier.empty()) {
      _status_ = false;

      return;
    }

    node = frontier.top();
    frontier.pop();

    if (_goalTest_(node)) {
      _status_ = true;
      _extractSolution_(node);

      return;
    }

    explored.push_back(node);

    for (const auto &action: _actions_(node)) {
      child = new Node(State(node->state).update(action), action, node, node->gValue + 1);
      _visitedNodes_++;
      visited = false;

      for (const auto &item: explored) {
        if (child->state == item->state) {
          visited = true;
          break;
        }
      }

      if (not visited) {
        child->fValue = child->heuristic(_goal_);
        frontier.push(child);
      }
    }
  }
}

void PuzzleSolver::_RBFS_(bool tieBreak) noexcept {
  Node *root = new Node(_puzzle_);
  root->fValue = root->heuristic(_goal_) + tieBreak * std::uniform_real_distribution<float>(-1, 1)(generator);

  _visitedNodes_++;

  std::pair<bool, float> result = _RBFSHelper_(root, MAXFLOAT, tieBreak);
  _status_ = result.first;
}

std::pair<bool, float> PuzzleSolver::_RBFSHelper_(Node *node, float limit, bool tieBreak) noexcept {
  Node *child, *best;
  std::pair<bool, float> result;
  float alternative;

  if (_goalTest_(node)) {
    _extractSolution_(node);

    return {true, node->fValue};
  }

  std::priority_queue<Node *, std::vector<Node *>, CompareNodePointers> children;

  for (const auto &action: _actions_(node)) {
    child = new Node(State(node->state).update(action), action, node, node->gValue + 1);
    child->fValue = std::max(
            child->heuristic(_goal_) + tieBreak * std::uniform_real_distribution<float>(-1, 1)(generator),
            node->fValue
    );

    _visitedNodes_++;
    children.push(child);
  }

  if (children.empty()) return {false, MAXFLOAT};

  while (true) {
    best = children.top();
    children.pop();

    if (best->fValue > limit) return {false, best->fValue};

    alternative = children.empty() ? MAXFLOAT : children.top()->fValue;
    result = _RBFSHelper_(best, std::min(limit, alternative));

    if (result.first) {
      return result;
    } else {
      best->fValue = result.second;
      children.push(best);
    }
  }
}

State PuzzleMaker::make(const State &goal, int depth) const noexcept {
  while (true) {
    State newPuzzle = goal;

    _shuffle_(newPuzzle, depth * 3);
    PuzzleSolver solver(newPuzzle, goal);
    solver.solve("A*");

    if (solver.report().solution.size() == depth) return newPuzzle;
  }
}

void PuzzleMaker::_shuffle_(State &state, int times) const noexcept {
  Direction new_, old = Direction::NA;
  std::vector<Direction> candidates;
  std::uniform_int_distribution<int> distribution;

  for (int i = 0; i < times; i++) {
    candidates.clear();

    if (state.blank.first > 0 and old != Direction::UP) candidates.push_back(Direction::UP);
    if (state.blank.first < state.size - 1 and old != Direction::DOWN) candidates.push_back(Direction::DOWN);
    if (state.blank.second > 0 and old != Direction::LEFT) candidates.push_back(Direction::LEFT);
    if (state.blank.second < state.size - 1 and old != Direction::RIGHT) candidates.push_back(Direction::RIGHT);

    new_ = candidates[distribution(generator) % candidates.size()];
    state.update(new_);
    old = new_;
  }
}

void Tester::test(const State &goal, int depth, int iteration) noexcept {
  _AStarResults_.reserve(iteration);
  _RBFSResults_.reserve(iteration);
  _RBFSTieBreakResults_.reserve(iteration);

  for (int i = 0; i < iteration; i++) {
    State puzzle = _maker_.make(goal, depth);

    PuzzleSolver AStarSolver(puzzle, goal);
    AStarSolver.solve("A*");
    _AStarResults_.push_back(AStarSolver.report());

    PuzzleSolver RBFSSolver(puzzle, goal);
    RBFSSolver.solve("RBFS");
    _RBFSResults_.push_back(RBFSSolver.report());

    PuzzleSolver RBFSTieBreakSolver(puzzle, goal);
    RBFSTieBreakSolver.solve("RBFS_TieBreak");
    _RBFSTieBreakResults_.push_back(RBFSTieBreakSolver.report());
  }
}

TestResult Tester::report() const noexcept {
  std::vector<int> AStarVisitedNodes, RBFSVisitedNodes, RBFTieBreakVisitedNodes;
  float AStarMeanVisitedNodes = 0, RBFSMeanVisitedNodes = 0, RBFSTieBreakMeanVisitedNodes = 0;
  float AStarSDVisitedNodes = 0, RBFSSDVisitedNodes = 0, RBFSTieBreakSDVisitedNodes = 0;
  int iteration = _AStarResults_.size(), depth = _AStarResults_[0].solution.size();
  AStarVisitedNodes.reserve(iteration);
  RBFSVisitedNodes.reserve(iteration);
  RBFTieBreakVisitedNodes.reserve(iteration);

  for (int i = 0; i < iteration; i++) {
    AStarMeanVisitedNodes += _AStarResults_[i].visitedNodes;
    RBFSMeanVisitedNodes += _RBFSResults_[i].visitedNodes;
    RBFSTieBreakMeanVisitedNodes += _RBFSTieBreakResults_[i].visitedNodes;

    AStarSDVisitedNodes += _AStarResults_[i].visitedNodes * _AStarResults_[i].visitedNodes;
    RBFSSDVisitedNodes += _RBFSResults_[i].visitedNodes * _RBFSResults_[i].visitedNodes;
    RBFSTieBreakSDVisitedNodes += _RBFSTieBreakResults_[i].visitedNodes * _RBFSTieBreakResults_[i].visitedNodes;

    AStarVisitedNodes.push_back(_AStarResults_[i].visitedNodes);
    RBFSVisitedNodes.push_back(_RBFSResults_[i].visitedNodes);
    RBFTieBreakVisitedNodes.push_back(_RBFSTieBreakResults_[i].visitedNodes);
  }

  AStarMeanVisitedNodes /= iteration;
  RBFSMeanVisitedNodes /= iteration;
  RBFSTieBreakMeanVisitedNodes /= iteration;
  AStarSDVisitedNodes /= iteration;
  RBFSSDVisitedNodes /= iteration;
  RBFSTieBreakSDVisitedNodes /= iteration;
  AStarSDVisitedNodes -= AStarMeanVisitedNodes * AStarMeanVisitedNodes;
  RBFSSDVisitedNodes -= RBFSMeanVisitedNodes * RBFSMeanVisitedNodes;
  RBFSTieBreakSDVisitedNodes -= RBFSTieBreakMeanVisitedNodes * RBFSTieBreakMeanVisitedNodes;

  return {iteration, depth, AStarMeanVisitedNodes, RBFSMeanVisitedNodes, RBFSTieBreakMeanVisitedNodes,
          sqrt(AStarSDVisitedNodes), sqrt(RBFSSDVisitedNodes), sqrt(RBFSTieBreakSDVisitedNodes),
          AStarVisitedNodes, RBFSVisitedNodes, RBFTieBreakVisitedNodes};
}