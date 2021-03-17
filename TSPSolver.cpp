#include "TSPSolver.h"

std::mt19937 generator(0);

float Node::heuristic() noexcept {
  int lastVisit = state.visited.back();
  state.visited.pop_back();

  std::vector<point_t> vertices;
  int vertexCount;
  vertices.reserve(state.map->size());

  for (int i = 0; i < state.map->size(); i++) {
    if (std::find(state.visited.begin(), state.visited.end(), i) != state.visited.end()) continue;

    vertices.push_back((*state.map)[i]);
  }

  state.visited.push_back(lastVisit);

  if (vertices.size() < 2) return gValue;

  vertexCount = vertices.size();
  std::vector<std::vector<vwpair_t>> adjacencyMatrix(vertexCount, std::vector<vwpair_t>());

  for (int i = 0; i < vertexCount; i++) {
    adjacencyMatrix[i].reserve(vertexCount);

    for (int j = 0; j < vertexCount; j++) {
      if (i == j) continue;

      adjacencyMatrix[i].push_back({j, State::L2norm(vertices[i], vertices[j])});
    }
  }

  return _MSTEstimate_(adjacencyMatrix) + gValue;
}

float Node::_MSTEstimate_(const std::vector<std::vector<vwpair_t>> &adjacencyMatrix) const noexcept {
  std::priority_queue<vwpair_t, std::vector<vwpair_t>, CompareWVPairs> priorityQueue;
  std::vector<float> keys(adjacencyMatrix.size(), MAXFLOAT);
  std::vector<bool> inMST(adjacencyMatrix.size(), false);
  int u, v;
  float weight;

  priorityQueue.push({0, 0});
  keys[0] = 0;

  while (not priorityQueue.empty()) {
    u = priorityQueue.top().second;

    priorityQueue.pop();
    inMST[u] = true;

    for (const auto &pair: adjacencyMatrix[u]) {
      v = pair.first;
      weight = pair.second;

      if (not inMST[v] and keys[v] > weight) {
        keys[v] = weight;
        priorityQueue.push({v, keys[v]});
      }
    }
  }

  float estimate = 0;

  for (const auto &key: keys) estimate += key;

  return estimate;
}

std::vector<int> TSPSolver::_actions_(const Node *node) noexcept {
  std::vector<int> validActions;
  validActions.reserve(node->state.map->size());

  for (int i = 0; i < node->state.map->size(); i++) {
    if (std::find(node->state.visited.begin(), node->state.visited.end(), i) == node->state.visited.end()) {
      validActions.push_back(i);
    }
  }

  return validActions;
}

void TSPSolver::_extractSolution_(const Node *node) noexcept {
  if (node->parent != nullptr) _extractSolution_(node->parent);

  _solution_.push_back(node->action);
}

void TSPSolver::_AStar_() noexcept {
  std::priority_queue<Node *, std::vector<Node *>, CompareNodePointers> frontier;
  std::vector<Node *> explored;
  Node *child, *node = new Node(_initialState_, _startPoint_);
  bool visited;
  _visitedNodes_++;

  if (_goalTest_(node)) {
    _status_ = true;
    _extractSolution_(node);

    return;
  }

  node->fValue = node->heuristic();
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
      child = new Node(
              State(node->state).update(action), action, node,
              node->gValue + State::L2norm((*node->state.map)[node->action], (*node->state.map)[action])
      );
      _visitedNodes_++;
      visited = false;

      for (const auto &item: explored) {
        if (child->state == item->state) {
          visited = true;
          break;
        }
      }

      if (not visited) {
        child->fValue = child->heuristic();
        frontier.push(child);
      }
    }
  }
}

void TSPSolver::_RBFS_() noexcept {
  Node *root = new Node(_initialState_, _startPoint_);
  root->fValue = root->heuristic();

  _visitedNodes_++;

  std::pair<bool, float> result = _RBFSHelper_(root, MAXFLOAT);
  _status_ = result.first;
}

std::pair<bool, float> TSPSolver::_RBFSHelper_(Node *node, float limit) noexcept {
  Node *child, *best;
  std::pair<bool, float> result;
  float alternative;

  if (_goalTest_(node)) {
    _extractSolution_(node);

    return {true, node->fValue};
  }

  std::priority_queue<Node *, std::vector<Node *>, CompareNodePointers> children;

  for (const auto &action: _actions_(node)) {
    child = new Node(
            State(node->state).update(action), action, node,
            node->gValue + State::L2norm((*node->state.map)[node->action], (*node->state.map)[action])
    );
    child->fValue = std::max(child->heuristic(), node->fValue);

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

State MapMaker::make(int pointCount) const noexcept {
  auto *map = new std::vector<point_t>();
  std::uniform_real_distribution<float> distribution(0, 1);

  map->reserve(pointCount);

  for (int i = 0; i < pointCount; i++) map->push_back({distribution(generator), distribution(generator)});

  return State(map, {std::uniform_int_distribution<int>()(generator) % (int) map->size()});
}

void Tester::test(int pointCount, int iteration) noexcept {
  _AStarResults_.reserve(iteration);
  _RBFSResults_.reserve(iteration);

  for (int i = 0; i < iteration; i++) {
    State initialState = _maker_.make(pointCount);

    TSPSolver AStarSolver(initialState);
    AStarSolver.solve("A*");
    _AStarResults_.push_back(AStarSolver.report());

    TSPSolver RBFSSolver(initialState);
    RBFSSolver.solve("RBFS");
    _RBFSResults_.push_back(RBFSSolver.report());
  }
}

TestResult Tester::report() const noexcept {
  std::vector<int> AStarVisitedNodes, RBFSVisitedNodes;
  float AStarMeanVisitedNodes = 0, RBFSMeanVisitedNodes = 0;
  float AStarSDVisitedNodes = 0, RBFSSDVisitedNodes = 0;
  int iteration = _AStarResults_.size(), pointCount = _AStarResults_[0].solution.size();
  AStarVisitedNodes.reserve(iteration);
  RBFSVisitedNodes.reserve(iteration);

  for (int i = 0; i < iteration; i++) {
    AStarMeanVisitedNodes += _AStarResults_[i].visitedNodes;
    RBFSMeanVisitedNodes += _RBFSResults_[i].visitedNodes;

    AStarSDVisitedNodes += _AStarResults_[i].visitedNodes * _AStarResults_[i].visitedNodes;
    RBFSSDVisitedNodes += _RBFSResults_[i].visitedNodes * _RBFSResults_[i].visitedNodes;

    AStarVisitedNodes.push_back(_AStarResults_[i].visitedNodes);
    RBFSVisitedNodes.push_back(_RBFSResults_[i].visitedNodes);
  }

  AStarMeanVisitedNodes /= iteration;
  RBFSMeanVisitedNodes /= iteration;
  AStarSDVisitedNodes /= iteration;
  RBFSSDVisitedNodes /= iteration;
  AStarSDVisitedNodes -= AStarMeanVisitedNodes * AStarMeanVisitedNodes;
  RBFSSDVisitedNodes -= RBFSMeanVisitedNodes * RBFSMeanVisitedNodes;

  return {iteration, pointCount, AStarMeanVisitedNodes, RBFSMeanVisitedNodes, sqrt(AStarSDVisitedNodes),
          sqrt(RBFSSDVisitedNodes), AStarVisitedNodes, RBFSVisitedNodes};
}