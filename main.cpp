#include <iostream>
//#include "PuzzleSolver.h"
#include "TSPSolver.h"

int main() {
//  State goal({{0, 1, 2},
//              {3, 4, 5},
//              {6, 7, 8}}, {0, 0}, 3);
//
//  Tester tester;
//
//  tester.test(goal, 25, 10);
//
//  auto result = tester.report();
//
//  std::cout << "### TEST REPORT ###" << std::endl;
//  std::cout << "  @iteration: " << result.iteration << std::endl;
//  std::cout << "  @depth    : " << result.depth << std::endl;
//  std::cout << "  @visited nodes (mean)" << std::endl;
//  std::cout << "    @A*              : " << result.AStarMeanVisitedNodes << std::endl;
//  std::cout << "    @RBFS            : " << result.RBFSMeanVisitedNodes << std::endl;
//  std::cout << "    @RBFS (tie break): " << result.RBFSTieBreakMeanVisitedNodes << std::endl;
//  std::cout << "  @visited nodes (SD)" << std::endl;
//  std::cout << "    @A*              : " << result.AStarSDVisitedNodes << std::endl;
//  std::cout << "    @RBFS            : " << result.RBFSSDVisitedNodes << std::endl;
//  std::cout << "    @RBFS (tie break): " << result.RBFSTieBreakSDVisitedNodes << std::endl;
//  std::cout << "  @visited nodes (raw data)" << std::endl;
//  std::cout << "    @A*              : ";
//
//  for (const auto &count: result.AStarVisitedNodes) std::cout << count << ", ";
//
//  std::cout << std::endl;
//  std::cout << "    @RBFS            : ";
//
//  for (const auto &count: result.RBFSVisitedNodes) std::cout << count << ", ";
//
//  std::cout << std::endl;
//  std::cout << "    @RBFS (tie break): ";
//
//  for (const auto &count: result.RBFSTiebreakVisitedNodes) std::cout << count << ", ";
//
//  std::cout << std::endl;


  Tester tester;

  tester.test(100, 10);

  auto result = tester.report();

  std::cout << "### TEST REPORT ###" << std::endl;
  std::cout << "  @iteration  : " << result.iteration << std::endl;
  std::cout << "  @point count: " << result.pointCount << std::endl;
  std::cout << "  @visited nodes (mean)" << std::endl;
  std::cout << "    @A*  : " << result.AStarMeanVisitedNodes << std::endl;
  std::cout << "    @RBFS: " << result.RBFSMeanVisitedNodes << std::endl;
  std::cout << "  @visited nodes (SD)" << std::endl;
  std::cout << "    @A*  : " << result.AStarSDVisitedNodes << std::endl;
  std::cout << "    @RBFS: " << result.RBFSSDVisitedNodes << std::endl;
  std::cout << "  @visited nodes (raw data)" << std::endl;
  std::cout << "    @A*  : ";

  for (const auto &count: result.AStarVisitedNodes) std::cout << count << '\t';

  std::cout << std::endl;
  std::cout << "    @RBFS: ";

  for (const auto &count: result.RBFSVisitedNodes) std::cout << count << '\t';

  std::cout << std::endl;

  return 0;
}
