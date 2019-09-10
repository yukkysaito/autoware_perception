#ifndef SUCCESSIVE_SHORTEST_PATH_H_
#define SUCCESSIVE_SHORTEST_PATH_H_

#include <unordered_map>
#include <vector>

namespace assignment_problem
{

// See IMPORTANT NOTE at the top of the file.
void MaximizeLinearAssignment(const std::vector<std::vector<double>> &cost,
                              std::unordered_map<int, int> *direct_assignment,
                              std::unordered_map<int, int> *reverse_assignment);
} // namespace assignment_problem

#endif // SUCCESSIVE_SHORTEST_PATH_H_