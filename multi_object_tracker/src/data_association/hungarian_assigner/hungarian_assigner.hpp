// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2018 Apex.AI, Inc.
/// \file
/// \brief Header for hungarian algorithm for optimal linear assignment
#ifndef HUNGARIAN_ASSIGNER__HUNGARIAN_ASSIGNER_HPP_
#define HUNGARIAN_ASSIGNER__HUNGARIAN_ASSIGNER_HPP_


/// \brief Ensure Eigen does not allocate memory dynamically
#define EIGEN_NO_MALLOC
/// \brief Prevent Eigen from yelling at you for larged fix-sized matrices
#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include "Eigen/Core"
#include "visibility_control.hpp"
#include <utility>
#include <limits>
#include <array>

namespace autoware
{
namespace fusion
{
/// \brief this namespace is for all functions, structs, classes and constants in the
///        hungarian_assigner package
namespace hungarian_assigner
{

/// \brief indexing matches what matrices use
using index_t = Eigen::Index;

/// \brief implementation of the hungarian/kuhn-munkres/jacobi algorithm for
/// minimum weight assignment problem in O(N^3 ) time
/// \tparam Capacity maximum number of things that can be matched, sets matrix size
template<int Capacity>
class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c
{
  static_assert(Capacity > 0, "Capacity must be positive");
  /// \brief markers for each matrix slot, not strongly typed so it can decay to numeric type
  enum mark_e : int8_t
  {
    NO_LINK = 0,   ///< no cost has been assigned, so assignment is assumed to be impossible
    UNMARKED = 1,  ///< this location is just a normal weight
    ZERO = 2,      ///< zero, but unmarked
    PRIMED = 3,    ///< primed zero
    STARRED = 4    ///< starred zero, corresponds to an assignment
  };

  /// \brief this definition is for internal book-keeping
  using index2_t = std::pair<index_t, index_t>;

  /// \brief This exception is for a bad edge case when the assigner tries to add a new zero
  ///        but the only uncovered entries in the matrix are not valid links/weights
  class HUNGARIAN_ASSIGNER_LOCAL no_uncovered_values_c : public std::runtime_error
  {
public:
    no_uncovered_values_c();
  };  // class no_uncovered_values_c

public:
  /// \brief This index denotes a worker for which no job assignment was possible
  static constexpr index_t UNASSIGNED = std::numeric_limits<index_t>::max();

  /// \brief constructor
  hungarian_assigner_c();

  /// \brief constructor, equivalent of construct(); set_size(num_rows, num_cols)
  /// \param[in] num_rows number of rows/jobs
  /// \param[in] num_cols number of columns/workers
  hungarian_assigner_c(const index_t num_rows, const index_t num_cols);

  /// \brief set the size of the matrix. Must be less than capacity. This should be done before
  ///        set_weight() calls
  /// \param[in] num_rows number of rows/jobs
  /// \param[in] num_cols number of columns/workers
  /// \throw std::length_error if num_rows or num_cols is bigger than capacity
  /// \throw std::domain_error if matrix shape is skinny
  void set_size(const index_t num_rows, const index_t num_cols);

  /// \brief set weight, update book-keeping for min in row
  ///        This function is meant to be called asynchronously over jdx, so
  ///        a thread should have fixed ownership over a given idx. Note that
  ///        you can call this function again for a same index pair, but you
  ///        will break assumptions if you set it to a higher weight.
  /// \param[in] weight the weight for assignment of job idx to worker jdx
  /// \param[in] idx the index of the job
  /// \param[in] jdx the index of the worker
  /// \throw std::out_of_range if idx or jdx are outside of range specified by set_size()
  void set_weight(const float weight, const index_t idx, const index_t jdx);

  /// \brief reset book-keeping and weight matrix, must be called after
  ///        assign(), and before set_weight()
  void reset();

  /// \brief reset and set_size, equivalent to reset(); set_size(num_rows, num_cols);
  /// \param[in] num_rows number of rows/jobs
  /// \param[in] num_cols number of columns/workers
  void reset(const index_t num_rows, const index_t num_cols);

  /// \brief compute minimum cost assignment
  /// \return whether or not it was successful. If assignment was unsuccessful (for example, the
  ///         case when a row has no valid assignments), then get_assignment() may return
  ///         UNASSIGNED
  /// \throw std::exception if some unspecified error happens internally, should never happen
  bool assign();

  // every row/job is guaranteed to have a worker
  /// \brief dictate what the assignment for a given row/task is, should be called after assign().
  ///        If assign() returned true, then every job/row is guaranteed to have a worker/column.
  ///        If assign() returned false, then a row may not have a worker/column
  /// \param[in] idx the index for the task, starting at 0
  /// \return the index for the assigned job, starting at 0
  ///         UNASSIGNED if assign() returned false and the idx job has no possible assignments
  /// \throw std::range_error if idx is out of bounds
  index_t get_assignment(const index_t idx) const;

  // get unassigned workers/columns. User should know how many unassigned there will be apriori
  /// \brief says what jobs have been unassigned
  /// \param[in] idx the i'th unassigned job, starts from 0 to num_jobs - num_workers
  /// \return the index of the i'th unassigned job
  /// \throw std::range_error if idx is out of bounds (i.e. there are no unassigned rows)
  index_t get_unassigned(const index_t idx) const;

private:
  /// \brief do steps 1-3, return true if perfect assignment found
  HUNGARIAN_ASSIGNER_LOCAL bool reduce_rows_and_init_zeros_and_check_result();

  /// \brief do step 5 and 3, return true if perfect assignment found
  HUNGARIAN_ASSIGNER_LOCAL bool increment_starred_zeroes_and_check_result(index2_t loc);

  /// \brief step 4 and 6: find an uncovered zero, add it if necessary, throws exception if it can't
  HUNGARIAN_ASSIGNER_LOCAL index2_t prime_uncovered_zero();

  /// \brief step 6: reduce matrix by smallest uncovered value, return false if no uncovered value
  HUNGARIAN_ASSIGNER_LOCAL bool add_new_zero(index2_t & loc);

  /// \brief check if assignment is complete
  HUNGARIAN_ASSIGNER_LOCAL bool are_all_columns_covered() const;

  /// \brief find a zero that is uncovered by rows or columns
  HUNGARIAN_ASSIGNER_LOCAL bool find_uncovered_zero(index2_t & loc) const;

  /// \brief find minimal value that is not covered, false if none found
  HUNGARIAN_ASSIGNER_LOCAL bool find_minimum_uncovered_value(
    index2_t & loc,
    float & min_val) const;

  /// \brief update internal bookkeeping for which rows and columns are uncovered
  HUNGARIAN_ASSIGNER_LOCAL void update_uncovered_rows_and_cols();

  Eigen::Matrix<float, Capacity, Capacity> m_weight_matrix;
  Eigen::Matrix<int8_t, Capacity, Capacity> m_mark_matrix;
  Eigen::Matrix<index_t, Capacity, 1> m_row_min_idx;
  index_t m_num_rows;
  index_t m_num_cols;
  Eigen::Matrix<float, Capacity, 1> m_row_min_weights;
  Eigen::Matrix<index_t, Capacity, 1> m_assignments;
  Eigen::Matrix<bool, Capacity, 1> m_is_col_covered;
  Eigen::Matrix<bool, Capacity, 1> m_is_row_covered;
  index_t m_num_uncovered_rows;
  index_t m_num_uncovered_cols;
  Eigen::Matrix<index_t, Capacity, 1> m_uncovered_rows;
  Eigen::Matrix<index_t, Capacity, 1> m_uncovered_cols;
  Eigen::Matrix<index2_t, 2 * Capacity, 1> m_augment_path;
  Eigen::Matrix<index2_t, Capacity, 1> m_primed_zero_locs;
  index_t m_num_primed_zeros;
};  // class hungarian_assigner_c

}  // namespace hungarian_assigner
}  // namespace fusion
}  // namespace autoware
#endif  // HUNGARIAN_ASSIGNER__HUNGARIAN_ASSIGNER_HPP_
