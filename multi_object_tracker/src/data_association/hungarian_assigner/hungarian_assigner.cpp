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
/// \brief source file for hungarian algorithm for optimal linear assignment

//lint -e537 cpplint complains otherwise NOLINT
#include <limits>
#include <algorithm>
#include "hungarian_assigner.hpp"
#include <iostream>

namespace autoware
{
namespace fusion
{
namespace hungarian_assigner
{

///
template <int Capacity>
hungarian_assigner_c<Capacity>::no_uncovered_values_c::no_uncovered_values_c()
    : std::runtime_error("Hungarian Assigner cannot add new zero: no uncovered values!")
{
}

///
template <int Capacity>
constexpr index_t hungarian_assigner_c<Capacity>::UNASSIGNED;

///
template <int Capacity>
hungarian_assigner_c<Capacity>::hungarian_assigner_c(
    const index_t num_rows,
    const index_t num_cols)
    : m_weight_matrix(
          Eigen::MatrixXf::Constant(
              Capacity,
              Capacity,
              std::numeric_limits<float>::max())),
      m_mark_matrix(
          Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(
              Capacity,
              Capacity,
              static_cast<int8_t>(NO_LINK))),
      m_row_min_idx(Eigen::Matrix<index_t, Capacity, 1>::Constant(UNASSIGNED)),
      m_num_rows(num_rows),
      m_num_cols(num_cols),
      m_row_min_weights(Eigen::ArrayXf::Constant(Capacity, std::numeric_limits<float>::max())),
      m_assignments(Eigen::Matrix<index_t, Capacity, 1>::Zero()),
      m_is_col_covered(Eigen::Matrix<bool, Capacity, 1>::Zero()), // zero/false initialized
      m_is_row_covered(Eigen::Matrix<bool, Capacity, 1>::Zero()), // zero/false initialized
      m_num_uncovered_rows(),                                     // zero initialization
      m_num_uncovered_cols(),                                     // zero initialization
      m_num_primed_zeros()                                        // zero initialization
{
  // all other arrays should be zero initialized
  // m_row_min_idx.fill(UNASSIGNED);
}

///
template <int Capacity>
hungarian_assigner_c<Capacity>::hungarian_assigner_c()
    : hungarian_assigner_c(index_t(), index_t()) // zero initialization
{
}

///
template <int Capacity>
void hungarian_assigner_c<Capacity>::set_size(const index_t num_rows, const index_t num_cols)
{
  if ((num_rows > Capacity) || (num_cols > Capacity))
  {
    throw std::length_error("Cannot make hungarian assigner bigger than capacity");
  }
  if (num_rows > num_cols)
  {
    throw std::domain_error("Cost matrix must be fat or square");
  }
  m_num_rows = num_rows;
  m_num_cols = num_cols;
  // initialize assignments to "unassigned"
  const index_t max_assignments = std::max(num_rows, num_cols);
  for (index_t idx = index_t(); idx < max_assignments; ++idx)
  {
    m_assignments[idx] = hungarian_assigner_c::UNASSIGNED;
  }
}

///
template <int Capacity>
void hungarian_assigner_c<Capacity>::set_weight(
    const float weight,
    const index_t idx,
    const index_t jdx)
{
  if ((idx >= m_num_rows) || (jdx >= m_num_cols))
  {
    throw std::out_of_range("Cannot set weight outside of range");
  }
  // rely on eigen's matrix to throw an error
  m_weight_matrix(idx, jdx) = weight;
  m_mark_matrix(idx, jdx) = static_cast<int8_t>(UNMARKED);
  // update min column weight, threadsafe if access different rows async
  if (weight < m_row_min_weights(idx))
  {
    m_row_min_weights(idx) = weight;
    m_row_min_idx[idx] = jdx;
  }
}

///
template <int Capacity>
void hungarian_assigner_c<Capacity>::reset()
{
  // set all weights to infinity
  m_weight_matrix.block(index_t(), index_t(), m_num_cols, m_num_cols) =
      Eigen::MatrixXf::Constant(m_num_cols, m_num_cols, std::numeric_limits<float>::max());
  m_row_min_weights.segment(0, m_num_cols) =
      Eigen::ArrayXf::Constant(m_num_cols, std::numeric_limits<float>::max());
  // reset assignment
  m_mark_matrix.block(index_t(), index_t(), m_num_cols, m_num_cols) =
      Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(
          m_num_cols,
          m_num_cols,
          static_cast<int8_t>(NO_LINK));
  // reset size
  m_num_cols = index_t();
  m_num_rows = index_t();
  m_num_primed_zeros = index_t();
  m_is_col_covered.fill(false);
  m_is_row_covered.fill(false);
  m_row_min_idx.fill(UNASSIGNED);
}

///
template <int Capacity>
void hungarian_assigner_c<Capacity>::reset(const index_t num_rows, const index_t num_cols)
{
  reset();
  set_size(num_rows, num_cols);
}

///
template <int Capacity>
bool hungarian_assigner_c<Capacity>::assign()
{
  ////////////////////////////
  // // Debug print
  // std::cout << std::endl;
  // std::cout << "m_weight_matrix (before)" << std::endl;
  // std::cout << m_weight_matrix << std::endl;
  ////////////////////////////

  bool ret = false;
  if (m_num_rows <= m_num_cols)
  {
    // pad 0's in unbalanced case
    if (m_num_rows < m_num_cols)
    {
      const index_t num_pad = m_num_cols - m_num_rows;
      m_weight_matrix.block(m_num_rows, index_t(), num_pad, m_num_cols) =
          Eigen::MatrixXf::Zero(num_pad, m_num_cols);
      m_row_min_weights.segment(m_num_rows, num_pad) =
          Eigen::ArrayXf::Zero(num_pad);
      m_mark_matrix.block(m_num_rows, index_t(), num_pad, m_num_cols) =
          Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic>::Constant(
              num_pad,
              m_num_cols,
              static_cast<int8_t>(ZERO));
    }
    // actually solve the assignment problem
    try
    {
      if (!reduce_rows_and_init_zeros_and_check_result())
      {
        // this loop runs N times because each iteration adds a starred zero/assignment
        for (index_t idx = index_t(); idx < m_num_cols; ++idx)
        {
          const typename hungarian_assigner_c<Capacity>::index2_t loc = prime_uncovered_zero();
          if (increment_starred_zeroes_and_check_result(loc))
          {
            ret = true;
            break;
          }
        }
      }
      else
      {
        ret = true;
      }
    }
    catch (const no_uncovered_values_c &)
    {
      // nothing to do, just brekaing out of loop
    }
  }

  ////////////////////////////
  // // Debug print
  // std::cout << std::endl;
  // std::cout << "m_weight_matrix (after)" << std::endl;
  // std::cout << m_weight_matrix << std::endl;

  // std::cout << std::endl;
  // std::cout << "m_mark_matrix (after)" << std::endl;
  // std::cout << m_mark_matrix << std::endl;

  // std::cout << std::endl;
  // std::cout << "m_assignments (after)" << std::endl;
  // std::cout << m_assignments << std::endl;

  ////////////////////////////

  return ret;
}

///
template <int Capacity>
index_t hungarian_assigner_c<Capacity>::get_assignment(const index_t idx) const
{
  if ((idx >= m_num_rows) || (idx >= Capacity))
  {
    throw std::range_error("Querying out of bounds assignment index");
  }
  return m_assignments[idx];
}

///
template <int Capacity>
index_t hungarian_assigner_c<Capacity>::get_unassigned(const index_t idx) const
{
  const index_t jdx = idx + m_num_rows;
  if (jdx >= m_num_cols)
  {
    throw std::range_error("Querying out of bounds assignment index");
  }
  return m_assignments[jdx];
}

////////////////////////////////////////////////////////////////////////////////
// private methods
////////////////////////////////////////////////////////////////////////////////
template <int Capacity>
bool hungarian_assigner_c<Capacity>::reduce_rows_and_init_zeros_and_check_result()
{
  // we should already know what the minimum weight is for each row
  // decrement each row by given weight
  auto blk = m_weight_matrix.block(0, 0, m_num_rows, m_num_cols);
  blk.colwise() -=
      m_row_min_weights.segment(0, m_num_rows);
  // don't do subtraction for the zero padded rows
  for (index_t row_idx = index_t(); row_idx < m_num_rows; ++row_idx)
  {
    const index_t col_idx = m_row_min_idx[row_idx];
    if (col_idx != UNASSIGNED)
    {
      m_mark_matrix(row_idx, col_idx) = static_cast<int8_t>(ZERO);
      if (!m_is_col_covered[col_idx])
      {
        // row and column is covered by me
        m_is_col_covered[col_idx] = true;
        // update STAR
        m_mark_matrix(row_idx, col_idx) = static_cast<int8_t>(STARRED);
        // update assignment
        m_assignments[row_idx] = col_idx;
      }
    }
  }
  // find consistent zeros for zero padded rows
  for (index_t idx = m_num_rows; idx < m_num_cols; ++idx)
  {
    for (index_t jdx = index_t(); jdx < m_num_cols; ++jdx)
    {
      if (!(m_is_col_covered[jdx]))
      {
        m_is_col_covered[jdx] = true;
        m_assignments[idx] = jdx;
        m_mark_matrix(idx, jdx) = static_cast<int8_t>(STARRED);
        break;
      }
    }
  }
  // reset row cover here
  m_is_row_covered.fill(false);
  // check if we're done
  return are_all_columns_covered();
}

///
template <int Capacity>
bool hungarian_assigner_c<Capacity>::increment_starred_zeroes_and_check_result(index2_t loc)
{
  // TODO(c.ho) maybe keep augment_path local
  // update path:
  index_t path_size = 1U;
  m_augment_path[0U] = loc;
  // this will run at most N times because each iteration adds a starred zero, of which
  // there are at most N-1 before entering this
  for (index_t idx = index_t(); idx < m_num_cols; ++idx)
  {
    // find star in col
    bool found = false;
    for (index_t jdx = index_t(); jdx < m_num_cols; ++jdx)
    {
      if (m_mark_matrix(jdx, loc.second) == static_cast<int8_t>(STARRED))
      {
        found = true;
        loc.first = jdx;
        m_augment_path[path_size] = loc;
        ++path_size;
        break;
      }
    }
    // if not found, break
    if (!found)
    {
      break;
    }
    // update row, col in path: find prime in row
    found = false;
    for (index_t jdx = index_t(); jdx < m_num_cols; ++jdx)
    {
      if (m_mark_matrix(loc.first, jdx) == static_cast<int8_t>(PRIMED))
      {
        loc.second = jdx;
        m_augment_path[path_size] = loc;
        ++path_size;
        found = true;
        break;
      }
    }
    if (!found)
    {
      // should never hit
      throw std::runtime_error("Guaranteed to find prime in row!");
    }
  }
  // flip rows and columns along path
  for (index_t idx = index_t(); idx < path_size; ++idx)
  {
    const index_t row_idx = m_augment_path[idx].first;
    const index_t col_idx = m_augment_path[idx].second;
    if (m_mark_matrix(row_idx, col_idx) == static_cast<int8_t>(STARRED))
    {
      m_mark_matrix(row_idx, col_idx) = static_cast<int8_t>(ZERO);
    }
    else
    {
      m_mark_matrix(row_idx, col_idx) = static_cast<int8_t>(STARRED);
      m_assignments[row_idx] = col_idx;
    }
  }

  // update column cover according to assignments
  m_is_col_covered.fill(false);
  for (index_t idx = index_t(); idx < m_num_cols; ++idx)
  {
    const index_t assignment = m_assignments[idx];
    if (UNASSIGNED != assignment)
    {
      m_is_col_covered[assignment] = true;
    }
  }
  // reset row cover, array is member so no need for pointer
  m_is_row_covered.fill(false);
  // erase all primes
  for (index_t idx = index_t(); idx < m_num_primed_zeros; ++idx)
  {
    loc = m_primed_zero_locs[idx];
    if (m_mark_matrix(loc.first, loc.second) == static_cast<int8_t>(PRIMED))
    {
      m_mark_matrix(loc.first, loc.second) = static_cast<int8_t>(ZERO);
    }
  }
  m_num_primed_zeros = index_t();

  // check if we're done
  return are_all_columns_covered();
}

///
template <int Capacity>
//lint -e9094 can't hide this further, will get compile error otherwise NOLINT
typename hungarian_assigner_c<Capacity>::index2_t
hungarian_assigner_c<Capacity>::prime_uncovered_zero()
{
  index2_t loc;
  // upper bound number of iterations to N, since entering outer loop is only columns
  // each iteration can swap one col to row, which can happen at most row times, so N
  for (index_t idx = index_t(); idx < m_num_cols; ++idx)
  {
    update_uncovered_rows_and_cols();
    if (!find_uncovered_zero(loc))
    {
      if (!add_new_zero(loc))
      {
        // can't add a new zero. I'm done
        throw no_uncovered_values_c();
      }
    }
    // prime zero and look for star in row
    m_mark_matrix(loc.first, loc.second) = static_cast<int8_t>(PRIMED);
    if (m_num_primed_zeros < Capacity)
    {
      m_primed_zero_locs[m_num_primed_zeros] = loc;
      ++m_num_primed_zeros;
    }
    else
    {
      // should never hit
      throw std::runtime_error("Assertion failed: more primes than number of columns!");
    }
    bool found = false;
    index_t star_col;
    for (index_t jdx = index_t(); jdx < m_num_cols; ++jdx)
    {
      if (m_mark_matrix(loc.first, jdx) == static_cast<int8_t>(STARRED))
      {
        star_col = jdx;
        found = true;
        break;
      }
    }
    // swap column cover for row cover OR done
    if (found)
    {
      m_is_row_covered[loc.first] = true;
      m_is_col_covered[star_col] = false;
    }
    else
    {
      break;
    }
  }
  return loc;
}

///
template <int Capacity>
bool hungarian_assigner_c<Capacity>::add_new_zero(index2_t &loc)
{
  // find minimum nonzero'd value
  float min_val;
  const bool ret = find_minimum_uncovered_value(loc, min_val);
  if (ret)
  {
    // add to covered rows
    for (index_t idx = index_t(); idx < m_num_cols; ++idx)
    {
      if (m_is_row_covered[idx])
      {
        m_weight_matrix.block(idx, index_t(), 1U, m_num_cols) +=
            Eigen::MatrixXf::Constant(1, m_num_cols, min_val);
      }
    }
    // subtract from uncovered columns
    for (index_t idx = index_t(); idx < m_num_cols; ++idx)
    {
      if (!m_is_col_covered[idx])
      {
        m_weight_matrix.block(index_t(), idx, m_num_cols, 1U) -=
            Eigen::MatrixXf::Constant(m_num_cols, 1, min_val);
      }
    }
    // add as a zero
    m_mark_matrix(loc.first, loc.second) = static_cast<int8_t>(ZERO);
  }
  return ret;
}

///
template <int Capacity>
bool hungarian_assigner_c<Capacity>::are_all_columns_covered() const
{
  bool ret = true;
  for (index_t idx = index_t(); idx < m_num_cols; ++idx)
  {
    ret = (m_is_col_covered[idx]) && ret;
  }
  return ret;
}

///
template <int Capacity>
bool hungarian_assigner_c<Capacity>::find_uncovered_zero(index2_t &loc) const
{
  bool found = false;
  for (index_t idx = index_t(); idx < m_num_uncovered_rows; ++idx)
  {
    for (index_t jdx = index_t(); jdx < m_num_uncovered_cols; ++jdx)
    {
      const index_t row_idx = m_uncovered_rows[idx];
      const index_t col_idx = m_uncovered_cols[jdx];
      const int8_t mark = static_cast<int8_t>(m_mark_matrix(row_idx, col_idx));
      if ((mark != static_cast<int8_t>(NO_LINK)) &&
          ((mark == static_cast<int8_t>(ZERO)) ||
           (mark == static_cast<int8_t>(PRIMED))))
      {
        found = true;
        loc.first = row_idx;
        loc.second = col_idx;
        break;
      }
    }
    if (found)
    {
      break;
    }
  }

  return found;
}

///
template <int Capacity>
bool hungarian_assigner_c<Capacity>::find_minimum_uncovered_value(
    index2_t &loc,
    float &min_val) const
{
  // I don't need to update uncovered because this will always be called
  // after find_uncovered_zero()
  // update_uncovered_rows_and_cols();
  bool ret = false;
  min_val = std::numeric_limits<float>::max();
  for (index_t idx = index_t(); idx < m_num_uncovered_rows; ++idx)
  {
    for (index_t jdx = index_t(); jdx < m_num_uncovered_cols; ++jdx)
    {
      const index_t row_idx = m_uncovered_rows[idx];
      const index_t col_idx = m_uncovered_cols[jdx];
      if (m_mark_matrix(row_idx, col_idx) != static_cast<int8_t>(NO_LINK))
      {
        const float val = m_weight_matrix(row_idx, col_idx);
        if (val < min_val)
        {
          ret = true;
          min_val = val;
          loc.first = row_idx;
          loc.second = col_idx;
        }
      }
    }
  }
  return ret;
}

///
/// TODO(c.ho) might be able to update this incrementally
template <int Capacity>
void hungarian_assigner_c<Capacity>::update_uncovered_rows_and_cols()
{
  m_num_uncovered_rows = index_t();
  m_num_uncovered_cols = index_t();
  for (index_t idx = index_t(); idx < m_num_cols; ++idx)
  {
    if (!(m_is_row_covered[idx]))
    {
      m_uncovered_rows[m_num_uncovered_rows] = idx;
      ++m_num_uncovered_rows;
    }
  }
  for (index_t idx = index_t(); idx < m_num_cols; ++idx)
  {
    if (!(m_is_col_covered[idx]))
    {
      m_uncovered_cols[m_num_uncovered_cols] = idx;
      ++m_num_uncovered_cols;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// precompile
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<16U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<32U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<64U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<96U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<128U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<192U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<256U>;
template class HUNGARIAN_ASSIGNER_PUBLIC hungarian_assigner_c<512U>;

} // namespace hungarian_assigner
} // namespace fusion
} // namespace autoware
