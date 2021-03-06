// Copyright 2017-2020 the Autoware Foundation, Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \file
/// \brief This file defines the ray ground filter algorithm

#ifndef RAY_GROUND_CLASSIFIER__RAY_GROUND_CLASSIFIER_HPP_
#define RAY_GROUND_CLASSIFIER__RAY_GROUND_CLASSIFIER_HPP_

#include <ray_ground_classifier/algorithm.hpp>
#include <ray_ground_classifier/types.hpp>
#include <ray_ground_classifier/ray_ground_point_classifier.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

using autoware::common::types::PointPtrBlock;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

/// \brief Given a ray, partitions into ground and nonground points
class RayGroundClassifier
{
public:
  /// \brief Default constructor
  /// \param[in] cfg Ray ground filter configuration parameters
  explicit RayGroundClassifier(const Config & cfg);
  /// \brief Copy constructor
  explicit RayGroundClassifier(const RayGroundClassifier & original);

  // ********************* Partition m_sort_array (ray member stored in this class) into blocks *********************
  // TODO adi this is all unused!  the entire part of using an internal ray is unused!!
  // The only advantage of using the stored ray is that it will sort it using Autoware's Quicksort
/*
  /// \brief Inserts a point into the internal datastructure, intended to be used with filter.
  ///        Points inserted sequentially should have the semantics of being a part of the same ray
  /// \param[in] pt The point to insert
  /// \throw std::runtime_error If the internal datastructure is full
  void insert(const PointXYZIF * pt);
  /// \brief Inserts a point into the internal datastructure, intended to be used with filter.
  ///        Points inserted sequentially should have the semantics of being a part of the same ray
  /// \param[in] pt The point to insert
  /// \throw std::runtime_error If the internal datastructure is full
  void insert(const PointXYZIFR & pt);

  /// \brief check if the provided output blocks can definitely fit the result of a partition
  /// \param[in] ground_block The block that should hold the resulting ground points
  /// \param[in] nonground_block The block that should hold the resulting nonground points
  /// \return false if provided output blocks could not fit ray
  bool8_t can_fit_result(
    const PointPtrBlock & ground_block,
    const PointPtrBlock & nonground_block) const;

  /// \brief Partitions points from a single ray that were inserted using insert as ground or
  ///        nonground
  /// \param[inout] ground_block Gets appended with ground partition of the ray. The size parameter
  ///                            is honored and modified.
  /// \param[inout] nonground_block Gets appended with nonground partition of the ray. The size
  ///                               parameter is honored and modified.
  /// \param[in] presorted If true, does not do a sorting step. Should only be true if the ray
  ///                      points are inserted in order of increasing radial distance, and in
  ///                      order of increasing height in the case of a tie
  /// \throw std::runtime_error If blocks cannot fit result
  void partition(
    PointPtrBlock & ground_block,
    PointPtrBlock & nonground_block,
    const bool8_t presorted);

  /// \brief Partitions points from raw block into ground points and nonground points
  /// \param[in] raw_block A full point block of one or more rays. Rays are assumed to be contiguous
  ///                      in the block, e.g. a ray label sequence of 1-1-2-2-1-1 is viewed as 3
  ///                      rays
  /// \param[out] ground_block Gets filled with ground partition of the ray. The size parameter
  ///                          is overwritten
  /// \param[out] nonground_block Gets filled with nonground partition of the ray. The size
  ///                             parameter is overwritten
  void structured_partition(
    const PointPtrBlock & raw_block,
    PointPtrBlock & ground_block,
    PointPtrBlock & nonground_block);
*/
  // ******************************** Partition a given ray into blocks ********************************
  /// \brief check if the provided output blocks can definitely fit the result of a partition
  /// \param[in] ground_block The block that should hold the resulting ground points
  /// \param[in] nonground_block The block that should hold the resulting nonground points
  /// \param[in] ray The ray to be partitioned
  /// \return false if provided output blocks could not fit ray
  bool8_t can_fit_result(const Ray & ray, const PointPtrBlock & ground_block, const PointPtrBlock & nonground_block) const;

  /// \brief Logic for making labels consistent throughout a single ray
  /// \param[in] ray Datastructure which holds a sorted ray
  /// \param[inout] ground_block Gets appended with ground points, size is respected and modified
  /// \param[inout] nonground_block Gets appended with nonground points, size is respected and
  ///                               modified
  /// \throw std::runtime_error If blocks cannot fit result
  void partition(
    const Ray & ray,
    PointPtrBlock & ground_block,
    PointPtrBlock & nonground_block);

private:
  /// \brief Inserts point to a block
  RAY_GROUND_CLASSIFIER_LOCAL static void insert(PointPtrBlock & block, const PointXYZIF * pt);

  /// actual ground filter
  RayGroundPointClassifier m_point_classifier;

  // ********************* Partition m_sort_array (ray member stored in this class) into blocks *********************
  /*
  /// \brief Sorts internally stored ray
  RAY_GROUND_CLASSIFIER_LOCAL void sort_ray();

  /// \brief Logic for making labels consistent throughout a single ray, uses internal ray
  /// \param[inout] ground_block Gets appended with ground points, size is respected and modified
  /// \param[inout] nonground_block Gets appended with nonground points, size is respected and
  ///                               modified
  RAY_GROUND_CLASSIFIER_LOCAL void segment_ray(
    PointPtrBlock & ground_block,
    PointPtrBlock & nonground_block);

  /// worker array
  Ray m_sort_array;

  /// Iterative quick sorter for rays
  autoware::common::algorithm::QuickSorter<Ray> m_ray_sorter;
*/
};  // class RayGroundClassifier
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // RAY_GROUND_CLASSIFIER__RAY_GROUND_CLASSIFIER_HPP_
