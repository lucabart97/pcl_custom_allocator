/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl/common/common.h>

#include <ostream>
#include <pcl/tk_allocator.h>
#include <vector>

namespace pcl {

/** Utility class interface which is used for creating and evaluating features. */
template <class FeatureType, class DataSet, class ExampleIndex>
class PCL_EXPORTS FeatureHandler {
public:
  /** Destructor. */
  virtual ~FeatureHandler(){};

  /** Creates random features.
   *
   * \param[in] num_of_features the number of random features to create
   * \param[out] features the destination for the created features
   */
  virtual void
  createRandomFeatures(const std::size_t num_of_features,
                       std::vector<FeatureType>& features) = 0;

  /** Evaluates a feature on the specified data.
   *
   * \param[in] feature the features to evaluate
   * \param[in] data_set the data set on which the feature is evaluated
   * \param[in] examples the examples which specify on which parts of the data set the
   *            feature is evaluated
   * \param[out] results the destination for the results of the feature evaluation
   * \param[out] flags flags that are supplied together with the
   *             results
   */
  virtual void
  evaluateFeature(const FeatureType& feature,
                  DataSet& data_set,
                  std::vector<ExampleIndex>& examples,
                  std::vector<float>& results,
                  std::vector<unsigned char>& flags) const = 0;

  /** Evaluates a feature on the specified data.
   *
   * \param[in] feature the features to evaluate
   * \param[in] data_set the data set on which the feature is evaluated
   * \param[in] example the examples which specify on which parts of the data set the
   *            feature is evaluated
   * \param[out] result the destination for the results of the feature evaluation
   * \param[out] flag flags that are supplied together with the results
   */
  virtual void
  evaluateFeature(const FeatureType& feature,
                  DataSet& data_set,
                  const ExampleIndex& example,
                  float& result,
                  unsigned char& flag) const = 0;

  /** Generates evaluation code for the specified feature and writes it to the specified
   *  stream
   *
   * \param[in] feature the feature for which code is generated
   * \param[out] stream the destination for the code
   */
  virtual void
  generateCodeForEvaluation(const FeatureType& feature,
                            ::std::ostream& stream) const = 0;
};

} // namespace pcl
