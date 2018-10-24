#pragma once
/*! \file components/samplers/cliff_sampler.h
  \brief The CLiFFMap Sampler based on rejection sampling.

  Rejection sampling.
*/

#include <memory>
#include <smp/region.hpp>
#include <smp/samplers/base.hpp>

#include <cliffmap_ros/cliffmap.hpp>

namespace smp {
namespace samplers {

//! Implements the sampler components that relies on uniform sampling.
/*!
  A sampler component that implements uniform sampling.

  \ingroup samplers
*/
template <class State, int NUM_DIMENSIONS> class CLiFF : public Base<State> {

  Region<NUM_DIMENSIONS> support;
  Region<NUM_DIMENSIONS> goal_region;
  double goal_bias{0.0};

  cliffmap_ros::CLiFFMapConstPtr cliffmap;

  unsigned int rejections;

  /**
   * When this parameter is set, only a goal-biased uniform sampling is done.
   */
  bool no_cliff_sampling{false};

  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> uniform_distribution;

public:
  CLiFF();
  CLiFF(const cliffmap_ros::CLiFFMapConstPtr &map);
  ~CLiFF();

  int sample(State **state_sample_out);

  int sampleV2(State **state_sample_out);

  unsigned int get_total_rejections();

  inline void dontUseCLiFFSampling() { no_cliff_sampling = true; }

  int set_goal_bias(double bias, const Region<NUM_DIMENSIONS> &goal_region);

  inline void reset_rejections() { rejections = 0; }

  /**
   * \brief Sets the dimensions and position of the rectangular bounding box of
   *        the support.
   *
   * Uniform distribution only makes sense in a bounded support, which can be
   * set
   * using this function. This sampler function only draws samples from a
   * rectangular
   * box in the Euclidean space with dimensions NUM_DIMENSIONS, which is a
   * template
   * parameter to the uniform sampler class. If the support variable is not set,
   * i.e.,
   * this function is never called, then the support is initialized to the unit
   * cube
   * centered at the origin by default.
   *
   * @param support_in New support for the uniform sampling distribution.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  int set_support(const cliffmap_ros::CLiFFMapConstPtr &map);
};
} // namespace samplers
} // namespace smp

#include <smp/samplers/cliff_impl.hpp>
