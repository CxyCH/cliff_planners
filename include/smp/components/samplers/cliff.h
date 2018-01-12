#pragma once
/*! \file components/samplers/cliff_sampler.h
  \brief The CLiFFMap Sampler based on rejection sampling.

  Rejection sampling.
*/

#include <memory>
#include <smp/common/region.h>
#include <smp/components/samplers/base.h>

#include <cliffmap_ros/cliffmap.hpp>

namespace smp {

//! Implements the sampler components that relies on uniform sampling.
/*!
  A sampler component that implements uniform sampling.

  \ingroup samplers
*/
template <class typeparams, int NUM_DIMENSIONS>
class sampler_cliff : public sampler_base<typeparams> {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

  typedef region<NUM_DIMENSIONS> region_t;

  region_t support;

  cliffmap_ros::CLiFFMapConstPtr cliffmap;

  unsigned int rejections;

public:
  sampler_cliff();
  sampler_cliff(const cliffmap_ros::CLiFFMapConstPtr &map);
  ~sampler_cliff();

  int sm_update_insert_vertex(vertex_t *vertex_in);

  int sm_update_insert_edge(edge_t *edge_in);

  int sm_update_delete_vertex(vertex_t *vertex_in);

  int sm_update_delete_edge(edge_t *edge_in);

  int sample(state_t **state_sample_out);
  
  int sampleV2(state_t **state_sample_out);

  unsigned int get_total_rejections();

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
}
