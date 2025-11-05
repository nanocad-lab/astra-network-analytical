/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/Type.h"
#include "congestion_aware/BasicTopology.h"

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * Implements a 2DTorus topology.
 *
 * 2DTorus(8) example:
 *    
 *   0 - 1 - 2 - 3
 *   |   |   |   | 
 *   7 - 6 - 5 - 4
 *   
 *
 * The number of NPUs and devices are both 8.
 *
 * If ring is uni-directional, then each chunk can flow through:
 * 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 0
 *
 * If the ring is bi-directional, then each chunk can flow through:
 * 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 0
 * 0 <- 1 <- 2 <- 3 <- 4 <- 5 <- 6 <- 7 <- 0
 */
class Mesh2D final : public BasicTopology {
 public:
  /**
   * Constructor.
   *
   * @param npus_count number of npus in the torus (dim × dim)
   * @param bandwidth  link bandwidth
   * @param latency    link latency
   * @param bidirectional true if torus is bidirectional
   * @param is_multi_dim  true if part of multidimensional topology
   * @param faulty_links  list of faulty links as tuples (src, dst, weight)
   */
  Mesh2D(int npus_count,
          Bandwidth bandwidth,
          Latency latency,
          bool bidirectional = true,
          bool is_multi_dim = false,
          const std::vector<std::tuple<int, int, double>>& faulty_links = {}) noexcept;

  /**
   * Alternate constructor for convenience (used by Helper.cpp)
   */
  Mesh2D(int npus_count,
          Bandwidth bandwidth,
          Latency latency,
          const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
      : Mesh2D(npus_count, bandwidth, latency, true, false, faulty_links) {}

    /**
     * Implementation of route function in Topology.
     */
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

    /**
     * Get connection policies of the ring topology.
     * Each connection policy is represented as a pair of (src, dest) device ids.
     * For a 4-node ring, the connection policies are:
     * - if bidirectional: (0,1), (1,2), (2,3), (3,0), (1,0), (2,1), (3,2), (0,3)
     *
     * @return list of connection policies
     */
    [[nodiscard]] std::vector<ConnectionPolicy> get_connection_policies() const noexcept override;

  private:
    /// true if the ring is bidirectional, false otherwise
    double fault_derate(int src, int dst) const;

    bool bidirectional;
    std::vector<std::tuple<int, int, double>> faulty_links;
};

}  // namespace NetworkAnalyticalCongestionAware
