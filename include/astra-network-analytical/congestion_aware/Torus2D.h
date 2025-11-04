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
 *    ________________
 *   |_0 - 1 - 2 - 3_|
 *     |   |   |   | 
 *    _7 - 6 - 5 - 4_
 *   |_______________|
 */
class Torus2D final : public BasicTopology {
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
  Torus2D(int npus_count,
          Bandwidth bandwidth,
          Latency latency,
          bool bidirectional = true,
          bool is_multi_dim = false,
          const std::vector<std::tuple<int, int, double>>& faulty_links = {}) noexcept;

  /**
   * Alternate constructor for convenience (used by Helper.cpp)
   */
  Torus2D(int npus_count,
          Bandwidth bandwidth,
          Latency latency,
          const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
      : Torus2D(npus_count, bandwidth, latency, true, false, faulty_links) {}

  /**
   * Implementation of route function in Topology.
   */
  [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

  /**
   * Get connection policies of the torus topology.
   */
  [[nodiscard]] std::vector<ConnectionPolicy> get_connection_policies() const noexcept override;

 private:
  //bool is_down(int src, int dst) const;
  double fault_derate(int src, int dst) const;

  bool bidirectional;
  std::vector<std::tuple<int, int, double>> faulty_links;
};

}  // namespace NetworkAnalyticalCongestionAware
