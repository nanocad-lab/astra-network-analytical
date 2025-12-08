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
 * Implements a ring topology.
 *
 * Ring(8) example:
 * 0 - 1 - 2 - 3
 * |           |
 * 7 - 6 - 5 - 4
 *
 * The number of NPUs and devices are both 8.
 */
class Ring final : public BasicTopology {
 public:
    /**
     * Constructor.
     *
     * @param npus_count number of npus in a ring
     * @param bandwidth bandwidth of link
     * @param latency latency of link
     * @param bidirectional true if ring is bidirectional
     * @param is_multi_dim  true if part of multidimensional topology
     * @param non_recursive_topo
     * @param faulty_links  list of faulty links as tuples (src, dst, weight)
     */
    Ring(int npus_count,
         Bandwidth bandwidth,
         Latency latency,
         bool bidirectional = true,
         bool is_multi_dim = false,
         int non_recursive_topo = 1,
         const std::vector<std::tuple<int, int, double>>& faulty_links = {}) noexcept;

    /**
     * Alternate constructor for convenience
     */
    Ring(int npus_count,
         Bandwidth bandwidth,
         Latency latency,
         const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
        : Ring(npus_count, bandwidth, latency, true, false, 1, faulty_links) {}

    /**
     * Implementation of route function in Topology.
     */
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

    /**
     * Get connection policies of the ring topology.
     */
    [[nodiscard]] std::vector<ConnectionPolicy> get_connection_policies() const noexcept override;

 private:
    double fault_derate(int src, int dst) const;

    bool bidirectional;
    std::vector<std::tuple<int, int, double>> faulty_links;
    int non_recursive_topo;
};

}  // namespace NetworkAnalyticalCongestionAware
