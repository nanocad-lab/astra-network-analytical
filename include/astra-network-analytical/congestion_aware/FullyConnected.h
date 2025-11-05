/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/Type.h"
#include "congestion_aware/BasicTopology.h"

namespace NetworkAnalyticalCongestionAware {

/**
 * Implements a FullyConnected topology.
 *
 * FullyConnected(4) example:
 *    0
 *  / | \
 * 3 -|- 1
 *  \ | /
 *   2
 *
 * Therefore, the number of NPUs and devices are both 4.
 *
 * Arbitrary send between two pair of NPUs will take 1 hop.
 */
class FullyConnected final : public BasicTopology {
 public:
    /**
     * Constructor.
     *
     * @param npus_count number of npus 
     * @param bandwidth bandwidth of link
     * @param latency latency of link
     * @param bidirectional true if is bidirectional
     * @param is_multi_dim  true if part of multidimensional topology
     * @param faulty_links  list of faulty links as tuples (src, dst, weight)
     */
    FullyConnected(int npus_count,
         Bandwidth bandwidth,
         Latency latency,
         bool bidirectional = true,
         bool is_multi_dim = false,
         const std::vector<std::tuple<int, int, double>>& faulty_links = {}) noexcept;

    /**
     * Alternate constructor for convenience
     */
    FullyConnected(int npus_count,
         Bandwidth bandwidth,
         Latency latency,
         const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
        : FullyConnected(npus_count, bandwidth, latency, true, false, faulty_links) {}
    /**
     * Implementation of route function in Topology.
     */
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

    /**
     * Get connection policies
     * Each connection policy is represented as a pair of (src, dest) device ids.
     * For a 4-node topology, the connection policies are:
     * - (0,1), (0,2), (0,3), (1,0), (1,2), (1,3), (2,0), (2,1), (2,3), (3,0), (3,1), (3,2)
     *
     * @return list of connection policies
     */
    [[nodiscard]] std::vector<ConnectionPolicy> get_connection_policies() const noexcept override;

    double fault_derate(int src, int dst) const;

    bool bidirectional;
    std::vector<std::tuple<int, int, double>> faulty_links;
};

}  // namespace NetworkAnalyticalCongestionAware
