/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/FullyConnected.h"
#include <cassert>

using namespace NetworkAnalyticalCongestionAware;

FullyConnected::FullyConnected(const int npus_count,
           const Bandwidth bandwidth,
           const Latency latency,
           const bool bidirectional,
           const bool is_multi_dim,
           const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
    : bidirectional(bidirectional),
      BasicTopology(npus_count, npus_count, bandwidth, latency, is_multi_dim),
      faulty_links(faulty_links) {   // initialize faulty links
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    FullyConnected::basic_topology_type = TopologyBuildingBlock::FullyConnected;


    // fully-connect every src-dest pairs
    if (!is_multi_dim) {
        for (auto src = 0; src < npus_count; src++) {
            for (auto dest = 0; dest < npus_count; dest++) {
                if (src != dest) {
                    if(fault_derate(src, dest) != 0)
                        connect(src, dest, bandwidth * fault_derate(src, dest), latency, false);
                    else
                        connect(src, dest, bandwidth, latency, false);  //might be removable
                }
            }
        }
    }
}

Route FullyConnected::route(const DeviceId src, const DeviceId dest) const noexcept {
    // assert npus are in valid range
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    // construct route
    // directly connected
    auto route = Route();
    route.push_back(devices[src]);
    route.push_back(devices[dest]);

    return route;
}

std::vector<ConnectionPolicy> FullyConnected::get_connection_policies() const noexcept {
    std::vector<ConnectionPolicy> policies;

    for (auto src = 0; src < npus_count; src++) {
        for (auto dest = 0; dest < npus_count; dest++) {
            if (src != dest) {
                policies.emplace_back(src, dest);
            }
        }
    }

    return policies;
}

double FullyConnected::fault_derate(int src, int dst) const{
    for (const auto& link : faulty_links) {
        int a = std::get<0>(link);
        int b = std::get<1>(link);
        double health = std::get<2>(link);

        // If this link exists and health != 0.0 → it's soft fault
        if ((a == src && b == dst) || (a == dst && b == src)) {
            return health;
        }
    }
    return 1.0;
}

