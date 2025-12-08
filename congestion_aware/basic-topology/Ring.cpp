/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Ring.h"
#include <cassert>

using namespace NetworkAnalyticalCongestionAware;

Ring::Ring(const int npus_count,
           const Bandwidth bandwidth,
           const Latency latency,
           const bool bidirectional,
           const bool is_multi_dim,
           const int non_recursive_topo,
           const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
    : bidirectional(bidirectional),
      BasicTopology(npus_count, npus_count, bandwidth, latency, is_multi_dim),
      non_recursive_topo(non_recursive_topo),
      faulty_links(faulty_links) 
{   // initialize faulty links
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    Ring::basic_topology_type = TopologyBuildingBlock::Ring;

    if (!is_multi_dim) {
        double scale_factor = 2.0;
        //connect npus in a ring
        for (auto i = 0; i < npus_count - 1; i++) {
            if(fault_derate(i, i+1) != 0){
                    if (fault_derate(i, i+1) != 1)
                        std::cout<<"HERE IS FAULTY_LINK"<<std::endl;
                    connect(i, i+1, bandwidth * fault_derate(i, i+1) * scale_factor, latency, bidirectional);
            }
                else
                    connect(i, i+1, bandwidth * scale_factor, latency, bidirectional);  //might be removable
        }
        if(fault_derate(npus_count-1, 0) != 0)
                connect(npus_count-1, 0, bandwidth * fault_derate(npus_count-1, 0) * scale_factor, latency, bidirectional);
            else
                connect(npus_count-1, 0, bandwidth * scale_factor, latency, bidirectional);  //might be removable
    }

    // this also works
    // std::vector<ConnectionPolicy> policies = get_connection_policies();
    // for (const auto& policy : policies) {
    //     connect(policy.src, policy.dst, bandwidth, latency, /*bidirectional=*/false);
    // }
}

Route Ring::route(DeviceId src, DeviceId dest) const noexcept {
    // assert npus are in valid range
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    // construct empty route
    auto route = Route();

    auto step = 1;  // default direction: clockwise
    if (bidirectional) {
        // check whether going anticlockwise is shorter
        auto clockwise_dist = dest - src;
        if (clockwise_dist < 0) {
            clockwise_dist += npus_count;
        }
        const auto anticlockwise_dist = npus_count - clockwise_dist;

        if (anticlockwise_dist < clockwise_dist) {
            // traverse the ring anticlockwise
            step = -1;
        }
    }

    // construct the route
    auto current = src;
    while (current != dest) {
        // traverse the ring until reaches dest
        route.push_back(devices.at(current));
        current = (current + step);

        // wrap around
        if (current < 0) {
            current += npus_count;
        } else if (current >= npus_count) {
            current -= npus_count;
        }
    }

    // arrives at dest
    route.push_back(devices.at(dest));

    // return the constructed route
    return route;
}

std::vector<ConnectionPolicy> Ring::get_connection_policies() const noexcept {
    std::vector<ConnectionPolicy> policies;

    for (int i = 0; i < npus_count; i++) {
        policies.emplace_back(i, (i + 1) % npus_count);
    }

    if (bidirectional) {
        for (int i = 0; i < npus_count; i++) {
            policies.emplace_back((i + 1) % npus_count, i);
        }
    }

    return policies;
}

double Ring::fault_derate(int src, int dst) const {
    for (const auto& link : faulty_links) {
        int a = std::get<0>(link);
        int b = std::get<1>(link);
        double health = std::get<2>(link);

        if ((a == src && b == dst) || (a == dst && b == src)) {
            return health;
        }
    }
    return 1.0;  // ✅ default: healthy link
}

