/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/MultiDimTopology.h"
#include "congestion_aware/Helper.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <numeric>
#include <utility>

namespace NetworkAnalyticalCongestionAware {

MultiDimTopology::MultiDimTopology(const std::vector<std::tuple<int, int, double>> faulty_links, const std::vector<int> non_recursive_topo) noexcept 
: Topology() , faulty_links{faulty_links}, m_non_recursive_topo{non_recursive_topo}{
    // initialize values
    m_topology_per_dim.clear();
    npus_count_per_dim = {};

    // initialize topology shape
    this->npus_count = 1;
    this->devices_count = 1;
    this->dims_count = 0;
    this->m_cluster = (m_non_recursive_topo.at(non_recursive_topo.size()-1) == 1);
}

// Route MultiDimTopology::route(DeviceId src, DeviceId dest) const noexcept {
//     // build up diemnsion
//     std::vector<int> routing_dimensions;
//     for (int dim_to_transfer = dims_count - 1; dim_to_transfer >= 0; dim_to_transfer--) {
//         routing_dimensions.push_back(dim_to_transfer);
//     }

//     // call
//     return routeHelper(src, dest, routing_dimensions);

// }



Route MultiDimTopology::route(DeviceId src, DeviceId dest) const noexcept {
    if (m_cluster) {
        return routeCluster(src, dest);
    }
    else
    {
        return routeNormal(src, dest);
    }
}

Route MultiDimTopology::routeCluster(DeviceId src, DeviceId dest) const noexcept {
    // build up dimension
    std::vector<int> normal_routing_dimensions; // right to left, top to bottom
    for (int dim_to_transfer = dims_count - 1; dim_to_transfer >= 0; dim_to_transfer--) {
        normal_routing_dimensions.push_back(dim_to_transfer);
    }

    std::vector<int> reverse_routing_dimensions; // left to right, bottom to top
    for (int dim_to_transfer = 0; dim_to_transfer < dims_count; dim_to_transfer++) {
        reverse_routing_dimensions.push_back(dim_to_transfer);
    }

    MultiDimAddress src_addr = translate_address(src);
    // address [0 ... 0 Q ... Z] when src is [A ... P Q ... Z]
    //std::vector<int> non_recursive_topo = {0, 0, 1};
    MultiDimAddress src_cluster_agent_addr{src_addr};
    for (int dim = 0; dim < dims_count; dim++)
    {
        if (m_non_recursive_topo.at(dim) == 0) {
            src_cluster_agent_addr.at(dim) = 0;
        }
        else {
            break;
        }
    }
    DeviceId src_cluster_agent_id = translate_address_back(src_cluster_agent_addr);

    // address [0 ... 0 Z] when src is [A ... P Q ... Z]
    auto top_cluster_agent_addr = MultiDimAddress();
    for (int dim = 0; dim < dims_count; dim++)
    {
        top_cluster_agent_addr.push_back(0);
    }
    assert(top_cluster_agent_addr.size() == dims_count);
    top_cluster_agent_addr.at(dims_count - 1) = src_addr.at(dims_count - 1);
    DeviceId top_cluster_agent_id = translate_address_back(top_cluster_agent_addr);
   

    // do two routing and connect them together
    Route route_to_agent, cluster_route, agent_to_dest;
    if (src != src_cluster_agent_id) {
        route_to_agent = routeHelper(src, src_cluster_agent_id, normal_routing_dimensions);
    }
    if (src_cluster_agent_id != top_cluster_agent_id) {
        cluster_route = routeHelper(src_cluster_agent_id, top_cluster_agent_id, reverse_routing_dimensions);
    }
    if (top_cluster_agent_id != dest) {
        agent_to_dest = routeHelper(top_cluster_agent_id, dest, normal_routing_dimensions);
    }

    // concatenate route while removing duplicate
    auto final_route = route_to_agent;
    if (!cluster_route.empty())
    {
        if (!final_route.empty())
        {
            cluster_route.pop_front();
        }
        final_route.splice(final_route.end(), cluster_route);
    }
    if (!agent_to_dest.empty())
    {
        if (!final_route.empty())
        {
            agent_to_dest.pop_front();
        }
        final_route.splice(final_route.end(), agent_to_dest);
    }
    return final_route;
}

Route MultiDimTopology::routeNormal(DeviceId src, DeviceId dest) const noexcept {
        std::vector<int> routing_dimensions;
    for (int dim_to_transfer = dims_count - 1; dim_to_transfer >= 0; dim_to_transfer--) {
        routing_dimensions.push_back(dim_to_transfer);
    }

    // call
    return routeHelper(src, dest, routing_dimensions);
}


Route MultiDimTopology::routeHelper(DeviceId src, DeviceId dest, const std::vector<int>& routing_dimensions) const noexcept {
    //std::cout << "[DEBUG] Beginning of the function - source and destination: " << src << dest << std::endl;

    // // assert npus are in valid range
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);
    // assert(src == translate_address_back(translate_address(src)));

    // translate src and dest to multi-dim address
    const auto src_address = translate_address(src);
    const auto dest_address = translate_address(dest);

    // // construct empty route
    auto route = Route();
    MultiDimAddress last_dest_address{src_address};
    DeviceId last_dest{src};

    for (const auto dim_to_transfer : routing_dimensions) {
        // if index in the current dimension is the same, skip
        if (src_address.at(dim_to_transfer) != dest_address.at(dim_to_transfer)) {
           
            // find destination in next dimension
            MultiDimAddress next_dim_dest_address{last_dest_address};
            next_dim_dest_address.at(dim_to_transfer) = dest_address.at(dim_to_transfer);

            // create internal route from current dimension
            auto* const topology = m_topology_per_dim.at(dim_to_transfer).get();
            auto internal_route =
                topology->route(last_dest_address.at(dim_to_transfer),
                                next_dim_dest_address.at(dim_to_transfer));  // route on that dimension
            auto route_in_dim = Route();

            // translate internal route device id to global device IDs and push to route in this dimension
            for (const auto& internal_device_number : internal_route) {
                // translate to global device ID
                MultiDimAddress internal_device_address{last_dest_address};
                internal_device_address.at(dim_to_transfer) = internal_device_number->get_id();

                // check if switch
                DeviceId global_device_id = -1;
                if (is_switch(internal_device_address)) {
                    // use translation unit for switch
                    if (m_switch_translation_unit.has_value()) {
                        global_device_id =
                            m_switch_translation_unit.value().translate_address_to_id(internal_device_address);
                    } else {
                        std::cerr << "[Error] (network/analytical/congestion_aware/MultiDimTopology): "
                                  << "SwitchTranslationUnit is not initialized." << std::endl;
                        std::exit(-1);
                    }
                } else {
                    // normal device
                    global_device_id = translate_address_back(internal_device_address);
                }
                assert(0 <= global_device_id && global_device_id < devices_count);

                // push to route in this dimension
                route_in_dim.push_back(devices.at(global_device_id));
            }

            // we have finished the route_in_dim
            std::vector<DeviceId> route_id;
            for (const auto device : route_in_dim) {
                route_id.push_back(device->get_id());
            }
            //std::cout << "[DEBUG] Route in dimension before fault check: ";
            //for (auto id : route_id) std::cout << id << " ";
            //std::cout << std::endl;

            bool meet_fault = false;
            for (int i = 0; i < (int)route_id.size() - 1; i++) {
                double derate = fault_derate(route_id.at(i), route_id.at(i + 1));
                //std::cout << "[DEBUG] Checking link (" << route_id.at(i)
                //        << " -> " << route_id.at(i + 1)
                //        << "), derate = " << derate << std::endl;

                if (derate == 0.0) {
                    //std::cout << "[DEBUG] Fault detected between "
                    //        << route_id.at(i) << " and " << route_id.at(i + 1) << std::endl;

                    auto begin_itr = route_in_dim.begin();
                    //auto end_itr = route_in_dim.begin();
                    std::advance(begin_itr, i+1);
                    //std::advance(end_itr, route_in_dim.size()-1);
                    route_in_dim.erase(begin_itr, route_in_dim.end());
                    route_id.erase(route_id.begin()+i+1, route_id.end());
                    meet_fault = true;

                    //std::cout << "[DEBUG] Truncated route_in_dim after fault at position " << i
                    //        << ". Remaining: ";
                    //for (auto d : route_in_dim)
                    //    std::cout << d->get_id() << " ";
                    //std::cout << std::endl;

                    break;
                }
            }

            // Remove duplicate at the junction of segments
            if (!route.empty() && !route_in_dim.empty()) {
                //std::cout << "[DEBUG] Removing duplicate junction node "
                //        << route_in_dim.front()->get_id() << std::endl;
                route_in_dim.pop_front();
            }

            // Append to total routing
            if (!route_in_dim.empty()) {
                //std::cout << "[DEBUG] Appending route_in_dim to main route. route_in_dim size = "
                //        << route_in_dim.size() << std::endl;
            }
            route.splice(route.end(), route_in_dim);

            if (meet_fault) {
                int last_id = route_id.back(); // global id before fault
                //std::cout << "[DEBUG] Fault met. last_id = " << last_id << std::endl;

                const auto last_device_addr = translate_address(last_id);
                //std::cout << "[DEBUG] last_device_addr: ";
                //for (auto v : last_device_addr) std::cout << v << " ";
                //std::cout << std::endl;

                auto new_dest_addr{last_device_addr};
                int next_dim = (dim_to_transfer + 1) % new_dest_addr.size();
                new_dest_addr.at(next_dim) =
                    (new_dest_addr.at(next_dim) + 1) % npus_count_per_dim.at(next_dim);

                //std::cout << "[DEBUG] Adjusted new_dest_addr after fault: ";
                //for (auto v : new_dest_addr) std::cout << v << " ";
                //std::cout << std::endl;
                
                //std::cout << "[DEBUG] destination value: " << dest << std::endl;
                auto new_dest = translate_address_back(new_dest_addr);
                //std::cout << "[DEBUG] New destination after fault reroute: " << new_dest << std::endl;

                // use reverse order
                std::vector<int> new_routing_dimension{routing_dimensions};
                int swapped_dim = (dim_to_transfer == 0) ? new_routing_dimension.size() - 1 : dim_to_transfer - 1;
                std::swap(new_routing_dimension.at(dim_to_transfer), new_routing_dimension.at(swapped_dim));

                // find new route
                auto new_route = this->routeHelper(new_dest, dest, new_routing_dimension);
                //std::cout << "[DEBUG] Reroute path after fault: ";
                //for (auto d : new_route)
                //    std::cout << d->get_id() << " ";
                //std::cout << std::endl;

                route.splice(route.end(), new_route); // apppend new path
                return route;
            }

            // update last dest
            last_dest_address = next_dim_dest_address;
            last_dest = translate_address_back(last_dest_address);
        }
    }

    assert(route.front()->get_id() == src && route.back()->get_id() == dest);
    return route;
}

void MultiDimTopology::append_dimension(std::unique_ptr<BasicTopology> topology) noexcept {
    // increment dims_count
    this->dims_count++;

    // increase npus_count
    const auto topology_size = topology->get_npus_count();
    this->npus_count *= topology_size;

    // increase device count
    this->devices_count *= topology->get_devices_count();

    // append bandwidth
    const auto bandwidth = topology->get_bandwidth_per_dim().at(0);
    this->bandwidth_per_dim.push_back(bandwidth);

    // push back topology and npus_count
    assert(topology->get_basic_topology_type() != TopologyBuildingBlock::Undefined);
    m_topology_per_dim.push_back(std::move(topology));
    this->npus_count_per_dim.push_back(topology_size);
}

void MultiDimTopology::make_connections() noexcept {
    if (!m_switch_translation_unit.has_value()) {
        std::cerr << "[Error] (network/analytical/congestion_aware/MultiDimTopology): "
                  << "SwitchTranslationUnit is not initialized." << std::endl;
        std::exit(-1);
    }

    for (int dim = 0; dim < dims_count; dim++) {
        // intra-dim connections
        const auto topology = m_topology_per_dim.at(dim).get();
        const auto policies = topology->get_connection_policies();
        assert(policies.size() != 0);
        bool non_recursive = false;
        std::vector<std::pair<MultiDimAddress, MultiDimAddress>> address_pairs;

        for (const auto& policy : policies) {
            if (!non_recursive) {
                address_pairs = generateAddressPairs(npus_count_per_dim, policy, dim);
            }

            // --- RECURSIVE MODE: only first nodes of lower dimensions ---
            else {
                address_pairs = generateAddressPairs_only_first_nodes(npus_count_per_dim, policy, dim);
            }
            //std::vector<std::pair<MultiDimAddress, MultiDimAddress>> address_pairs =
            //    generateAddressPairs(npus_count_per_dim, policy, dim);
            for (const auto& address_pair : address_pairs) {
                // translate to device ID, depending on switch or not

                const auto src = is_switch(address_pair.first)
                                     ? m_switch_translation_unit.value().translate_address_to_id(address_pair.first)
                                     : translate_address_back(address_pair.first);
                const auto dest = is_switch(address_pair.second)
                                      ? m_switch_translation_unit.value().translate_address_to_id(address_pair.second)
                                      : translate_address_back(address_pair.second);
                assert(0 <= src && src < devices_count);
                assert(0 <= dest && dest < devices_count);
                

                // make connection
                const auto bandwidth = bandwidth_per_dim.at(dim);
                const auto latency = topology->get_link_latency();
                if(fault_derate(src, dest) != 0)
                    connect(src, dest, bandwidth * fault_derate(src, dest), latency, false);
                else
                    connect(src, dest, bandwidth, latency, false);  //might be removable
            }
        }
    }
}

void MultiDimTopology::initialize_all_devices() noexcept {
    // instantiate all devices
    const auto total_num_devices = get_total_num_devices();

    for (auto i = 0; i < total_num_devices; i++) {
        devices.push_back(std::make_shared<Device>(i));
    }
}

MultiDimAddress MultiDimTopology::translate_address(const DeviceId npu_id) const noexcept {
    // If units-count if [2, 8, 4], and the given id is 47, then the id should be
    // 47 // 16 = 2, leftover = 47 % 16 = 15
    // 15 // 2 = 7, leftover = 15 % 2 = 1
    // 1 // 1 = 1, leftover = 0
    // therefore the address is [1, 7, 2]

    // create empty address
    auto multi_dim_address = MultiDimAddress();
    for (int i = 0; i < dims_count; i++) {
        multi_dim_address.push_back(-1);
    }

    auto leftover = npu_id;
    auto denominator = npus_count;

    for (int dim = dims_count - 1; dim >= 0; dim--) {
        // change denominator
        denominator /= npus_count_per_dim.at(dim);

        // get and update address
        const auto quotient = leftover / denominator;
        leftover %= denominator;

        // update address
        multi_dim_address.at(dim) = quotient;
    }

    // check address translation
    for (int i = 0; i < dims_count; i++) {
        assert(0 <= multi_dim_address.at(i));
        assert(multi_dim_address.at(i) < npus_count_per_dim.at(i));
    }

    // return retrieved address
    return multi_dim_address;
}

DeviceId MultiDimTopology::translate_address_back(const MultiDimAddress multi_dim_address) const noexcept {
    DeviceId device_id = 0;
    assert(multi_dim_address.size() == dims_count);
    for (int top_dim = dims_count - 1; top_dim >= 0; top_dim--) {
        // product from beginning to the top_dim - 1
        DeviceId total_npus_in_group = std::accumulate(npus_count_per_dim.begin(), npus_count_per_dim.begin() + top_dim,
                                                       static_cast<DeviceId>(1), std::multiplies<DeviceId>());

        // Add the contribution to the total device ID
        device_id += total_npus_in_group * multi_dim_address.at(top_dim);
    }
    return device_id;
}

int MultiDimTopology::get_dim_to_transfer(const MultiDimAddress& src_address,
                                          const MultiDimAddress& dest_address) const noexcept {
    for (int dim = 0; dim < dims_count; dim++) {
        // check the dim that has different address
        if (src_address.at(dim) != dest_address.at(dim)) {
            return dim;
        }
    }

    // shouldn't reach here
    std::cerr << "[Error] (network/analytical/congestion_unaware): "
              << "src and dest have the same address" << std::endl;
    std::exit(-1);
}

int MultiDimTopology::get_total_num_devices() const noexcept {
    assert(npus_count_per_dim.size() == dims_count);

    // get partial product of npus_count_per_dim
    std::vector<int> partial_product(dims_count, -1);
    partial_product.at(partial_product.size() - 1) = 1;
    for (int i = partial_product.size() - 2; i >= 0; i--) {
        partial_product.at(i) = partial_product.at(i + 1) * npus_count_per_dim.at(i + 1);
    }

    assert(npus_count == partial_product.at(0) * npus_count_per_dim.at(0));
    int total_npu_device = npus_count;
    int total_switch_device = 0;

    // get total switch number
    for (int i = 0; i < dims_count; i++) {
        if (m_topology_per_dim.at(i)->get_basic_topology_type() == TopologyBuildingBlock::Switch) {
            // each switch type add one additional device
            total_switch_device += partial_product.at(i);
        }
    }

    return total_npu_device + total_switch_device;
}

bool MultiDimTopology::is_switch(const MultiDimAddress& address) const noexcept {
    assert(address.size() == npus_count_per_dim.size());

    // element-wise check if all address is less than npus_count_per_dim
    // switch should have one and only one equal to dim
    auto result =
        std::mismatch(npus_count_per_dim.begin(), npus_count_per_dim.end(), address.begin(), [](int r, int a) {
            // The predicate: Check if the address element 'a' is less than the npus_count_per_dim element 'r'
            return a < r;
        });

    // If result.first is equal to range.end(), it means std::mismatch
    // reached the end without finding a mismatch (i.e., a pair where a >= r).
    return result.first != npus_count_per_dim.end();
}

void MultiDimTopology::build_switch_length_mapping() noexcept {
    if (!m_switch_translation_unit.has_value()) {
        std::vector<bool> is_switch_dim;
        is_switch_dim.resize(dims_count);

        // create a boolean mask for whether each dimension is a switch
        std::transform(m_topology_per_dim.begin(), m_topology_per_dim.end(), is_switch_dim.begin(),
                       [](const std::unique_ptr<BasicTopology>& topology) {
                           return topology->get_basic_topology_type() == TopologyBuildingBlock::Switch;
                       });

        m_switch_translation_unit.emplace(npus_count_per_dim, is_switch_dim);
    }
}
double MultiDimTopology::fault_derate(int src, int dst) const{
    for (const auto& link : faulty_links) {
        int a = std::get<0>(link);
        int b = std::get<1>(link);
        double health = std::get<2>(link);

        // If this link exists and health != 0.0 → it's soft fault
        if ((a == src && b == dst) || (a == dst && b == src)) {
            return health;
        }
        //else
        //    return 1;
    }
    return 1;
}


// std::vector<std::pair<MultiDimAddress, MultiDimAddress>>
// generateAddressPairs_only_first_nodes(const std::vector<int>& npus_count_per_dim,
//                                       const ConnectionPolicy& policy,
//                                       int dim)
// {
//     std::vector<std::pair<MultiDimAddress, MultiDimAddress>> result;

//     MultiDimAddress addr(npus_count_per_dim.size());

//     // Only vary dim, all other dims = 0
//     for (int d = 0; d < (int)npus_count_per_dim[dim]; d++) {
//         addr[dim] = d;
//         MultiDimAddress src = addr;
//         MultiDimAddress dst = addr;

//         src[dim] = policy.src;
//         dst[dim] = policy.dst;

//         result.emplace_back(src, dst);
//     }

//     return result;
// }


void MultiDimTopology::make_non_recursive_connections() noexcept {
    if (!m_switch_translation_unit.has_value()) {
        std::cerr << "[Error] (network/analytical/congestion_aware/MultiDimTopology): "
                  << "SwitchTranslationUnit is not initialized." << std::endl;
        std::exit(-1);
    }

    for (int dim = 0; dim < dims_count; dim++) {
    const auto topology = m_topology_per_dim.at(dim).get();
    const auto policies = topology->get_connection_policies();
    assert(!policies.empty());

    bool recursive_dim = (m_non_recursive_topo.at(dim) == 1);

        for (const auto& policy : policies) {

            std::vector<std::pair<MultiDimAddress, MultiDimAddress>> address_pairs;

            // --- NORMAL MODE ---
            if (!recursive_dim) {
                address_pairs =
                    generateAddressPairs(npus_count_per_dim, policy, dim);
            }

            // --- RECURSIVE MODE: only first nodes of lower dimensions ---
            else {
                address_pairs =
                    generateAddressPairs(npus_count_per_dim, policy, dim);
            }

            // Create links for all generated pairs
            for (const auto& ap : address_pairs) {

                const auto src = is_switch(ap.first)
                                    ? m_switch_translation_unit.value().translate_address_to_id(ap.first)
                                    : translate_address_back(ap.first);

                const auto dest = is_switch(ap.second)
                                    ? m_switch_translation_unit.value().translate_address_to_id(ap.second)
                                    : translate_address_back(ap.second);

                assert(0 <= src && src < devices_count);
                assert(0 <= dest && dest < devices_count);

                const auto bw = bandwidth_per_dim.at(dim);
                const auto lat = topology->get_link_latency();

                if (fault_derate(src, dest) != 0)
                    connect(src, dest, bw * fault_derate(src, dest), lat, false);
                else
                    connect(src, dest, bw, lat, false);
            }
        }
    }

}



};  // namespace NetworkAnalyticalCongestionAware

