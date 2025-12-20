/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/NetworkParser.h"
#include <cassert>
#include <iostream>

using namespace NetworkAnalytical;

NetworkParser::NetworkParser(const std::string& path) noexcept : dims_count(-1) {
    // initialize values
    npus_count_per_dim = {};
    bandwidth_per_dim = {};
    latency_per_dim = {};
    topology_per_dim = {};
    faulty_links = {};
    non_recursive_topo = {};

    try {
        // load network config file
        const auto network_config = YAML::LoadFile(path);

        // parse network configs
        parse_network_config_yml(network_config);
    } catch (const YAML::BadFile& e) {
        // loading network config file failed
        std::cerr << "[Error] (network/analytical) " << e.what() << std::endl;
        std::exit(-1);
    }
}

int NetworkParser::get_dims_count() const noexcept {
    assert(dims_count > 0);

    return dims_count;
}

std::vector<int> NetworkParser::get_npus_counts_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(npus_count_per_dim.size() == dims_count);

    return npus_count_per_dim;
}

std::vector<Bandwidth> NetworkParser::get_bandwidths_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(bandwidth_per_dim.size() == dims_count);

    return bandwidth_per_dim;
}

std::vector<Latency> NetworkParser::get_latencies_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(latency_per_dim.size() == dims_count);

    return latency_per_dim;
}

std::vector<TopologyBuildingBlock> NetworkParser::get_topologies_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(topology_per_dim.size() == dims_count);

    return topology_per_dim;
}
std::vector<std::tuple<int, int, double>> NetworkParser::get_faulty_links() const noexcept {
    return faulty_links;
}
std::vector<int> NetworkParser::get_non_recursive_topo() const noexcept {
    return non_recursive_topo;
}

void NetworkParser::parse_network_config_yml(const YAML::Node& network_config) noexcept {
    // parse topology_per_dim
    const auto topology_names = parse_vector<std::string>(network_config["topology"]);
    for (const auto& topology_name : topology_names) {
        const auto topology_dim = NetworkParser::parse_topology_name(topology_name);
        topology_per_dim.push_back(topology_dim);
    }

    // set dims_count
    dims_count = static_cast<int>(topology_per_dim.size());

    // parse vector values
    npus_count_per_dim = parse_vector<int>(network_config["npus_count"]);
    bandwidth_per_dim = parse_vector<Bandwidth>(network_config["bandwidth"]);
    latency_per_dim = parse_vector<Latency>(network_config["latency"]);
    // Parse non_recursive_topo with format priority
    if (network_config["non_recursive_from"]) {
        // NEW FORMAT: crossover index - dimensions >= crossover are non-recursive
        int crossover = network_config["non_recursive_from"].as<int>();
        // Validate range
        if (crossover < 0 || crossover > dims_count) {
            std::cerr << "[Error] (network/analytical) " << "non_recursive_from (" << crossover
                      << ") must be between 0 and dims_count (" << dims_count << ")" << std::endl;
            std::exit(-1);
        }
        non_recursive_topo.resize(dims_count, 0);
        for (int d = crossover; d < dims_count; d++) {
            non_recursive_topo[d] = 1;
        }
    } else if (network_config["non_recursive_topology"]) {
        // LEGACY FORMAT: explicit array [0, 0, 1, 1]
        non_recursive_topo = parse_vector<int>(network_config["non_recursive_topology"]);
    } else {
        // DEFAULT: all dimensions are recursive (no cluster mode)
        non_recursive_topo.resize(dims_count, 0);
    }

    // check the validity of the parsed network config
    check_validity();

    //faulty link support
    if (network_config["faulty_links"]) {
        for (const auto& link_node : network_config["faulty_links"]) {
            if (link_node.IsSequence() && link_node.size() >= 3) {
                std::cout<<"the source is"<< link_node[0] << "and dest is"<<link_node[1]<<"fault rate is:"<<link_node[2]<<std::endl;
                int src = link_node[0].as<int>();
                int dst = link_node[1].as<int>();
                double reliability = link_node[2].as<double>();
                faulty_links.emplace_back(src, dst, reliability);
            } else {
                std::cerr << "[Warning] (network/analytical) Invalid faulty_links format. Expected [src, dst, weight].\n";
            }
        }
    }
}

TopologyBuildingBlock NetworkParser::parse_topology_name(const std::string& topology_name) noexcept {
    assert(!topology_name.empty());

    if (topology_name == "Ring") {
        return TopologyBuildingBlock::Ring;
    }

    if (topology_name == "FullyConnected") {
        return TopologyBuildingBlock::FullyConnected;
    }

    if (topology_name == "Switch") {
        return TopologyBuildingBlock::Switch;
    }

    if (topology_name == "Bus") {
        return TopologyBuildingBlock::Bus;
    }

    if (topology_name == "BinaryTree") {
        return TopologyBuildingBlock::BinaryTree;
    }

    if (topology_name == "DoubleBinaryTree") {
        return TopologyBuildingBlock::DoubleBinaryTree;
    }

    if (topology_name == "Mesh") {
        return TopologyBuildingBlock::Mesh;
    }

    if (topology_name == "HyperCube") {
        return TopologyBuildingBlock::HyperCube;
    }
    if (topology_name == "Torus2D") {
        return TopologyBuildingBlock::Torus2D;
    }
    if (topology_name == "Mesh2D") {
        return TopologyBuildingBlock::Mesh2D;
    }
    if (topology_name == "KingMesh2D") {
        return TopologyBuildingBlock::KingMesh2D;
    }

    // shouldn't reach here
    std::cerr << "[Error] (network/analytical) " << "Topology name " << topology_name << " not supported" << std::endl;
    std::exit(-1);
}

void NetworkParser::check_validity() const noexcept {
    // dims_count should match
    if (dims_count != npus_count_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) " << "length of npus_count (" << npus_count_per_dim.size()
                  << ") doesn't match with dimensions (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    if (dims_count != bandwidth_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) " << "length of bandwidth (" << bandwidth_per_dim.size()
                  << ") doesn't match with dims_count (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    if (dims_count != latency_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) " << "length of latency (" << latency_per_dim.size()
                  << ") doesn't match with dims_count (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    // npus_count should be all positive
    for (const auto& npus_count : npus_count_per_dim) {
        if (npus_count <= 1) {
            std::cerr << "[Error] (network/analytical) " << "npus_count (" << npus_count << ") should be larger than 1"
                      << std::endl;
            std::exit(-1);
        }
    }

    // bandwidths should be all positive
    for (const auto& bandwidth : bandwidth_per_dim) {
        if (bandwidth <= 0) {
            std::cerr << "[Error] (network/analytical) " << "bandwidth (" << bandwidth << ") should be larger than 0"
                      << std::endl;
            std::exit(-1);
        }
    }

    // latency should be non-negative
    for (const auto& latency : latency_per_dim) {
        if (latency < 0) {
            std::cerr << "[Error] (network/analytical) " << "latency (" << latency << ") should be non-negative"
                      << std::endl;
            std::exit(-1);
        }
    }

    // Validate non_recursive_topo
    if (!non_recursive_topo.empty()) {
        // Size must match dims_count
        if (non_recursive_topo.size() != dims_count) {
            std::cerr << "[Error] (network/analytical) " << "length of non_recursive_topology ("
                      << non_recursive_topo.size() << ") doesn't match with dims_count ("
                      << dims_count << ")" << std::endl;
            std::exit(-1);
        }

        // Values must be 0 or 1, and must be consecutive 0s followed by 1s
        bool seen_one = false;
        for (size_t i = 0; i < non_recursive_topo.size(); i++) {
            int v = non_recursive_topo[i];
            if (v != 0 && v != 1) {
                std::cerr << "[Error] (network/analytical) "
                          << "non_recursive_topology values must be 0 or 1, got " << v
                          << " at dimension " << i << std::endl;
                std::exit(-1);
            }
            if (seen_one && v == 0) {
                std::cerr << "[Error] (network/analytical) "
                          << "non_recursive_topology must be consecutive 0s followed by 1s. "
                          << "Found 0 at dimension " << i << " after seeing 1." << std::endl;
                std::exit(-1);
            }
            if (v == 1) seen_one = true;
        }
    }
}
