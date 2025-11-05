/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Mesh2D.h"
#include <cassert>
#include <cmath>


using namespace NetworkAnalyticalCongestionAware;

Mesh2D::Mesh2D(const int npus_count,
                 const Bandwidth bandwidth,
                 const Latency latency,
                 const bool bidirectional,
                 const bool is_multi_dim,
                 const std::vector<std::tuple<int, int, double>>& faulty_links) noexcept
    : bidirectional(bidirectional),
      BasicTopology(npus_count, npus_count, bandwidth, latency, is_multi_dim), faulty_links(faulty_links) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);


    Mesh2D::basic_topology_type = TopologyBuildingBlock::Mesh2D;

    if (!is_multi_dim) {
        // Assume npus_count forms a perfect square
        const int dim = static_cast<int>(std::sqrt(npus_count));
        assert(dim * dim == npus_count && "2D Mesh requires a square grid");

        for (int row = 0; row < dim; ++row) {
            for (int col = 0; col < dim; ++col) {
                int current = row * dim + col;

                // --- Connect right (no wrap-around) ---
                if (col + 1 < dim) {
                    int right = row * dim + (col + 1);
                    if(fault_derate(current, right) != 0)
                    connect(current, right, bandwidth * fault_derate(current, right), latency, bidirectional);
                else
                    connect(current, right, bandwidth, latency, bidirectional);  //might be removable
                }

                // --- Connect down (no wrap-around) ---
                if (row + 1 < dim) {
                    int down = (row + 1) * dim + col;
                    if(fault_derate(current, down) != 0)
                    connect(current, down, bandwidth * fault_derate(current, down), latency, bidirectional);
                else
                    connect(current, down, bandwidth, latency, bidirectional);  //might be removable
                }
            }
        }
    } else {
        // Fallback to 1D Mesh
        for (int i = 0; i < npus_count - 1; ++i) {
            connect(i, i + 1, bandwidth, latency, bidirectional);
        }
        //connect(npus_count - 1, 0, bandwidth, latency, bidirectional);
    }

    // this also works
    // std::vector<ConnectionPolicy> policies = get_connection_policies();
    // for (const auto& policy : policies) {
    //     connect(policy.src, policy.dst, bandwidth, latency, /*bidirectional=*/false);
    // }
}

Route Mesh2D::route(DeviceId src, DeviceId dest) const noexcept {
    Route route;
    const int dim = static_cast<int>(std::sqrt(npus_count));
    int sx = src % dim, sy = src / dim;
    int dx = dest % dim, dy = dest / dim;

    route.push_back(devices.at(src));
    int cur = src;

    while (cur != dest) {
        int cx = cur % dim, cy = cur / dim;

        int step_x = 0, step_y = 0;
        if (cx != dx) {
            step_x = (dx > cx) ? +1 : -1;
        } else if (cy != dy) {
            step_y = (dy > cy) ? +1 : -1;
        }

        int next;
        if (step_x != 0) {
            int nx = cx + step_x;
            if (nx >= 0 && nx < dim) {
                next = cy * dim + nx;
                if (fault_derate(cur, next) == 0.0) {
                    // Detour one step in Y
                    int ny_up = cy + 1;
                    int ny_down = cy - 1;
                    if (ny_up < dim && fault_derate(cur, ny_up * dim + cx) != 0.0)
                        next = ny_up * dim + cx;
                    else if (ny_down >= 0 && fault_derate(cur, ny_down * dim + cx) != 0.0)
                        next = ny_down * dim + cx;
                    else
                        break;  // no valid detour
                }
            } else {
                break;  // out of bounds
            }
        } else if (step_y != 0) {
            int ny = cy + step_y;
            if (ny >= 0 && ny < dim) {
                next = ny * dim + cx;
                if (fault_derate(cur, next) == 0.0) {
                    // Detour one step in X
                    int nx_right = cx + 1;
                    int nx_left = cx - 1;
                    if (nx_right < dim && fault_derate(cur, cy * dim + nx_right) != 0.0)
                        next = cy * dim + nx_right;
                    else if (nx_left >= 0 && fault_derate(cur, cy * dim + nx_left) != 0.0)
                        next = cy * dim + nx_left;
                    else
                        break;  // no valid detour
                }
            } else {
                break;  // out of bounds
            }
        } else {
            break;  // no progress
        }

        route.push_back(devices.at(next));
        cur = next;
    }

    return route;
}


std::vector<ConnectionPolicy> Mesh2D::get_connection_policies() const noexcept {
    std::vector<ConnectionPolicy> policies;

    const int dim = static_cast<int>(std::sqrt(npus_count));
    assert(dim * dim == npus_count && "2D mesh requires npus_count to be a perfect square");

    // Each node connects to its right and down neighbor (no wrap-around)
    for (int row = 0; row < dim; ++row) {
        for (int col = 0; col < dim; ++col) {
            int current = row * dim + col;

            // Connect to right neighbor (if not on the right edge)
            if (col + 1 < dim) {
                int right = row * dim + (col + 1);
                policies.emplace_back(current, right);
            }

            // Connect to down neighbor (if not on the bottom edge)
            if (row + 1 < dim) {
                int down = (row + 1) * dim + col;
                policies.emplace_back(current, down);
            }
        }
    }

    // If bidirectional, add reverse edges too
    if (bidirectional) {
        for (int row = 0; row < dim; ++row) {
            for (int col = 0; col < dim; ++col) {
                int current = row * dim + col;

                // Connect to left neighbor (if not on the left edge)
                if (col - 1 >= 0) {
                    int left = row * dim + (col - 1);
                    policies.emplace_back(current, left);
                }

                // Connect to up neighbor (if not on the top edge)
                if (row - 1 >= 0) {
                    int up = (row - 1) * dim + col;
                    policies.emplace_back(current, up);
                }
            }
        }
    }

    return policies;
}

double Mesh2D::fault_derate(int src, int dst) const{
    for (const auto& link : faulty_links) {
        int a = std::get<0>(link);
        int b = std::get<1>(link);
        double health = std::get<2>(link);

        // If this link exists and health != 0.0 → it's soft fault
        if ((a == src && b == dst) || (a == dst && b == src)) {
            return health;
        }
        else
            //std::cerr << "[Warning] (network/analytical) No link between destination pair" <<src <<"and" <<dst<< "Faulty link omitted.\n";
            return 1;
    }

    return 1;
}
