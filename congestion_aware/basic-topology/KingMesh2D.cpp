/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/KingMesh2D.h"
#include <cassert>
#include <cmath>


using namespace NetworkAnalyticalCongestionAware;


const int npus_count_x = 8;         //MODIFY npus_ dimensions
const int npus_count_y = 2;         //MODIFY npus_ dimensions


KingMesh2D::KingMesh2D(const int npus_count,
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


    KingMesh2D::basic_topology_type = TopologyBuildingBlock::KingMesh2D;

    if (!is_multi_dim) {
        // Assume npus_count forms a perfect square
        const int dim_x = npus_count_x;
        const int dim_y = npus_count_y;

        assert(dim_x * dim_y == npus_count && "KingMesh dimensions are not equal to npus count (topology)");

        for (int row = 0; row < dim_y; ++row) {
            for (int col = 0; col < dim_x; ++col) {
                int current = row * dim_x + col;

                // --- Connect right (no wrap-around) ---
                if (col + 1 < dim_x) {
                    int right = row * dim_x + (col + 1);
                    if(fault_derate(current, right) != 0)
                        connect(current, right, bandwidth * fault_derate(current, right), latency, bidirectional);
                    else
                        connect(current, right, bandwidth, latency, bidirectional);  //might be removable
                } 

                // --- Connect down (no wrap-around) ---
                if (row + 1 < dim_y) {
                    int down = (row + 1) * dim_x + col;
                    if(fault_derate(current, down) != 0)
                        connect(current, down, bandwidth * fault_derate(current, down), latency, bidirectional);
                    //else
                        //connect(current, down, bandwidth, latency, false);  //might be removable
                    if (col + 1 < dim_x){
                        int diag_right = (row + 1) * dim_x + (col + 1);
                        if(fault_derate(current, down) != 0)
                            connect(current, diag_right, bandwidth * fault_derate(current, diag_right), latency, bidirectional);
                        //else
                        //    connect(current, diag_right, bandwidth, latency, bidirectional);  //might be removable
                    }
                    if (col > 0){
                        int diag_left = (row + 1) * dim_x + (col - 1);
                        if(fault_derate(current, down) != 0)
                            connect(current, diag_left, bandwidth * fault_derate(current, diag_left), latency, bidirectional);
                        //else
                        //    connect(current, diag_left, bandwidth, latency, bidirectional);  //might be removable
                    }
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

Route KingMesh2D::route(DeviceId src, DeviceId dest) const noexcept {
    Route route;
    const int dim_x = npus_count_x;
    const int dim_y = npus_count_y;

    assert(dim_x * dim_y == npus_count && "KingMesh2D requires perfect square npus_count");

    int sx = src % dim_x, sy = src / dim_x;
    int dx = dest % dim_x, dy = dest / dim_x;

    route.push_back(devices.at(src));
    int cur = src;
    std::cout<<"src and dest are: "<<src<<"and"<<dest<<std::endl;
    while (cur != dest) {
        int cx = cur % dim_x, cy = cur / dim_x;

        int step_x = 0, step_y = 0;
        if (cx < dx) step_x = +1;
        else if (cx > dx) step_x = -1;
        if (cy < dy) step_y = +1;
        else if (cy > dy) step_y = -1;

        int next = -1;

        // --- Prefer diagonal move if both steps are nonzero ---
        if (step_x != 0 && step_y != 0) {
            int nx = cx + step_x;
            int ny = cy + step_y;
            if (nx >= 0 && nx < dim_x && ny >= 0 && ny < dim_y) {
                next = ny * dim_x + nx;
                // If diagonal link is faulty, fallback to single-axis move
                if (fault_derate(cur, next) == 0.0) {
                    // Prefer maintaining the same general direction
                    int nx_alt = cx + step_x;
                    int ny_alt = cy + step_y;

                    bool moved = false;
                    if (nx_alt >= 0 && nx_alt < dim_x) {
                        int next_x = cy * dim_x + nx_alt;
                        if (fault_derate(cur, next_x) != 0.0) {
                            next = next_x;
                            moved = true;
                        }
                    }
                    if (!moved && ny_alt >= 0 && ny_alt < dim_y) {
                        int next_y = ny_alt * dim_x + cx;
                        if (fault_derate(cur, next_y) != 0.0) {
                            next = next_y;
                            moved = true;
                        }
                    }
                    if (!moved) break; // No valid move
                }
            } else {
                next = -1;
            }
        }

        // --- If not diagonal or diagonal not possible, move along X or Y ---
        if (next == -1) {
            if (step_x != 0) {
                int nx = cx + step_x;
                if (nx >= 0 && nx < dim_x) {
                    next = cy * dim_x + nx;
                    if (fault_derate(cur, next) == 0.0) {
                        // Detour vertically (same direction if possible)
                        int ny_up = cy + 1;
                        int ny_down = cy - 1;
                        if (ny_up < dim_y && fault_derate(cur, ny_up * dim_x + cx) != 0.0)
                            next = ny_up * dim_x + cx;
                        else if (ny_down >= 0 && fault_derate(cur, ny_down * dim_x + cx) != 0.0)
                            next = ny_down * dim_x + cx;
                        else
                            break;
                    }
                }
            } else if (step_y != 0) {
                int ny = cy + step_y;
                if (ny >= 0 && ny < dim_y) {
                    next = ny * dim_x + cx;
                    if (fault_derate(cur, next) == 0.0) {
                        // Detour horizontally (same direction if possible)
                        int nx_right = cx + 1;
                        int nx_left = cx - 1;
                        if (nx_right < dim_x && fault_derate(cur, cy * dim_x + nx_right) != 0.0)
                            next = cy * dim_x + nx_right;
                        else if (nx_left >= 0 && fault_derate(cur, cy * dim_x + nx_left) != 0.0)
                            next = cy * dim_x + nx_left;
                        else
                            break;
                    }
                }
            }
        }

        if (next == -1) break;  // no valid move
        route.push_back(devices.at(next));
        cur = next;
    }

    return route;
}



std::vector<ConnectionPolicy> KingMesh2D::get_connection_policies() const noexcept {
    std::vector<ConnectionPolicy> policies;

    const int dim_x = npus_count_x;
    const int dim_y = npus_count_y;

    assert(dim_x * dim_y == npus_count && "KingMesh2D requires npus_count to be a perfect square");

    // --- Each node connects to its 8-neighborhood (no wrap-around) ---
    for (int row = 0; row < dim_y; ++row) {
        for (int col = 0; col < dim_x; ++col) {
            int current = row * dim_x + col;

            // Explore all 8 directions
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    // Skip self
                    if (dx == 0 && dy == 0) continue;

                    int new_row = row + dy;
                    int new_col = col + dx;

                    // Boundary check (no wrap-around)
                    if (new_row >= 0 && new_row < dim_y && new_col >= 0 && new_col < dim_x) {
                        int neighbor = new_row * dim_x + new_col;
                        policies.emplace_back(current, neighbor);
                    }
                }
            }
        }
    }

    // --- If bidirectional, add reverse edges explicitly ---
    if (bidirectional) {
        std::vector<ConnectionPolicy> reverse_policies;
        reverse_policies.reserve(policies.size());

        for (const auto& p : policies) {
            reverse_policies.emplace_back(p.dst, p.src);
        }

        // Append reverse edges
        policies.insert(policies.end(), reverse_policies.begin(), reverse_policies.end());
    }

    return policies;
}

double KingMesh2D::fault_derate(int src, int dst) const{
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
