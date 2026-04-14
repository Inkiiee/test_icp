#include "map_backend.h"

#include <cmath>
#include <algorithm>

namespace rcl_map_backend_type{
    Weight::Weight():hit_count(0), miss_count(0), last_seen_frame(0), x(0), y(0){}
}

namespace rcl_map_backend{
    MapBackend::MapBackend(double resolution): pos_r(resolution) {}
    MapBackend::~MapBackend(){}

    weight_map_index_type MapBackend::get_map_index(double x, double y) const{
        int cell_x = static_cast<int>(std::ceil(x / pos_r));
        int cell_y = static_cast<int>(std::ceil(y / pos_r));
        return {cell_x, cell_y};
    }

    void MapBackend::touchWeightCell(weight_map_index_type index){
        if(wm.find(index) == wm.end()){
            Weight w;
            w.x = index.first * pos_r;
            w.y = index.second * pos_r;
            wm[index] = w;
        }
    }

    void MapBackend::addPos(const std::vector<double>& x, const std::vector<double>& y){
        for(size_t i=0; i<x.size(); i++){
            weight_map_index_type index = get_map_index(x[i], y[i]);
            auto& w = wm[index];
            if(w.hit_count == 0 && w.miss_count == 0){
                w.x = index.first * pos_r;
                w.y = index.second * pos_r;
            }
            w.hit_count += 1;
            w.last_seen_frame = frame_index;
        }
    }

    void MapBackend::markFreeRay(weight_map_index_type origin, weight_map_index_type target, const std::unordered_set<weight_map_index_type, PairHash>& hit_cells){
        int x0 = origin.first;
        int y0 = origin.second;
        int x1 = target.first;
        int y1 = target.second;

        constexpr int kMaxMiss = 20;
        constexpr int kOccupiedThreshold = 3; // hit이 이 이상이면 이미 장애물 → miss 안 찍음

        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while(x0 != x1 || y0 != y1){
            weight_map_index_type current = {x0, y0};
            if(current != origin && hit_cells.find(current) == hit_cells.end()){
                auto it = wm.find(current);
                if(it != wm.end() && it->second.hit_count >= kOccupiedThreshold){
                    // 이미 장애물로 확인된 셀 → ray가 관통할 수 없음 → ray 중단
                    break;
                }
                touchWeightCell(current);
                Weight& w = wm[current];
                if(w.miss_count < kMaxMiss) w.miss_count += 1;
            }

            int err2 = 2 * err;
            if(err2 > -dy){
                err -= dy;
                x0 += sx;
            }
            if(err2 < dx){
                err += dx;
                y0 += sy;
            }
        }
    }

    void MapBackend::updateOccupancyMap(double sensor_x, double sensor_y, const std::vector<double>& x, const std::vector<double>& y){
        std::unordered_set<weight_map_index_type, PairHash> hit_cells;
        hit_cells.reserve(x.size());
        weight_map_index_type origin = get_map_index(sensor_x, sensor_y);

        constexpr int kMaxCount = 20;  // hit/miss 상한 → 최근 관측에 비중

        for(size_t i=0; i<x.size(); i++){
            weight_map_index_type index = get_map_index(x[i], y[i]);
            hit_cells.insert(index);
        }

        // hit 기록 (상한 적용)
        for(const auto& index : hit_cells){
            touchWeightCell(index);
            Weight& w = wm[index];
            if(w.hit_count < kMaxCount) w.hit_count += 1;
            w.last_seen_frame = frame_index;
            markFreeRay(origin, index, hit_cells);
        }
    }

    bool MapBackend::isStaticCell(const Weight& weight) const{
        int total = weight.hit_count + weight.miss_count;
        if(total == 0) return false;

        double occupied_ratio = static_cast<double>(weight.hit_count) / total;
        return occupied_ratio >= 0.55;
    }

    void MapBackend::getPos(std::vector<double>& x, std::vector<double>& y, bool static_only){
        x.clear();
        y.clear();
        x.reserve(wm.size());
        y.reserve(wm.size());
        
        for(const auto& entry : wm){
            const Weight& weight = entry.second;
            if(static_only && !isStaticCell(weight)) continue;

            double px = weight.x;
            double py = weight.y;

            x.push_back(px);
            y.push_back(py);
        }
    }

    void MapBackend::getAdjacentPos(const RobotBasePose& pose, std::vector<double>& x, std::vector<double>& y, double radius, bool static_only){
        x.clear();
        y.clear();

        // 반경 내 그리드 셀 범위를 직접 계산하여 해당 셀만 조회
        // O(전체맵) → O((2*radius/pos_r)^2)
        int min_cx = static_cast<int>(std::floor((pose.tx - radius) / pos_r));
        int max_cx = static_cast<int>(std::ceil((pose.tx + radius) / pos_r));
        int min_cy = static_cast<int>(std::floor((pose.ty - radius) / pos_r));
        int max_cy = static_cast<int>(std::ceil((pose.ty + radius) / pos_r));

        int range = (max_cx - min_cx + 1) * (max_cy - min_cy + 1);
        x.reserve(std::min(range, static_cast<int>(wm.size())));
        y.reserve(std::min(range, static_cast<int>(wm.size())));

        double r2 = radius * radius;
        for(int cx = min_cx; cx <= max_cx; cx++){
            for(int cy = min_cy; cy <= max_cy; cy++){
                auto it = wm.find({cx, cy});
                if(it == wm.end()) continue;

                const Weight& weight = it->second;
                if(static_only && !isStaticCell(weight)) continue;

                double dx = weight.x - pose.tx;
                double dy = weight.y - pose.ty;
                if(dx * dx + dy * dy <= r2){
                    x.push_back(weight.x);
                    y.push_back(weight.y);
                }
            }
        }
    }

    void MapBackend::clearMap(){
        wm.clear();
    }

    void MapBackend::incrementFrameIndex(){
        frame_index++;
    }
}