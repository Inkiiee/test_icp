#include "scan_match_backend.h"
#include "slam_basic.h"
#include "my_pose_graph.h"

#include <cmath>
#include <QDebug>

namespace rcl_scan_match_backend{
    using namespace rcl_scan_match;
    using namespace rcl_slam_basic_type;
    using namespace rcl_slam_basic_transform;
    using namespace rcl_pose_graph;
    using namespace rcl_map_backend;

    ScanMatchBackend::ScanMatchBackend(Bridge* b, double pos_r, QObject* parent): QObject(parent), bridge{b}, world_map(pos_r), local_map(pos_r){
        map_x = 0, map_y = 0, map_theta = 0;
        odom_x = 0, odom_y = 0, odom_theta = 0;
        imu_theta = 0;

        QObject::connect(bridge, &Bridge::scanDataReceived, this, &ScanMatchBackend::lidarUpdate, Qt::ConnectionType::QueuedConnection);
        QObject::connect(bridge, &Bridge::odomDataReceived, this, &ScanMatchBackend::odomUpdate, Qt::ConnectionType::QueuedConnection);
        // QObject::connect(bridge, &Bridge::imuHeadingReceived, this, &ScanMatchBackend::imuUpdate, Qt::ConnectionType::QueuedConnection);
    }
    ScanMatchBackend::~ScanMatchBackend(){}

    std::vector<sub_map>* ScanMatchBackend::getSubMaps(){
        return &sub_maps;
    }
    PoseGraph* ScanMatchBackend::getPoseGraph(){
        return &pose_graph;
    }
    MapBackend* ScanMatchBackend::getWorldMap(){
        return &world_map;
    }
    MapBackend* ScanMatchBackend::getLocalMap(){
        return &local_map;
    }
    std::mutex* ScanMatchBackend::getSharedDataMutex(){
        return &shared_data_mutex_;
    }

    void ScanMatchBackend::odomUpdate(double x, double y, double /*z*/, double /*rx*/, double /*ry*/, double rz){
        double change_x = x - odom_x;
        double change_y = y - odom_y;
        double heading = rz;
        double change_theta = normalizeAngle(heading - odom_theta);

        double local_dx = cos(odom_theta) * change_x + sin(odom_theta) * change_y;
        double local_dy = -sin(odom_theta) * change_x + cos(odom_theta) * change_y;
        map_x = map_x + (cos(map_theta) * local_dx - sin(map_theta) * local_dy);
        map_y = map_y + (sin(map_theta) * local_dx + cos(map_theta) * local_dy);
        map_theta = normalizeAngle(map_theta + change_theta);

        odom_x = x;
        odom_y = y;
        odom_theta = heading;
    }

    void ScanMatchBackend::imuUpdate(double yaw){
        imu_theta = normalizeAngle(yaw);
    }

    void ScanMatchBackend::poseOptimized(int index, const Eigen::Matrix3d& delta){
        double delta_theta = std::atan2(delta(1, 0), delta(0, 0));
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex_);

            // 최적화 이후 새로 추가된 포즈들에 delta 보정 적용
            for(int i=index + 1; i < static_cast<int>(pose_graph.getPoseCount()); i++){
                auto p = pose_graph.getPose(i);
                Eigen::Vector3d vec(p.tx, p.ty, 1);
                vec = delta * vec;
                p.tx = vec.x();
                p.ty = vec.y();
                p.theta = normalizeAngle(p.theta + delta_theta);
                pose_graph.setPose(i, p);  // 실제로 pose graph에 반영
            }

            Eigen::Vector3d vec(map_x, map_y, 1);
            vec = delta * vec;
            map_x = vec.x();
            map_y = vec.y();
            map_theta = normalizeAngle(map_theta + delta_theta);
        }

        emit rebuildMapRequested();
    }

    double preciouse(double val, int num){
        double temp = (std::pow(10, num));
        return std::round(temp * val) / temp;
    }

    void ScanMatchBackend::lidarUpdate(const ScanAxis& xs, const ScanAxis& ys){
        // ── 1. 항상 최신 스캔을 버퍼에 저장 (O(1), 매우 빠름) ──
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            pending_scan_x_ = xs;
            pending_scan_y_ = ys;
            has_pending_scan_ = true;
        }

        // ── 2. 이미 처리 중이면 리턴 — 현재 처리가 끝난 뒤 다음 이벤트가 최신값을 처리 ──
        bool expected = false;
        if(!processing_busy_.compare_exchange_strong(expected, true)){
            return;
        }

        // ── 3. 버퍼에서 최신 스캔 가져오기 (Qt큐의 오래된 xs/ys가 아닌 진짜 최신값) ──
        ScanAxis latest_xs, latest_ys;
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if(!has_pending_scan_){
                processing_busy_ = false;
                return;
            }
            latest_xs = std::move(pending_scan_x_);
            latest_ys = std::move(pending_scan_y_);
            has_pending_scan_ = false;
        }

        // ── 거리 게이팅 ──
        double travel_xy = std::sqrt((map_x - last_match_x_) * (map_x - last_match_x_) + (map_y - last_match_y_) * (map_y - last_match_y_));
        double travel_theta = std::abs(normalizeAngle(map_theta - last_match_theta_));
        bool moved_enough = (travel_xy >= kMinTravelDistance || travel_theta >= kMinTravelAngle);

        if(!moved_enough){
            std::vector<double> pixel_x(latest_xs.begin(), latest_xs.end()), pixel_y(latest_ys.begin(), latest_ys.end());
            rotationAndTranslation(map_x, map_y, map_theta, pixel_x, pixel_y);
            local_map.addPos(pixel_x, pixel_y);
            emit scanUpdated(pixel_x, pixel_y);
            emit predictedPose(map_x, map_y, map_theta);
            processing_busy_ = false;
            return;
        }

        if(frame_index++ % 10 == 0){
            rcl_map_backend_type::sub_map sm;
            local_map.getPos(sm.x, sm.y);

            RobotBasePose currentPose{map_x, map_y, map_theta};
            // 로컬 좌표계에서의 센서 원점 (로컬 기준이므로 0,0)
            sm.sensor_x = 0;
            sm.sensor_y = 0;
            int current_index = 0;
            {
                std::lock_guard<std::mutex> lock(shared_data_mutex_);

                // 서브맵 포인트는 이미 월드 좌표 → addWorldMap에 센서 위치도 전달
                world_map.updateOccupancyMap(map_x, map_y, sm.x, sm.y);

                // Local 좌표계로 서브맵으로 저장시.
                RobotBasePose inv = inversePose(currentPose);
                rotationAndTranslation(inv.tx, inv.ty, inv.theta, sm.x, sm.y);
                sub_maps.push_back(sm);

                current_index = pose_graph.getPoseCount();
                pose_graph.addPose(currentPose);

                // 연속 노드 간 오도메트리 엣지 추가 (루프 최적화 시 궤적 유지에 필수)
                if(current_index > 0){
                    pose_graph.addEdge(current_index - 1, current_index, 1.0, 1.0, 1.0, false);
                }

                local_map.clearMap();
            }

            frame_index = 1;
            lut_valid_ = false;  // 맵이 바뀌었으니 LUT 캐시 무효화
            qDebug()<<"Sub "<<preciouse(map_x, 3)<<" "<<preciouse(map_y, 3)<<" "<<preciouse(map_theta, 3);
            qDebug()<<"Odom "<<preciouse(odom_x, 3)<<" "<<preciouse(odom_y, 3)<<" "<<preciouse(odom_theta, 3);

            emit subMapUpdated(current_index);
        }

        // 매칭 대상: local_map + world_map에서 로봇 근처 다운샘플된 포인트
        match_ref_map_.clearMap();
        std::vector<double> pixel_x, pixel_y, world_x, world_y;
        local_map.getPos(world_x, world_y);
        match_ref_map_.addPos(world_x, world_y);

        // world_map에서 반경 내 포인트를 다운샘플(0.1m 해상도)하여 추출
        // → pose 100+에서도 reference 포인트 수가 일정하게 유지됨
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex_);
            world_map.getAdjacentPosDownsampled(RobotBasePose(map_x, map_y, map_theta), world_x, world_y, 5.0, 0.1, true);
        }
        match_ref_map_.addPos(world_x, world_y);
        match_ref_map_.getPos(world_x, world_y);

        if(!world_x.empty()){
            std::vector<double> scan_x(latest_xs.begin(), latest_xs.end()), scan_y(latest_ys.begin(), latest_ys.end());

            // odom 변위가 충분할 때만 CSM, 그 외엔 NDT만
            double disp_xy = std::sqrt((map_x - last_csm_x_) * (map_x - last_csm_x_) + (map_y - last_csm_y_) * (map_y - last_csm_y_));
            double disp_theta = std::abs(normalizeAngle(map_theta - last_csm_theta_));
            bool need_csm = (disp_xy > 0.15 || disp_theta > 0.1 || !lut_valid_);

            Param p;
            if(need_csm){
                // LUT 재빌드 & CSM → NDT
                cached_lut_ = scan_matcher.buildLookupTable(world_x, world_y, 0.02, 0.05);
                lut_valid_ = true;
                p = scan_matcher.runCSM(scan_x, scan_y, world_x, world_y, map_x, map_y, map_theta,
                        0.3, 0.2, 0.05, 0.02, 0.005, 0.002, 0.02, 0.05);
                p = scan_matcher.runNDT(scan_x, scan_y, world_x, world_y, p.tx, p.ty, p.theta, 0.1, 0.05, 30, 1e-6);
                last_csm_x_ = p.tx;
                last_csm_y_ = p.ty;
                last_csm_theta_ = p.theta;
            } else {
                // NDT만 (odom이 좋은 초기값)
                p = scan_matcher.runNDT(scan_x, scan_y, world_x, world_y, map_x, map_y, map_theta, 0.1, 0.05, 30, 1e-6);
            }
            p.theta = normalizeAngle(p.theta);

            // 매칭 결과가 odom 예측과 너무 다르면 가중 평균 (급격한 점프 방지)
            double jump_xy = std::sqrt((p.tx - map_x) * (p.tx - map_x) + (p.ty - map_y) * (p.ty - map_y));
            double jump_theta = std::abs(normalizeAngle(p.theta - map_theta));
            if(jump_xy > 0.3 || jump_theta > 0.15){
                constexpr double alpha = 0.3;
                p.tx = map_x + alpha * (p.tx - map_x);
                p.ty = map_y + alpha * (p.ty - map_y);
                p.theta = normalizeAngle(map_theta + alpha * normalizeAngle(p.theta - map_theta));
            }

            map_x = p.tx;
            map_y = p.ty;
            map_theta = p.theta;
        }

        pixel_x.assign(latest_xs.begin(), latest_xs.end());
        pixel_y.assign(latest_ys.begin(), latest_ys.end());
        rotationAndTranslation(map_x, map_y, map_theta, pixel_x, pixel_y);

        local_map.addPos(pixel_x, pixel_y);

        last_match_x_ = map_x;
        last_match_y_ = map_y;
        last_match_theta_ = map_theta;

        emit scanUpdated(pixel_x, pixel_y);
        emit predictedPose(map_x, map_y, map_theta);

        processing_busy_ = false;
    }
}
