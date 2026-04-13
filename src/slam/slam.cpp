#include "slam.h"
#include "slam_basic.h"

#include <QThread>

namespace rcl_slam{
    using namespace rcl_scan_match_backend;
    using namespace rcl_loop_detecter;
    using namespace rcl_painter;
    using namespace rcl_slam_basic_transform;

    SlamSystem::SlamSystem(Bridge* b, double r, QObject* parent): QObject(parent), pos_r{r}, bridge{b} {
        backend = new ScanMatchBackend(bridge, pos_r, this);
        loop_detecter = new LoopDetecter(backend->getPoseGraph(), backend->getSubMaps(), backend->getSharedDataMutex(), this);
        painter = new Painter(backend->getPoseGraph(), backend->getWorldMap(), backend->getLocalMap(), pos_r, backend->getSharedDataMutex());

        QObject::connect(
            loop_detecter, 
            &LoopDetecter::optimizedPoseUpdated, 
            backend,
            &ScanMatchBackend::poseOptimized,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend, 
            &ScanMatchBackend::subMapUpdated,
            loop_detecter, 
            &LoopDetecter::detectLoop,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend, 
            &ScanMatchBackend::predictedPose, 
            painter, 
            &Painter::predictedPoseUpdate,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend, 
            &ScanMatchBackend::scanUpdated,
            painter,
            &Painter::scanUpdate,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend,
            &ScanMatchBackend::rebuildMapRequested,
            this,
            &SlamSystem::rebuildMap,
            Qt::ConnectionType::QueuedConnection
        );

        painter->show();

        QThread * thread1 = new QThread(this);
        backend->moveToThread(thread1);
        thread1->start();

        QThread * thread2 = new QThread(this);
        loop_detecter->moveToThread(thread2);
        thread2->start();

        QThread * thread3 = new QThread(this);
        this->moveToThread(thread3);
        thread3->start();
    }
    SlamSystem::~SlamSystem(){
        backend->deleteLater();
        loop_detecter->deleteLater();
        painter->deleteLater();
    }

    void SlamSystem::rebuildMap(){
        auto world_map = backend->getWorldMap();
        auto pose_graph = backend->getPoseGraph();
        auto sub_maps = backend->getSubMaps();
        auto data_mutex = backend->getSharedDataMutex();

        std::lock_guard<std::mutex> lock(*data_mutex);

        world_map->clearMap();
        for(size_t i=0; i<pose_graph->getPoseCount(); i++){
            const auto& sm = (*sub_maps)[i];
            std::vector<double> xpos(sm.x.begin(), sm.x.end());
            std::vector<double> ypos(sm.y.begin(), sm.y.end());

            rotationAndTranslation(pose_graph->getPose(i).tx, pose_graph->getPose(i).ty, pose_graph->getPose(i).theta, xpos, ypos);

            // 센서 원점도 같이 변환
            double sx = sm.sensor_x, sy = sm.sensor_y;
            double ct = std::cos(pose_graph->getPose(i).theta), st = std::sin(pose_graph->getPose(i).theta);
            double world_sx = ct * sx - st * sy + pose_graph->getPose(i).tx;
            double world_sy = st * sx + ct * sy + pose_graph->getPose(i).ty;

            world_map->updateOccupancyMap(world_sx, world_sy, xpos, ypos);
        }
    }
}
