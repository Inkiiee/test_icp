#ifndef PAINTER_H
#define PAINTER_H

#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QPixmap>
#include <vector>

#include "map_backend.h"
#include "my_pose_graph.h"

class SharedMem;

namespace rcl_painter{
    class Painter : public QWidget
    {
        Q_OBJECT
    private:
        double pos_r;
        bool is_lidar_visible = true;
        QPixmap pixmap, lader_pixmap, world_pixmap;
        QLabel* label;
        SharedMem* shared_mem = nullptr;
        rcl_pose_graph::PoseGraph* pose_graph = nullptr;
        rcl_map_backend::MapBackend* world_map = nullptr;
        rcl_map_backend::MapBackend* local_map = nullptr;

        void drawScan(const std::vector<double>& xs, const std::vector<double>& ys);
        void drawPose(double x, double y, double theta);
        void drawWorldMap();
    protected:
        void paintEvent(QPaintEvent* event) override;
    public:
        explicit Painter(rcl_pose_graph::PoseGraph* pg, rcl_map_backend::MapBackend* wm, rcl_map_backend::MapBackend* lm, double r=0.05, QWidget *parent = nullptr);
        virtual ~Painter();

        void setSharedMem(SharedMem* sm);
    public Q_SLOTS:
        void scanUpdate(const std::vector<double>& xs, const std::vector<double>& ys);
    public Q_SLOTS:
        void predictedPoseUpdate(double x, double y, double theta);
    };
}
#endif
