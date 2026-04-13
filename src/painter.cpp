#include "painter.h"
#include "teleopt.hpp"

#include <QDebug>
#include <QPushButton>

#include <cstdlib>

int map_size_x = 400;
int map_size_y = 400;

namespace rcl_painter{
    using namespace rcl_pose_graph;
    using namespace rcl_map_backend;

    Painter::Painter(PoseGraph* pg, MapBackend* wm, MapBackend* lm, double r, QWidget *parent):QWidget(parent), pos_r{r}, pixmap(map_size_x, map_size_y), lader_pixmap(map_size_x, map_size_y), world_pixmap(map_size_x, map_size_y) {
        pose_graph = pg;
        world_map = wm;
        local_map = lm;

        pixmap.fill(Qt::transparent);
        lader_pixmap.fill(Qt::transparent);
        world_pixmap.fill(Qt::gray);
        QPushButton* btn = new QPushButton("Toggle Lidar", this);
        QObject::connect(btn, &QPushButton::clicked, [this](){
            is_lidar_visible = !is_lidar_visible;
        });
        btn->setGeometry(10, 10, 100, 30);

        label = new QLabel("", this);
        label->setGeometry(120, 10, 200, 30);
    }

    Painter::~Painter(){}

    void Painter::drawScan(const std::vector<double>& xs, const std::vector<double>& ys){
        lader_pixmap.fill(Qt::transparent);
        QPainter painter(&lader_pixmap);
        QPen pen(Qt::green);
        pen.setWidth(1);
        painter.setPen(pen);

        double min = -16, max = 16;
        for(size_t i=0; i<xs.size(); i++){
            double mx = (xs[i] - min) / (max - min) * map_size_x;
            double my = (-ys[i] - min) / (max - min) * map_size_y;
            painter.drawPoint(mx, my);
        }
        update();
    }
    void Painter::drawPose(double x, double y, double theta){
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        QPen pen(Qt::red);
        pen.setWidth(1);
        painter.setPen(pen);

        double min = -16, max = 16;
        for(int i=0; i<static_cast<int>(pose_graph->getPoseCount()); i++){
            auto p = pose_graph->getPose(i);
            double mx = (p.tx - min) / (max - min) * map_size_x;
            double my = (-p.ty - min) / (max - min) * map_size_y;
            painter.drawPoint(mx, my);
        }

        // 현재 위치 강조
        static std::vector<double> arrow_x{-2, -1, 0, 1, 2, 2, 2,  2,  2, 3, 3,  3, 4};
        static std::vector<double> arrow_y{ 0,  0, 0, 0, 2, 1, 0, -1, -2, 1, 0, -1, 0};

        for(size_t i=0; i<arrow_x.size(); i++){
            double mx = (arrow_x[i]) * pos_r * 3;
            double my = (arrow_y[i]) * pos_r * 3;
            double rx = std::cos(theta) * mx - std::sin(theta) * my + x;
            double ry = std::sin(theta) * mx + std::cos(theta) * my + y;
            double px = (rx - min) / (max - min) * map_size_x;
            double py = (-ry - min) / (max - min) * map_size_y;
            painter.drawPoint(px, py);
        }
    }

    void Painter::drawWorldMap(){
        world_pixmap.fill(Qt::gray);
        QPainter painter(&world_pixmap);
        QPen pen(Qt::black);
        pen.setWidth(1);
        painter.setPen(pen);

        double min = -16, max = 16;
        std::vector<double> x, y;
        world_map->getPos(x, y, true);
        for(size_t i=0; i<x.size(); i++){
            double mx = (x[i] - min) / (max - min) * map_size_x;
            double my = (-y[i] - min) / (max - min) * map_size_y;
            painter.drawPoint(mx, my);
        }
        local_map->getPos(x, y);
        for(size_t i=0; i<x.size(); i++){
            double mx = (x[i] - min) / (max - min) * map_size_x;
            double my = (-y[i] - min) / (max - min) * map_size_y;
            painter.drawPoint(mx, my);
        }
        update();
    }

    void Painter::paintEvent(QPaintEvent* event){
        if(shared_mem){
            label->setText(QString("speed: %1, theta: %2").arg(shared_mem->get_speed()).arg(shared_mem->get_theta()));
        }

        QPainter painter(this);
        painter.drawPixmap(0, 0, 600, 600, world_pixmap);
        if(is_lidar_visible) painter.drawPixmap(0, 0, 600, 600, lader_pixmap);
        painter.drawPixmap(0, 0, 600, 600, pixmap);

        QWidget::paintEvent(event);
    }

    void Painter::setSharedMem(SharedMem* sm){
        shared_mem = sm;
    } 

    void Painter::scanUpdate(const std::vector<double>& xs, const std::vector<double>& ys){
        drawScan(xs, ys);
        repaint();
    }
    void Painter::predictedPoseUpdate(double x, double y, double theta){
        drawWorldMap();
        drawPose(x, y, theta);
        repaint();
    }
}