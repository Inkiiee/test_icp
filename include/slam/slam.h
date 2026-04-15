#ifndef __RCL_SLAM_H__
#define __RCL_SLAM_H__

#include "scan_match_backend.h"
#include "loop_detecter.h"
#include "bridge.h"
#include "painter.h"

#include <QObject>

class SharedMem;

namespace rcl_slam{
    class SlamSystem: public QObject{
        Q_OBJECT
    private:
        double pos_r;

        rcl_scan_match_backend::ScanMatchBackend* backend;
        rcl_loop_detecter::LoopDetecter* loop_detecter;
        rcl_painter::Painter* painter;
        Bridge* bridge = nullptr;
    public:
        SlamSystem(Bridge* b, double r=0.05,QObject* parent=nullptr);
        ~SlamSystem();

        void setSharedMem(SharedMem* sm);
    
    public Q_SLOTS:
        void rebuildMap();
    };
}



#endif // __RCL_SLAM_H__