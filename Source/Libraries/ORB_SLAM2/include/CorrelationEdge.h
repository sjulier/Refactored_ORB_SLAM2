#ifndef MAPPOINTEDGE_H
#define MAPPOINTEDGE_H

#pragma once
#include <atomic>

namespace ORB_SLAM2 {

class MapPoint;

struct Edge {
    MapPoint* pA;
    MapPoint* pB;
    std::atomic<uint32_t> counter{1};

    Edge(MapPoint* a, MapPoint* b) : pA(a), pB(b) {}
    MapPoint* Other(MapPoint* self) const { return self==pA ? pB : pA; }
};

} // namespace ORB_SLAM2

#endif //MAPPOINTEDGE_H
