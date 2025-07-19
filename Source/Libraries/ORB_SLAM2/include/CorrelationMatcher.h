#ifndef CORRELATIONMATCHER_H
#define CORRELATIONMATCHER_H

// EdgeBuilder.h
#pragma once
#include "Frame.h"

namespace ORB_SLAM2 {

    /**
     * @brief For a given frame F, perform KD-Tree radius search between two feature channels:
     *        For each successfully matched point in channel chA, search for points in channel chB
     *        within a pixel distance < th_px. For each match found, call MapPoint::AddEdge() to
     *        increment edge weight between the corresponding MapPoints.
     *
     * @param F       The current frame (can be a Frame or a KeyFrame)
     * @param chA     Source feature channel index
     * @param chB     Target feature channel index (can be greater or less than chA)
     * @param th_px   Pixel radius threshold (default is 2.0f)
     *
     * @return The number of newly added or updated edges, useful for debugging or statistics
     */

    size_t BuildCorrelationEdges(Frame& F,
                                 int    chA,
                                 int    chB,
                                 float  th_px = 2.0f);

} // namespace

#endif //CORRELATIONMATCHER_H
