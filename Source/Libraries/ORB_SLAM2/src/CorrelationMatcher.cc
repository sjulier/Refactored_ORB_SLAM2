// CorrelationMatcher.cc
#include "CorrelationMatcher.h"
#include "CorrelationEdge.h"
#include <opencv2/flann.hpp>

namespace ORB_SLAM2 {

float BuildCorrelationEdges(Frame& F, int chA, int chB, float th_px, size_t th_str)
{
    const int N = F.Ntype;
    // Check input validity
    if(chA<0 || chA>=N || chB<0 || chB>=N || chA==chB) return 0;

    using Node = std::pair<cv::Point2f, MapPoint*>;
    std::vector<Node> vSrc, vDst;

    /* ---------- 1) Collect the matched points ---------- */
    auto collect = [&](int ch, std::vector<Node>& v){
        const auto& C = F.Channels[ch];
        for(int i=0;i<C.N;++i){
            MapPoint* p = C.mvpMapPoints[i];

            // *** Filter conditions ***
            if(!p)                    continue;          // No MapPoint pointer
            if(p->isBad())            continue;          // Has been marked as bad
            if(C.mvbOutlier[i])       continue;          // Judged as outlier
            if(p->Observations() < 1)  continue;         // Temporary VO point

            v.emplace_back(C.mvKeysUn[i].pt, p);         // (pixel coordinates, MapPoint*)
        }
    };
    collect(chA, vSrc);
    collect(chB, vDst);

    // std::cout << "Channel " << chA << " valid points: " << vSrc.size() << std::endl;
    // std::cout << "Channel " << chB << " valid points: " << vDst.size() << std::endl;

    if(vSrc.empty() || vDst.empty()) return 0;      // No available points

    /*

    // ---------- 2) Create KD-Tree on target channel ---------- //
    cv::Mat data(vDst.size(), 2, CV_32F);
    std::vector<MapPoint*> idmap; idmap.reserve(vDst.size());
    for(size_t i=0;i<vDst.size(); ++i){
        data.at<float>(i,0) = vDst[i].first.x;
        data.at<float>(i,1) = vDst[i].first.y;
        idmap.push_back(vDst[i].second);
    }
    cv::flann::Index kd(data, cv::flann::KDTreeIndexParams(1));

    // ---------- 3) Radius search and accumulate edges ---------- //
    std::vector<int>   idxs;
    std::vector<float> dists;
    size_t nEdges = 0;

    // DEBUG
    size_t totHits = 0;
    size_t maxHits = 0;

    for(const auto& ns : vSrc){
        float q[2] = { ns.first.x, ns.first.y };
        idxs.clear(); dists.clear();

        cv::Mat query(1, 2, CV_32F, q);
        kd.radiusSearch(query,
                idxs,
                dists,
                th_px,
                static_cast<int>(vDst.size()),
                cv::flann::SearchParams(-1, 0, false));

        // DEBUG
        totHits += idxs.size();
        maxHits  = std::max(maxHits, idxs.size());

        MapPoint* pSrc = ns.second;
        for(int id : idxs){
            if(pSrc == idmap[id]) continue;             // Skip the same point
            pSrc->AddEdge(idmap[id]);                   // Symmetric increatment count
            ++nEdges;
        }
    }

    */

    size_t nCorr = 0, nEdges = 0, totHits = 0, maxHits = 0;

    for(const auto& ns : vSrc){
        int hits = 0;
        for(const auto& nd : vDst){
            float dx = ns.first.x - nd.first.x;
            float dy = ns.first.y - nd.first.y;
            if(dx*dx + dy*dy <= th_px*th_px){
                ++hits;
                if (ns.second != nd.second) {
                    size_t counts = ns.second->AddEdge(nd.second);
                    ++nEdges;
                    if (counts >= th_str)
                        ++nCorr;
                }
            }
        }
        totHits += hits;
        maxHits  = std::max(maxHits, (size_t)hits);
    }

    std::cout << "[CM] totHits=" << totHits
              << " avgHits=" << (double)totHits/vSrc.size()
              << " maxHits=" << maxHits
              << " validEdges: " << nEdges
              << " nCorr: " << nCorr << '\n';

    return (2.0f * nCorr) / static_cast<float>(std::min(vSrc.size(), vDst.size()));
}

} // namespace ORB_SLAM2
