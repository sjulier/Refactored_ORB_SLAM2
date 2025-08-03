// CorrelationMatcher.cc
#include "CorrelationMatcher.h"
#include "CorrelationEdge.h"
#include <fstream>
#include <opencv2/flann.hpp>
#include <map>
#include <iomanip>

namespace ORB_SLAM2 {

CorrelationMatcher::CorrelationMatcher() {
    mvStats.reserve(5000);
}

/*
void CorrelationMatcher::Finalize() {
    if(mvStats.empty()) return;

    std::ofstream log("CorrelationStatus.txt");
    log << "#frame chA chB RI nCorr nA nB\n";

    double sCorr=0,sA=0,sB=0;
    for(const auto& s: mvStats){
        log << s.frameId << ' '<<s.chA<<' '<<s.chB<<' '
            << s.ri      << ' '<<s.nCorr<<' '
            << s.nA      << ' '<<s.nB   << '\n';

        sCorr += s.nCorr; sA += s.nA; sB += s.nB;
    }

    const double F = mvStats.size();
    const double avgC = sCorr/F, avgA = sA/F, avgB = sB/F;
    const double RIg_MNR  = avgC / std::min(avgA, avgB);
    const double RIg_GNR  = avgC / std::sqrt(avgA * avgB);
    const double RIg_DICE = 2.0 * avgC / (avgA + avgB);

    log << "\n# ---------- Global Summary ----------\n";
    log << "# CorrFrames: " << F << '\n';
    log << "avg_CorrEdges: " << avgC << '\n';
    log << "avg_match_A: " << avgA << '\n';
    log << "avg_match_B: " << avgB << '\n';
    log << "global_RI_MNR: " << RIg_MNR << '\n';
    log << "global_RI_GNR: " << RIg_GNR << '\n';
    log << "global_RI_DICE: " << RIg_DICE << '\n';
}
*/

void CorrelationMatcher::Finalize() {
    if (mvStats.empty()) return;

    std::ofstream log("CorrelationStatus.txt");
    auto& out = log; // used for uniform log and stdout output

    std::ostream& log_and_cout = std::cout;

    log << "#frame chA chB nCorr nA nB MNR GNR DICE\n";

    float sCorr = 0, sA = 0, sB = 0;
    const float F = mvStats.size();

    struct Stat {
        float sumCorr = 0, sumA = 0, sumB = 0;
		float sumMNR = 0, sumGNR = 0, sumDICE = 0;
        int count = 0;
    };
    std::map<std::pair<int, int>, Stat> channelStats;

    for (const auto& s : mvStats) {
        log << s.frameId << ' ' << s.chA << ' ' << s.chB << ' '
            << s.nCorr << ' ' << s.nA << ' ' << s.nB << ' '
            << s.MNR << ' ' << s.GNR << ' ' << s.DICE << '\n';

        sCorr += s.nCorr;
        sA += s.nA;
        sB += s.nB;

        auto key = std::make_pair(s.chA, s.chB);
        auto& cs = channelStats[key];
        cs.sumCorr += s.nCorr;
        cs.sumA += s.nA;
        cs.sumB += s.nB;
		cs.sumMNR += s.MNR;
		cs.sumGNR += s.GNR;
		cs.sumDICE += s.DICE;
        cs.count++;
    }

    const float avgC = sCorr / F;
    const float avgA = sA / F;
    const float avgB = sB / F;
    const float RIg_MNR = avgC / std::min(avgA, avgB);
    const float RIg_GNR = avgC / std::sqrt(avgA * avgB);
    const float RIg_DICE = 2.0 * avgC / (avgA + avgB);

    log << "\n# ---------- Global Summary ----------\n";
    log << "# CorrFrames: " << F << '\n';
    log << "avg_CorrEdges: " << avgC << '\n';
    log << "avg_match_A: " << avgA << '\n';
    log << "avg_match_B: " << avgB << '\n';
    log << "global_RI_MNR: " << RIg_MNR << '\n';
    log << "global_RI_GNR: " << RIg_GNR << '\n';
    log << "global_RI_DICE: " << RIg_DICE << '\n';

    std::cout << "\n========== Global Summary ==========\n";
    std::cout << "Frames: " << F << "\n";
    std::cout << "avg_CorrEdges: " << avgC << "\n";
    std::cout << "avg_match_A: " << avgA << "\n";
    std::cout << "avg_match_B: " << avgB << "\n";
    std::cout << "global_RI_MNR: " << RIg_MNR << "\n";
    std::cout << "global_RI_GNR: " << RIg_GNR << "\n";
    std::cout << "global_RI_DICE: " << RIg_DICE << "\n";

    log << "\n# ---------- Per-Channel Summary ----------\n";
    std::cout << "\n========== Per-Channel Summary ==========\n";
    std::cout << std::setw(6) << "chA" << std::setw(6) << "chB"
              << std::setw(12) << "avgCorr" << std::setw(12) << "avgA"
              << std::setw(12) << "avgB" << std::setw(12) << "avgRI_MNR"
              << std::setw(12) << "avgRI_GNR" << std::setw(12) << "avgRI_DICE" << "\n";

    for (const auto& [key, stat] : channelStats) {
        const float f = stat.count;
        const float avgC = stat.sumCorr / f;
        const float avgA = stat.sumA / f;
        const float avgB = stat.sumB / f;
		// Averaged Index Version
        // const float RIm_MNR = avgC / std::min(avgA, avgB);
        // const float RIm_GNR = avgC / std::sqrt(avgA * avgB);
        // const float RIm_DICE = 2.0 * avgC / (avgA + avgB);
		// Index Averaged Version
		const float RIm_MNR = stat.sumMNR / f;
        const float RIm_GNR = stat.sumGNR / f;
        const float RIm_DICE = stat.sumDICE / f;

        log << "chA: " << key.first << " chB: " << key.second
            << " | avgC: " << avgC
            << " avgA: " << avgA
            << " avgB: " << avgB
            << " MNR: " << RIm_MNR
            << " GNR: " << RIm_GNR
            << " DICE: " << RIm_DICE << '\n';

        std::cout << std::setw(6) << key.first << std::setw(6) << key.second
                  << std::setw(12) << avgC
                  << std::setw(12) << avgA
                  << std::setw(12) << avgB
                  << std::setw(12) << RIm_MNR
                  << std::setw(12) << RIm_GNR
                  << std::setw(12) << RIm_DICE << "\n";
    }
}

float CorrelationMatcher::BuildEdges(Frame& F, int chA, int chB, float th_px, size_t th_str)
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
	size_t nCorr = 0;
	const float radius = th_px * th_px;

	for (const auto& ns : vSrc) {
    	float q[2] = { ns.first.x, ns.first.y };
    	cv::Mat query(1, 2, CV_32F, q);

    	idxs.clear();  dists.clear();
    	int nFound = kd.radiusSearch(query,
                                     idxs,
                                 	 dists,
                                 	 radius,
                                 	 vDst.size(),
                                 	 cv::flann::SearchParams(INT_MAX, 0, false));
    	MapPoint* pSrc = ns.second;
    	for (int k = 0; k < nFound; ++k) {
        	int id = idxs[k];
        	MapPoint* pDst = idmap[id];
        	if (pSrc == pDst) continue;

        	size_t cnt = pSrc->AddEdge(pDst);
        	if (cnt >= th_str)
            	++nCorr;
    	}
	}

	/*
	// --------------- Brute force exhaustive matching -------------- //

    size_t nCorr = 0;
    // size_t nEdges = 0, totHits = 0, maxHits = 0;

    for(const auto& ns : vSrc){
        int hits = 0;
        for(const auto& nd : vDst){
            float dx = ns.first.x - nd.first.x;
            float dy = ns.first.y - nd.first.y;
            if(dx*dx + dy*dy <= th_px*th_px){
                //++hits; // DEBUG LOGGING
                if (ns.second != nd.second) {
                    size_t counts = ns.second->AddEdge(nd.second);
                    //++nEdges; // DEBUG LOGGING
                    if (counts >= th_str)
                        ++nCorr;
                }
            }
        }
        //totHits += hits; // DEBUG LOGGING
        //maxHits  = std::max(maxHits, (size_t)hits); // DEBUG LOGGING
    }

	*/

    // DEBUG LOGGING
	/*
    std::cout << "[CM] totHits=" << totHits
              << " avgHits=" << (double)totHits/vSrc.size()
              << " maxHits=" << maxHits
              << " validEdges: " << nEdges
              << " nCorr: " << nCorr << '\n';
	*/

    // Save correlation status for evaluation and logging
    size_t nA = vSrc.size();
    size_t nB = vDst.size();
	float fnA = (float) nA;
	float fnB = (float) nB;
    float MNR = (float) nCorr / std::min(fnA, fnB);
	float GNR = (float) nCorr / std::sqrt(fnA * fnB);
	float DICE = 2.0f * (float) nCorr / (fnA + fnB);
    CorrFrameStat cs{F.mnId, chA, chB, nCorr, nA, nB, MNR, GNR, DICE};
    mvStats.emplace_back(cs);

    return MNR;
}

} // namespace ORB_SLAM2
