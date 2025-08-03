#include <iostream>
#include <fstream>
#include <chrono>
#include <fbow.h>
#include <vocabulary_creator.h>

#include "FeatureExtractorFactory.h"
using namespace ORB_SLAM2;

static std::vector<std::string> LoadImageList(const std::string &list)
{
    std::ifstream ifs(list);
    std::vector<std::string> v;
    for (std::string l; std::getline(ifs, l); )
        if (!l.empty()) v.push_back(l);
    return v;
}

int main(int argc, char **argv)
{
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <ExtractorName> <imagelist.txt> <out.fbow> <k> <L> [settings.yaml]\n";
        return 1;
    }
    std::string yamlFile = (argc >= 7) ? argv[6] : "";

    std::string extName  = argv[1];
    std::string listFile = argv[2];
    std::string outFile  = argv[3];
    int k  = std::stoi(argv[4]);
    int L  = std::stoi(argv[5]);

    /* ---------- 1) Create Extractor ---------- */
    cv::FileNode cfg;
    if (!yamlFile.empty()) {
        cv::FileStorage fs(yamlFile, cv::FileStorage::READ);
        if (!fs.isOpened()) { std::cerr << "Cannot open " << yamlFile << "\n"; return 2; }
        cfg = fs.root();
    }
    auto pExtractor = std::unique_ptr<FeatureExtractor>(
        FeatureExtractorFactory::Instance().Create(extName, cfg, true));
    if (!pExtractor) { std::cerr << "Extractor '" << extName << "' not registered.\n"; return 3; }
	std::cout << "[ FBoW Training ] Training on '" << extName << "' Feature ... " << std::endl;

    /* ---------- 2) Read Images & Extract Discriptors ---------- */
    auto vImg = LoadImageList(listFile);
    std::cout << "Total images: " << vImg.size() << "\n";
    std::vector<cv::Mat> all_desc;
    size_t total_feat = 0;

    auto t0 = std::chrono::steady_clock::now();

    size_t idx = 0;
    for (auto &path : vImg)
    {
        cv::Mat im = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (im.empty()) { std::cerr << "Skip " << path << "\n"; continue; }

        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        (*pExtractor)(im, cv::noArray(), kps, desc);

        if (!desc.empty()) {
            total_feat += desc.rows;
            all_desc.push_back(desc);
        }

        if (++idx % 100 == 0)
        {
            auto now = std::chrono::steady_clock::now();
            double secs = std::chrono::duration<double>(now - t0).count();
            std::cout << "\r[" << idx << "/" << vImg.size()
                      << "]  feats: " << total_feat
                      << "  (" << idx / secs << " img/s) " << std::flush;
        }
    }
    std::cout << "\nDone collecting descriptors: " << total_feat << "\n";

    if (all_desc.empty()) { std::cerr << "No descriptors collected.\n"; return 3; }

    /* ---------- 3) Train FBoW  ---------- */
    fbow::VocabularyCreator::Params param;
    param.k        = k;
    param.L        = L;
    param.nthreads = std::thread::hardware_concurrency();
    param.verbose  = true;

    fbow::Vocabulary voc;
    std::cout << "Training vocabulary (k=" << k << ", L=" << L << ") ...\n";
    fbow::VocabularyCreator().create(voc, all_desc, extName, param);

    /* ---------- 4) Save FBoW ---------- */
    voc.saveToFile(outFile);
    std::cout << "\nSaved vocabulary to " << outFile << "\n";
    return 0;
}
