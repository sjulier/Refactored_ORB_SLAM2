#ifndef FBOWVOCABULARY_H
#define FBOWVOCABULARY_H

#pragma once
#include <fbow.h>

class FbowVocabulary {
public:
    bool load(const std::string& path){
        voc_.readFromFile(path); return true;
    }
    void transform(const cv::Mat& desc,
                   fbow::fBow& bow,
                   fbow::fBow2& feat,
                   int level=4){
        voc_.transform(desc, level, bow, feat);
    }
    double score(const fbow::fBow& a, const fbow::fBow& b) const{
        return fbow::fBow::score(a,b);
    }
    size_t size() const { return voc_.size(); }
private:
    fbow::Vocabulary voc_;
};

#endif //FBOWVOCABULARY_H
