#ifndef _CANDIDATE_H_
#define _CANDIDATE_H_

#include "opencv2/imgproc/imgproc.hpp"

struct Candidate
{
    cv::Point2d position;
    int val_left;
    int val_position;
    int val_right;
};

#endif