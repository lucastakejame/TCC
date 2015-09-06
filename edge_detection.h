#ifndef _EDGE_DETECTION_H_
#define _EDGE_DETECTION_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "candidate.h"

using namespace std;
using namespace cv;

void get_A_B(const Mat& mat_src, const Mat& mat_dy, vector<Candidate>& pts_candidate);
void edge_detection_fine(const Mat& mat_src, vector<Candidate>& pts_cnddt);
vector<Candidate> get_edge_candidates(const Mat& mat_src, const Mat& mat_dy, int col, int threshold);
void filter(const Mat& mat_src, Mat& mat_dy, int trace_width, float alpha);


#endif