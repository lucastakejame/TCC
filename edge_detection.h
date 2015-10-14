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

struct Trace
{
    vector<double> pts_rise_edge;
    vector<double> pts_fall_edge;

    // double tolerance = 100; // called m in the thesis, its the distance of tolerance around the reference candidate

    // time slice variables

    double pos_estimated_rise; // this will change from column to column
    double pos_estimated_fall; //

    vector<double> candidates_r_k;
    vector<double> candidates_f_k;

    vector<double> mean_trace_width; // trace width estimated to that time slice (column).
};

#define cerr(x) cerr << x;
#define cerrln(x) cerr << x << endl;
#define cerrv(x) cerr <<  #x << " : " << x << endl;

#define testing 1
#define trust(a_true_condition) if (testing && !(a_true_condition)) {cerr("dont trust line: ");cerr(__LINE__);cerr(" from file ");cerrln(__FILE__);};

int find_min(const Mat& mat_col, int idx_begin, int length_sweep);
int find_max(const Mat& mat_col, int idx_begin, int length_sweep);
double get_median(vector<double>& distances);


Mat col_average(const Mat& mat_src, int idx_col_first, int n_cols);

void filter(const Mat& mat_src, Mat& mat_dy, int trace_width, float alpha);

vector<Candidate> get_edge_candidates(const Mat& mat_src_col, const Mat& mat_dy_col, float threshold);
void get_A_B(const Mat& mat_src_col, const Mat& mat_dy_col, vector<Candidate>& pts_candidate);

void edge_detection_coarse(const Mat& mat_src_col, const Mat& mat_dy_col, vector<Candidate>& pts_candidate);

void edge_detection_fine(const Mat& mat_src_col, vector<Candidate>& pts_candidate);

Mat n_first_last_col_average(const Mat& mat_ring_first, const Mat& mat_ring_last, int range_average);

vector<Trace> initialize_traces(const Mat& mat_ring_first, const Mat& mat_ring_last, int range_average);

double closest_to_estimation(vector<double> possible_candidates, double estimated_edge);

void define_traces(const Mat& mat_src, vector<Candidate> pts_candidate, vector<Trace>& traces, int analyzed_col);

vector<Trace> trace_following(const Mat& mat_src, const Mat& mat_src2, Mat& mat_visual);


#endif