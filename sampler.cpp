// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include <stdlib.h>
// #include <stdio.h>
// #include <iostream>
// #include <vector>

// #include "cvplot.h"

// #include "make_wav.h"

// /*
// Points are (x, y)

// mat.at<type>  are (y, x)
// */

// using namespace std;
// using namespace cv;

// struct Candidate
// {
//     Point2d position;
//     int val_left;
//     int val_position;
//     int val_right;
// };

// struct Trace
// {
//     vector<Point> pts_rise_edge;
//     vector<Point> pts_fall_edge;
// };

// typedef unsigned char Uint8;

// static Mat resize_mat(const Mat& src);
// static void filter(const Mat& mat_src, Mat& mat_dy);
// static vector<Point> extreme_values_from_col(const Mat& mat_src, int pos_column);
// static vector<Point> mark_extremes(const Mat& mat_src, int col, int threshold);

// int find_min(const Mat& mat_col_header /*1D array*/, int idx_begin, int length_sweep)
// {
//     int min;

//     min = 255;

//     for (int idx = idx_begin; idx < idx_begin + length_sweep; ++idx)
//     {
//         int val = mat_col_header.at<unsigned char>(idx);
//         if(min > val)
//         {
//             min = val;
//         }
//     }

//     return min;
// }

// int find_max(const Mat& mat_col_header /*1D array*/, int idx_begin, int length_sweep)
// {
//     int max;

//     max = 0;

//     for (int idx = idx_begin; idx < idx_begin + length_sweep; ++idx)
//     {
//         int val = mat_col_header.at<unsigned char>(idx);
//         if(max < val)
//         {
//             max = val;
//         }
//     }

//     return max;
// }

// // resize matrix to show
// static Mat resize_mat(const Mat& src)
// {
//     const int max_size = 500;

//     Mat src_resize;

//     float sz_new_width = max_size;
//     float sz_new_height = sz_new_width*src.rows/src.cols;

//     Size size(sz_new_width, sz_new_height);

//     if(src.cols > max_size || src.rows > max_size)
//     {
//         resize(src, src_resize, size, 0, 0, CV_INTER_AREA);
//     }
//     else
//     {
//         resize(src, src_resize, size, 0, 0, CV_INTER_LINEAR);
//     }

//     return src_resize;
// }


// // This function purpuse is to get the d/dy derivative matrix from source matrix
// static void filter(const Mat& mat_src, Mat& mat_dy)
// {
//     const int TRACE_WIDTH = 130;
//     const float ALPHA = 0.2;

//     int sz_kernel = ALPHA*TRACE_WIDTH;

//     if(sz_kernel%2 == 0)
//         sz_kernel++;


//     // set kernel
//     Mat mat_kernel = Mat::ones( sz_kernel, 1, CV_32F );
//     for (int i = 0; i < sz_kernel/2; ++i)
//     {
//         mat_kernel.at<float>(i, 0) = -1;
//     }

//     mat_kernel.at<float>(sz_kernel/2, 0) = 0;

//     // normalize values.
//     for (int i = 0; i < sz_kernel; ++i)
//     {
//         mat_kernel.at<float>(i, 0) /= (float)sz_kernel;
//     }

//     // Apply filter
//     Point arg_anchor = Point( -1, -1 );
//     double arg_delta = 0;
//     int arg_ddepth = CV_32F;
//     // int arg_ddepth = -1;
//     filter2D(mat_src, mat_dy, arg_ddepth , mat_kernel, arg_anchor, arg_delta, BORDER_DEFAULT );
// }

// static vector<Point> mark_extremes(const Mat& mat_src, int col, int threshold)
// {
//     vector<Point> pts_max;

//     for (int i = 0; i < mat_src.rows-3; ++i)
//     {
//         // get if derivative is roughly 0.
//         float delta1 = mat_src.at<float>(i+1, col) - mat_src.at<float>(i, col);
//         float delta2 = mat_src.at<float>(i+3, col) - mat_src.at<float>(i+2, col);

//         // if its about 0 and its bigger than the threshold, its marked.
//         if(delta1 * delta2 <= 0 && fabs(mat_src.at<float>(i+2, col)) > threshold )
//         {
//             pts_max.push_back(Point(col,i+2));
//             i += 3;
//         }
//     }

//     // fprintf(stderr, "Number of max found: %i\n", pts_max.size());
//     // fprintf(stderr, "threshold: %i\n", threshold);

//     if(pts_max.size() < 2 && threshold > 1)
//     {
//         return mark_extremes(mat_src, col, threshold*0.9);
//     }
//     else
//     {
//         return pts_max;
//     }
// }

// void edge_detection_thresh2(const Mat& mat_src, const Mat& mat_dy, vector<Candidate>& pts_candidate)
// {
//     int idx_last_marked = 0; // used as boundary
//     for (int i = 0; i <= pts_candidate.size(); ++i)
//     {
//         int idx_src_col;
//         int idx_src_row;
//         float val_col_xtrm;

//         if(i < pts_candidate.size())
//         {
//             idx_src_col = pts_candidate[i].position.x;  // column from mat_src
//             idx_src_row = pts_candidate[i].position.y;  // idx in a column seen as array in mat_src

//             val_col_xtrm = mat_dy.at<float>(idx_src_row, idx_src_col); // value of extremum in mat_dy
//         }
//         else // thats after the last point.
//         {
//             idx_src_col = pts_candidate[i-1].position.x;  // column from mat_src
//             idx_src_row = mat_src.rows;  // idx in a column seen as array in mat_src

//             val_col_xtrm = -mat_dy.at<float>(pts_candidate[i-1].position.y, pts_candidate[i-1].position.x); // value of extremum in mat_dy
//         }

//         if(val_col_xtrm > 0) // if its a positive edge (crescent from left to right)
//         {
//             int min = find_min(mat_src.col(idx_src_col), idx_last_marked, idx_src_row - idx_last_marked);

//             // set candidate neighbour values (A and B from edge model)
//             if(i > 0)
//             {
//                 pts_candidate[i-1].val_right = min;
//             }
//             if(i < pts_candidate.size())
//             {
//                 pts_candidate[i].val_left = min;
//             }
//         }
//         else // if its a negative edge
//         {
//             int max = find_max(mat_src.col(idx_src_col), idx_last_marked, idx_src_row - idx_last_marked);

//             // set candidate neighbour values (A and B from edge model)
//             if(i > 0)
//             {
//                 pts_candidate[i-1].val_right = max;
//             }
//             if(i < pts_candidate.size())
//             {
//                 pts_candidate[i].val_left = max;
//             }
//         }

//         idx_last_marked = idx_src_row;
//     }
// }

// // static vector<float> extract_audio_cludge(const Mat& mat_src)
// // {
// //     vector<float> max_vals;
// //     vector<float> dif;

// //     for (int i = 0; i < mat_src.cols; ++i)
// //     {
// //         Point result = get_max_cludge(mat_src, i, 0, 150);
// //         int index = result.y;
// //         max_vals.push_back(index);
// //     }

// //     for (int i = 0; i < max_vals.size()-1; ++i)
// //     {
// //         dif.push_back((float)(max_vals[i+1] - max_vals[i])/);
// //     }

// //     // int max = 0;
// //     // for (int i = 0; i < dif.size()-1; ++i)
// //     // {
// //     //     if(fabs(dif[i]) > max)
// //     //         max = fabs(dif[i]);
// //     // }

// //     // cout << "MAX VARIANCE: " << max << endl;

// //     return dif;

// // }

// /** @function main */
// int main ( int argc, char** argv )
// {
//     Mat mat_src;
//     Mat mat_dy;

//     // Load an image
//     mat_src = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE);

//     // This force the image to be in a landscape format
//     if(mat_src.rows > mat_src.cols)
//     {
//         mat_src = mat_src.t();
//     }

//     // negative src so the traces are white.
//     bitwise_not(mat_src, mat_src);

//     mat_dy.create(mat_src.rows, mat_src.cols, CV_32FC1);

//     if( !mat_src.data )
//         { return -1; }

//     // get the smooth derivative from mat_src.
//     filter(mat_src, mat_dy);

// ///
//     // Trace fake_trace; // will hold edges from 2 different traces, just as exercise.

//     // for (int i = 0; i < mat_dy.cols; ++i)
//     // {
//     //     vector<Point> pts_max = mark_extremes(mat_dy, i, 30 /*threshold example*/);

//     //     if(pts_max.size() > 2)
//     //     {
//     //         cout << "pts_max: " << pts_max.size() << endl;
//     //         fake_trace.pts_rise_edge.push_back(pts_max[1]);
//     //         fake_trace.pts_fall_edge.push_back(pts_max[2]);
//     //     }
//     //     else
//     //     {
//     //         fake_trace.pts_rise_edge.push_back(pts_max[0]);
//     //         fake_trace.pts_fall_edge.push_back(pts_max[1]);
//     //     }

//     // }

//     // cout << "faketrace:\n" << fake_trace.pts_rise_edge << endl;
// ///

//     vector<Point> pts_max = mark_extremes(mat_dy, 0, 30 /*threshold example*/);
//     edge_detection_thresh(mat_src, pts_max);

//     // vector<int> result;
//     // result = extract_audio_cludge(mat_dy);

//     // cout << "result.size(): " << result.size() << endl;

//     // short int* vec = new short int[result.size()];

//     // for (int k = 0; k < result.size(); ++k)
//     // {
//     //     vec[k] = (short int)1000*result[k];
//     //     // vec[k] = (short int)(32000*((float)result[k]/70));
//     //     fprintf(stderr, "vec[%i]: %i\n", k, vec[k]);
//     // }

//     // write_wav("testee.wav", result.size(), vec, 104000);


//     // vector<Point> max_pnts = extreme_values_from_col(mat_dy, 0, 0, mat_dy.rows);

//     // namedWindow( "Tits", CV_WINDOW_AUTOSIZE );
//     // imshow( "Tits", resize_mat(mat_dy) );

//     waitKey(0);

//     return 0;
// }