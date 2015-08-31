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

// // struct PandV
// // {
// //     Point pt;
// //     float val;
// // };

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

// void edge_detection_thresh(const Mat& mat_src, vector<Point> pts_max)
// {
//     // for (int i = 0; i < pts_max.size(); ++i)
//     int i = 0;
//     {
//         int idx_range = 8;
//         int idx_profile = pts_max[i].y;
//         int idx_init = idx_profile - idx_range;
//         int idx_end = idx_profile + idx_range;

//         if(idx_init < 0)
//         {
//             idx_init = 0;
//         }
//         if(idx_end >= mat_src.rows)
//         {
//             idx_end = mat_src.rows-1;
//         }

//         int min = 255;
//         int max = 0;
//         for (int idx = idx_init; idx < idx_end; ++idx)
//         {
//             int val = mat_src.at<unsigned char>(idx, i);
//             if(min > val)
//             {
//                 min = val;
//             }
//             if(max < val)
//             {
//                 max = val;
//             }

//         }

//         fprintf(stderr, "min: %i\n", min);
//         fprintf(stderr, "max: %i\n", max);

//     }


// }


// // static Point get_max_cludge(const Mat& mat_src, int col, int index_start, int index_end)
// // {
// //     float max = 0;
// //     Point pnt_max = Point(0,0);

// //     if(index_start < 0)
// //         index_start = 0;

// //     if(index_end > mat_src.rows)
// //         index_end = mat_src.rows;

// //     for (int i = index_start; i < index_end; ++i)
// //     {
// //         if(mat_src.at<int>(i, col) > max)
// //         {
// //             max = mat_src.at<int>(i, col);
// //             pnt_max.x = col;
// //             pnt_max.y = i;
// //         }
// //     }

// //     return pnt_max;
// // }

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

// // static vector<Point> extreme_values_from_col(const Mat& mat_src, int pos_column, int row_init, int row_end)
// // {
// //     vector<Point> vec_max;
// //     Point pnt_max1(0,0);
// //     Point pnt_max2(0,0);
// //     Point pnt_max3(0,0);
// //     Point pnt_max4(0,0);

// //     int max1 = 0;
// //     int max2 = 0;
// //     int max3 = 0;
// //     int max4 = 0; //biggest

// //     Mat mat_dy;

// //     if(row_end > mat_src.rows)
// //         row_end = mat_src.rows;

// //     filter(mat_src, mat_dy);

// //     for (int pos_row = row_init; pos_row < row_end-1; ++pos_row)
// //     {
// //         int val1_dy = mat_dy.at<int>(pos_row, pos_column);
// //         int val2_dy = mat_dy.at<int>(pos_row+1, pos_column);

// //         if(val1_dy * val2_dy <= 0)
// //         {
// //             int val1_src = mat_src.at<int>(pos_row, pos_column);
// //             int val2_src = mat_src.at<int>(pos_row+1, pos_column);

// //             int local_max;
// //             Point pnt_local_max;

// //             if(fabs(val2_src) > fabs(val1_src))
// //             {
// //                 local_max = val2_src;
// //                 pnt_local_max = Point(pos_row+1, pos_column);
// //             }
// //             else
// //             {
// //                 local_max = val1_src;
// //                 pnt_local_max = Point(pos_row, pos_column);
// //             }

// //             if(fabs(local_max) > fabs(max1))
// //             {
// //                 if(fabs(local_max) > fabs(max4))
// //                 {
// //                     max1 = max2;
// //                     max2 = max3;
// //                     max3 = max4;
// //                     max4 = local_max;

// //                     pnt_max1 = pnt_max2;
// //                     pnt_max2 = pnt_max3;
// //                     pnt_max3 = pnt_max4;
// //                     pnt_max4 = pnt_local_max;
// //                 }
// //                 else if(fabs(local_max) > fabs(max3))
// //                 {
// //                     max1 = max2;
// //                     max2 = max3;
// //                     max3 = local_max;

// //                     pnt_max1 = pnt_max2;
// //                     pnt_max2 = pnt_max3;
// //                     pnt_max3 = pnt_local_max;
// //                 }
// //                 else if(fabs(local_max) > fabs(max2))
// //                 {
// //                     max1 = max2;
// //                     max2 = local_max;

// //                     pnt_max1 = pnt_max2;
// //                     pnt_max2 = pnt_local_max;
// //                 }
// //                 else
// //                 {
// //                     max1 = local_max;
// //                     pnt_max1 = pnt_local_max;
// //                 }
// //             }

// //             pos_row++;
// //             pos_row++;
// //         }
// //     }

// //     vec_max.push_back(pnt_max1);
// //     vec_max.push_back(pnt_max2);
// //     vec_max.push_back(pnt_max3);
// //     vec_max.push_back(pnt_max4);

// //     return vec_max;
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