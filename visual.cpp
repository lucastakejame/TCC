#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "edge_detection.h"
#include "cvplot.h"
#include "candidate.h"

using namespace std;
using namespace cv;

const int LIMIT_ALPHA = 10;
const int LIMIT_SZ_KERNEL = 20;
int LIMIT_SEL_COL;
const int TRACE_WIDTH = 130;

static int g_prm_avrg = 30;
static int g_prm_alpha = 2;
static int g_prm_sz_kernel = 5;
static int g_prm_sel_col = 0;

Mat g_mat_src;
Mat g_mat_src2;
Mat g_mat_dy;

static void cb_avrg(int param, void* user_data);
static void cb_filter(int param, void* user_data);
static void cb_col_profile(int param, void* user_data);
static void col_profile(const Mat& mat_src, int col_src, const char* window_name, unsigned char red, unsigned char green, unsigned char blue);
static void mark_col(Mat src, int src_col);

// resize matrix to show
Mat resize_mat(const Mat& src)
{
    const int max_size = 500;

    Mat src_resize;

    float sz_new_width = max_size;
    float sz_new_height = sz_new_width*src.rows/src.cols;

    Size size(sz_new_width, sz_new_height);

    if(src.cols > max_size || src.rows > max_size)
    {
        resize(src, src_resize, size, 0, 0, CV_INTER_AREA);
    }
    else
    {
        resize(src, src_resize, size, 0, 0, CV_INTER_LINEAR);
    }

    return src_resize;
}

static void cb_avrg(int param, void* user_data)
{
    CvPlot::clear("Average");
    CvPlot::clear("Average 2");

    Mat mat_n_average;
    Mat mat_dy;
    // mat_n_average = col_average(g_mat_src, 0, g_prm_avrg);

    if(g_prm_avrg < 1)
        g_prm_avrg = 1;

    mat_n_average = n_first_last_col_average(g_mat_src, g_mat_src2, g_prm_avrg);
/*    filter(mat_n_average, mat_dy, TRACE_WIDTH, 0.2);

    vector<Candidate> pts_candidate;
    edge_detection_coarse(mat_n_average.col(0), mat_dy.col(0), pts_candidate);
    edge_detection_fine(mat_n_average.col(0), pts_candidate);

    Mat mat_edges = Mat::zeros(mat_n_average.rows, 1, g_mat_src.type());

    for (int i = 0; i < pts_candidate.size(); ++i)
    {
        int idx = pts_candidate[i].position;
        mat_edges.at<unsigned char>(idx) = pts_candidate[i].val_position;
    }


    col_profile(mat_dy, 0, "Average_DY", 100, 50, 30);*/

    vector<Trace> traces = initialize_traces(g_mat_src, g_mat_src2, g_prm_avrg);

    Mat mat_edges = Mat::zeros(mat_n_average.rows, 1, g_mat_src.type());

    cerrv(traces.size())

    for (int i = 0; i < traces.size(); ++i)
    {
        int idx;

        idx = traces[i].pos_estimated_rise;
        mat_edges.at<unsigned char>(idx) = 200;
        cerrv(idx)
        idx = traces[i].pos_estimated_fall;
        mat_edges.at<unsigned char>(idx) = 200;
        cerrv(idx)

    }

    col_profile(mat_n_average, 0, "Average", 100, 50, 30);
    col_profile(mat_edges, 0, "Average 2", 200, 50, 50);
}

// This callback purpuse is to get the d/dy derivative matrix from source matrix
static void cb_filter(int param, void* user_data)
{
    float alpha;
    if(g_prm_alpha < 1)
        alpha = 0.1;
    else
        alpha = (float)g_prm_alpha/10;

    filter(g_mat_src, g_mat_dy, TRACE_WIDTH, alpha);
    cb_col_profile(0,0);
}

static void cb_col_profile(int param, void* user_data)
{
    Mat show_mat_dy = resize_mat(g_mat_dy);
    // normalized sel_col param, bounded in [0,1]
    float norm_sel_col = (float)g_prm_sel_col/LIMIT_SEL_COL;
    int orig_mat_col = g_prm_sel_col;

    Mat mat_dst = Mat::zeros(g_mat_dy.rows, 1, g_mat_dy.type()); // mark the extremes from mat_dy
    Mat mat_edges = Mat::zeros(g_mat_src.rows, 1, g_mat_src.type()); // mark the found edges

    vector<Candidate> pts_candidate;
    edge_detection_coarse(g_mat_src.col(orig_mat_col), g_mat_dy.col(orig_mat_col), pts_candidate);
    edge_detection_fine(g_mat_src.col(orig_mat_col), pts_candidate);

    for (int i = 0; i < pts_candidate.size(); ++i)
    {
        int idx = pts_candidate[i].position;
        mat_dst.at<float>(idx) = g_mat_dy.at<float>(idx, orig_mat_col);
    }

    for (int i = 0, k = 0; i != pts_candidate.size(); ++i)
    {
        if(i < pts_candidate.size())
        {
            while(k < pts_candidate[i].position)
            {
                // mat_edges.at<unsigned char>(k) = pts_candidate[i].val_left;
                k++;
            }
            mat_edges.at<unsigned char>(k) = pts_candidate[i].val_position;
            k++;
        }
        else
        {
            while(k < mat_edges.rows)
            {
                // mat_edges.at<unsigned char>(k) = pts_candidate[i-1].val_right;
                k++;
            }
        }

    }


    CvPlot::clear("Perfil");
    CvPlot::clear("Perfil Src");

    col_profile(g_mat_dy, orig_mat_col, "Perfil", 255, 0, 0);
    col_profile(mat_dst, 0, "Perfil", 0, 0, 255);
    col_profile(g_mat_src, orig_mat_col, "Perfil Src", 0, 255, 0);
    col_profile(mat_edges, 0, "Perfil Src", 125, 0, 125);

    int show_mat_col;
    if(show_mat_col > orig_mat_col)
        show_mat_col = orig_mat_col*(float)(show_mat_dy.cols)/(g_mat_dy.cols);
    else
        show_mat_col = (float)(show_mat_dy.cols-1)*norm_sel_col;

    mark_col(show_mat_dy, show_mat_col);

    imshow( "dy derivative", show_mat_dy );

}

static void col_profile(const Mat& mat_src, int col_src, const char* window_name, unsigned char red, unsigned char green, unsigned char blue)
{

    switch(mat_src.type())
    {
        case CV_8UC1:
        {
            unsigned char* input = (unsigned char*)(mat_src.data);
            input = input + col_src;
            CvPlot::plot(window_name, input, mat_src.rows, mat_src.cols, red, green, blue);
        }
        break;
        case CV_32SC1:
        {
            int* input = (int*)(mat_src.data);
            input = input + col_src;
            CvPlot::plot(window_name, input, mat_src.rows, mat_src.cols, red, green, blue);
        }
        break;
        case CV_32FC1:
        {
            float* input = (float*)(mat_src.data);
            input = input + col_src;
            CvPlot::plot<float>(window_name, input, mat_src.rows, mat_src.cols, red, green, blue);
        }
        break;
        default:
        {
            unsigned char* input = (unsigned char*)(mat_src.data);
            input = input + col_src;
            CvPlot::plot(window_name, input, mat_src.rows, mat_src.cols, red, green, blue);
        }
        break;
    }

    CvPlot::label("Column Profile");
}

static void mark_col(Mat mat_src, int col_src)
{
    Point start = Point(col_src,0);
    Point end = Point(col_src, mat_src.rows-1);
    int thickness = 1;
    int line_type = 8;
    line(mat_src, start, end, Scalar(255), thickness, line_type);
}

/*
Changes mat to landscape format
negative
*/
void prepare_mat(Mat& src)
{
    if(src.rows > src.cols)
    {
        src = src.t();
    }

    bitwise_not(src, src);
}

/** @function main */
int main ( int argc, char** argv )
{
    vector<Mat> vec_mat;
    Mat fig0;
    Mat fig1;
    Mat fig2;
    Mat fig3;
    Mat fig4;
    Mat fig5;
    Mat fig6;
    Mat fig7;

    if(argc > 2)
    {
        switch(argc)
        {
            case 9:
            {
                fig7 = imread(argv[8], CV_LOAD_IMAGE_GRAYSCALE);
            }
            case 8:
            {
                fig6 = imread(argv[7], CV_LOAD_IMAGE_GRAYSCALE);
            }
            case 7:
            {
                fig5 = imread(argv[6], CV_LOAD_IMAGE_GRAYSCALE);
            }
            case 6:
            {
                fig4 = imread(argv[5], CV_LOAD_IMAGE_GRAYSCALE);
            }
            case 5:
            {
                fig3 = imread(argv[4], CV_LOAD_IMAGE_GRAYSCALE);
            }
            case 4:
            {
                fig2 = imread(argv[3], CV_LOAD_IMAGE_GRAYSCALE);
            }
            case 3:
            {
                fig0 = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
                fig1 = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
            }
            break;
        }
    }

    Mat h_concat;

    if(argc >= 3)
    {
        vconcat(fig0, fig1, h_concat);
        g_mat_src2 = fig1;
    }
    if(argc >= 4)
    {
        vconcat(h_concat, fig2, h_concat);
        g_mat_src2 = fig2;
    }
    if(argc >= 5)
    {
        vconcat(h_concat, fig3, h_concat);
        g_mat_src2 = fig3;
    }
    if(argc >= 6)
    {
        vconcat(h_concat, fig4, h_concat);
        g_mat_src2 = fig4;
    }
    if(argc >= 7)
    {
        vconcat(h_concat, fig5, h_concat);
        g_mat_src2 = fig5;
    }
    if(argc >= 8)
    {
        vconcat(h_concat, fig6, h_concat);
        g_mat_src2 = fig6;
    }
    if(argc >= 9)
    {
        vconcat(h_concat, fig7, h_concat);
        g_mat_src2 = fig7;
    }

    prepare_mat(h_concat);
    prepare_mat(g_mat_src2);

    g_mat_src = h_concat;
/*
    namedWindow( "ring", CV_WINDOW_AUTOSIZE );
    imshow("ring",  resize_mat(h_concat));

    waitKey(0);

    return 0;


    //COMEÇO
    // Load an image
    g_mat_src = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE);

    if(argc > 2)
    {
        g_mat_src2 = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE);

        if(g_mat_src2.rows > g_mat_src2.cols)
        {
            g_mat_src2 = g_mat_src2.t();
        }

        bitwise_not(g_mat_src2, g_mat_src2);
    }

    // This force the image to be in a landscape format
    if(g_mat_src.rows > g_mat_src.cols)
    {
        g_mat_src = g_mat_src.t();
    }    // This force the image to be in a landscape format

    LIMIT_SEL_COL = g_mat_src.cols-1;

    // negative src so the traces are white.
    bitwise_not(g_mat_src, g_mat_src);

    g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_32FC1);

    if( !g_mat_src.data )
        { return -1; }

*/

    g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_32FC1);

    Mat mat_traces = Mat::zeros(g_mat_src.rows, g_mat_src.cols, CV_8UC3);

    waitKey(0);

    vector<Trace> traces = trace_following(g_mat_src, g_mat_src2, mat_traces);

    // for (int i = 0; i < traces.size(); ++i)
    // {
    //     for (int j = 0; j < traces[i].pts_rise_edge.size(); ++j)
    //     {
    //         if(j == 21) //temp
    //         {
    //             cerrv(i)
    //             cerrv(traces[i].pts_fall_edge[j])
    //         }

    //         if(traces[i].pts_rise_edge[j] > 0)
    //         {
    //             mat_traces.col(j).at<Vec3b>((int)traces[i].pts_rise_edge[j])[1] = 255;
    //         }
    //         if(traces[i].pts_fall_edge[j] > 0)
    //         {
    //             mat_traces.col(j).at<Vec3b>((int)traces[i].pts_fall_edge[j])[1] = 255;
    //         }
    //     }
    // }

    // namedWindow( "traces", CV_WINDOW_AUTOSIZE );
    // imshow( "traces", resize_mat(mat_traces) );
    imwrite( "output.bmp", mat_traces);

    waitKey(0);


    return 1;

    // Create window
    namedWindow( "dy derivative", CV_WINDOW_AUTOSIZE );
    namedWindow( "original", CV_WINDOW_AUTOSIZE );

    // Create trackbars
    // createTrackbar( "N Average", "original", &g_prm_avrg, 30, cb_avrg);
    createTrackbar( "kernel alpha", "original", &g_prm_alpha, LIMIT_ALPHA, cb_filter);
    createTrackbar( "Column section", "original", &g_prm_sel_col, LIMIT_SEL_COL, cb_col_profile);

    // open refreshed windows
    cb_filter(0, 0);
    // cb_avrg(0, 0);
    imshow( "original", resize_mat(g_mat_src) );

    waitKey(0);
    // while(cin.get() != 'x')

    return 0;
}