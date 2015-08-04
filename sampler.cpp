#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "cvplot.h"

using namespace std;
using namespace cv;

const int LIMIT_ALPHA = 9;
const int LIMIT_SZ_KERNEL = 20;
const int LIMIT_SEL_COL = 9999;
const int TRACE_WIDTH = 12;

static int g_prm_alpha = 2;
static int g_prm_sz_kernel = 5;
static int g_prm_sel_col = 4040;


Mat g_mat_src;
Mat g_mat_dy;

static void cb_filter(int param, void* user_data);
static void cb_col_profile(int param, void* user_data);
static void col_profile(const Mat& src, int src_col, const char* window_name);
static void paint_col(Mat src, int src_col);


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

// This callback purpuse is to get the d/dy derivative matrix from source matrix
static void cb_filter(int param, void* user_data)
{
    float alpha;
    if(g_prm_alpha < 1)
        alpha = 0.1;
    else
        alpha = (float)g_prm_alpha/10;

    int sz_kernel = alpha*TRACE_WIDTH;

    if(sz_kernel%2 == 0)
        sz_kernel++;

    Mat mat_kernel = Mat::ones( sz_kernel, 1, CV_32F );
    for (int i = 0; i < sz_kernel/2; ++i)
    {
        mat_kernel.at<float>(i, 0) = -1;
    }
    mat_kernel.at<float>(sz_kernel/2, 0) = 0;

    // Apply filter
    Point arg_anchor = Point( -1, -1 );
    double arg_delta = 0;
    int arg_ddepth = -1;
    filter2D(g_mat_src, g_mat_dy, arg_ddepth , mat_kernel, arg_anchor, arg_delta, BORDER_DEFAULT );

    cb_col_profile(0,0);
}

static void cb_col_profile(int param, void* user_data)
{
    Mat show_mat_dy = resize_mat(g_mat_dy);

    // normalized sel_col param, bounded in [0,1]
    float norm_sel_col = (float)g_prm_sel_col/LIMIT_SEL_COL;

    int orig_mat_col = (float)(g_mat_dy.cols-1)*norm_sel_col;

    col_profile(g_mat_dy, orig_mat_col, "Perfil");
    col_profile(g_mat_src, orig_mat_col, "Perfil src");

    int show_mat_col;
    if(show_mat_col > orig_mat_col)
        show_mat_col = orig_mat_col*(float)(show_mat_dy.cols)/(g_mat_dy.cols);
    else
        show_mat_col = (float)(show_mat_dy.cols-1)*norm_sel_col;

    paint_col(show_mat_dy, show_mat_col);

    imshow( "filter2D Demo", show_mat_dy );

}

static void col_profile(const Mat& mat_src, int col_src, const char* window_name)
{
    unsigned char *input = (unsigned char*)(mat_src.data);
    input = input + col_src;
    CvPlot::clear(window_name);
    CvPlot::plot(window_name, input, mat_src.rows, mat_src.cols, 253, 150, 114);
    CvPlot::label("Column Profile");
}

static void paint_col(Mat mat_src, int col_src)
{
    Point start = Point(col_src,0);
    Point end = Point(col_src, mat_src.rows-1);
    int thickness = 1;
    int line_type = 8;
    line(mat_src, start, end, Scalar(255), thickness, line_type);
}

/** @function main */
int main ( int argc, char** argv )
{
    // Load an image
    g_mat_src = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE);

    // This force the image to be in a landscape format
    if(g_mat_src.rows > g_mat_src.cols)
    {
        g_mat_src = g_mat_src.t();
    }

    // negative src
    bitwise_not(g_mat_src, g_mat_src);

    // g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_32FC1);
    g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_8UC1);

    if( !g_mat_src.data )
        { return -1; }

    // Create window
    namedWindow( "filter2D Demo", CV_WINDOW_AUTOSIZE );
    namedWindow( "original", CV_WINDOW_AUTOSIZE );

    // Create trackbars
    createTrackbar( "kernel alpha", "original", &g_prm_alpha, LIMIT_ALPHA, cb_filter);
    createTrackbar( "Column section", "original", &g_prm_sel_col, LIMIT_SEL_COL, cb_col_profile);

    // open refreshed windows
    cb_filter(0, 0);
    imshow( "original", resize_mat(g_mat_src) );

    waitKey(0);

    return 0;
}