#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "cvplot.h"

using namespace std;
using namespace cv;

const int limit_scal = 99;
const int limit_sz_kernel = 20;
const int limit_sel_col = 9999;


// void resize_imshow(const char* window_name, const Mat& src)
// {
//     const int max_size = 500;


//     if(src.cols > max_size || src.rows > max_size)
//     {
//         Mat src_resize;

//         float sz_new_width = max_size;
//         float sz_new_height = sz_new_width*src.rows/src.cols;

//         Size size(sz_new_width, sz_new_height);

//         resize(src, src_resize, size);

//         imshow(window_name, src_resize);
//     }
//     else
//     {
//         imshow(window_name, src);
//     }
// }

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

// Filter Parameters
class FParams
{
public:
    static int prm_sel_col; // prm_: parameter used in GUI
    static int prm_sz_kernel;
    static int prm_scal;

    Mat* pt_mat_src;
    Mat* pt_mat_dst;
    Mat  mat_kernel;

    FParams(Mat* src, Mat* dst):pt_mat_src(src), pt_mat_dst(dst)
    {
    }

    static void cb_filter(int param, void* user_data)
    {
        FParams* p_hndl = (FParams *)user_data; // pointer handle

        Mat show_dst = resize_mat(*(p_hndl->pt_mat_dst));

        p_hndl->mat_kernel = Mat::zeros( FParams::prm_sz_kernel, FParams::prm_sz_kernel, CV_32F );
        for (int i = 0; i < FParams::prm_sz_kernel/2; ++i)
        {
            p_hndl->mat_kernel.at<float>(i, FParams::prm_sz_kernel/2) = -((float)FParams::prm_scal/10);
            p_hndl->mat_kernel.at<float>( (FParams::prm_sz_kernel-1) - i, FParams::prm_sz_kernel/2) = ((float)FParams::prm_scal/10);
        }

        // Apply filter

        if(FParams::prm_sz_kernel)
        {
            Point arg_anchor = Point( -1, -1 );
            double arg_delta = 0;
            int arg_ddepth = -1;

            filter2D(*(p_hndl->pt_mat_src), *(p_hndl->pt_mat_dst), arg_ddepth , p_hndl->mat_kernel, arg_anchor, arg_delta, BORDER_DEFAULT );
        }

        cb_col_contour(0,p_hndl);

    }

    static void cb_col_contour(int param, void* user_data)
    {
        FParams* p_hndl = (FParams *)user_data;

        Mat show_mat = resize_mat(*(p_hndl->pt_mat_dst));

        float norm_sel_col = (float)FParams::prm_sel_col/limit_sel_col; // normalized sel_col param. []
        int orig_mat_col = (float)(p_hndl->pt_mat_dst->cols-1)*norm_sel_col;
        int show_mat_col;
        if(show_mat_col > orig_mat_col)
            show_mat_col = orig_mat_col*(float)(show_mat.cols)/(p_hndl->pt_mat_dst->cols);
        else
            show_mat_col = (float)(show_mat.cols-1)*norm_sel_col;

        unsigned char *input = (unsigned char*)(p_hndl->pt_mat_dst->data);
        input = input + orig_mat_col;
        CvPlot::clear("Perfil");
        CvPlot::plot("Perfil", input, p_hndl->pt_mat_dst->rows, p_hndl->pt_mat_dst->cols, 253, 150, 114);
        CvPlot::label("Column Section");

        Point start = Point(show_mat_col,0);
        Point end = Point(show_mat_col, show_mat.rows-1);
        int thickness = 1;
        int line_type = 8;
        line( show_mat, start, end, Scalar(255), thickness, line_type );

        imshow( "filter2D Demo", show_mat );

    }
};


int FParams::prm_scal = 10;
int FParams::prm_sz_kernel = 5;
int FParams::prm_sel_col = 4040;


/** @function main */
int main ( int argc, char** argv )
{
/// Declare variables
    Mat mat_src, mat_dy; // mat_: matrix

/// Load an image
    mat_src = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE);

    if(mat_src.rows > mat_src.cols)
    {
        Mat temp = mat_src.t();
        mat_src = temp;
    }

    mat_dy.create(mat_src.rows, mat_src.cols, CV_8UC1);

    FParams hndl_filter(&mat_src, &mat_dy); // handle for filter parameters

    if( !mat_src.data )
        { return -1; }

/// Create window
    namedWindow( "filter2D Demo", CV_WINDOW_AUTOSIZE );
    namedWindow( "original", CV_WINDOW_AUTOSIZE );

/// Create trackbars
    createTrackbar( "kernel scalar", "original", &FParams::prm_scal, limit_scal, FParams::cb_filter, &hndl_filter);
    createTrackbar( "kernel size", "original", &FParams::prm_sz_kernel, limit_sz_kernel, FParams::cb_filter, &hndl_filter);
    createTrackbar( "Column section", "original", &FParams::prm_sel_col, limit_sel_col, FParams::cb_col_contour, &hndl_filter);


/// open refreshed windows
    FParams::cb_filter(0, &hndl_filter);
    imshow( "original", resize_mat(mat_src) );

    waitKey(0);

    return 0;
}