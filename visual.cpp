#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "cvplot.h"

#define cerr(x) cerr << x << endl;
#define cerrv(x) cerr <<  #x << " : " << x << endl;


using namespace std;
using namespace cv;

const int LIMIT_ALPHA = 10;
const int LIMIT_SZ_KERNEL = 20;
int LIMIT_SEL_COL;
const int TRACE_WIDTH = 130;

static int g_prm_alpha = 2;
static int g_prm_sz_kernel = 5;
static int g_prm_sel_col = 0;


Mat g_mat_src;
Mat g_mat_dy;

struct Candidate
{
    Point2d position;
    int val_left;
    int val_position;
    int val_right;
};

static void cb_filter(int param, void* user_data);
static void cb_col_profile(int param, void* user_data);
static void col_profile(const Mat& mat_src, int col_src, const char* window_name, unsigned char red, unsigned char green, unsigned char blue);
static void paint_col(Mat src, int src_col);
static Mat mark_max(const Mat& mat_src, int col, int threshold);
Mat edge_detection_thresh(const Mat& mat_src, const Mat& mat_dy, vector<Point> pts_col_xtrm);
Mat edge_detection_thresh2(const Mat& mat_src, const Mat& mat_dy, vector<Candidate>& pts_candidate);


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

    // setting up the differentiator kernel
    Mat mat_kernel = Mat::ones( sz_kernel, 1, CV_32F );
    for (int i = 0; i < sz_kernel/2; ++i)
    {
        mat_kernel.at<float>(i, 0) = -1;
    }

    mat_kernel.at<float>(sz_kernel/2, 0) = 0;

    // normalizing
    for (int i = 0; i < sz_kernel; ++i)
    {
        mat_kernel.at<float>(i, 0) /= (float)sz_kernel;
    }

    // Apply filter
    Point arg_anchor = Point( -1, -1 );
    double arg_delta = 0;
    int arg_ddepth = CV_32F;
    // int arg_ddepth = -1;
    filter2D(g_mat_src, g_mat_dy, arg_ddepth , mat_kernel, arg_anchor, arg_delta, BORDER_DEFAULT );

    cb_col_profile(0,0);
}

static void cb_col_profile(int param, void* user_data)
{
    Mat show_mat_dy = resize_mat(g_mat_dy);

    // normalized sel_col param, bounded in [0,1]
    float norm_sel_col = (float)g_prm_sel_col/LIMIT_SEL_COL;
    int orig_mat_col = g_prm_sel_col;
    // int orig_mat_col = (float)(g_mat_dy.cols-1)*norm_sel_col;


    Mat mat_dst = mark_max(g_mat_dy, orig_mat_col, 35); // mark the extremes from mat_dy

    vector<Candidate> pts_cnddt;

    for (int i = 0; i < mat_dst.rows; ++i)
    {
        if(mat_dst.at<float>(i, orig_mat_col) != 0)
        {
            Candidate temp = {Point(orig_mat_col, i), 0, g_mat_src.at<unsigned char>(i, orig_mat_col), 0};
            pts_cnddt.push_back(temp);
        }
    }

    Mat mat_A_B = edge_detection_thresh2(g_mat_src, g_mat_dy, pts_cnddt);

    for (int i = 0; i < mat_A_B.rows; ++i)
    {
        mat_A_B.at<unsigned char>(i, orig_mat_col) = 0;
    }

    // refined edge location
    int countt = 0;
    for (int i = 0; i < pts_cnddt.size(); ++i)
    {
        double threshold = 0.7; // beta, in the edge model.
        double val_edge;
        int row_edge = pts_cnddt[i].position.y;
        int col_edge = pts_cnddt[i].position.x;

        int val_smaller;
        int val_bigger;

        double pos_new_edge;

        if(pts_cnddt[i].val_left < pts_cnddt[i].val_right)
        {
            // val_right is A, val_left is B
            val_edge = threshold*fabs(pts_cnddt[i].val_right - pts_cnddt[i].val_left) + pts_cnddt[i].val_left;

            if(val_edge < pts_cnddt[i].val_position)
            {
                int k = (int)row_edge;
                while(g_mat_src.at<unsigned char>(k, col_edge) > val_edge)
                {
                    k--;
                }

                val_smaller = g_mat_src.at<unsigned char>(k, col_edge);
                val_bigger = g_mat_src.at<unsigned char>(k+1, col_edge);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (val_edge-val_smaller)/(val_bigger-val_smaller) + k;

            }
            else
            {
                int k = (int)row_edge;
                while(g_mat_src.at<unsigned char>(k, col_edge) < val_edge)
                {
                    k++;
                }
                val_smaller = g_mat_src.at<unsigned char>(k-1, col_edge);
                val_bigger = g_mat_src.at<unsigned char>(k, col_edge);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (val_edge-val_smaller)/(val_bigger-val_smaller) + k-1;

            }
        }
        else
        {
            val_edge = threshold*(pts_cnddt[i].val_left - pts_cnddt[i].val_right) + pts_cnddt[i].val_right;

            if(val_edge < pts_cnddt[i].val_position)
            {
                int k = (int)row_edge;
                while(g_mat_src.at<unsigned char>(k, col_edge) > val_edge)
                {
                    k++;
                }
                val_smaller = g_mat_src.at<unsigned char>(k, col_edge);
                val_bigger = g_mat_src.at<unsigned char>(k-1, col_edge);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (double)k - (val_edge-val_smaller)/(val_bigger-val_smaller);

            }
            else
            {
                int k = (int)row_edge;
                while(g_mat_src.at<unsigned char>(k, col_edge) < val_edge)
                {
                    k--;
                }
                val_smaller = g_mat_src.at<unsigned char>(k+1, col_edge);
                val_bigger = g_mat_src.at<unsigned char>(k, col_edge);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (double)k - (val_edge-val_smaller)/(val_bigger-val_smaller);

            }
        }


        if(pos_new_edge > 0)
        {
            mat_A_B.at<unsigned char>((int)pos_new_edge, pts_cnddt[i].position.x) = val_edge;
            countt++;
        }
    }
    cerrv(countt);
    cerrv(pts_cnddt.size());
    cerrv(pts_cnddt.size() - countt);




    CvPlot::clear("Perfil");
    CvPlot::clear("Perfil Src");

    col_profile(g_mat_dy, orig_mat_col, "Perfil", 255, 0, 0);
    col_profile(mat_dst, orig_mat_col, "Perfil", 0, 0, 255);
    col_profile(g_mat_src, orig_mat_col, "Perfil Src", 0, 255, 0);
    col_profile(mat_A_B, orig_mat_col, "Perfil Src", 125, 0, 125);

    int show_mat_col;
    if(show_mat_col > orig_mat_col)
        show_mat_col = orig_mat_col*(float)(show_mat_dy.cols)/(g_mat_dy.cols);
    else
        show_mat_col = (float)(show_mat_dy.cols-1)*norm_sel_col;

    paint_col(show_mat_dy, show_mat_col);

    imshow( "filter2D Demo", show_mat_dy );

}

static void col_profile(const Mat& mat_src, int col_src, const char* window_name, unsigned char red, unsigned char green, unsigned char blue)
{
    // CvPlot::clear(window_name);


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

static void paint_col(Mat mat_src, int col_src)
{
    Point start = Point(col_src,0);
    Point end = Point(col_src, mat_src.rows-1);
    int thickness = 1;
    int line_type = 8;
    line(mat_src, start, end, Scalar(255), thickness, line_type);
}

static Mat mark_max(const Mat& mat_src, int col, int threshold)
{
    std::vector<Point> pts_max;

    Mat mat_dst;
    mat_dst.create(mat_src.rows, mat_src.cols, mat_src.type());


    for (int i = 0; i < mat_src.rows-3; ++i)
    {
        // get if derivative is roughly 0.
        float delta1 = mat_src.at<float>(i+1, col) - mat_src.at<float>(i, col);
        float delta2 = mat_src.at<float>(i+3, col) - mat_src.at<float>(i+2, col);


        // if its about 0 and its bigger than the threshold, its marked.
        if(delta1 * delta2 <= 0 && fabs(mat_src.at<float>(i+2, col)) > threshold )
        {
            mat_dst.at<float>(i+2, col) = mat_src.at<float>(i+2, col);
            pts_max.push_back(Point(col,i+2));
            i += 3;
        }
        else
        {
            mat_dst.at<float>(i+2, col) = 0;
        }

    }

    // {
        static int count = 0;
        static float media = 0;

        media = (media*count + pts_max.size()) / (count+1);
        count++;

        // fprintf(stderr, "Number of max found: %i\n", pts_max.size());
        // fprintf(stderr, "Média: %i\n", (int)media);
    // }

    // cout << pts_max << endl;
    // fprintf(stderr, "threshold: %i\n", threshold);

    if(pts_max.size() < media && threshold > 10)
    {
        mat_dst.release();
        return mark_max(mat_src, col, threshold*0.9);
    }
    else
    {
        return mat_dst;
    }
}

Mat edge_detection_thresh(const Mat& mat_src, const Mat& mat_dy, vector<Point> pts_col_xtrm)
{
 /*
 pts_col_xtrm saves the extremes from a mat_src's column
 */
    Mat mat_A_B;
    mat_A_B.create(mat_src.rows, mat_src.cols, mat_src.type());

    int last_idx_end = 0;
    for (int i = 0; i < pts_col_xtrm.size(); ++i)
    {
        int idx_range = 50;
        int idx_src_col = pts_col_xtrm[i].x;  // column from mat_src
        int idx_src_row = pts_col_xtrm[i].y;  // idx in a column seen as array in mat_src
        int idx_init = idx_src_row - idx_range;
        int idx_end = idx_src_row + idx_range;
        bool min_to_max = false;

        if(idx_init < 0)
        {
            idx_init = 0;
        }
        else // if it is not the first extremum.
        {
            if(i > 0 && idx_init < last_idx_end)
            {
                idx_init = last_idx_end;
            }
        }
        if(idx_end >= mat_src.rows)
        {
            idx_end = mat_src.rows-1;
        }
        if(i < pts_col_xtrm.size()-1)
        {
            if(idx_end > pts_col_xtrm[i+1].y)
            {
                idx_end = pts_col_xtrm[i+1].y;
            }
        }

        if(mat_dy.at<float>(idx_src_row, idx_src_col) > 0)
        {
            min_to_max = true;
        }

        last_idx_end = idx_end;

        int min = 255;
        int max = 0;
        for (int idx = idx_init; idx < idx_src_row; ++idx)
        {
            int val = mat_src.at<unsigned char>(idx, idx_src_col);
            if(min_to_max)
            {
                if(min > val)
                {
                    min = val;
                }
            }
            else
            {
                if(max < val)
                {
                    max = val;
                }
            }

        }
        for (int idx = idx_src_row; idx < idx_end; ++idx)
        {
            int val = mat_src.at<unsigned char>(idx, idx_src_col);
            if(min_to_max)
            {
                if(max < val)
                {
                    max = val;
                }
            }
            else
            {
                if(min > val)
                {
                    min = val;
                }
            }

        }
        for (int idx = idx_init; idx < idx_end; ++idx)
        {
            if(min_to_max)
            {
                if(idx < idx_src_row)
                {
                    mat_A_B.at<unsigned char>(idx, idx_src_col) = min;
                }
                else
                {
                    mat_A_B.at<unsigned char>(idx, idx_src_col) = max;
                }
            }
            else
            {
                if(idx < idx_src_row)
                {
                    mat_A_B.at<unsigned char>(idx, idx_src_col) = max;
                }
                else
                {
                    mat_A_B.at<unsigned char>(idx, idx_src_col) = min;
                }
            }
        }

    }

    return mat_A_B;

}

Mat edge_detection_thresh2(const Mat& mat_src, const Mat& mat_dy, vector<Candidate>& pts_candidate)
{
    Mat mat_A_B;
    mat_A_B.create(mat_src.rows, mat_src.cols, mat_src.type());

    int idx_last_marked = 0; // used as boundary
    for (int i = 0; i <= pts_candidate.size(); ++i)
    {
        int idx_src_col;
        int idx_src_row;
        float val_col_xtrm;

        if(i < pts_candidate.size())
        {
            idx_src_col = pts_candidate[i].position.x;  // column from mat_src
            idx_src_row = pts_candidate[i].position.y;  // idx in a column seen as array in mat_src

            val_col_xtrm = mat_dy.at<float>(idx_src_row, idx_src_col); // value of extremum in mat_dy
        }
        else // thats after the last point.
        {
            idx_src_col = pts_candidate[i-1].position.x;  // column from mat_src
            idx_src_row = mat_src.rows;  // idx in a column seen as array in mat_src

            val_col_xtrm = -mat_dy.at<float>(pts_candidate[i-1].position.y, pts_candidate[i-1].position.x); // value of extremum in mat_dy
        }

        int min = 255, max = 0;
        if(val_col_xtrm > 0) // if its a positive edge (crescent from left to right)
        {
            for (int idx = idx_last_marked; idx < idx_src_row; ++idx)
            {
                int val = mat_src.at<unsigned char>(idx, idx_src_col);
                if(min > val)
                {
                    min = val;
                }
            }
            for (int idx = idx_last_marked; idx < idx_src_row; ++idx)
            {
                mat_A_B.at<unsigned char>(idx, idx_src_col) = min;
            }

            // set candidate neighbour values (A and B from edge model)
            if(i > 0)
            {
                pts_candidate[i-1].val_right = min;
            }
            if(i < pts_candidate.size())
            {
                pts_candidate[i].val_left = min;
            }
        }
        else // if its a negative edge
        {
            for (int idx = idx_last_marked; idx < idx_src_row; ++idx)
            {
                int val = mat_src.at<unsigned char>(idx, idx_src_col);
                if(max < val)
                {
                    max = val;
                }
            }
            for (int idx = idx_last_marked; idx < idx_src_row; ++idx)
            {
                mat_A_B.at<unsigned char>(idx, idx_src_col) = max;
            }
            // set candidate neighbour values (A and B from edge model)
            if(i > 0)
            {
                pts_candidate[i-1].val_right = max;
            }
            if(i < pts_candidate.size())
            {
                pts_candidate[i].val_left = max;
            }
        }

        idx_last_marked = idx_src_row;

    }

    return mat_A_B;
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

    LIMIT_SEL_COL = g_mat_src.cols-1;

    // negative src so the traces are white.
    bitwise_not(g_mat_src, g_mat_src);

    // g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_32SC1);
    g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_32FC1);
    // g_mat_dy.create(g_mat_src.rows, g_mat_src.cols, CV_8UC1);

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