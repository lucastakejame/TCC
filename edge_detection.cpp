#include "edge_detection.h"

int find_min(const Mat& mat, int col, int idx_begin, int length_sweep)
{
    Mat mat_col_header = mat.col(col);

    int min;

    min = 255;

    for (int idx = idx_begin; idx < idx_begin + length_sweep; ++idx)
    {
        int val = mat_col_header.at<unsigned char>(idx);
        if(min > val)
        {
            min = val;
        }
    }

    return min;
}

int find_max(const Mat& mat, int col, int idx_begin, int length_sweep)
{
    Mat mat_col_header = mat.col(col);

    int max;

    max = 0;

    for (int idx = idx_begin; idx < idx_begin + length_sweep; ++idx)
    {
        int val = mat_col_header.at<unsigned char>(idx);
        if(max < val)
        {
            max = val;
        }
    }

    return max;
}

void filter(const Mat& mat_src, Mat& mat_dy, int trace_width, float alpha)
{
    int sz_kernel = alpha*trace_width;

    if(sz_kernel%2 == 0)
        sz_kernel++;


    // set kernel
    Mat mat_kernel = Mat::ones( sz_kernel, 1, CV_32F );
    for (int i = 0; i < sz_kernel/2; ++i)
    {
        mat_kernel.at<float>(i, 0) = -1;
    }

    mat_kernel.at<float>(sz_kernel/2, 0) = 0;

    // normalize values.
    for (int i = 0; i < sz_kernel; ++i)
    {
        mat_kernel.at<float>(i, 0) /= (float)sz_kernel;
    }

    // Apply filter
    Point arg_anchor = Point( -1, -1 );
    double arg_delta = 0;
    int arg_ddepth = CV_32F;
    // int arg_ddepth = -1;
    filter2D(mat_src, mat_dy, arg_ddepth , mat_kernel, arg_anchor, arg_delta, BORDER_DEFAULT );
}

vector<Candidate> get_edge_candidates(const Mat& mat_src, const Mat& mat_dy, int col, int threshold)
{
    Mat mat_src_col = mat_src.col(col);
    Mat mat_dy_col = mat_dy.col(col);

    vector<Candidate> pts_cnddt;

    for (int i = 0; i < mat_dy_col.rows-3; ++i)
    {
        // get if derivative is roughly 0.
        float delta1 = mat_dy_col.at<float>(i+1) - mat_dy_col.at<float>(i);
        float delta2 = mat_dy_col.at<float>(i+3) - mat_dy_col.at<float>(i+2);

        // if its about 0 and its bigger than the threshold, its marked.
        if(delta1 * delta2 <= 0 && fabs(mat_dy_col.at<float>(i+2)) > threshold )
        {
            Candidate temp = {Point(col, i+2), 0, mat_src_col.at<unsigned char>(i+2), 0};
            pts_cnddt.push_back(temp);
            i += 3;
        }
    }

    static int count = 0;
    static float media = 0;

    media = (media*count + pts_cnddt.size()) / (count+1);
    count++;


    if(pts_cnddt.size() < media && threshold > 10)
    {
        return get_edge_candidates(mat_src, mat_dy, col, threshold*0.9);
    }
    else
    {
        return pts_cnddt;
    }
}

void get_A_B(const Mat& mat_src, const Mat& mat_dy, vector<Candidate>& pts_candidate)
{
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

        if(val_col_xtrm > 0) // if its a positive edge (crescent from left to right)
        {
            int min = find_min(mat_src, idx_src_col, idx_last_marked, idx_src_row - idx_last_marked);

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
            int max = find_max(mat_src, idx_src_col, idx_last_marked, idx_src_row - idx_last_marked);

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
}


void edge_detection_fine(const Mat& mat_src, vector<Candidate>& pts_cnddt) // candidates must have left and right values.
{
    // refined edge location
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
                while(mat_src.at<unsigned char>(k, col_edge) > val_edge)
                {
                    k--;
                }

                val_smaller = mat_src.at<unsigned char>(k, col_edge);
                val_bigger = mat_src.at<unsigned char>(k+1, col_edge);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (val_edge-val_smaller)/(val_bigger-val_smaller) + k;

            }
            else
            {
                int k = (int)row_edge;
                while(mat_src.at<unsigned char>(k, col_edge) < val_edge)
                {
                    k++;
                }
                val_smaller = mat_src.at<unsigned char>(k-1, col_edge);
                val_bigger = mat_src.at<unsigned char>(k, col_edge);

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
                while(mat_src.at<unsigned char>(k, col_edge) > val_edge)
                {
                    k++;
                }
                val_smaller = mat_src.at<unsigned char>(k, col_edge);
                val_bigger = mat_src.at<unsigned char>(k-1, col_edge);

                pos_new_edge = (double)k - (val_edge-val_smaller)/(val_bigger-val_smaller);

            }
            else
            {
                int k = (int)row_edge;
                while(mat_src.at<unsigned char>(k, col_edge) < val_edge)
                {
                    k--;
                }
                val_smaller = mat_src.at<unsigned char>(k+1, col_edge);
                val_bigger = mat_src.at<unsigned char>(k, col_edge);

                pos_new_edge = (double)k - (val_edge-val_smaller)/(val_bigger-val_smaller);

            }
        }
    }
}

