#include "edge_detection.h"

Mat* g_temp_mat;
int g_idx_col;

int find_min(const Mat& mat_col, int idx_begin, int length_sweep)
{
    int min;

    min = 255;

    for (int idx = idx_begin; idx < idx_begin + length_sweep; ++idx)
    {
        int val = mat_col.at<unsigned char>(idx);
        if(min > val)
        {
            min = val;
        }
    }

    return min;
}

int find_max(const Mat& mat_col, int idx_begin, int length_sweep)
{
    int max;

    max = 0;

    for (int idx = idx_begin; idx < idx_begin + length_sweep; ++idx)
    {
        int val = mat_col.at<unsigned char>(idx);
        if(max < val)
        {
            max = val;
        }
    }

    return max;
}

double get_median(vector<double>& distances)
{
    double median;
    size_t size = distances.size();

    sort(distances.begin(), distances.end());

    if (size % 2 == 0)
    {
        median = (distances[size / 2 - 1] + distances[size / 2]) / 2;
    }
    else
    {
        median = distances[size / 2];
    }

    return median;

}

Mat col_average(const Mat& mat_src, int idx_col_first, int n_cols)
{
    Mat mat_n_average;
    mat_n_average.create(mat_src.rows, 1, mat_src.type());
    for (int i = 0; i < mat_n_average.rows; ++i)
    {
        float average = 0;
        for (int j = idx_col_first; j < idx_col_first+n_cols; ++j)
        {
            average += mat_src.col(j).at<unsigned char>(i);
        }

        mat_n_average.at<unsigned char>(i) = average/n_cols;
    }

    return mat_n_average;

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

    // Apply filter
    Point arg_anchor = Point( -1, -1 );
    double arg_delta = 0;
    int arg_ddepth = CV_32F;
    filter2D(mat_src, mat_dy, arg_ddepth , mat_kernel, arg_anchor, arg_delta, BORDER_DEFAULT );

    for (int i = 0; i < mat_dy.cols; ++i)
    {
        float maxabs = 0;
        for (int j = 0; j < mat_dy.rows; ++j)
        {
            if(maxabs < fabs(mat_dy.col(i).at<float>(j)))
            {
                maxabs = fabs(mat_dy.col(i).at<float>(j));
            }
        }
        mat_dy.col(i) = mat_dy.col(i)/maxabs;
    }
}

// Return a vector with edge candidates located on the mat_dy_col
vector<Candidate> get_edge_candidates(const Mat& mat_src_col, const Mat& mat_dy_col, float threshold)
{
    vector<Candidate> pts_candidate;

    for (int i = 0; i < mat_dy_col.rows-3; ++i)
    {
        // get if derivative is roughly 0.
        float delta1 = mat_dy_col.at<float>(i+1) - mat_dy_col.at<float>(i);
        float delta2 = mat_dy_col.at<float>(i+3) - mat_dy_col.at<float>(i+2);

        // if its about 0 and its bigger than the threshold, its marked.
        if(delta1 * delta2 <= 0 && fabs(mat_dy_col.at<float>(i+2)) > threshold )
        {
            Candidate temp = {
                                (double)i+2,
                                0, // val_left
                                mat_src_col.at<unsigned char>(i+2), //val_position
                                0, // val_right
                                (mat_dy_col.at<float>(i+2) > 0) // 0 if fall edge, 1 if rising edge
                            };
            pts_candidate.push_back(temp);
            i += 3;
        }
    }

    static int count = 0;
    static float media = 0;

    media = (media*count + pts_candidate.size()) / (count+1);
    count++;

    if(pts_candidate.size() < media && threshold > 0.1)
    {
        return get_edge_candidates(mat_src_col, mat_dy_col, threshold*0.9);
    }
    else
    {
        return pts_candidate;
    }
}

void get_A_B(const Mat& mat_src_col, const Mat& mat_dy_col, vector<Candidate>& pts_candidate)
{
    int idx_last_marked = 0; // used as boundary

    trust(pts_candidate.size() > 0);

    for (int i = 0; i <= pts_candidate.size(); ++i)
    {
        int idx_src_row;
        float val_col_xtrm;

        if(i < pts_candidate.size())
        {
            idx_src_row = pts_candidate[i].position;  // idx in a column seen as array in mat_src

            val_col_xtrm = mat_dy_col.at<float>(idx_src_row); // value of extremum in mat_dy
        }
        else // thats after the last point.
        {
            idx_src_row = mat_src_col.rows;  // idx in a column seen as array in mat_src

            val_col_xtrm = -mat_dy_col.at<float>(pts_candidate[i-1].position); // value of extremum in mat_dy
        }

        if(val_col_xtrm > 0) // if its a positive edge (crescent from left to right)
        {
            int min = find_min(mat_src_col, idx_last_marked, idx_src_row - idx_last_marked);

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
            int max = find_max(mat_src_col, idx_last_marked, idx_src_row - idx_last_marked);

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

void edge_detection_coarse(const Mat& mat_src_col, const Mat& mat_dy_col, vector<Candidate>& pts_candidate)
{
    pts_candidate = get_edge_candidates(mat_src_col, mat_dy_col, 0.3);

    get_A_B(mat_src_col, mat_dy_col, pts_candidate);
}

void edge_detection_fine(const Mat& mat_src_col, vector<Candidate>& pts_candidate) // candidates must have left and right values.
{
    // refined edge location
    for (int i = 0; i < pts_candidate.size(); ++i)
    {
        double threshold = 0.7; // beta, in the edge model.
        double val_edge;
        int row_edge = (int) pts_candidate[i].position;

        int val_smaller;
        int val_bigger;

        double pos_new_edge;

        if(pts_candidate[i].val_left < pts_candidate[i].val_right)
        {
            // val_right is A, val_left is B
            val_edge = threshold*fabs(pts_candidate[i].val_right - pts_candidate[i].val_left) + pts_candidate[i].val_left;

            if(val_edge < pts_candidate[i].val_position)
            {
                int k = (int)row_edge;
                while(mat_src_col.at<unsigned char>(k) > val_edge)
                {
                    k--;
                }

                val_smaller = mat_src_col.at<unsigned char>(k);
                val_bigger = mat_src_col.at<unsigned char>(k+1);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (val_edge-val_smaller)/(val_bigger-val_smaller) + k;

            }
            else
            {
                int k = (int)row_edge;
                while(mat_src_col.at<unsigned char>(k) < val_edge)
                {
                    k++;
                }
                val_smaller = mat_src_col.at<unsigned char>(k-1);
                val_bigger = mat_src_col.at<unsigned char>(k);

                // interpolation to find edge position with subpixel accuracy

                pos_new_edge = (val_edge-val_smaller)/(val_bigger-val_smaller) + k-1;

            }
        }
        else
        {
            val_edge = threshold*(pts_candidate[i].val_left - pts_candidate[i].val_right) + pts_candidate[i].val_right;

            if(val_edge < pts_candidate[i].val_position)
            {
                int k = (int)row_edge;
                while(mat_src_col.at<unsigned char>(k) > val_edge)
                {
                    k++;
                }
                val_smaller = mat_src_col.at<unsigned char>(k);
                val_bigger = mat_src_col.at<unsigned char>(k-1);

                pos_new_edge = (double)k - (val_edge-val_smaller)/(val_bigger-val_smaller);

            }
            else
            {
                int k = (int)row_edge;
                while(mat_src_col.at<unsigned char>(k) < val_edge)
                {
                    k--;
                }
                val_smaller = mat_src_col.at<unsigned char>(k+1);
                val_bigger = mat_src_col.at<unsigned char>(k);

                pos_new_edge = (double)k - (val_edge-val_smaller)/(val_bigger-val_smaller);

            }
        }
    }
}

int get_n_traces(vector<Candidate> pts_candidate)
{
    vector<double> distances;

    int groove_bottom_count = 0;

    for (int i = 0; i < pts_candidate.size() - 1; ++i)
    {
        double distance = pts_candidate[i+1].position - pts_candidate[i].position;

        distances.push_back(distance);

        if(distance < 30 && i > 1 && i < pts_candidate.size() - 2)
        {
            groove_bottom_count++;
        }
    }

    return 2*groove_bottom_count;
}

Mat n_first_last_col_average(const Mat& mat_ring_first, const Mat& mat_ring_last, int range_average)
{
    return col_average(mat_ring_first, 0, range_average)/2
        + col_average(mat_ring_last, mat_ring_last.cols - range_average - 1, range_average)/2;
}

vector<Trace> initialize_traces(const Mat& mat_ring_first, const Mat& mat_ring_last, int range_average)
{
    vector<Trace> traces;

    // calculates average of n first (first col_average ) and n last (second) columns from the ring
    Mat mat_avrg = n_first_last_col_average(mat_ring_first, mat_ring_last, range_average);

    // get edges candidates for initialization
    Mat mat_dy;
    vector<Candidate> pts_candidate;
    filter(mat_avrg, mat_dy, 130, 0.2);
    edge_detection_coarse(mat_avrg.col(0), mat_dy.col(0), pts_candidate);
    edge_detection_fine(mat_avrg.col(0), pts_candidate);

    // cerrv(pts_candidate.size());


    // count number of grooves and traces
    // create traces.
    vector<double> profile_sections;
    for (int i = 0; i < pts_candidate.size() - 1; ++i)
    {
        double distance = pts_candidate[i+1].position - pts_candidate[i].position; // distance between edges
        profile_sections.push_back(distance);
        if(
            distance < 30 // distance between edges of about 20 groove bottom
            && i > 1 // i > 1 means that, supposedly, the 1st section is a trace
            && i < pts_candidate.size() - 2 // means that its not the last section
            )
        {
            vector<double> pts_rise1;
            vector<double> pts_fall1;

            Trace temp1;
            temp1.pts_rise_edge = pts_rise1;
            temp1.pts_fall_edge = pts_fall1;
            temp1.pos_estimated_rise = (double) pts_candidate[i-1].position;
            temp1.pos_estimated_fall = (double) pts_candidate[i].position;
            temp1.tolerance = 0;
            temp1.trace_width = 0;

            traces.push_back(temp1);

            vector<double> pts_rise2;
            vector<double> pts_fall2;

            Trace temp2;
            temp2.pts_rise_edge = pts_rise2;
            temp2.pts_fall_edge = pts_fall2;
            temp2.pos_estimated_rise = (double) pts_candidate[i+1].position;
            temp2.pos_estimated_fall = (double) pts_candidate[i+2].position;
            temp2.tolerance = 0;
            temp2.trace_width = 0;

            traces.push_back(temp2);
        }
    }


    cerrln(pts_candidate.size());

    double median_width = get_median(profile_sections);

    for (int i = 0; i < traces.size(); ++i)
    {
        traces[i].trace_width = median_width;
    }

    return traces;
}

double closest_to_estimation(vector<Candidate> pts_candidate, double estimated_edge, double tolerance_min, double tolerance_max)
{
    double result;
    vector<double> possible_candidates;
    for (int i = 0; i < pts_candidate.size(); ++i)
    {
        double candidate = pts_candidate[i].position;
        g_temp_mat->col(g_idx_col).at<Vec3b>((int)candidate)[0] = 255;
        if(tolerance_min <= candidate && candidate <= tolerance_max)
        {
            possible_candidates.push_back(candidate);
        }
        else if(candidate > tolerance_max)
        {
            break;
        }
    }

    if(possible_candidates.size() == 0)
    {
        result = -1; //undefined
    }
    else if(possible_candidates.size() == 1)
    {
        result = possible_candidates[0];
    }
    else
    {
        double min = 0x7FFFFFFF;
        int idx_choosen = 0;
        for (int k = 0; k < possible_candidates.size(); ++k)
        {
            g_temp_mat->col(g_idx_col).at<Vec3b>((int)possible_candidates[k])[2] = 255;

            if(min > fabs(possible_candidates[k] - estimated_edge))
            {
                min = fabs(possible_candidates[k] - estimated_edge);
                idx_choosen = k;
            }
        }

        result = possible_candidates[idx_choosen];
    }
    return result;
}


void define_traces(const Mat& mat_src, vector<Candidate> pts_candidate, vector<Trace>& traces, int analyzed_col)
{
    const int tolerance = 15;

    int idx_candidate = 0;

    for(int i = 0; i < traces.size(); ++i)
    {
        double tolerance_min = traces[i].pos_estimated_rise - tolerance;
        double tolerance_max = traces[i].pos_estimated_rise + tolerance;

        if(tolerance_min < 0)
        {
            tolerance_min = 0;
        }
        if(i > 0 && tolerance_min < traces[i-1].pos_estimated_fall)
        {
            tolerance_min = traces[i-1].pos_estimated_fall;
        }

        if(tolerance_max > mat_src.rows - 1)
        {
            tolerance_max = mat_src.rows - 1;
        }
        if(i < traces.size()-1 && tolerance_max > traces[i+1].pos_estimated_fall)
        {
            tolerance_max = traces[i+1].pos_estimated_fall;
        }

        double candidate_k = closest_to_estimation(pts_candidate, traces[i].pos_estimated_rise, tolerance_min, tolerance_max);

        traces[i].pts_rise_edge.push_back(candidate_k);

        if(candidate_k > 0)
        {
            traces[i].pos_estimated_rise = candidate_k;
        }

        tolerance_min = traces[i].pos_estimated_fall - tolerance;
        tolerance_max = traces[i].pos_estimated_fall + tolerance;

        if(tolerance_min < 0)
        {
            tolerance_min = 0;
        }
        if(i > 0 && tolerance_min < traces[i-1].pos_estimated_rise)
        {
            tolerance_min = traces[i-1].pos_estimated_rise;
        }

        if(tolerance_max > mat_src.rows - 1)
        {
            tolerance_max = mat_src.rows - 1;
        }
        if(i < traces.size()-1 && tolerance_max > traces[i+1].pos_estimated_rise)
        {
            tolerance_max = traces[i+1].pos_estimated_rise;
        }

        candidate_k = closest_to_estimation(pts_candidate, traces[i].pos_estimated_fall, tolerance_min, tolerance_max);

        traces[i].pts_fall_edge.push_back(candidate_k);

        if(candidate_k > 0)
        {
            traces[i].pos_estimated_fall = candidate_k;
        }
    }
}


void print_percent_undef(vector<Trace> traces)
{
    int count = 0;
    for (int i = 0; i < traces.size(); ++i)
    {
        for (int j = 0; j < traces[i].pts_rise_edge.size(); ++j)
        {
            if(traces[i].pts_rise_edge[j] < 0)
            {
                count++;
            }
        }
        cout << "trace rise [" << i << "] has " <<  100*(float)count/traces[i].pts_rise_edge.size() << "% of undefined points" <<  endl;
        count = 0;
        for (int j = 0; j < traces[i].pts_fall_edge.size(); ++j)
        {
            if(traces[i].pts_fall_edge[j] < 0)
            {
                count++;
            }
        }
        cout << "trace fall [" << i << "] has " <<  100*(float)count/traces[i].pts_rise_edge.size() << "% of undefined points" <<  endl;
        count = 0;
    }

}


vector<Trace> trace_following(const Mat& mat_src, const Mat& mat_src2, Mat& mat_visual)
{

    g_temp_mat = &mat_visual;

    vector<Trace> traces = initialize_traces(mat_src, mat_src2, 30);

    Mat mat_dy;
    filter(mat_src, mat_dy, traces[0].trace_width, 0.2);

    // this for processes the image columnwise.
    for (int i = 0; i < mat_src.cols; ++i)
    {
        g_idx_col = i;
        vector<Candidate> pts_candidate;
        edge_detection_coarse(mat_src.col(i), mat_dy.col(i), pts_candidate);
        edge_detection_fine(mat_src.col(i), pts_candidate);

        define_traces(mat_src, pts_candidate, traces, i);
    }

    print_percent_undef(traces);

    return traces;
}