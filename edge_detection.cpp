#include "edge_detection.h"
#include "make_wav.h"
#include <sys/time.h>

// variaveis teste
// const int gg_tolerance = 5;
// const int gg_tolerance = 8;
const int gg_tolerance = 14;



Mat* g_temp_mat;
int g_idx_col;
bool debug_paint = false;

void draw_pixel(int x, int y, unsigned char red, unsigned char green, unsigned char blue)
{
    if(debug_paint)
    {
        g_temp_mat->col(x).at<Vec3b>(y)[0] = blue;
        g_temp_mat->col(x).at<Vec3b>(y)[1] = green;
        g_temp_mat->col(x).at<Vec3b>(y)[2] = red;
    }
}

void draw_red(int x, int y)
{
    if(debug_paint)
    {
        if( x < g_temp_mat->cols && y < g_temp_mat->rows)
        {
                g_temp_mat->col(x).at<Vec3b>(y)[2] = 255;
        }
    }
}
void draw_green(int x, int y)
{
    if(debug_paint)
    {
        if( x < g_temp_mat->cols && y < g_temp_mat->rows)
        {
                g_temp_mat->col(x).at<Vec3b>(y)[1] = 255;
        }
    }
}
void draw_blue(int x, int y)
{
    if(debug_paint)
    {
        if( x < g_temp_mat->cols && y < g_temp_mat->rows)
        {
                g_temp_mat->col(x).at<Vec3b>(y)[0] = 255;
        }
    }
}

/*
Encontra valor minimo no array entrege partind de idx_begin por lenght_sweep casas
*/
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

/*
Encontra valor máximo no array entrege partind de idx_begin por lenght_sweep casas
*/
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


/*
Retorna o valor da mediana do vetor entregue
*/
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


/*
Retorna um array em que cada elemento é a média dos elementos
correspondentes em n_cols partindo da coluna idx_col_first
*/
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


/*
Associa ao mat_dy o filtro derivativo na direção perpendicular às trilhas
*/
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

/*
Retorna um vetor com todos possíveis candidatos a borda localizados na mat_dy_col
*/
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
            Candidate temp;

            temp.position = (double)i+2;
            temp.val_left = 0;
            temp.val_position = mat_src_col.at<unsigned char>(i+2);
            temp.val_right = 0;
            temp.fall_rise = (mat_dy_col.at<float>(i+2) > 0);

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


/*
Essa função é relacionada ao modelo de trilha apresentado na tese
A e B são, respectivamente, o topo do perfil da trilha numa imagem negativa ( o que chamamos de traço )
e o valor da base da colina no perfil da trilha.
*/
void get_A_B(const Mat& mat_src_col, const Mat& mat_dy_col, vector<Candidate>& pts_candidate)
{
    int idx_last_marked = 0; // usado como fronteira para marcar até onde foi varrido no array

    trust(pts_candidate.size() > 0);

    for (int i = 0; i <= pts_candidate.size(); ++i)
    {
        int idx_src_row;
        float val_col_xtrm;

        if(i < pts_candidate.size()) // se ainda for um ponto válido.
        {
            idx_src_row = pts_candidate[i].position;  // idx in a column seen as array in mat_src

            val_col_xtrm = mat_dy_col.at<float>(idx_src_row); // value of extremum (value in the candidate position) in mat_dy
        }
        else // depois do último ponto do vetor, serve apenas pra completar o trecho do último candidato até o final da matriz.
        {
            idx_src_row = mat_src_col.rows;  // idx in a column seen as array in mat_src

            val_col_xtrm = -mat_dy_col.at<float>(pts_candidate[i-1].position); // value of extremum in mat_dy
        }

        if(val_col_xtrm > 0) // Borda de subida
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
        else // Borda de descida
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
        int pos_old_edge = (int) pts_candidate[i].position;

        int val_smaller;
        int val_bigger;

        double pos_new_edge;


        if(pts_candidate[i].val_left < pts_candidate[i].val_right) // if its a rise edge
        {


            // val_right is A, val_left is B
            // hence v = t*(A-B) + B
            val_edge = threshold*fabs(pts_candidate[i].val_right - pts_candidate[i].val_left) + pts_candidate[i].val_left;

            if(val_edge < pts_candidate[i].val_position) // edge canditate is at val_position's left
            {

                int k = (int)pos_old_edge;
                while(mat_src_col.at<unsigned char>(k) > val_edge)
                {
                    k--;
                }

                val_smaller = mat_src_col.at<unsigned char>(k);
                val_bigger = mat_src_col.at<unsigned char>(k+1);

                // interpolation to find edge position with subpixel accuracy

                int inclination = val_bigger-val_smaller;
                if(inclination != 0)
                {
                    pos_new_edge = (val_edge-val_smaller)/(inclination) + k;
                }

            }
            else
            {

                int k = (int)pos_old_edge;
                while(mat_src_col.at<unsigned char>(k) < val_edge)
                {
                    k++;
                }
                val_smaller = mat_src_col.at<unsigned char>(k-1);
                val_bigger = mat_src_col.at<unsigned char>(k);

                // interpolation to find edge position with subpixel accuracy

                int inclination = val_bigger-val_smaller;
                if(inclination != 0)
                {
                    pos_new_edge = (val_edge-val_smaller)/(inclination) + k-1;
                }

            }
        }
        else //if its a fall edge
        {

            val_edge = threshold*(pts_candidate[i].val_left - pts_candidate[i].val_right) + pts_candidate[i].val_right;

            if(val_edge < pts_candidate[i].val_position)
            {
                int k = (int)pos_old_edge;
                while(mat_src_col.at<unsigned char>(k) > val_edge)
                {
                    k++;
                }
                val_smaller = mat_src_col.at<unsigned char>(k);
                val_bigger = mat_src_col.at<unsigned char>(k-1);

                int inclination = val_bigger-val_smaller;
                if(inclination != 0)
                {
                    pos_new_edge = (double)k - (val_edge-val_smaller)/(inclination);
                }

            }
            else
            {
                int k = (int)pos_old_edge;
                while(mat_src_col.at<unsigned char>(k) < val_edge)
                {
                    k--;
                }
                val_smaller = mat_src_col.at<unsigned char>(k+1);
                val_bigger = mat_src_col.at<unsigned char>(k);

                int inclination = val_bigger-val_smaller;
                if(inclination != 0)
                {
                    pos_new_edge = (double)k - (val_edge-val_smaller)/(inclination);
                }

            }
        }

        pts_candidate[i].position = pos_new_edge;////// Quando descomento isso o perfil da trilha e a borda encontrada ficam estranhos


        draw_blue(g_idx_col ,pos_new_edge);
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
            distance < 50 // threshold distance between edges of groove bottom
            && i > 1 // i > 1 means that, supposedly, the 1st section is a trace
            && i < pts_candidate.size() - 2 // means that its not the last section
            )
        {
            Trace temp1;
            temp1.pos_estimated_rise = (double) pts_candidate[i-1].position;
            temp1.pos_estimated_fall = (double) pts_candidate[i].position;

            traces.push_back(temp1);

            Trace temp2;
            temp2.pos_estimated_rise = (double) pts_candidate[i+1].position;
            temp2.pos_estimated_fall = (double) pts_candidate[i+2].position;

            traces.push_back(temp2);

        }
    }

    printf("Número de bordas detectadas : %i\n",pts_candidate.size());
    printf("Número de traços considerados : %i\n\n",traces.size());

    double median_width = get_median(profile_sections);

    for (int i = 0; i < traces.size(); ++i)
    {
        traces[i].mean_trace_width.push_back(median_width);
    }

    return traces;
}

vector<double> in_range_candidates(vector<Candidate> pts_candidate, double estimated_edge, double tolerance_min, double tolerance_max)
{
    vector<double> possible_candidates;
    for (int i = 0; i < pts_candidate.size(); ++i)
    {
        double candidate = pts_candidate[i].position;
        if(tolerance_min <= candidate && candidate <= tolerance_max)
        {
            possible_candidates.push_back(candidate);
        }
        else if(candidate > tolerance_max)
        {
            break;
        }
    }

    return possible_candidates;
}

double closest_to_estimation(vector<double> possible_candidates, double estimated_edge)
{
    double result;

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
            // draw_pixel(g_idx_col, possible_candidates[k], 255, 0, 0);

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

double estimated_displacement(vector<Trace> traces, int idx_trace, int col, bool fall_rise)
{
    double displacement;
    if(col > 0)
    {
        if(fall_rise = false && traces[idx_trace].pts_rise_edge[col] > 0 && traces[idx_trace].pts_rise_edge[col-1] > 0) // check displacement on same trace, other edge
        {
            displacement = traces[idx_trace].pts_rise_edge[col] - traces[idx_trace].pts_rise_edge[col-1];
        }
        else if(fall_rise = true && traces[idx_trace].pts_fall_edge[col] > 0 && traces[idx_trace].pts_fall_edge[col-1] > 0) // check displacement on same trace, other edge
        {
            displacement = traces[idx_trace].pts_fall_edge[col] - traces[idx_trace].pts_fall_edge[col-1];
        }
          // same groove, other trace. If idx_trace is even, other trace in the groove is idx_trace+1, otherwise it is idx_trace-1.
        else if( idx_trace % 2 == 0 && traces[idx_trace+1].pts_rise_edge[col] > 0 && traces[idx_trace+1].pts_rise_edge[col-1] > 0) // other rise edge
        {
            displacement = traces[idx_trace+1].pts_rise_edge[col] - traces[idx_trace+1].pts_rise_edge[col-1];
        }
        else if( idx_trace % 2 == 0 && traces[idx_trace+1].pts_fall_edge[col] > 0 && traces[idx_trace+1].pts_fall_edge[col-1] > 0)
        {
            displacement = traces[idx_trace+1].pts_fall_edge[col] - traces[idx_trace+1].pts_fall_edge[col-1];
        }
        else if( idx_trace % 2 != 0 && traces[idx_trace-1].pts_rise_edge[col] > 0 && traces[idx_trace-1].pts_rise_edge[col-1] > 0)
        {
            displacement = traces[idx_trace-1].pts_rise_edge[col] - traces[idx_trace-1].pts_rise_edge[col-1];
        }
        else if( idx_trace % 2 != 0 && traces[idx_trace-1].pts_fall_edge[col] > 0 && traces[idx_trace-1].pts_fall_edge[col-1] > 0) // check displacement on same trace, other edge
        {
            displacement = traces[idx_trace-1].pts_fall_edge[col] - traces[idx_trace-1].pts_fall_edge[col-1];
        }
        else
        {
            double mean_displacement = 0;
            int mean_counter = 0;
            for (int l = 0; l < traces.size(); ++l)
            {
                if(traces[l].pts_rise_edge[col] > 0 // if this edge point is defined
                    && traces[l].pts_rise_edge[col-1] > 0) // if last one is defined
                {
                    mean_displacement += traces[l].pts_rise_edge[col] - traces[l].pts_rise_edge[col-1];
                    mean_counter++;
                }
                if(traces[l].pts_fall_edge[col] > 0
                    && traces[l].pts_fall_edge[col-1] > 0)
                {
                    mean_displacement += traces[l].pts_fall_edge[col] - traces[l].pts_fall_edge[col-1];
                    mean_counter++;
                }
            }


            // cerrv(mean_displacement)
            // cerrv(mean_counter)

            displacement = mean_displacement/mean_counter;
        }

    }
    else
    {
        displacement = 0;
    }

    if(col == 69 && (int)traces[idx_trace].pos_estimated_fall == 2121) //temp
    {
        cerrv(displacement);
    }

    return displacement;
}

void backtracking(Trace trace, /*Trace*/
                bool rise_or_fall, /*rise - 0, fall - 1*/
                int starting_col /*first column in backtracking process*/)
{
    const float zeta = 0.05; /* % of mean width*/
    int j = starting_col;
    bool backtracking = true;

        while(backtracking && j >= 0)
        {
            if(!rise_or_fall) /*rise*/
            {
                trace.pts_rise_edge[j] = -1;
            }
            else /*fall*/
            {
                trace.pts_fall_edge[j] = -1;
            }

            j = j - 1;
            double trace_width = fabs(trace.pts_rise_edge[j] - trace.pts_fall_edge[j]);

            if(fabs(trace.mean_trace_width[j] - trace_width) < zeta*trace.mean_trace_width[j])
            {
                backtracking = false;
            }
        }

}

void print_progress(int analyzed_col, int total_cols)
{
    static int last_progress = -1;
    static int prog_counter = 0;
    double progress;
    prog_counter++;
    if(analyzed_col == total_cols)
    {
        progress = 100;
    }
    else
    {
        progress = 100*analyzed_col/total_cols;
    }
    if(last_progress != (int)progress)
    {
        cerr << "\r";
        cerr << "Fatias de tempo analisadas : " << (int)progress << "%";
        // printf("\r");
        // printf("%6i/%6i progress: %i", analyzed_col, total_cols, (int)progress);
    }

    last_progress = progress;
}


void define_traces(const Mat& mat_src, vector<Candidate> pts_candidate, vector<Trace>& traces, int analyzed_col)
{
    // clock_t t0, t1, t2, t3, t4, t5, t6;
    // double delta0 = 0, delta1 = 0, delta2 = 0, delta3 = 0;

    int tolerance = gg_tolerance;/// Com 10 é sussa, ja nesse nivel ja faz o choosen edge ser desviado por riscos laterais, precisa implementar o backtracking process;


    for(int i = 0; i < traces.size(); ++i)
    {
        // calculation of mean width

        if(analyzed_col > 0)
        {
            const double sigma = 1/1000;

            traces[i].mean_trace_width.push_back(
                sigma*fabs(traces[i].pos_estimated_fall - traces[i].pos_estimated_rise)
                + (1.0f-sigma)*(traces[i].mean_trace_width[analyzed_col-1])
                );

        }

        // tolerance range defined

        double tolerance_min = traces[i].pos_estimated_rise - tolerance;
        double tolerance_max = traces[i].pos_estimated_rise + tolerance;

        /// tunning the tolerance boundary.
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
        // if(i < traces.size()-1 && tolerance_max > traces[i+1].pos_estimated_fall)
        // {
        //     tolerance_max = traces[i+1].pos_estimated_fall;
        // }

        // gets candidate closest to estimation point
        // double candidate_rise = closest_to_estimation(pts_candidate, traces[i].pos_estimated_rise, tolerance_min, tolerance_max);
        // vector<double> candidates_rise = in_range_candidates(pts_candidate, traces[i].pos_estimated_rise, tolerance_min, tolerance_max);

        draw_red(analyzed_col, traces[i].pos_estimated_rise);
        draw_pixel(analyzed_col, tolerance_min, 60, 60, 60);
        draw_pixel(analyzed_col, tolerance_max, 60, 60, 60);

        traces[i].candidates_r_k = in_range_candidates(pts_candidate, traces[i].pos_estimated_rise, tolerance_min, tolerance_max);

        double choosen_rise = closest_to_estimation(traces[i].candidates_r_k, traces[i].pos_estimated_rise);


        traces[i].pts_rise_edge.push_back(choosen_rise);

        // if(choosen_rise > 0)
        // {
        //     traces[i].pos_estimated_rise = choosen_rise;
        // }

        /////////////////////


        tolerance_min = traces[i].pos_estimated_fall - tolerance;
        tolerance_max = traces[i].pos_estimated_fall + tolerance;

        if(tolerance_min < 0)
        {
            tolerance_min = 0;
        }
        // if(i > 0 && tolerance_min < traces[i-1].pos_estimated_rise)
        // {
        //     tolerance_min = traces[i-1].pos_estimated_rise;
        // }

        if(tolerance_max > mat_src.rows - 1)
        {
            tolerance_max = mat_src.rows - 1;
        }
        if(i < traces.size()-1 && tolerance_max > traces[i+1].pos_estimated_rise)
        {
            tolerance_max = traces[i+1].pos_estimated_rise;
        }

        // double candidate_fall = closest_to_estimation(pts_candidate, traces[i].pos_estimated_fall, tolerance_min, tolerance_max);
        // vector<double> candidates_fall = in_range_candidates(pts_candidate, traces[i].pos_estimated_fall, tolerance_min, tolerance_max);

        draw_red(analyzed_col, traces[i].pos_estimated_fall);
        draw_pixel(analyzed_col, tolerance_min, 30, 30, 30);
        draw_pixel(analyzed_col, tolerance_max, 30, 30, 30);


        traces[i].candidates_f_k = in_range_candidates(pts_candidate, traces[i].pos_estimated_fall, tolerance_min, tolerance_max);

        double choosen_fall = closest_to_estimation(traces[i].candidates_f_k, traces[i].pos_estimated_fall);

        traces[i].pts_fall_edge.push_back(choosen_fall);

        // if(choosen_fall > 0)
        // {
        //     traces[i].pos_estimated_fall = choosen_fall;
        // }

        if(0) //temp
        // if(analyzed_col == 21) //temp
        {
            cerrv(i)
            cerrv(traces[i].candidates_r_k.size());
            cerrv(traces[i].pos_estimated_rise);
            cerrv(choosen_rise);
            cerr("");
            cerrv(traces[i].candidates_f_k.size());
            cerrv(traces[i].pos_estimated_fall);
            cerrv(choosen_fall);
            cerr("");
        }
    }

    // for cases where trace has more than 1 candidate in its acceptable range
    // t0 = clock();

    for (int i = 0; i < traces.size(); ++i)
    {
        bool condition_rise = false;
        bool condition_fall = false;

        // t1 = clock();

        if(traces[i].candidates_r_k.size() > 1) /*test rise condition*/
        {
            double width_dif_ref = fabs(fabs(traces[i].pos_estimated_fall - traces[i].pts_rise_edge[analyzed_col]) - traces[i].mean_trace_width[analyzed_col]);

            for (int k = 0; k < traces[i].candidates_r_k.size(); ++k)
            {
                double width_dif_k = fabs(fabs(traces[i].pos_estimated_fall - traces[i].candidates_r_k[k]) - traces[i].mean_trace_width[analyzed_col]);
                if(width_dif_k < width_dif_ref)
                {
                    condition_rise = true;
                    // backtracking(traces[i], 0 /*rise*/, analyzed_col);
                    // draw_pixel(analyzed_col, traces[i].candidates_r_k[k], 120, 120, 120);
                    // traces[i].pts_rise_edge[analyzed_col] = traces[i].candidates_r_k[k];
                    // width_dif_k = fabs(fabs(traces[i].pos_estimated_fall - traces[i].candidates_r_k[k]) - traces[i].mean_trace_width[analyzed_col]);
                }
            }

        }

        // t2 = clock();
        // delta0 += t2-t1;

        if(traces[i].candidates_f_k.size() > 1) /*test fall condition*/
        {
            double width_dif_ref = fabs(fabs(traces[i].pts_fall_edge[analyzed_col] - traces[i].pos_estimated_rise) - traces[i].mean_trace_width[analyzed_col]);

            for (int k = 0; k < traces[i].candidates_f_k.size(); ++k)
            {
                double width_dif_k = fabs(fabs(traces[i].candidates_f_k[k] - traces[i].pos_estimated_rise) - traces[i].mean_trace_width[analyzed_col]);
                if(width_dif_k < width_dif_ref)
                {
                    condition_fall = true;
                    // draw_pixel(analyzed_col, traces[i].candidates_f_k[k], 90, 90, 90);
                    // traces[i].pts_fall_edge[analyzed_col] = traces[i].candidates_r_k[k];
                }
            }
        }

        if(condition_rise && !condition_fall)
        {
            backtracking(traces[i], 0 /*rise*/, analyzed_col);
        }
        else if(!condition_rise && condition_fall)
        {
            backtracking(traces[i], 1 /*rise*/, analyzed_col);
        }

        // t3 = clock();
        // delta1 += t3-t2;

        // ESSE BLOCO DE CODIGO GASTA MTO TEMPO NO MEIO DA ANALISE

        if(traces[i].pts_rise_edge[analyzed_col] > 0) // if its defined
        {
            traces[i].pos_estimated_rise = traces[i].pts_rise_edge[analyzed_col];
        }
        // else
        // {
        //     double displacement = estimated_displacement(traces, i, analyzed_col, true /*rise*/);
        //     traces[i].pos_estimated_rise += displacement;
        //     if(traces[i].pos_estimated_rise > 4096)
        //     {
        //         cerr("ESTORO")
        //         cerrv(i)
        //         cerrv(analyzed_col)
        //         cerrv(traces[i].pos_estimated_rise)
        //         cerrv(displacement)
        //     }
        // }

        // t4 = clock();
        // delta2 += t4-t3;

        // ESSE BLOCO DE CODIGO GASTA MTO TEMPO NO MEIO DA ANALISE
        if(traces[i].pts_fall_edge[analyzed_col] > 0)
        {
            traces[i].pos_estimated_fall = traces[i].pts_fall_edge[analyzed_col];
        }
        // else
        // {
        //     double displacement = estimated_displacement(traces, i, analyzed_col, false /*fall*/);
        //     traces[i].pos_estimated_fall += displacement;
        //     if(traces[i].pos_estimated_fall > 4096)
        //     {
        //         cerr("ESTORO")
        //         cerrv(i)
        //         cerrv(analyzed_col)
        //         cerrv(traces[i].pos_estimated_fall)
        //         cerrv(displacement)
        //     }
        // }

        // t5 = clock();
        // delta3 += t5-t4;
    }
    // t6 = clock();
/*
    double deltaFinal = t6-t0;

    printf("%i/%i\n", analyzed_col, mat_src.cols);
    printf("1o bloco em %%: %f\n", 100.0*delta0/deltaFinal);
    printf("2o bloco em %%: %f\n", 100.0*delta1/deltaFinal);
    printf("3o bloco em %%: %f\n", 100.0*delta2/deltaFinal);
    printf("4o bloco em %%: %f\n", 100.0*delta3/deltaFinal);
    printf("total: %f\n\n", deltaFinal/1000.0f);*/

}


void print_percent_undef(vector<Trace> traces)
{
    int count_undef = 0;
    int count_total = 0;
    for (int i = 0; i < traces.size(); ++i)
    {
        for (int j = 0; j < traces[i].pts_rise_edge.size(); ++j)
        {
            if(traces[i].pts_rise_edge[j] < 0)
            {
                count_undef++;
            }
            count_total++;
        }
        for (int j = 0; j < traces[i].pts_fall_edge.size(); ++j)
        {
            if(traces[i].pts_fall_edge[j] < 0)
            {
                count_undef++;
            }
            count_total++;
        }
    }
    printf("Pontos indefinidos : %.1f %\n\n", (double)100*count_undef/count_total);

}

void undefined_points_treatment(vector<Trace>& traces)
{

    bool redefined_edge0 = false;
    bool redefined_edge1 = false;
    bool redefined_edge2 = false;
    bool redefined_edge3 = false;

    double width = traces[0].pts_fall_edge[0] - traces[0].pts_rise_edge[0];
    double bottom = traces[1].pts_rise_edge[0] - traces[0].pts_fall_edge[0];

    for (int i = 0; i < traces.size(); ++i)
    {
        for (int j = 0; j < traces[i].pts_rise_edge.size(); ++j)
        {

            if(traces[i].pts_rise_edge[j] < 0)  // edge 0 in groove
            {
                if(i % 2 == 0)
                {
                    if(traces[i].pts_fall_edge[j] > 0) // edge 1 defined?
                    {
                        traces[i].pts_rise_edge[j] = traces[i].pts_fall_edge[j] - width;
                    }
                    else if(traces[i+1].pts_rise_edge[j] > 0) // edge 2 defined?
                    {
                        traces[i].pts_rise_edge[j] = traces[i+1].pts_rise_edge[j] - (width + bottom);
                    }
                    else if(traces[i+1].pts_fall_edge[j] > 0) // edge 3 defined?
                    {
                        traces[i].pts_rise_edge[j] = traces[i+1].pts_fall_edge[j] - (2*width + bottom);
                    }
                    else // falta implementar caso encontre ponto definido num tempo a frente
                    {
                        traces[i].pts_rise_edge[j] = traces[i].pts_rise_edge[j-1];
                    }
                    redefined_edge0 = true;
                }
                else // edge 2 in groove
                {
                    if(traces[i].pts_fall_edge[j] > 0) // edge 3 defined?
                    {
                        traces[i].pts_rise_edge[j] = traces[i].pts_fall_edge[j] - width;
                    }
                    else if(!redefined_edge0)
                    {
                        traces[i].pts_rise_edge[j] = traces[i-1].pts_rise_edge[j] + (width + bottom);
                    }
                    else if(!redefined_edge1)
                    {
                        traces[i].pts_rise_edge[j] = traces[i-1].pts_fall_edge[j] + bottom;
                    }
                    else // falta implementar caso encontre ponto definido num tempo a frente
                    {
                        traces[i].pts_rise_edge[j] = traces[i].pts_rise_edge[j-1];
                    }
                    redefined_edge2 = true;
                }
                draw_green(j, traces[i].pts_rise_edge[j]);
            }

            if(traces[i].pts_fall_edge[j] < 0)
            {
                if(i % 2 == 0) // edge 1 in groove
                {
                    if(traces[i+1].pts_rise_edge[j] > 0) // edge 2 defined?
                    {
                        traces[i].pts_fall_edge[j] = traces[i+1].pts_rise_edge[j] - bottom;
                    }
                    else if(traces[i+1].pts_fall_edge[j] > 0) // edge 3 defined?
                    {
                        traces[i].pts_fall_edge[j] = traces[i+1].pts_fall_edge[j] - (width + bottom);
                    }
                    else if(!redefined_edge0)
                    {
                        traces[i].pts_fall_edge[j] = traces[i].pts_rise_edge[j] + width;
                    }
                    else // falta implementar caso encontre ponto definido num tempo a frente
                    {
                        traces[i].pts_fall_edge[j] = traces[i].pts_fall_edge[j-1];
                    }
                    redefined_edge1 = true;
                }
                else // edge 3 in groove
                {
                    if(!redefined_edge0)
                    {
                        traces[i].pts_fall_edge[j] = traces[i-1].pts_rise_edge[j] + (2*width + bottom);
                    }
                    else if(!redefined_edge1)
                    {
                        traces[i].pts_fall_edge[j] = traces[i-1].pts_fall_edge[j] + (width + bottom);
                    }
                    else if(!redefined_edge2)
                    {
                        traces[i].pts_fall_edge[j] = traces[i].pts_rise_edge[j] + width;
                    }
                    else
                    {
                        traces[i].pts_fall_edge[j] = traces[i].pts_fall_edge[j-1];
                    }
                    redefined_edge0 = false; // reseting.
                    redefined_edge1 = false; // reseting.
                    redefined_edge2 = false; // reseting.
                    redefined_edge3 = false; // reseting.
                }
                draw_green(j, traces[i].pts_fall_edge[j]);
            }

        }
    }
}

vector<double> unify_groove(vector<Trace> traces)
{

    vector<double> groove;

    double dist_to_last_groove = 0;

    for(int i = 0; i < traces.size()-1; i += 2)
    {
        for (int j = 0; j < traces[i].pts_rise_edge.size(); ++j)
        {
            double mean = (traces[i].pts_fall_edge[j] + traces[i].pts_rise_edge[j] +
                            traces[i+1].pts_fall_edge[j] + traces[i+1].pts_rise_edge[j])/4 + dist_to_last_groove;

            groove.push_back(mean);
        }

        if(i < traces.size()-3)
        {
            dist_to_last_groove -= traces[i].pts_fall_edge[0] - traces[i+2].pts_fall_edge[0];
        }
    }


    // derivada
    for (int idx = 0; idx < groove.size()-1; ++idx)
    {
        groove[idx] = groove[idx+1] - groove[idx];
    }
    groove[groove.size()-1] = groove.size()-2;


    //resampling


    int n_samples = groove.size()-1;

    double sampling_time = 0;

    double old_samplerate = 104000;
    double new_samplerate = 44100;

    printf("Reamostragem de %.0f para %.0f Hertz\n", old_samplerate, new_samplerate);

    double old_period = 1/old_samplerate;
    double new_period = 1/new_samplerate;

    double total_time = n_samples * old_period;

    vector<double> resampled;

    while(sampling_time < total_time)
    {
        double decimal_index = sampling_time/old_period;

        int int_index = (int) decimal_index;

        double delta = decimal_index - int_index;

        double new_sample = groove[int_index] + (groove[int_index+1] - groove[int_index]) * delta;

        resampled.push_back(new_sample);

        sampling_time += new_period;
    }

    return resampled;
}

void export_wave(vector<double> pre_audio, int samplerate, const char* filename)
{
    double amplitude = 10000;

    double max = 0;
    for (int i = 0; i < pre_audio.size(); ++i)
    {
        if(max < fabs(pre_audio[i]))
        {
            max = fabs(pre_audio[i]);
        }
    }

    short *wav_buff = new short int[pre_audio.size()];

    for (int i = 0; i < pre_audio.size(); ++i)
    {
        // pre_audio[i] /= 0.1;

        wav_buff[i] = short(pre_audio[i] * amplitude);

    }

    write_wav((char*)filename, pre_audio.size(), wav_buff, samplerate);
    // write_wav("tcc.wav", pre_audio.size(), wav_buff, samplerate);

    return;

}

vector<Trace> trace_following(const Mat& mat_src, const Mat& mat_src2, Mat& mat_visual)
{

    g_temp_mat = &mat_visual;

    cerrln("Inicializando traços.\n");
    vector<Trace> traces = initialize_traces(mat_src, mat_src2, 30);

    Mat mat_dy;
    cerrln("Obtendo matriz de derivada.");
    filter(mat_src, mat_dy, traces[0].mean_trace_width[0], 0.2);

    cerrln("Inicio do laço de detecção de candidatos.\n")

    // this for processes the image columnwise.
    for (int i = 0; i < mat_src.cols; ++i)
    {
        print_progress(i, mat_src.cols);

        g_idx_col = i;
        vector<Candidate> pts_candidate;

        edge_detection_coarse(mat_src.col(i), mat_dy.col(i), pts_candidate);
        edge_detection_fine(mat_src.col(i), pts_candidate);

        define_traces(mat_src, pts_candidate, traces, i);
    }

    cerrln("\n")
    print_percent_undef(traces);

    cerrln("Tratando pontos indefinidos.")
    undefined_points_treatment(traces);

    cerrln("Unificando segmentos de trilha.")
    vector<double> pre_audio = unify_groove(traces);

    const char* filename = "audio_extraido.wav";

    printf("Exportando audio para o arquivo \"%s\"", filename);
    export_wave(pre_audio, 44100, filename);


    return traces;
}