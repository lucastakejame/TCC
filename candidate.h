#ifndef _CANDIDATE_H_
#define _CANDIDATE_H_

struct Candidate
{
    double position; // distance from origin of the array.
    int val_left;
    int val_position;
    int val_right;
    bool fall_rise; // 0 - falling edge ,1 - rising edge
};


#endif