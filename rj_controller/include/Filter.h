#ifndef FILTER_H
#define FILTER_H

#include <vector>

class Filter {
    float filtered_value;

public:
    //establishes vector for storing values
    std::vector<float> values;
    void single_value_data(float &single_value, int max_size, float percent);

    float get_filtered_value() {return filtered_value;}

};



#endif
