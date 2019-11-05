#include <Filter.h>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

void Filter::single_value_data(float &single_value, int max_size, float percent){
    // find summation of the values 
    float sum_values = 0.0f;
    for (int iterator = 0; iterator < values.size(); iterator++) {
            sum_values += values[iterator];
    }

    // calculate the average and filter the values
    float average_value;
    if (values.size() == 0) {
        filtered_value = single_value;
    }
    else {
        average_value = float (sum_values / values.size());
        filtered_value = (average_value * (1- percent)) + (single_value * percent);
    }

    // deletes front value and adds the new value to the end
    if (values.size() < max_size) {
        values.push_back(single_value);
    }
    else if (values.size() >= max_size) {
        values.erase(values.begin());
        values.push_back(single_value);
    }
}


