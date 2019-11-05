#ifndef LATERALGAINS_H
#define LATERALGAINS_H

#include <ostream>

class LateralGains {
public:
    float k_lat;
    float k_head;
    float k_o;

    LateralGains() : k_lat(0), k_head(0), k_o(0) {}
    LateralGains(float k_lateral, float k_heading, float k_omega) :
        k_lat(k_lateral), k_head(k_heading), k_o(k_omega) {}
    friend std::ostream& operator<<(std::ostream &out, const LateralGains &gains);
};

inline std::ostream& operator<<(std::ostream &out, const LateralGains &gains) {
    out << "k_lat: " << gains.k_lat << "\t k_head: " << gains.k_head << "\t k_o: " << gains.k_o;
    return out;
}

#endif
