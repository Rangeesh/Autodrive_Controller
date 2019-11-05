#ifndef GAINS_H
#define GAINS_H

class Gains {
public:
    float k_lat;
    float k_head;
    float k_o;

    Gains() : k_lat(0), k_head(0), k_o(0) {}
    Gains(float k_lateral, float k_heading, float k_omega) :
        k_lat(k_lateral), k_head(k_heading), k_o(k_omega) {}
    friend std::ostream& operator<<(std::ostream &out, const Gains &gains);
};

inline std::ostream& operator<<(std::ostream &out, const Gains &gains) {
    out << "k_lat: " << gains.k_lat << "\t k_head: " << gains.k_head << "\t k_o: " << gains.k_o;
    return out;
}

#endif