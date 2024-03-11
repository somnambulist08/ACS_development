#include "filter.hh"

void pt1Filter::init(float cutoff, float dt) {
    float omega = 2.0f * 3.14159265359f * cutoff * dt;
    k = omega / (omega + 1.0f);
    state = 0.0f;
};

float pt1Filter::apply(float input) {
        state += k * (input - state);
        return state;
};

