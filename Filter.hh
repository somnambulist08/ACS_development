#include <math.h>

class pt1Filter {
public:
    init(float cutoff, float dt);
    float apply(float input);
private:
    float k;
    float state;
};