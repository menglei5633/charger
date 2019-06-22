#include <unistd.h>
#include "basic.h"

using namespace std;


int main() {
    srand(time(NULL));
    Params params;
    params.m = 200;
    params.n = 50;
    params.x_size = 10;
    params.y_size = 10;
    params.radius = 5;
    params.A_o = 60*M_PI / 180;
    params.A_s = 30*M_PI / 180;
    params.nColor = 4;

    Algorithm a = Algorithm(params);

    auto result = a.centralizedAlgorithm();

    return 0;
}