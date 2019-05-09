#include "basic.h"


int main() {
    srand((unsigned)time(0));
    Params params;
    params.m = 30;
    params.n = 3;
    params.x_size = 10;
    params.y_size = 10;
    params.radius = 6;
    params.A_o = 120*M_PI / 180;
    params.A_s = 60*M_PI / 180;
    params.nColor = 4;
    cout<<params.A_s<<endl;
    cout<<(params.A_s*180/M_PI)<<endl;

    Algorithm algorithm = Algorithm(params);
    // auto set = algorithm.getDominantCoverageSet(0);
    // cout<<"支配集大小：";
    // for (int i = 0; i < set.size(); i++) {
    //     cout<<set[i].size() << " ";
    // }
    // cout<<endl;
    auto result = algorithm.centralizedAlgorithm();
    cout<<"策略选择"<<endl;
    for (int i = 0; i < result.size(); i++) {
        for (int j = 0; j < result[i].size(); j++) {
            cout<< result[i][j] << " ";
        }
        cout<<endl;
    }

    return 0;
}