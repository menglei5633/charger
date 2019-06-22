#include <thread>
#include <unistd.h>
#include "basic.h"
#include "test.h"

using namespace std;





int main() {
    srand((unsigned)time(0));
    Params params;
    params.m = 100;
    params.n = 20;
    params.x_size = 20;
    params.y_size = 20;
    params.radius = 10;
    params.A_o = 60*M_PI / 180;
    params.A_s = 30*M_PI / 180;
    params.nColor = 4;
    // cout<<params.A_s<<endl;
    // cout<<(params.A_s*180/M_PI)<<endl;
    vector<vector<vector<double>>>  result(4);    //4个线程
    for (int i = 0; i<4; i++) {   //线程
        result[i] = vector<vector<double>>(4);
        for (int j = 0; j<4; j++) {
            result[i][j] = vector<double>(12);
            for (int k = 0; k<12; k++) {
                result[i][j][k] = 0;
            }
        }
    }
    bool is_complete[4];   //线程
    for (int i =0; i< 4; i++) {  //线程
        is_complete[i] = false;
    }
    // cout<<"aaaaaa"<<endl;
    for (int i = 0; i< 4; i++) {   //线程
        int num_of_time = 25;
        // if (i %2 == 0) {
        //     num_of_time = 12;
        // }
        thread t(Test::Test_A_s, params, num_of_time, &result[i], is_complete, i);
        t.detach(); 
    }
    // cout<<"bbbb"<<endl;
    while(true) {
        bool is_break = true;
        for (int i = 0; i<4; i++) {   //线程
            cout<<is_complete[i]<<" ";
            if (!is_complete[i]) {
                is_break = false;
                break;
            }
        }
        cout<<endl;
        if (is_break) {
            break;
        }
        sleep(5);
    }
    vector<vector<double>> r(4);
    for (int i = 0; i<4; i++) {
        r[i] = vector<double>(12);
        for (int j =0; j<12; j++) {
            r[i][j] = 0;
        }
    }
    for (int i = 0; i<4; i++) {
        for (int j =0; j<12; j++) {
            for (int k = 0; k<4; k++) {    //线程
                r[i][j] += result[k][i][j];
            }
        }
    }
    for (int i = 0; i<4; i++) {
        for (int j =0; j<12; j++) {
            r[i][j] /= 100;
        }
    }
    //写入文件
    ofstream f("./test_A_s_result.txt");
    for (int i = 0; i<4; i++) {
        for (int j = 0; j< 12; j++) {
            f<< r[i][j]<< " ";
        }
        f<<endl;
    }
    f.close();
    // algorithm.writeChargerSensorToFile("./data.txt");
    // algorithm.test();
    return 0;
}