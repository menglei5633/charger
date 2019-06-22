#include <istream>
#include "basic.h"

using namespace std;

class Test {
public:
    static void Test_A_s(Params params, int time, vector<vector<double>>* r, bool* com, int id) {
        // cout<<"test...."<<endl;
        vector<vector<double>> result(4);
        for (int i = 0; i< 4; i++) {
            for (int j = 30; j<=360; j+=30) {
                result[i].push_back(0);
            }
        }
        // cout<<"dddd"<<endl; 
        Algorithm a = Algorithm(params);
        for (int t = 0; t <time; t++) {
            // cout<<"fffff"<<endl; 
            a.reBuild();
            // cout<<"eeeeee"<<endl;
            cout<<"线程"<<id<<"开始"<<(t+1)<<"次"<<endl;
            for (int A_s = 30; A_s <= 360; A_s += 30) {
                a.A_s  = 1.0*A_s*M_PI/180;
                // a.A_s = 2*M_PI;
                a.is_all_set = false;      //需重新计算支配集
                a.nColor = 4;
                // cout<<"cccc"<<endl;
                double U  = a.getUOfResult(a.centralizedAlgorithm());
                // double U  = a.getUOfResult(a.distributedAlgorithm());
                // cout<<U<<endl;  
                // cout<<"cccc"<<endl;
                double U_cover = a.getUOfResult(a.greedyCoverAlgorithm());
                // cout<<"dddd"<<endl;
                double U_u = a.getUOfResult(a.greedyUtilityAlgorithm());
                
                a.nColor = 1;
                double U_  = a.getUOfResult(a.centralizedAlgorithm());
                // double U_  = a.getUOfResult(a.distributedAlgorithm());
                // cout<<"bbbbb"<<endl;
                // cout<<(A_s/30-1)<<endl;
                result[0][A_s/30 - 1] += U;
                result[1][A_s/30 - 1] += U_cover;
                result[2][A_s/30 - 1] += U_u;
                result[3][A_s/30 - 1] += U_;
                // cout<<"aaaa"<<endl;
            }
            cout<<"线程"<<id<<"完成"<<(t+1)<<"次"<<endl;
        }
        
        for (int i = 0; i<4; i++) {
            for (int j = 0; j<12; j++) {
                (*r)[i][j] = result[i][j];
            }
        }
        
        com[id] = true;
        cout<<"线程结束"<<endl;
    }
};