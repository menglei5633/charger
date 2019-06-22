#ifndef ALGORITHM
#define ALGORITHM
#include <vector>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <string.h>
#include <fstream>
#include <semaphore.h>
#include <thread>
#include <unistd.h>
// #include <functional>
using namespace std;


struct Sensorinfo{
	double tx,ty,te,tdir;   //x,y,所需电量,方向
};

struct Chargerinfo{
	double cx,cy;   //x,y
};



struct Params {
    int n, m;
    int x_size, y_size;
    int radius;
    double A_s, A_o;
    int nColor;
    string profile = "";
};

class Algorithm {
public:
    //常量：
    const double Emax=200/24,Emin=50/24;
    const double alpha=10000,beta=40;

    vector<Sensorinfo> sensor_list;
    vector<Chargerinfo> charger_list;
    int n;    //charger个数
    int m;     //sensor个数
    int x_size;
    int y_size;
    double A_s;    //charger扇形的张角，弧度值
    double A_o;    //sensor扇形的张角，弧度制
    int radius;   //charger和sensor里半径较小的半径
    int nColor;   //颜色的个数
    bool* cover;   //记录每个charger可以覆盖的sensor，是一个大小为nxm的矩阵
    double* dists;  //记录每个charger和每个sensor间的距离， 是一个大小为nxm的矩阵
    double* P;   //记录每个charger对每个sensor充电电量，覆盖不到的为0，是一个大小为nxm的矩阵
    double* N;   //记录sensor之间分享电量的效率，大于0小于1

    double* E_s;  //记录sensor之间分享的电量

    vector<vector<vector<int>>> all_domain_set;
    bool is_all_set = false;




    Algorithm(Params params);
    ~Algorithm();

    

    double getdist(int ch,int se);  //计算charger和sensor间的距离

    vector<vector<int>> getDominantCoverageSet(int ch);  //计算charger ch 的dominant coverage set
    vector<vector<vector<int>>> getAllDominantCoverageSet();  //计算所有charger的dominant coverage set

    vector<vector<int>> centralizedAlgorithm();  //集中式算法, 返回每个charger充电的sensor
    vector<vector<int>> distributedAlgorithm();  //分布式算法，返回每个charger充电的sensor
    vector<vector<int>> greedyUtilityAlgorithm();   //效用贪心算法
    vector<vector<int>>  greedyCoverAlgorithm();    //覆盖贪心算法
    double getUOfResult(vector<vector<int>> result);   //根据选择的结果计算效用，有能量分享
    double getUOfResult1(vector<vector<int>> result);  //无能量分享

    void writeChargerSensorToFile(string filename);  //将charger和sensor数据写入文件
    void reBuild();

    void test() {
        vector<int> v1;
        v1.push_back(1);
        v1.push_back(2);
        vector<int> v2;
        v2.push_back(1);
        v2.push_back(3);
        auto v3 = vectors_intersection(v1, v2);
        auto v4 = vectors_set_union(v1, v2);
        for (int i = 0; i< v3.size(); i++) {
            cout<<v3[i]<<" ";
        }
        cout<<endl;
        for (int i = 0; i< v4.size(); i++) {
            cout<<v4[i]<<" ";
        }
        cout<<endl;
    }


    
    

private:
    void generate_random();  //随机生成charger和sensor
    void readProfile(string file); //从文件读取，未实现
    double getAngle(int ch, int se);  //计算充电器和sensor的角度，由charger指向sensor
    double getAngle2(int se, int ch);  //计算充电器和sensor的角度，由sensor指向charger
    bool checkSensorCoverCharger(int se, int ch);  // 判断sensor是否覆盖charger,仅判断角度覆盖
    double getP(int ch, int se);  //计算充电器对sensor的充电量
    void getCover(); // 计算cover矩阵

    double F_exactly(vector<vector<int>> G, vector<vector<vector<int>>> all_domain_set);  //F函数
    double F_exactly_distribute(int ich, vector<vector<int>> Q, vector<int> neighbor);  //分布式算法F值


    vector<double> greedyEnergySharingStrategy(vector<double> charger_value);  //电量分享贪心算法
    vector<double> distributeGreedyEnergySharingStrategy(vector<double> charger_value,
                                                        vector<int> all_ner_set);  //分布式电量分享贪心算法
    double U(   vector<double> sensor_charger_value);  //计算电量效用
    double U_(vector<double> sensor_charger_value, vector<int> sensor_set);  //计算部分sensor的电量效用

    double det_f(int ich, vector<vector<int>> Q, vector<int> neighbor,
                vector<int> currColor, int* poicy);   //分布式算法中，计算charger ich 的最大det F值和对应的策略


    struct Message{
        int time_slot;
        bool is_update;  //false-null, true-UPD
        int ich;
        double det_F;
        int poicy;
        int color;
    };
    sem_t g_semaphore;
    sem_t print;
    static void distributeThread(vector<vector<Message>>* messages, vector<vector<double>>* currDetf, 
                                vector<vector<int>>* currPoicy, vector<int>* currColor, vector<int>* Q, 
                                vector<int> neighbor, int ich, bool* com,  Algorithm* alg);



    

    double mindo_(double a, double b) {
        return a<b?a:b;
    }

    double minin_(int a, int b) {
        return a<b?a:b;
    }

    struct sensor_dev
    {
        int id;
        double angle;
    };

    struct sensor_dev2
    {
        int id;
        double is_share;
    };



    static bool cmp2(sensor_dev2 a, sensor_dev2 b) {  //降序
        if (a.is_share < b.is_share) {
            return false;
        } else {
            return true;
        }
    }
    

    static bool cmp(sensor_dev a, sensor_dev b) {  //升序
        if (a.angle < b.angle) {
            return true;
        } else {
            return false;
        }
    }

    static vector<int> vectors_intersection(vector<int> v1,vector<int> v2){   //两个vector求交集
        vector<int> v;
        sort(v1.begin(),v1.end());
        sort(v2.begin(),v2.end());   
        set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(),back_inserter(v));//求交集 
        return v;
    }
    static vector<int> vectors_set_union(vector<int> v1,vector<int> v2){   //并集
        vector<int> v;
        sort(v1.begin(),v1.end());   
        sort(v2.begin(),v2.end());   
        set_union(v1.begin(),v1.end(),v2.begin(),v2.end(),back_inserter(v));//求交集 
        return v;
    }
    static bool is_element_in_vector(vector<int> v,int element){   //判断是否存在
        vector<int>::iterator it;
        it=find(v.begin(),v.end(),element);
        if (it!=v.end()){
            return true;
        }
        else{
            return false;
        }
    }
};

#endif