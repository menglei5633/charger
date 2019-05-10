#include <vector>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <string.h>
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
    const double Emax=20000/24,Emin=5000/24;
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

    Algorithm(Params params);
    ~Algorithm();

    

    double getdist(int ch,int se);  //计算charger和sensor间的距离

    vector<vector<int>> getDominantCoverageSet(int ch);  //计算charger ch 的dominant coverage set
    vector<vector<vector<int>>> getAllDominantCoverageSet();  //计算所有charger的dominant coverage set

    vector<vector<int>> centralizedAlgorithm();  //集中式算法
    vector<vector<int>> distributedAlgorithm();  //分布式算法


    
    

private:
    void generate_random();  //随机生成charger和sensor
    void readProfile(string file); //从文件读取
    double getAngle(int ch, int se);  //计算充电器和sensor的角度，由charger指向sensor
    double getAngle2(int se, int ch);  //计算充电器和sensor的角度，由sensor指向charger
    bool checkSensorCoverCharger(int se, int ch);  // 判断sensor是否覆盖charger,仅判断角度覆盖
    double getP(int ch, int se);  //计算充电器对sensor的充电量
    void getCover(); // 计算cover矩阵

    double F_exactly(vector<vector<int>> G, vector<vector<vector<int>>> all_domain_set);  //F函数

    vector<double> greedyEnergySharingStrategy(vector<double> charger_value);  //电量分享贪心算法
    double U(vector<double> sensor_charger_value);  //计算电量效用

    double det_f(int ich, vector<vector<int>> Q, int poicy, vector<vector<int>> neighbor);   //分布式算法中，计算charger ich 的local f值

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
};