#include "basic.h"

Algorithm::Algorithm(Params params){
    this->n = params.n;
    this->m = params.m;
    this->x_size = params.x_size;
    this->y_size = params.y_size;
    this->radius = params.radius;
    this->A_s = params.A_s;
    this->A_o = params.A_o;
    this->nColor = params.nColor;
    this->cover = new bool[n*m];
    this->dists = new double[n*m];
    this->N = new double[m*m];
    // this->E_s = new double[n*m];
    // memset(E_s, 0, sizeof(E_s));

    for (int i = 0; i< m*m; i++) {
        if ((rand() / double(RAND_MAX)) < 0.5) 
            this->N[i] = rand() / double(RAND_MAX);
        else
            this->N[i] = 0;
    }

    if (params.profile.empty()) {
        this->generate_random();
    } else {
        this->readProfile(params.profile);
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            dists[i*m + j] = getdist(i, j);
        }
    }
    getCover();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            P[i*m + j] = getP(i, j);
        }
    }
}

double Algorithm::getdist(int ch,int se) {
    
    double dist = sqrt((charger_list[ch].cx - sensor_list[se].tx) * 
                (charger_list[ch].cx - sensor_list[se].tx) +
                (charger_list[ch].cy - sensor_list[se].ty) *
                (charger_list[ch].cy - sensor_list[se].ty));
    // cout<<"计算距离"<<ch << " " << se << ":" << dist <<endl;
    return dist;
}

void Algorithm::generate_random(){
    cout<< "随机生成..."<<endl;
    for (int i=0;i<n;i++) {
        Chargerinfo c;
		c.cx=rand() / double(RAND_MAX) * x_size;
		c.cy=rand() / double(RAND_MAX) * y_size;
		cout<<c.cx<<" "<<c.cy<<endl;
        charger_list.push_back(c);
	}
    for (int i=0; i<m; i++){
        Sensorinfo s;
        s.tx=rand() / double(RAND_MAX) * x_size;
		s.ty=rand() / double(RAND_MAX) * y_size;
		s.te=rand() / double(RAND_MAX) * (Emax-Emin) + Emin;
		s.tdir=rand() / double(RAND_MAX) * (2*M_PI);
        sensor_list.push_back(s);
        cout<<s.tx << " " << s.ty << " " << (s.tdir * 180 /M_PI) <<endl;
    }
}

bool Algorithm::checkSensorCoverCharger(int se, int ch) {
    double angle = getAngle2(se, ch);
    double angle_cha = abs(angle - sensor_list[se].tdir);
    // cout<<"角度差 "<< ch << " " << se << ": "<<angle_cha*180/M_PI <<endl;
    if (angle_cha <= A_o/2) {
        return true;
    } else {
        return false;
    }
}

double Algorithm::getP(int ch, int se) {
    if (cover[ch*m+se]) {
        return (1.0*alpha / (dists[ch*m+se]+beta) * (dists[ch*m+se]+beta));
    } else {
        return 0;
    }
}

void Algorithm::getCover() {
    cout<< "计算cover矩阵..."<<endl;
    for (int ich = 0; ich < n; ich ++) {
        for (int ise = 0; ise < m; ise ++) {
            if (dists[ich*m+ise] <= radius && checkSensorCoverCharger(ise, ich)) {
                cover[ich * m + ise] = true;
                
            } else {
                cover[ich * m + ise] = false;
                
            }
        }
        // cout<<"覆盖集: ";
        // for (int j = 0; j < m; j++) {
        //     if (cover[ich * m + j]) {
        //         cout<< j << " ";
        //     }
        // }
        // cout << endl;
    }
}

double Algorithm::getAngle(int ch, int se) {
    double yy = sensor_list[se].ty - charger_list[ch].cy;
    double xx = sensor_list[se].tx - charger_list[ch].cx;
    double angle = atan2(yy, xx);
    if (angle<0) {
        angle=M_PI*2+angle;
    } 
	return angle;
}

double Algorithm::getAngle2(int se, int ch) {
    double yy = charger_list[ch].cy - sensor_list[se].ty;
    double xx = charger_list[ch].cx - sensor_list[se].tx;
    double angle = atan2(yy, xx);
    if (angle<0) {
        angle=M_PI*2+angle;
    } 
	return angle;
}

vector<vector<int>> Algorithm::getDominantCoverageSet(int ch) {
    cout<<endl<<"计算charger"<<ch<<endl;
    vector<vector<int>> set;
    vector<sensor_dev> cover_set;

    
    for(int ise = 0; ise < m; ise++) {
        if (cover[ch*m + ise]) {
            sensor_dev se;
            se.id = ise;
            se.angle = getAngle(ch, ise);
            cover_set.push_back(se);
        }
    }
    if (cover_set.size() == 0) {
        return set;
    }

    sort(cover_set.begin(), cover_set.end(), cmp);

    for (int i  = 0; i < cover_set.size(); i++) {
        cout<< cover_set[i].angle*180/M_PI << endl;
    }

    int start  = 0;
    int end = -1;
    int first_set_end = -2;
    for (start = 0; start < cover_set.size(); start++) {
        cout<<"start : "<< start <<" ";
        double end_angle = cover_set[start].angle + A_s;
        cout<<"end_angle : "<<(end_angle*180/M_PI)<<" "; 
        vector<int> one_set;
        one_set.push_back(start);
        if (end_angle < M_PI * 2) {
            int j = start + 1;
            while(j < cover_set.size() && cover_set[j].angle <= end_angle) {
                one_set.push_back(j);
                j++;
            }
            if (j-1 > end) {
                set.push_back(one_set);
                end = j - 1;
            }
        } else {
            int j;
            end_angle = end_angle - M_PI*2;
            for (j = start + 1; j < cover_set.size(); j++) {
                one_set.push_back(j);
            }
            for (j = 0; j < cover_set.size(); j++) {
                if (cover_set[j].angle <= end_angle) {
                    one_set.push_back(j);
                } else {
                    break;
                }
            }
            if (j-1 == first_set_end) {
                set[0] = one_set;
                break;
            } else {
                j = j-1 + cover_set.size();
                if (j > end) {
                    set.push_back(one_set);
                    end = j;
                }
            }
        }
        cout<<"end : " << end<<endl; 

        if (start == 0) {
            first_set_end = end;
        }
    }
    for (int i = 0; i < set.size(); i++) {
        for (int j = 0; j < set[i].size(); j++) {
            set[i][j] = cover_set[set[i][j]].id;
        }
    }
    return set;
}

vector<vector<vector<int>>> Algorithm::getAllDominantCoverageSet() {
    vector<vector<vector<int>>> all_set;
    for (int i = 0; i < n; i++) {
        auto set = getDominantCoverageSet(i);
        all_set.push_back(set);
    }
    return all_set;
}

void Algorithm::readProfile(string file) {
    return;
}

double Algorithm::U(vector<double> sensor_charger_value) {
    double u = 0;
    for (int i = 0; i < m; i++) {
        u += mindo_(1, 1.0*sensor_charger_value[i] / sensor_list[i].te);
    }
    return u;
}

double Algorithm::F_exactly(vector<vector<int>> G, vector<vector<vector<int>>> all_domain_set) {
    //G里包含了每个charger的选择的策略，vector里按顺序对应每个color
    cout<<"F精确求"<<endl;
    int all_case_num = 1;
    for (int i = 0; i < G.size(); i++) {
        if (G[i].size() > 0)
            all_case_num = all_case_num * G[i].size();
    }
    // cout<<"all_case_num:" << all_case_num<<endl;
    vector<vector<double>> sensor_charger_values(all_case_num);
    for (int i = 0; i < all_case_num; i++) {
        for (int j = 0; j < m; j++) {
            sensor_charger_values[i].push_back(0);
        }
    } 
    int case_num_till_now = 1;
    int case_num_last = 1;
    for (int i = 0; i < G.size(); i++) {
        if (!(G[i].size() > 0))
            continue;
        case_num_last = case_num_till_now;
        case_num_till_now = case_num_till_now * G[i].size();
        int every_case_entity_num_now = all_case_num / case_num_till_now;
        int every_case_entity_num_last = all_case_num / case_num_last;
        for (int j = 0; j < case_num_last; j++) {
            for (int k = 0; k < G[i].size(); k++) {
                int start = k*every_case_entity_num_now + j*every_case_entity_num_last;
                int end = (k+1)*every_case_entity_num_now + j*every_case_entity_num_last;
                //当第i个charger选择第k个策略时，计算它给周围sensor的充电量
                int domain_set_id = G[i][k];
                for (int o = start; o < end; o++) {
                    for (int pp = 0; pp < all_domain_set[i][domain_set_id].size(); pp++) {
                        int sensor_id = all_domain_set[i][domain_set_id][pp];
                        // cout<<"sensor_id:"<<sensor_id<<endl;
                        sensor_charger_values[o][sensor_id] += P[i*m+sensor_id];
                    }
                }
            }
        } 
    }
    //已经计算完每个case的sensor充电量，接下来进行电量分享
    
    double all_u = 0;
    for (int i = 0; i < all_case_num; i++) {
        // cout<<"分享前"<<endl;
        // for (int j = 0; j < m; j++) {
        //     cout<<sensor_charger_values[i][j] << " ";
        // }
        // cout<<endl;
        sensor_charger_values[i] = this->greedyEnergySharingStrategy(sensor_charger_values[i]);
        // cout<<"分享后"<<endl;
        // for (int j = 0; j < m; j++) {
        //     cout<<sensor_charger_values[i][j] << " ";
        // }
        // cout<<endl;
        all_u += this->U(sensor_charger_values[i]);
        // cout<<"all_u" <<all_u<<endl;
    }
    // cout<<"end f"<<endl;
    return 1.0*all_u/all_case_num;
}

vector<double> Algorithm::greedyEnergySharingStrategy(vector<double> charger_value) {
    vector<int> charger_set; //记录被充电的sensor
    vector<int> no_charger_set;
    for (int i = 0; i<charger_value.size(); i++) {
        if (charger_value[i] >0 ) {
            charger_set.push_back(i);
        } else {
            no_charger_set.push_back(i);
        }
    }
    // cout<<"xx"<<endl;
    for (int ics = 0; ics < charger_set.size(); ics++) {
        vector<sensor_dev2> send_unity;
        int ics_id = charger_set[ics];
        for (int incs = 0; incs < no_charger_set.size(); incs++) {
            int incs_id = no_charger_set[incs];
            sensor_dev2 tmp;
            tmp.id = incs_id;
            tmp.is_share = (N[ics_id*m + incs] * sensor_list[ics_id].te - sensor_list[incs_id].te);
            send_unity.push_back(tmp);
        }
        // cout<<"cc"<<endl;
        sort(send_unity.begin(), send_unity.end(), this->cmp2);   //降序
        // cout<<"dd"<<endl;
        for (int j = 0; j < send_unity.size(); j++) {
            auto tmp = send_unity[j];
            if (charger_value[tmp.id] < sensor_list[tmp.id].te && tmp.is_share > 0 
                && charger_value[ics_id] > 0) {
                    double x = mindo_((1.0*(sensor_list[tmp.id].te - charger_value[tmp.id])/N[ics_id*m+tmp.id]),
                                        charger_value[ics_id]);
                    charger_value[ics_id] -= x;
                    charger_value[tmp.id] += x*N[ics_id*m+tmp.id];
                    // this->E_s[ics_id*m + tmp.id] = x;
            }
        }
    }
    return charger_value;
}

vector<vector<int>> Algorithm::centralizedAlgorithm() {
    auto all_domain_set = getAllDominantCoverageSet();
    cout<<"所有支配集:"<<endl;
    for (int i = 0; i<all_domain_set.size(); i++) {
        cout<<"charger "<<i<<"支配集个数: "<<all_domain_set[i].size()<<endl;
        for (int j = 0; j<all_domain_set[i].size(); j++) {
            for (int k = 0; k < all_domain_set[i][j].size(); k++) {
                cout<<all_domain_set[i][j][k]<<" ";
            }
            cout<<endl;
        }
    }
    vector<vector<vector<int>>> result(n);
    vector<vector<int>> G(n);
    for (int ico = 0; ico < nColor; ico++) {
        cout<<"color:"<<ico<<endl;
        for (int ich = 0; ich < n; ich++) {
            if (all_domain_set[ich].size() == 0) 
                continue;
            if (all_domain_set[ich].size() == 1) {  //无需比较，直接放入
                result[ich].push_back(all_domain_set[ich][0]);
                G[ich].push_back(0);
                continue;
            }
            cout<<"charger:"<<ich<<endl;
            int max_policy = -1;
            double max_u = 0;
            cout<<all_domain_set[ich].size()<<endl;
            
            for (int ipo = 0; ipo < all_domain_set[ich].size(); ipo++) {
                vector<vector<int>> G_(G);
                G_[ich].push_back(ipo);
                cout<<"G_:---------"<<endl;
                for (int ii = 0; ii < G_.size(); ii++) {
                    for (int jj = 0; jj < G_[ii].size(); jj++) {
                        cout <<G_[ii][jj] << " ";
                    }
                    cout<<endl;
                }
                cout<<"--------"<<endl;
                double u_ = this->F_exactly(G_, all_domain_set);
                cout<<"F 期望："<<u_<<endl;
                if (u_ > max_u) {
                    max_u = u_;
                    max_policy = ipo;
                }
            }
            result[ich].push_back(all_domain_set[ich][max_policy]);
            G[ich].push_back(max_policy);
        }
    }
    cout<<"G:---------"<<endl;
    for (int ii = 0; ii < G.size(); ii++) {
        for (int jj = 0; jj < G[ii].size(); jj++) {
            cout <<G[ii][jj] << " ";
        }
        cout<<endl;
    }
    cout<<"--------"<<endl;
    vector<vector<int>> result1(n);
    //sample
    for (int ich = 0; ich<n; ich++) {
        if (all_domain_set[ich].size() == 0) {
            continue;
        }
            
        int sample_color = rand() / double(RAND_MAX) * nColor;
        cout<<"sample "<<sample_color<<endl;
        result1[ich] = result[ich][sample_color];
    }
    return result1;
}

double Algorithm::det_f(int ich, const vector<vector<int>> Q, const int poicy, 
                        const vector<vector<int>> neighbors) {
    
}

vector<vector<int>> Algorithm::distributedAlgorithm() {
    vector<vector<int>> neighbor(n);
    for (int ich1 = 0; ich1 < n - 1; ich1++) {
        for (int ich2 = ich1+1; ich2 < n; ich2++) {
            bool is_neighbor = false;
            for (int ise = 0; ise < m; ise++) {
                if (!cover[ich1*m+ise])
                    continue;
                if (cover[ich2*m+ise]) {
                    is_neighbor = true;
                    break;
                } else {
                    for (int ise2 = 0; ise2 < m; ise2++) {
                        if (cover[ich2*m+ise2] && N[ise*m+ise2] > 0 && N[ise2*m+ise] > 0) {
                            is_neighbor = true;
                        }
                    }
                }
            }
            if (is_neighbor) {
                neighbor[ich1].push_back(ich2);
                neighbor[ich2].push_back(ich1);
            }
        }
    }
    //已经得到每个charger的邻居
    auto all_domain_set = getAllDominantCoverageSet();  //得到每个charger的支配集
    vector<int> poicties(n);   //记录每个charger所选择的策略
    vector<vector<int>> Q(n); //记录每个charger，每个颜色的策略，下标自动对应颜色
    vector<int> isUpdate(n);  //记录每个charger发送消息的类型， 0-没发送，1-null， 2-Update 
    for (int i = 0; i<n; i++) {
        for (int j =0; j<m; j++) {
            Q[i].push_back(-1);
        }
    }
    vector<int> currColor(n);   //记录每个charger当前所处的颜色阶段
    for (int i=0; i<n; i++) {
        currColor.push_back(0);
    }
    vector<double> currDetf(n);   //记录当前每个charger的DETf的值

    while(true) {

    }
}

Algorithm::~Algorithm() {
    if (N != NULL)
        delete[] N;
    if (cover != NULL)
        delete[] cover;
    if (dists != NULL)
        delete[] dists;
    if (P != NULL) 
        delete[] P;  
    // delete[] E_s;
}