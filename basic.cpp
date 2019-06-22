#include "basic.h"

Algorithm::Algorithm(Params params){
    srand(time(NULL));
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
    this->P = new double[m*n];
    // cout<<"内存申请完毕！！！"<<endl;
    // this->E_s = new double[n*m];
    // memset(E_s, 0, sizeof(E_s));

    for (int i = 0; i< m; i++) {
        for (int j = 0; j < m; j++) {
            if ((rand() / double(RAND_MAX)) < 0.5) {
                this->N[i*m +j] = rand() / double(RAND_MAX);
                this->N[j*m +i] = rand() / double(RAND_MAX);
                if (this->N[i*m +j] <= 0) {
                    this->N[i*m +j] = 0.1;
                }
                if (this->N[j*m +i] <= 0) {
                    this->N[j*m +i] = 0.1;
                }
                // cout<<1<<endl;
            } else {
                // cout<<0<<endl;
                this->N[i] = 0;
            }
        }

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
    // cout<<"gggggggg"<<endl; 
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            P[i*m + j] = getP(i, j);
        }
    }
    // cout<<"kkkkkkk"<<endl;
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
    // cout<< "随机生成..."<<endl;
    for (int i=0;i<n;i++) {
        Chargerinfo c;
		c.cx=rand() / double(RAND_MAX) * x_size;
		c.cy=rand() / double(RAND_MAX) * y_size;
		// cout<<c.cx<<" "<<c.cy<<endl;
        charger_list.push_back(c);
	}
    for (int i=0; i<m; i++){
        Sensorinfo s;
        s.tx=rand() / double(RAND_MAX) * x_size;
		s.ty=rand() / double(RAND_MAX) * y_size;
		s.te=rand() / double(RAND_MAX) * (Emax-Emin) + Emin;
		s.tdir=rand() / double(RAND_MAX) * (2*M_PI);
        sensor_list.push_back(s);
        // cout<<s.tx << " " << s.ty << " " << (s.tdir * 180 /M_PI) <<endl;
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
        // cout<<"ch: "<<ch<<" se: "<<se<<" alpha: "<<alpha << " "<<" beta: "<<beta<<" dist: "<<dists[ch*m+se]<<" result: "<<(1.0*alpha / ((dists[ch*m+se]+beta) * (dists[ch*m+se]+beta)))<<endl;
        return (1.0*alpha / ((dists[ch*m+se]+beta) * (dists[ch*m+se]+beta)));
    } else {
        return 0;
    }
}

void Algorithm::getCover() {
    // cout<< "计算cover矩阵..."<<endl;
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
    // cout<<endl<<"计算charger"<<ch<<endl;
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

    // for (int i  = 0; i < cover_set.size(); i++) {
    //     cout<< cover_set[i].angle*180/M_PI << endl;
    // }

    int start  = 0;
    int end = -1;
    int first_set_end = -2;
    for (start = 0; start < cover_set.size(); start++) {
        // cout<<"start : "<< start <<" ";
        double end_angle = cover_set[start].angle + A_s;
        // cout<<"end_angle : "<<(end_angle*180/M_PI)<<" "; 
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
                if (one_set.size() == cover_set.size()) {
                    break;
                }
            }
        } else {
            int j;
            end_angle = end_angle - M_PI*2;
            for (j = start + 1; j < cover_set.size(); j++) {
                one_set.push_back(j);
            }
            for (j = 0; j < start; j++) {
                if (cover_set[j].angle < end_angle) {
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
                    // cout<<one_set.size()<<" "<<cover_set.size()<<endl;
                    if (one_set.size() == cover_set.size()) {
                        break;
                    }
                }
            }
        }
        // cout<<"end : " << end<<endl; 

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
        // cout<<"计算效用: "<<sensor_charger_value[i]<<" "<<sensor_list[i].te<<" "<<(1.0*sensor_charger_value[i] / sensor_list[i].te)<<endl;
    }
    return u;
}

double Algorithm::U_(vector<double> sensor_charger_value, vector<int> sensor_set) {
    double u = 0;
    for (int i = 0; i < sensor_set.size(); i++) {
        int se_id = sensor_set[i];
        u += mindo_(1, 1.0*sensor_charger_value[se_id] / sensor_list[se_id].te);
    }
    return u;
}

double Algorithm::F_exactly(vector<vector<int>> G, vector<vector<vector<int>>> all_domain_set) {
    //G里包含了每个charger的选择的策略，vector里按顺序对应每个color
    // cout<<"F精确求"<<endl;
    int all_case_num = 1;

    bool large = false;
    // cout<<"G: ";
    for (int i = 0; i < G.size(); i++) {
        if (G[i].size() > 0)
            all_case_num = all_case_num * G[i].size();
        // cout<<G[i].size()<<" ";
        if (all_case_num > 500) {
            large = true;
            break;
        }
    }

    int sample_num = 60;    //采样的次数
    sample_num = all_case_num < sample_num? all_case_num : sample_num;
    vector<vector<double>> sensor_charger_values(sample_num);
    for (int i = 0; i < sample_num; i++) {
        for (int j = 0; j < m; j++) {
            sensor_charger_values[i].push_back(0);
        }
    } 
    // cout<<endl;
    if (large == false) {
        vector<int> num_for_each_case;
        int case_num_till_now = 1;
        // cout<<"num_for_each_case: ";
        for (int i = 0; i< G.size(); i++) {
            if (G[i].size() >0) {
                case_num_till_now *= G[i].size();
            }
            num_for_each_case.push_back((all_case_num/case_num_till_now));
            // cout<<(all_case_num/case_num_till_now)<<" ";
        }
        // cout<<endl;
        // cout<<"all_case_num:" << all_case_num<<endl;


        vector<int>  sample_ids;    //存放采样的case id

        
        for(int i = 0; i < sample_num; i++) {
            int sample_int = rand() / double(RAND_MAX) * all_case_num;
            while(is_element_in_vector(sample_ids, sample_int)) {
                sample_int++;
                if (sample_int >= all_case_num) {
                    sample_int = 0;
                }
            }
            sample_ids.push_back(sample_int);
        }
        // cout<<"aaaaaaa"<<endl;

        for (int i = 0; i< sample_num; i++) {
            int sample_id = sample_ids[i];
            // vector<int> charger_ps(G.size());   //记录每个charger选择的策略
            for (int ich = 0; ich< G.size(); ich++) {
                if (G[ich].size() <= 0) {
                    // charger_ps[ich] = -1;
                    continue;
                }
                int p = sample_id / num_for_each_case[ich];
                sample_id = sample_id % num_for_each_case[ich];
                // if (p >= all_domain_set[ich].size()) {
                //     cout<<p<<" "<<all_domain_set[ich].size()<<endl;
                // }
                int domain_id = G[ich][p];
                for (int j = 0; j< all_domain_set[ich][domain_id].size(); j++) {
                    int sensor_id = all_domain_set[ich][domain_id][j];
                    sensor_charger_values[i][sensor_id] += this->P[ich*m+sensor_id];
                }
            }
        }
        // cout<<"bbbb"<<endl;
    } else {
        //简单采样
        for (int i = 0; i< sample_num; i++) {
            for (int ich = 0; ich<G.size(); ich++) {
                if (G[ich].size() <= 0) {
                    continue;
                }
                int sample_p = rand() / double(RAND_MAX) * G[ich].size();
                int domain_id = G[ich][sample_p];
                for (int j = 0; j< all_domain_set[ich][domain_id].size(); j++) {
                    int sensor_id = all_domain_set[ich][domain_id][j];
                    sensor_charger_values[i][sensor_id] += this->P[ich*m+sensor_id];
                }
            }
        }
    }

    double all_u = 0;
    for (int i = 0; i < sample_num; i++) {
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
        // cout<<"U: "<<this->U(sensor_charger_values[i])<<endl;
        all_u += this->U(sensor_charger_values[i]);
        // cout<<"all_u" <<all_u<<endl;
    }
    all_u = 1.0*all_u/sample_num;
    

    // cout<<"end f"<<endl;
    return all_u;
}

// double Algorithm::F_exactly_distribute(int ich, vector<vector<int>> Q, vector<int> neighbor) {
//     int all_case_num = 1;
//     vector<int> ner_and_me;   //邻居和我
//     ner_and_me.push_back(ich);
//     for (int i = 0; i<neighbor.size(); i++) {
//         ner_and_me.push_back(neighbor[i]);
//     }
//     for (int i = 0; i< ner_and_me.size(); i++) {
//         all_case_num = all_case_num * Q[ner_and_me[i]].size();
//     }
//     vector<vector<double>> sensor_charger_values(all_case_num);
//     for (int i = 0; i < all_case_num; i++) {
//         for (int j = 0; j < m; j++) {
//             sensor_charger_values[i].push_back(0);
//         }
//     }
//     vector<int> cover_set;   //ich所覆盖的sensor
//     for (int i = 0; i<m; i++) {
//         if (cover[ich*m+i]) {
//             cover_set.push_back(i);
//         }
//     }
//     vector<int> all_ner_set;  //记录所有邻居和ich的所能充电的sensor
//     int case_num_till_now = 1;
//     int case_num_last = 1;
//     for (int ii = 0; ii < ner_and_me.size(); ii++) {
//         int i = ner_and_me[ii];   //i为charger编号
//         // cout<<"charger: "<<i<<endl;
//         vector<int> ner_cover_set; //记录邻居覆盖的sensor
//         for (int ise = 0; ise<m; ise++) {
//             if (cover[i*m+ise]) {
//                 ner_cover_set.push_back(ise);
//             }
//         }
//         all_ner_set = vectors_set_union(all_ner_set, ner_cover_set);
//         // if (!(Q_num[i] > 0))
//         //     continue;
//         case_num_last = case_num_till_now;
//         case_num_till_now = case_num_till_now * Q[i].size();
//         int every_case_entity_num_now = all_case_num / case_num_till_now;
//         int every_case_entity_num_last = all_case_num / case_num_last;
//         // cout<<"gg"<<endl;
//         for (int j = 0; j < case_num_last; j++) {
//             // int k = 0, k_=0;
//             for (int k=0; k<Q[i].size(); k++) {
//             // for (int k = 0; k < Q_num[i]; k++) {
//                 int start = k*every_case_entity_num_now + j*every_case_entity_num_last;
//                 int end = (k+1)*every_case_entity_num_now + j*every_case_entity_num_last;
//                 //当第i个charger选择第k个color时，计算它给周围sensor的充电量
//                 int domain_set_id = Q[i][k];
//                 // cout<<"jj"<<endl;
//                 if (domain_set_id == -1) {
//                     continue;  //不需要加
//                 }
//                 // cout<<"ii"<<endl;
//                 // cout<<"kk "<<domain_set_id<<endl;
//                 // cout<<"kk "<<domain_set_id<< " " <<all_domain_set[i][domain_set_id].size()<<endl;
//                 // cout<<start<<" "<<end<<endl;
//                 for (int o = start; o < end; o++) {
//                     for (int pp = 0; pp < all_domain_set[i][domain_set_id].size(); pp++) {
//                         int sensor_id = all_domain_set[i][domain_set_id][pp];
//                         // cout<<"sensor_id:"<<sensor_id<<endl;
//                         sensor_charger_values[o][sensor_id] += P[i*m+sensor_id];
//                     }
//                 }
//                 // cout<<"hh"<<endl;
//             }
//         } 
//     }
//     //已经计算完每个case的sensor充电量，接下来进行电量分享
//     // all_ner_set = vectors_set_union(all_ner_set, cover_set);
//     double all_u = 0;
//     for (int i = 0; i< all_case_num; i++) {
//         // cout<<"分享前"<<endl;
//         // for (int j = 0; j < m; j++) {
//         //     cout<<sensor_charger_values[i][j] << " ";
//         // }
//         // cout<<endl;
//         sensor_charger_values[i] = distributeGreedyEnergySharingStrategy(sensor_charger_values[i],
//                                                                             all_ner_set);
//         // cout<<"分享后"<<endl;
//         // for (int j = 0; j < m; j++) {
//         //     cout<<sensor_charger_values[i][j] << " ";
//         // }
//         // cout<<endl;
//         // for (int j = 0; j < m; j++) {
//         //     cout<<sensor_list[j].te << " ";
//         // }
//         // cout<<endl;
//         // cout<<this->U_(sensor_charger_values[i], cover_set)<<" ";
//         all_u += U_(sensor_charger_values[i], cover_set);
//     }
//     // cout<<endl;
//     // cout<<(1.0*all_u/all_case_num)<<endl;
//     return (1.0*all_u/all_case_num);
// }

double Algorithm::F_exactly_distribute(int ich, vector<vector<int>> Q, vector<int> neighbor) {
    int all_case_num = 1;
    vector<int> Q_num(n);    
    for (int i = 0; i <n; i++) {
        Q_num[i] = 0;
        for (int j = 0; j<nColor; j++) {
            if (Q[i][j] != -1) {
                Q_num[i]++;
            }
        }
    }
    vector<int> ner_and_me;   //邻居和我
    ner_and_me.push_back(ich);
    for (int i = 0; i<neighbor.size(); i++) {
        ner_and_me.push_back(neighbor[i]);
    }
    for (int i = 0; i<ner_and_me.size(); i++) {
        int ner_id = ner_and_me[i];
        if (Q_num[ner_id] > 0) {
            all_case_num = all_case_num*Q_num[ner_id];
        }
    }
    vector<vector<double>> sensor_charger_values(all_case_num);
    for (int i = 0; i < all_case_num; i++) {
        for (int j = 0; j < m; j++) {
            sensor_charger_values[i].push_back(0);
        }
    }

    vector<int> cover_set;   //ich所覆盖的sensor
    for (int i = 0; i<m; i++) {
        if (cover[ich*m+i]) {
            cover_set.push_back(i);
        }
    }

    vector<int> all_ner_set;  //记录所有邻居和ich的所能充电的sensor
    int case_num_till_now = 1;
    int case_num_last = 1;
    for (int ii = 0; ii < ner_and_me.size(); ii++) {
        int i = ner_and_me[ii];   //i为charger编号
        vector<int> ner_cover_set; //记录邻居覆盖的sensor
        for (int ise = 0; ise<m; ise++) {
            if (cover[i*m+ise]) {
                ner_cover_set.push_back(ise);
            }
        }
        all_ner_set = vectors_set_union(all_ner_set, ner_cover_set);
        if (!(Q_num[i] > 0))
            continue;
        case_num_last = case_num_till_now;
        case_num_till_now = case_num_till_now * Q_num[i];
        int every_case_entity_num_now = all_case_num / case_num_till_now;
        int every_case_entity_num_last = all_case_num / case_num_last;
        // cout<<"gg"<<endl;
        for (int j = 0; j < case_num_last; j++) {
            int k = 0, k_=0;
            while(k_<Q_num[i]) {
            // for (int k = 0; k < Q_num[i]; k++) {
                int start = k_*every_case_entity_num_now + j*every_case_entity_num_last;
                int end = (k_+1)*every_case_entity_num_now + j*every_case_entity_num_last;
                //当第i个charger选择第k个color时，计算它给周围sensor的充电量
                int domain_set_id = Q[i][k];
                // cout<<"jj"<<endl;
                if (domain_set_id == -1) {
                    k++;
                    continue;
                }
                // cout<<"ii"<<endl;
                // cout<<"kk "<<domain_set_id<< " " <<all_domain_set[i][domain_set_id].size()<<endl;
                // cout<<start<<" "<<end<<endl;
                for (int o = start; o < end; o++) {
                    for (int pp = 0; pp < all_domain_set[i][domain_set_id].size(); pp++) {
                        int sensor_id = all_domain_set[i][domain_set_id][pp];
                        // cout<<"sensor_id:"<<sensor_id<<endl;
                        sensor_charger_values[o][sensor_id] += P[i*m+sensor_id];
                    }
                }
                // cout<<"hh"<<endl;
                k++;
                k_++;
            }
        } 
    }
    // cout<<"ee"<<endl;
    //已经计算完每个case的sensor充电量，接下来进行电量分享
    // all_ner_set = vectors_set_union(all_ner_set, cover_set);
    double all_u = 0;
    for (int i = 0; i< all_case_num; i++) {
        // cout<<"分享前"<<endl;
        // for (int j = 0; j < m; j++) {
        //     cout<<sensor_charger_values[i][j] << " ";
        // }
        // cout<<endl;
        sensor_charger_values[i] = this->distributeGreedyEnergySharingStrategy(sensor_charger_values[i],
                                                                            all_ner_set);
        // sensor_charger_values[i] = this->greedyEnergySharingStrategy(sensor_charger_values[i]);
        // cout<<"分享后"<<endl;
        // for (int j = 0; j < m; j++) {
        //     cout<<sensor_charger_values[i][j] << " ";
        // }
        // cout<<endl;
        // for (int j = 0; j < m; j++) {
        //     cout<<sensor_list[j].te << " ";
        // }
        // cout<<endl;
        // cout<<"U: "<<this->U(sensor_charger_values[i])<<endl;
        all_u += this->U_(sensor_charger_values[i], cover_set);
    }
    return 1.0*all_u/all_case_num;
}


vector<double> Algorithm::greedyEnergySharingStrategy(vector<double> charger_value) {
    vector<int> charger_set; //记录被充电的sensor
    vector<int> no_charger_set;
    // cout<<"电量分享"<<endl;
    // for (int i =0; i<charger_value.size(); i++) {
    //     cout<< charger_value[i]<< " ";
    // }
    // cout<<endl;
    for (int i = 0; i<charger_value.size(); i++) {
        if (charger_value[i] >0 ) {
            charger_set.push_back(i);
        } else {
            no_charger_set.push_back(i);
        }
    }
    // cout<<"charged set: ";
    // for (int i = 0; i<charger_set.size(); i++) {
    //     cout<< charger_set[i]<< " ";
    // }
    // cout<<endl;
    // cout<<"no charged set: ";
    // for (int i = 0; i<no_charger_set.size(); i++) {
    //     cout<< no_charger_set[i]<< " ";
    // }
    // cout<<endl;
    // cout<<"xx"<<endl;
    for (int ics = 0; ics < charger_set.size(); ics++) {
        vector<sensor_dev2> send_unity;
        int ics_id = charger_set[ics];
        for (int incs = 0; incs < no_charger_set.size(); incs++) {
            int incs_id = no_charger_set[incs];
            sensor_dev2 tmp;
            tmp.id = incs_id;
            // cout<<"N["<<ics_id<<","<<incs_id<<"]: "<<N[ics_id*m + incs_id]<<"te1: "<<sensor_list[ics_id].te<< "te2: "<<sensor_list[incs_id].te<<endl;
            tmp.is_share = (N[ics_id*m + incs_id] * sensor_list[ics_id].te - sensor_list[incs_id].te);
            // cout<<"is_share: "<<tmp.is_share<<endl;
            send_unity.push_back(tmp);
        }
        // cout<<"cc"<<endl;
        sort(send_unity.begin(), send_unity.end(), this->cmp2);   //降序
        // cout<<"dd"<<endl;
        for (int j = 0; j < send_unity.size(); j++) {
            // cout<<"is_share: "<<send_unity[j].is_share<<" ";
            auto tmp = send_unity[j];
            if (charger_value[tmp.id] < sensor_list[tmp.id].te && tmp.is_share > 0 
                && charger_value[ics_id] > 0) {
                    double x = mindo_((1.0*(sensor_list[tmp.id].te - charger_value[tmp.id])/N[ics_id*m+tmp.id]),
                                        charger_value[ics_id]);
                    charger_value[ics_id] -= x;
                    charger_value[tmp.id] += x*N[ics_id*m+tmp.id];
                    // cout<<ics_id<<" 给 "<<tmp.id<<" 分享 "<<x<<" 电量 N "<<N[ics_id*m+tmp.id]<<endl;
                    // this->E_s[ics_id*m + tmp.id] = x;
            }
        }
        // cout<<endl;
    }
    return charger_value;
}

vector<double> Algorithm::distributeGreedyEnergySharingStrategy(vector<double> charger_value,
                                                        vector<int> all_ner_set) {
    //只考虑all_ner_set之间的分享，其余的sensor充电量肯定为0
    // cout<<"all_ner_set: ";
    // for (int i = 0; i<all_ner_set.size(); i++) {
    //     cout << all_ner_set[i] <<" ";
    // }
    // cout<<endl;
    vector<int> charger_set; //记录被充电的sensor
    vector<int> no_charger_set;
    for (int i = 0; i<charger_value.size(); i++) {
        if (charger_value[i] >0 ) {
            charger_set.push_back(i);
        } else {
            if (is_element_in_vector(all_ner_set, i))
                no_charger_set.push_back(i);
        }
    }
    // cout<<"no_charger_set: ";
    // for (int i = 0; i<no_charger_set.size(); i++) {
    //     cout << no_charger_set[i] <<" ";
    // }
    // cout<<endl;
    for (int ics=0; ics<charger_set.size(); ics++) {
        vector<sensor_dev2> send_unity;
        int ics_id = charger_set[ics];
        for (int incs = 0; incs < no_charger_set.size(); incs++) {
            int incs_id = no_charger_set[incs];
            sensor_dev2 tmp;
            tmp.id = incs_id;
            tmp.is_share = (N[ics_id*m + incs_id] * sensor_list[ics_id].te - sensor_list[incs_id].te);
            send_unity.push_back(tmp);
        }
        sort(send_unity.begin(), send_unity.end(), this->cmp2);   //降序
        // cout<<"dd"<<endl;
        for (int j = 0; j < send_unity.size(); j++) {
            auto tmp = send_unity[j];
            // cout<<tmp.is_share<<" ";
            if (charger_value[tmp.id] < sensor_list[tmp.id].te && tmp.is_share > 0 
                && charger_value[ics_id] > 0) {
                    double x = mindo_((1.0*(sensor_list[tmp.id].te - charger_value[tmp.id])/N[ics_id*m+tmp.id]),
                                        charger_value[ics_id]);
                    charger_value[ics_id] -= x;
                    charger_value[tmp.id] += x*N[ics_id*m+tmp.id];
                    // cout<<ics_id<<" 给 "<<tmp.id<<" 分享 "<<x<<" 电量 N "<<N[ics_id*m+tmp.id]<<endl;
                    // this->E_s[ics_id*m + tmp.id] = x;
            }
        }
        // cout<<endl;
    }
    return charger_value;
}

vector<vector<int>> Algorithm::centralizedAlgorithm() {
    if(!is_all_set) {
        all_domain_set = getAllDominantCoverageSet();
        is_all_set = true;
    }
    // cout<<"获得支配集"<<endl;
        
    // cout<<"所有支配集:"<<endl;
    // for (int i = 0; i<all_domain_set.size(); i++) {
    //     cout<<"charger 支配集个数: "<<all_domain_set[i].size()<<endl;
    //     // for (int j = 0; j<all_domain_set[i].size(); j++) {
    //     //     for (int k = 0; k < all_domain_set[i][j].size(); k++) {
    //     //         cout<<all_domain_set[i][j][k]<<" ";
    //     //     }
    //     //     cout<<endl;
    //     // }
    // }
    vector<vector<vector<int>>> result(n);  //每个charger对应一个vector<vector<int>>， 里面存放每个颜色选择的策略对应的覆盖集
    vector<vector<int>> G(n);   //每个charger对应一个vector<int>， 里面存放每个颜色选择的策略
    for (int ico = 0; ico < nColor; ico++) {
        // cout<<"color:"<<ico<<endl;
        for (int ich = 0; ich < n; ich++) {
            if (all_domain_set[ich].size() == 0) 
                continue;
            if (all_domain_set[ich].size() == 1) {  //无需比较，直接放入
                result[ich].push_back(all_domain_set[ich][0]);
                G[ich].push_back(0);
                continue;
            }
            // cout<<"charger:"<<ich<<endl;
            int max_policy = -1;
            double max_u = 0;
            // cout<<all_domain_set[ich].size()<<endl;
            
            for (int ipo = 0; ipo < all_domain_set[ich].size(); ipo++) {
                vector<vector<int>> G_(G);
                G_[ich].push_back(ipo);
                // cout<<"G_:---------"<<endl;
                // for (int ii = 0; ii < G_.size(); ii++) {
                //     for (int jj = 0; jj < G_[ii].size(); jj++) {
                //         cout <<G_[ii][jj] << " ";
                //     }
                //     cout<<endl;
                // }
                // cout<<"--------"<<endl;
                double u_ = this->F_exactly(G_, all_domain_set);
                // cout<<"F 期望："<<u_<<endl;
                if (u_ > max_u) {
                    max_u = u_;
                    max_policy = ipo;
                }
            }
            result[ich].push_back(all_domain_set[ich][max_policy]);
            G[ich].push_back(max_policy);
        }
    }
    // cout<<"G:---------"<<endl;
    // for (int ii = 0; ii < G.size(); ii++) {
    //     for (int jj = 0; jj < G[ii].size(); jj++) {
    //         cout <<G[ii][jj] << " ";
    //     }
    //     cout<<endl;
    // }
    // cout<<"--------"<<endl;
    vector<vector<int>> result1(n);
    //sample
    for (int ich = 0; ich<n; ich++) {
        if (all_domain_set[ich].size() == 0) {
            continue;
        }
            
        int sample_color = rand() / double(RAND_MAX) * nColor;
        // cout<<"sample "<<sample_color<<endl;
        result1[ich] = result[ich][sample_color];
    }
    return result1;
}

double Algorithm::det_f(int ich, vector<vector<int>> Q,  
                        vector<int> neighbors,
                        vector<int> currColor, int* poicy) {
    int color = currColor[ich];
    double max_F = 0;
    int max_poicy = -1;
    if (all_domain_set[ich].size() == 0) {
        (*poicy) = max_poicy;   //返回-1代表没有选择策略
        return 0;
    }
    vector<vector<int>>Q_(Q);
    Q_[ich][color] = -1;
    // for(int i = 0; i< n;i++) {
    //     for (int j = 0; j<nColor; j++) {
    //         cout<<Q_[i][j]<<" ";
    //     }
    //     cout<<endl;
    // }
    double F_ = F_exactly_distribute(ich, Q_, neighbors);
    // cout<<"F_before: "<<F_<<endl;
    for (int ip = 0; ip<all_domain_set[ich].size(); ip++) {
        
        Q_[ich][color] = ip;
        // for(int i = 0; i< n;i++) {
        //     for (int j = 0; j<nColor; j++) {
        //         cout<<Q_[i][j]<<" ";
        //     }
        //     cout<<endl;
        // }
        // cout<<"ccc"<<endl;
        double F = F_exactly_distribute(ich, Q_, neighbors);
        // cout<<"策略："<<ip<<"F: "<<F<<endl;
        // cout<<"F1: "<<F<<endl;
        if (F > max_F) {
            max_F = F;
            max_poicy = ip;
        }
        // cout<<"end loop"<<endl;
    }
    (*poicy) = max_poicy;
    return (max_F - F_);
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
                            break;
                        }
                    }
                    if (is_neighbor) {
                        break;
                    }
                }
            }
            if (is_neighbor) {
                // cout<<ich1<< " "<<ich2<<endl;
                neighbor[ich1].push_back(ich2);
                neighbor[ich2].push_back(ich1);
            }
        }
    }

    // 打印邻居信息
    // cout<<"邻居信息"<<endl;
    // for (int i = 0; i<n; i++) {
    //     cout<<"ich: "<<i<<"的邻居：";
    //     for (int j = 0;j<neighbor[i].size(); j++) {
    //         cout<<neighbor[i][j]<<" ";
    //     }
    //     cout<<endl;
    // }

    //已经得到每个charger的邻居
    if(!is_all_set) {
        all_domain_set = getAllDominantCoverageSet();
        is_all_set = true;
    }
    // cout<<"所有支配集:"<<endl;
    // for (int i = 0; i<all_domain_set.size(); i++) {
    //     cout<<"charger "<<i<<"支配集个数: "<<all_domain_set[i].size()<<endl;
    //     for (int j = 0; j<all_domain_set[i].size(); j++) {
    //         for (int k = 0; k < all_domain_set[i][j].size(); k++) {
    //             cout<<all_domain_set[i][j][k]<<" ";
    //         }
    //         cout<<endl;
    //     }
    // }
    vector<int> poicties(n);   //记录每个charger所选择的策略
    vector<vector<int>> Q(n); //记录每个charger，每个颜色的策略，下标自动对应颜色
    vector<vector<Message>>  messages(n);   //记录每个charger的收信箱
    for (int i = 0; i<n; i++) {
        for (int j =0; j<nColor; j++) {
            Q[i].push_back(-1);   //-1代表为空,没有选择策略
        }
    }

    vector<vector<int>> currColor(n);   //记录每个charger当前所处的颜色阶段
    for (int i=0; i<n; i++) {
        // currColor.push_back(vector<int>());
        for (int j = 0; j<n; j++) {
            currColor[i].push_back(0);
        }
    }
    vector<vector<vector<double>>> currDetf(n);   //记录当前每个charger每个颜色的DETf的值
    for (int ii = 0; ii <n; ii++) {
        for (int i = 0; i<n; i++) {
            currDetf[ii].push_back(vector<double>());
            for (int j =0; j<nColor; j++) {
                currDetf[ii][i].push_back(-1);
            }
        }
    }

    vector<vector<vector<int>>> currPoicy(n);  //记录每个charger每个颜色的e*
    // cout<<"!!!!!!!!!!!!!!!!  "<<n<<endl;
    for (int ii = 0; ii<n; ii++) {
        for (int i = 0; i<n; i++) {
            currPoicy[ii].push_back(vector<int>());
            for (int j =0; j<nColor; j++) {
                currPoicy[ii][i].push_back(-1);
            }
        }
    }
    // cout<<"当前策略:-------"<<endl;
    // for (int ii = 0; ii<currPoicy[0].size(); ii++) {
    //     for (int cc = 0; cc< nColor; cc++) {
    //         cout<<currPoicy[0][ii][cc]<<" ";
    //     }
    //     cout<<endl;
    // }
    // cout<<"------------"<<endl;


    // cout<<"开始多线程模拟"<<endl;
    
    for (int ico = 0; ico<nColor; ico++) {
        //一个时间片循环
        int time_slot = 0;
        // cout<<"颜色初始计算开始"<<endl;
        for (int ich = 0; ich < n; ich++) {
            //计算det_F和poicy
            int max_poicy;
            // cout<<"bbb"<<endl;
            double det_F = det_f(ich, currPoicy[ich], neighbor[ich], currColor[ich], &max_poicy);
            // cout<<"aaa"<<endl;
            // cout<<"det_f: "<<det_F<< " max_poicy: "<<max_poicy<<endl;
            currDetf[ich][ich][ico] = det_F;
            currPoicy[ich][ich][ico] = max_poicy;   //-1表示没有选择策略，因为支配集为空
            // cout<<"charger: "<<ich<<"  det_f: "<<det_F<<"  max_poicy: "<<max_poicy<<endl;
            Message m;
            m.ich = ich;
            m.is_update = false;
            m.time_slot = time_slot;
            m.det_F = det_F;
            m.poicy = max_poicy;
            m.color = ico;
            for (int iner=0; iner<neighbor[ich].size(); iner++) {
                messages[neighbor[ich][iner]].push_back(m);
            }
        }
        // cout<<"颜色初始计算结束"<<endl;
        // for (int  i = 0; i<n; i++) {
        //     cout<< currDetf[i][i][ico]<< " ";
        // }
        // cout<<endl<<endl;;

        //另一个时间片循环
        while (true) {
            time_slot++;
            bool is_break = true;
            for (int ich =0; ich<n; ich++) {
                // 随机延迟
                double delay = rand() / double(RAND_MAX) * 1;
                // cout<<delay<<endl;
                if (delay < 0.3) {
                    is_break = false;
                    // cout<<"delay"<<endl;
                    continue;
                }
                    

                // cout<<endl<<"charger: "<<ich<<endl;
                // cout<<"当前策略:-------"<<endl;
                // for (int ii = 0; ii<currPoicy[ich].size(); ii++) {
                //     for (int cc = 0; cc< nColor; cc++) {
                //         cout<<currPoicy[ich][ii][cc]<<" ";
                //     }
                //     cout<<endl;
                // }
                // cout<<"------------"<<endl;
                if (currColor[ich][ich] > ico) {
                    //进入下一个color的charger要等待
                    // cout<<"已经进入下一个color"<<endl;
                    continue;
                }
                    
                is_break = false;
                if (currDetf[ich][ich][ico] <= 0) {
                    currColor[ich][ich]++;  //ich进入下个颜色
                    // cout<<"det_f 为 0, 进入下一个color"<<endl;
                    continue;
                }

                //检查是否收集齐
                bool is_collect = true;
                for (int iner = 0; iner<neighbor[ich].size(); iner++) {
                    int ner_id = neighbor[ich][iner];
                    if (currDetf[ich][ner_id][ico] == -1) {
                        is_collect = false;
                        break;
                    }
                }
                if (is_collect) {
                    //检查自己是不是最大的
                    // cout<<"检查自己是不是最大的"<<endl;
                    // for (int iner = 0; iner < neighbor[ich].size(); iner++) {
                    //     int ner_id = neighbor[ich][iner];
                    //     // cout<<currColor[ich][ner_id]<<" ";
                    // }
                    // cout<<endl;
                    int max_ich = -1;
                    double max_det_F = -1;
                    // cout<<ich<<" : ";
                    // cout<<currDetf[ich][ich][ico]<< " | ";
                    // if (currDetf[ich][ich][ico] > max_det_F) {
                    //     max_det_F = currDetf[ich][ich][ico];
                    //     max_ich = ich;
                    // }
                    for (int iner = 0; iner < neighbor[ich].size(); iner++) {
                        int ner_id = neighbor[ich][iner];
                        // if (currColor[ich][ner_id] == ico) {
                        //     cout<<ner_id<<" "<<currDetf[ich][ner_id][ico]<<"   ";
                        // }
                        
                        if (currColor[ich][ner_id] == ico && currDetf[ich][ner_id][ico] > max_det_F) {
                            max_ich = ner_id;
                            max_det_F = currDetf[ich][ner_id][ico];
                        }
                    }
                    // cout<<endl;
                    // if (currDetf[ich][ich][ico] > max_det_F || currDetf[ich][ich][ico] == max_det_F && max_ich>ich) {
                    if (currDetf[ich][ich][ico] >= max_det_F) {
                        max_ich = ich;
                        max_det_F = currDetf[ich][ich][ico];
                    }
                    if (max_ich == ich) {
                        //发送UPD消息
                        Message m;
                        m.ich = ich;
                        m.color = ico;
                        m.time_slot = time_slot;
                        m.det_F = currDetf[ich][ich][ico];
                        m.is_update = true;
                        m.poicy = currPoicy[ich][ich][ico];
                        for (int iner=0; iner<neighbor[ich].size(); iner++) {
                            messages[neighbor[ich][iner]].push_back(m);
                        }
                        Q[ich][ico] = currPoicy[ich][ich][ico];
                        currColor[ich][ich]++;   //ich进入下一个color
                        // cout<<"进入下一个color"<<endl;
                        continue;
                    }
                }





                // cout<<"charger: "<<ich<<endl;
                //先处理当前time_slot之前的message
                // cout<<"处理消息： ich   time_slot   color  is_UPD"<<endl;
                for (int ime = 0; ime<messages[ich].size(); ime++) {
                    auto m = messages[ich][ime];
                    // cout<<"m: "<<m.ich << " "<<m.time_slot<< " "<<m.color<<" "<<m.is_update<<endl;
                    if (m.color != ico) {
                        // cout<<"删除上一个color的消息"<<endl;
                        messages[ich].erase(messages[ich].begin()+ ime);
                        ime--;
                        continue;
                    }
                    if (currColor[ich][m.ich] < ico)
                        currColor[ich][m.ich] = ico;
                    if (m.time_slot < time_slot && m.color == ico) {
                        if (m.is_update) {
                            // cout<< "UPD ";
                            //重新计算
                            // cout<<"接受到UPD消息，重新计算"<<endl;
                            currDetf[ich][m.ich][ico] = m.det_F;
                            currPoicy[ich][m.ich][ico] = m.poicy;
                            currColor[ich][m.ich]++;
                            int max_poicy;
                            double det_F = det_f(ich, currPoicy[ich], neighbor[ich], 
                                            currColor[ich], &max_poicy);
                            currDetf[ich][ich][ico] = det_F;
                            currPoicy[ich][ich][ico] = max_poicy;
                            // cout<<"当前策略:-------"<<endl;
                            // for (int ii = 0; ii<currPoicy[ich].size(); ii++) {
                            //     for (int cc = 0; cc< nColor; cc++) {
                            //         cout<<currPoicy[ich][ii][cc]<<" ";
                            //     }
                            //     cout<<endl;
                            // }
                            // cout<<"------------"<<endl;
                            Message m1;
                            m1.ich = ich;
                            m1.is_update = false;
                            m1.det_F = det_F;
                            m1.color = ico;
                            m1.poicy = max_poicy;
                            m1.time_slot = time_slot;
                            for (int iner=0; iner<neighbor[ich].size(); iner++) {
                                messages[neighbor[ich][iner]].push_back(m1);
                            }
                        } else {
                            currDetf[ich][m.ich][ico] = m.det_F;
                            currPoicy[ich][m.ich][ico] = m.poicy;
                            if (m.det_F <= 0) {
                                currColor[ich][m.ich]++;
                            }
                        }
                        // cout<<endl;
                        messages[ich].erase(messages[ich].begin()+ ime);
                        ime--;
                    }
                }
                
                
                
                
                // int aaa;
                // cin>>aaa;
                // cout<<endl;
            }
            if (is_break) {
                // cout<<"break!!!"<<endl;
                break;
            }
        }
    }
    // cout<<"Q已经得到"<<endl;
    // // Q已经得到,进行采样
    // for (int i = 0; i<Q.size(); i++) {
    //     for (int j =0; j<nColor; j++) {
    //         cout<<Q[i][j]<<" ";
    //     }
    //     cout<<endl;
    // }
    vector<vector<int>> result(n);
    for (int ich = 0; ich<n; ich++) {
        // cout<<"ich: "<<ich<<endl;
        if (all_domain_set[ich].size() == 0) {
            continue;
        }
        // for (int i = 0; i<Q[ich].size(); i++) {
        //     cout<<Q[ich][i]<<" ";
        // }
        // cout<<endl;

        // cout<<"aaa"<<endl;
        int num = 0;
        for (int i =0; i<Q[ich].size(); i++) {
            if (Q[ich][i] != -1){
                num++;
            }
        }
        // cout<<"bbb"<<endl;
        if (num == 0) {
            if (all_domain_set[ich].size() > 0) {
                // for (int iset = 0; iset < all_domain_set[ich].size(); iset++) {
                //     cout<<all_domain_set[ich][iset].size()<<" ";
                // }
                // cout<<endl;
                cout<<"error!!!!"<<endl;
                vector<vector<int>> set = all_domain_set[ich];
                int max_size = -1;
                int max_poicy = -1;
                for (int iset = 0; iset < set.size(); iset++) {
                    int size = set[iset].size();
                    if (size > max_size) {
                        max_size = size;
                        max_poicy = iset;
                    }
                }
                cout<<max_poicy<<endl;
                if (max_poicy != -1)
                    result[ich] = all_domain_set[ich][max_poicy];
                // int sample = rand() / double(RAND_MAX) * all_domain_set[ich].size();
                // result[ich] = all_domain_set[ich][sample];
            }
            continue;
        }
        int sample_color = rand() / double(RAND_MAX) * num;
        // cout<<sample_color<<" ";
        int k = 0, k_ = 0;
        while(k<sample_color+1) {
            if (Q[ich][k_] != -1) {
                k++;
            }
            k_++;
        }
        // cout<<k_<<endl;
        // if (Q[ich][sample_color] != -1)
        result[ich] = all_domain_set[ich][Q[ich][k_-1]];
    }
    // cout<<"分布式算法结束"<<endl;
    return result;
}

Algorithm::~Algorithm() {
    // cout<<"jbjabjbg"<<endl;
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

void Algorithm::writeChargerSensorToFile(string filename) {
    ofstream f(filename);
    if (!f)
        return;
    f<<n<<" "<<m<<endl;
    for (int i =0; i<n;i++) {
        f<<charger_list[i].cx<<" "<<charger_list[i].cy<<endl;
    }
    for (int i =0; i<m;i++) {
        f<<sensor_list[i].tx<<" "<<sensor_list[i].ty<<" "<<sensor_list[i].tdir<<" "<<sensor_list[i].te <<endl;
    }
    f<<(A_s*180/M_PI)<<" "<<(A_o*180/M_PI)<<endl;
    for (int i = 0; i < m; i++) {
        for (int j =0; j<m; j++) {
            f<<N[i*m+j]<<" ";
        }
        f<<endl;
    }
    for (int i = 0; i < n; i++) {
        for (int j =0; j<m; j++) {
            f<<cover[i*m+j]<<" ";
        }
        f<<endl;
    }
    f.close();
}

double Algorithm::getUOfResult(vector<vector<int>> result) {
    vector<double> sensor_charger_value(m);
    for (int i = 0; i<m; i++) {
        sensor_charger_value[i] = 0;
    }
    for (int ich=0; ich<result.size(); ich++) {
        for (int ise = 0; ise<result[ich].size(); ise++) {
            int se_id = result[ich][ise];
            sensor_charger_value[se_id] += P[ich*m+se_id];
        }
    }
    sensor_charger_value = this->greedyEnergySharingStrategy(sensor_charger_value);
    return (1.0*this->U(sensor_charger_value) / m);
}

double Algorithm::getUOfResult1(vector<vector<int>> result) {
    vector<double> sensor_charger_value(m);
    for (int i = 0; i<m; i++) {
        sensor_charger_value[i] = 0;
    }
    for (int ich=0; ich<result.size(); ich++) {
        for (int ise = 0; ise<result[ich].size(); ise++) {
            int se_id = result[ich][ise];
            sensor_charger_value[se_id] += P[ich*m+se_id];
        }
    }
    return (1.0*this->U(sensor_charger_value) / m);
}


vector<vector<int>> Algorithm::greedyUtilityAlgorithm() {
    // cout<<"贪心Utility开始"<<endl;
    if(!is_all_set) {
        all_domain_set = getAllDominantCoverageSet();
        is_all_set = true;
    }
    vector<vector<int>> result(n);
    for (int ich = 0; ich<n; ich++) {
        auto set = all_domain_set[ich];
        int max_poicy = -1;
        int max_u = -1;
        for (int iset = 0; iset < set.size(); iset++) {
            vector<double> sensor_charger_value(m);
            for (int i = 0; i< m; i++) {
                sensor_charger_value[i] = 0;
            }
            for (int i = 0; i< set[iset].size(); i++) {
                int se_id = set[iset][i];
                sensor_charger_value[se_id] += P[ich*m + se_id];
            }
            double u = this->U(sensor_charger_value);
            if (u > max_u) {
                max_u = u;
                max_poicy = iset;
            }
        }
        // cout<<max_poicy<<endl;
        if (max_poicy != -1)
            result[ich] = set[max_poicy];
    }
    // cout<<"贪心Utility结束"<<endl;
    return result;
}

vector<vector<int>>  Algorithm::greedyCoverAlgorithm() {
    // cout<<"贪心cover开始"<<endl;
    if(!is_all_set) {
        all_domain_set = getAllDominantCoverageSet();
        is_all_set = true;
    }
    vector<vector<int>> result(n);
    for (int ich = 0; ich<n; ich++) {
        vector<vector<int>> set = all_domain_set[ich];
        // cout<<"set: "<<set.size()<<endl;
        int max_poicy = -1;
        int max_num = -1;
        for (int iset = 0; iset < set.size(); iset++) {
            // cout<<"set_:"<<set[iset].size()<<endl;
            int size = set[iset].size();
            if (size > max_num) {
                // cout<<"aaa"<<endl;
                max_num = size;
                max_poicy = iset;
            }
        }
        // cout<<max_poicy<<endl;
        if (max_poicy != -1)
            result[ich] = set[max_poicy];
    }
    // cout<<"贪心cover结束"<<endl;
    return result;
}

void Algorithm::reBuild() {
    for (int i = 0; i< m*m; i++) {
        if ((rand() / double(RAND_MAX)) < 0.5) {
            this->N[i] = rand() / double(RAND_MAX);
            // cout<<1<<endl;
        } else {
            // cout<<0<<endl;
            this->N[i] = 0;
        }
    }
    vector<Chargerinfo> tmp1;
    charger_list.swap(tmp1);
    vector<Sensorinfo> tmp2;
    sensor_list.swap(tmp2);
    this->generate_random();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            dists[i*m + j] = getdist(i, j);
        }
    }
    
    getCover();
    // cout<<"gggggggg"<<endl; 
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            P[i*m + j] = getP(i, j);
        }
    }
}

// vector<vector<int>> Algorithm::distributedAlgorithm() {
//     vector<vector<int>> neighbor(n);
//     for (int ich1 = 0; ich1 < n - 1; ich1++) {
//         for (int ich2 = ich1+1; ich2 < n; ich2++) {
//             bool is_neighbor = false;
//             for (int ise = 0; ise < m; ise++) {
//                 if (!cover[ich1*m+ise])
//                     continue;
//                 if (cover[ich2*m+ise]) {
//                     is_neighbor = true;
//                     break;
//                 } else {
//                     for (int ise2 = 0; ise2 < m; ise2++) {
//                         if (cover[ich2*m+ise2] && N[ise*m+ise2] > 0 && N[ise2*m+ise] > 0) {
//                             is_neighbor = true;
//                             break;
//                         }
//                     }
//                     if (is_neighbor) {
//                         break;
//                     }
//                 }
//             }
//             if (is_neighbor) {
//                 // cout<<ich1<< " "<<ich2<<endl;
//                 neighbor[ich1].push_back(ich2);
//                 neighbor[ich2].push_back(ich1);
//             }
//         }
//     }

//     //打印邻居信息
//     cout<<"邻居信息"<<endl;
//     for (int i = 0; i<n; i++) {
//         cout<<"ich: "<<i<<"的邻居：";
//         for (int j = 0;j<neighbor[i].size(); j++) {
//             cout<<neighbor[i][j]<<" ";
//         }
//         cout<<endl;
//     }

//     //已经得到每个charger的邻居
//     if(!is_all_set) {
//         all_domain_set = getAllDominantCoverageSet();
//         is_all_set = true;
//     }
//     cout<<"所有支配集:"<<endl;
//     for (int i = 0; i<all_domain_set.size(); i++) {
//         cout<<"charger "<<i<<"支配集个数: "<<all_domain_set[i].size()<<endl;
//         for (int j = 0; j<all_domain_set[i].size(); j++) {
//             for (int k = 0; k < all_domain_set[i][j].size(); k++) {
//                 cout<<all_domain_set[i][j][k]<<" ";
//             }
//             cout<<endl;
//         }
//     }
//     vector<int> poicties(n);   //记录每个charger所选择的策略
//     vector<vector<int>> Q(n); //记录每个charger，每个颜色的策略，下标自动对应颜色
//     vector<vector<Message>>  messages(n);   //记录每个charger的收信箱
//     for (int i = 0; i<n; i++) {
//         for (int j =0; j<nColor; j++) {
//             Q[i].push_back(-1);   //-1代表为空,没有选择策略
//         }
//     }

//     vector<vector<int>> currColor(n);   //记录每个charger当前所处的颜色阶段
//     for (int i=0; i<n; i++) {
//         // currColor.push_back(vector<int>());
//         for (int j = 0; j<n; j++) {
//             currColor[i].push_back(0);
//         }
//     }
//     vector<vector<vector<double>>> currDetf(n);   //记录当前每个charger每个颜色的DETf的值
//     for (int ii = 0; ii <n; ii++) {
//         for (int i = 0; i<n; i++) {
//             currDetf[ii].push_back(vector<double>());
//             for (int j =0; j<nColor; j++) {
//                 currDetf[ii][i].push_back(-1);
//             }
//         }
//     }

//     vector<vector<vector<int>>> currPoicy(n);  //记录每个charger每个颜色的e*
//     // cout<<"!!!!!!!!!!!!!!!!  "<<n<<endl;
//     for (int ii = 0; ii<n; ii++) {
//         for (int i = 0; i<n; i++) {
//             currPoicy[ii].push_back(vector<int>());
//             for (int j =0; j<nColor; j++) {
//                 currPoicy[ii][i].push_back(-1);
//             }
//         }
//     }
//     // cout<<"当前策略:-------"<<endl;
//     // for (int ii = 0; ii<currPoicy[0].size(); ii++) {
//     //     for (int cc = 0; cc< nColor; cc++) {
//     //         cout<<currPoicy[0][ii][cc]<<" ";
//     //     }
//     //     cout<<endl;
//     // }
//     // cout<<"------------"<<endl;


//     // cout<<"开始多线程模拟"<<endl;
//     bool* com = new bool[n];
//     for (int ich = 0; ich < n; ich++) {
//         com[ich] = false;
//         thread t(Algorithm::distributeThread, &messages, &currDetf[ich], &currPoicy[ich], &currColor[ich],
//                 &Q[ich], neighbor[ich], ich, com, this);
//         t.detach(); 
//     }
//     while(true) {
//         bool is_break = true;
//         for (int i = 0; i< n; i++) {
//             if (!com[i]) {
//                 is_break = false;
//                 break;
//             }
//         }
//         if (is_break)
//             break;
//     }

//     cout<<"Q已经得到"<<endl;
//     //Q已经得到,进行采样
//     // for (int i = 0; i<Q.size(); i++) {
//     //     for (int j =0; j<nColor; j++) {
//     //         cout<<Q[i][j]<<" ";
//     //     }
//     //     cout<<endl;
//     // }
//     vector<vector<int>> result(n);
//     for (int ich = 0; ich<n; ich++) {
//         // cout<<"ich: "<<ich<<endl;
//         if (all_domain_set[ich].size() == 0) {
//             continue;
//         }
//         // for (int i = 0; i<Q[ich].size(); i++) {
//         //     cout<<Q[ich][i]<<" ";
//         // }
//         // cout<<endl;
//         int num = 0;
//         for (int i =0; i<Q[ich].size(); i++) {
//             if (Q[ich][i] != -1){
//                 num++;
//             }
//         }
//         if (num == 0) {
//             // if (all_domain_set[ich].size() > 0) {
//             //     cout<<"error!!!!!!!!"<<endl;
//             // }
//             continue;
//         }
//         int sample_color = rand() / double(RAND_MAX) * nColor;
//         // cout<<sample_color<<endl;
//         // int k = 0, k_ = 0;
//         // while(k<sample_color+1) {
//         //     if (Q[ich][k_] != -1) {
//         //         k++;
//         //     }
//         //     k_++;
//         // }
//         if (Q[ich][sample_color] != -1)
//             result[ich] = all_domain_set[ich][Q[ich][sample_color]];
//     }
//     // cout<<"分布式算法结束"<<endl;
//     return result;
// }



// void Algorithm::distributeThread(vector<vector<Message>>* messages, vector<vector<double>>* currDetf, 
//                                 vector<vector<int>>* currPoicy, vector<int>* currColor, vector<int>* Q,
//                                  vector<int> neighbor, int ich, bool* com, Algorithm* alg) {
//     auto all_domain_set = alg->all_domain_set;
//     int nColor = alg->nColor;
//     cout<<ich<<"开始"<<endl;
//     for (int c = 0; c < nColor; c++) {
//         int max_poicy;
//         // cout<<"bbb"<<endl;
//         double det_F = det_f(ich, *currPoicy, neighbor, all_domain_set, *currColor, &max_poicy, alg);
//         // sem_wait(&(alg->print));
//         cout<<ich<<" "<<det_F<<" "<<max_poicy<<endl;
//         // sem_post(&(alg->print));
//         //发送消息
//         Message m;
//         m.ich = ich;
//         m.is_update = false;
//         m.det_F = det_F;
//         m.poicy = max_poicy;
//         m.color = c;
//         cout<<ich<<"开始发消息"<<endl;
//         // sem_wait(&(alg->g_semaphore));
//         cout<<"size:"<<neighbor.size()<<endl;
//         for (int iner = 0; iner<neighbor.size(); iner++) {
//             int ner_id = neighbor[iner];
//             cout<<ich<<"发送消息!"<<endl;
//             (*messages)[ner_id].push_back(m);
//             cout<<ich<<"发送消息"<<endl;
//         }
//         // sem_post(&(alg->g_semaphore));
//         bool is_collect = false;
//         while (det_F > 0) {
//             is_collect = true;
//             for (int iner=0; iner<neighbor.size(); iner++) {
//                 int ner_id = neighbor[iner];
//                 if ((*currDetf)[ner_id][c] == -1) {
//                     is_collect = false;
//                     break;
//                 }
//             }
//             cout<<ich<<" "<<is_collect<<endl;
//             if (is_collect) {    //判断自己是不是最大的
//                 int max_ich = -1;
//                 double max_det_F = -1;
//                 // cout<<ich<<" : ";
//                 // cout<<currDetf[ich][ich][ico]<< " | ";
//                 // if (currDetf[ich][ich][ico] > max_det_F) {
//                 //     max_det_F = currDetf[ich][ich][ico];
//                 //     max_ich = ich;
//                 // }
//                 for (int iner = 0; iner < neighbor.size(); iner++) {
//                     int ner_id = neighbor[iner];
//                     // if (currColor[ich][ner_id] == ico) {
//                     //     cout<<ner_id<<" "<<currDetf[ich][ner_id][ico]<<"   ";
//                     // }
                    
//                     if ((*currColor)[ner_id] == c && (*currDetf)[ner_id][c] > max_det_F) {
//                         max_ich = ner_id;
//                         max_det_F = (*currDetf)[ner_id][c];
//                     }
//                 }
//                 // cout<<endl;
//                 if (det_F > max_det_F || det_F== max_det_F && max_ich>ich) {
//                     max_ich = ich;
//                     max_det_F = det_F;
//                 }
//                 if (max_ich == ich) {
//                     //发送UPD消息
//                     Message m;
//                     m.ich = ich;
//                     m.color = c;
//                     // m.time_slot = time_slot;
//                     m.det_F = det_F;
//                     m.is_update = true;
//                     m.poicy = max_poicy;
//                     // sem_wait(&(alg->g_semaphore));
//                     for (int iner=0; iner<neighbor.size(); iner++) {
//                         (*messages)[neighbor[iner]].push_back(m);
//                     }
//                     // sem_post(&(alg->g_semaphore));
//                     (*Q)[c] = max_poicy;
//                     (*currPoicy)[ich][c] = max_poicy;
//                     (*currColor)[ich]++;   //ich进入下一个color
//                     cout<<ich<<"进入下一个color"<<endl;
//                     break;
//                 }
//             }
//             //处理消息
//             for (int ime = 0; ime<(*messages)[ich].size(); ime++) {
//                 cout<<ich<<" "<<"处理消息"<<endl;
//                 auto m = (*messages)[ich][ime];
//                 cout<<"m: "<<m.ich << " "<<m.det_F<<" "<<m.color<<" "<<m.is_update<<endl;
//                 if (m.color < c) {
//                     cout<<"删除上一个color的消息"<<endl;
//                     messages[ich].erase(messages[ich].begin()+ ime);
//                     ime--;
//                     continue;
//                 } else if(m.color > c) {
//                     continue;
//                 }
//                 if ((*currColor)[m.ich] < c)
//                     currColor[ich][m.ich] = c;
//                 if (m.color == c) {
//                     if (m.is_update) {
//                         cout<<ich<<" UPD ";
//                         //重新计算
//                         // cout<<"接受到UPD消息，重新计算"<<endl;
//                         (*currDetf)[m.ich][c] = m.det_F;
//                         (*currPoicy)[m.ich][c] = m.poicy;
//                         (*currColor)[m.ich]++;
//                         // int max_poicy;
//                         det_F = det_f(ich, (*currPoicy), neighbor, 
//                                         all_domain_set, (*currColor), &max_poicy, alg);
//                         // (*currDetf)[ich][c] = det_F;
//                         // (*currPoicy)[ich][c] = max_poicy;
//                         // cout<<"当前策略:-------"<<endl;
//                         // for (int ii = 0; ii<currPoicy[ich].size(); ii++) {
//                         //     for (int cc = 0; cc< nColor; cc++) {
//                         //         cout<<currPoicy[ich][ii][cc]<<" ";
//                         //     }
//                         //     cout<<endl;
//                         // }
//                         // cout<<"------------"<<endl;
//                         Message m1;
//                         m1.ich = ich;
//                         m1.is_update = false;
//                         m1.det_F = det_F;
//                         m1.color = c;
//                         m1.poicy = max_poicy;
//                         // m1.time_slot = time_slot;
//                         // sem_wait(&(alg->g_semaphore));
//                         for (int iner=0; iner<neighbor.size(); iner++) {
//                             (*messages)[neighbor[iner]].push_back(m1);
//                         }
//                         // sem_post(&(alg->g_semaphore));
//                     } else {
//                         cout<<ich<<"  NULL"<<endl;
//                         (*currDetf)[m.ich][c] = m.det_F;
//                         (*currPoicy)[m.ich][c] = m.poicy;
//                         if (m.det_F <= 0) {
//                             currColor[ich][m.ich]++;
//                             (*currPoicy)[m.ich][c] = -1;
//                         }
//                     }
//                     // cout<<endl;
//                     messages[ich].erase(messages[ich].begin()+ ime);
//                     ime--;
//                 }
//             }
//         }
//         if (det_F <=0) {
//             (*currPoicy)[ich][c] = -1;
//         }
//     }
//     com[ich] = true;
// }