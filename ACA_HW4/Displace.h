#ifndef __DISPLACE_H__
#define __DISPLACE_H__


struct Displace{
    V3 p;
    vector<V3> q;
    Displace(){};
    Displace(V3 _p, vector<V3> _q):p(_p),q(_q){};
    Displace operator*(const float& alpha){
        V3 _p = p * alpha;
        vector<V3> _q;
        for (auto& x : q){
            _q.push_back(x*alpha);
        }
        return Displace(_p,_q);
    }
};

#endif