#pragma once
#include <cmath>
#include <iostream>
#define FOR(i,n,m) for (int i=(n);i<=(m);i++)
static float eps = 0.0001;

struct position{
    float p[3];
    position(){};
    position(float _p[3]){
        p[0]=_p[0], p[1]=_p[1], p[2]=_p[2];
    }
    position(float p0, float p1, float p2){
        p[0]=p0, p[1]=p1, p[2]=p2;
    }
    position operator*(position const& rhs){
        return position(p[1]*rhs.p[2] - p[2]*rhs.p[1],
                        p[2]*rhs.p[0] - p[0]*rhs.p[2],
                        p[0]*rhs.p[1] - p[1]*rhs.p[0]);
    }
    position operator+(position const& rhs){
        return position(p[0]+rhs.p[0], p[1]+rhs.p[1], p[2]+rhs.p[2]);
    }
    position operator-(position const& rhs){
        return position(p[0]-rhs.p[0], p[1]-rhs.p[1], p[2]-rhs.p[2]);
    }
    float operator%(position const& rhs){
        return p[0]*rhs.p[0] + p[1]*rhs.p[1] + p[2]*rhs.p[2];
    }
    position operator/(float const& x){
        return position(p[0]/x, p[1]/x, p[2]/x);
    }
    position operator*(float const& x){
        return position(p[0]*x, p[1]*x, p[2]*x);
    }
};

float norm(position &a){
    return sqrt(a.p[0]*a.p[0] + a.p[1]*a.p[1] + a.p[2]*a.p[2]);
}

float angle(V3 &pv, V3 &v){ // return value is radian
    V3 cross = pv.cross(v);
    return atan2(cross.norm()/pv.norm()/v.norm(), (pv.dot(v))/pv.norm()/v.norm());
}

struct matrix{
    float a[4][4];
    float v[16];
    matrix(){
        FOR (i,0,3) FOR (j,0,3) a[i][j]=0;
        FOR (i,0,3) a[i][i]=1;
    };
    matrix(float _a[4][4]){
        FOR (i,0,3) FOR (j,0,3) a[i][j]=_a[i][j];
    }
    matrix operator*(matrix const& rhs){
        matrix res=matrix();
        FOR (i,0,3) FOR (j,0,3){
            res.a[i][j]=0;
            FOR (k,0,3) res.a[i][j]+=a[i][k]*rhs.a[k][j];
        }
        return res;
    }
    void vec(){
        int t=0;
        FOR (i,0,3) FOR (j,0,3) v[t++]=a[i][j];
    }
    void print(){
        FOR (i,0,15) std::cout << v[i];
        std::cout << "\n";
    }
};

// reference at Jehee Lee math library
struct quater{
    float p[4];
    quater(){
        p[0]=1; FOR (i,1,3) p[i]=0;
    };
    quater(float p0, float p1, float p2, float p3){
        p[0]=p0, p[1]=p1, p[2]=p2, p[3]=p3;
    }
    quater(float _p[4]){
        FOR (i,0,3) p[i]=_p[i];
    }
    quater(float p0, position v){
        p[0]=p0, p[1]=v.p[0], p[2]=v.p[1], p[3]=v.p[2];
    }
    quater(float p0, V3 v){
        //v = v / v.norm();
        p[0]=p0, p[1]=v(0), p[2]=v(1), p[3]=v(2);
    }
    void print(){
        printf("%.5lf %.5lf %.5lf %.5lf\n",p[0],p[1],p[2],p[3]);
    }
    float dot(quater q2){
        float t=0.0;
        FOR (i,0,3) t+=p[i]*q2.p[i];
        return t;
    }
    quater inverse(){
        return quater(p[0],-p[1],-p[2],-p[3]);
    }
    quater operator*(float x){
        return quater(x*p[0], x*p[1], x*p[2], x*p[3]);
    }
    quater operator*(quater const& rhs){
        quater c;
        /*c.p[0] = p[0]*rhs.p[0] - p[1]*rhs.p[1] - p[2]*rhs.p[2] - p[3]*rhs.p[3];
        c.p[1] = p[0]*rhs.p[1] + p[1]*rhs.p[0] + p[2]*rhs.p[3] - p[3]*rhs.p[2];
        c.p[2] = p[0]*rhs.p[2] + p[2]*rhs.p[0] + p[3]*rhs.p[1] - p[1]*rhs.p[3];
        c.p[3] = p[0]*rhs.p[3] + p[3]*rhs.p[0] + p[1]*rhs.p[2] - p[2]*rhs.p[1];*/
        position v1=position(p[1],p[2],p[3]);
        position v2=position(rhs.p[1],rhs.p[2],rhs.p[3]);
        float w1=p[0], w2=rhs.p[0];
        return quater(w1*w2-v1%v2, v2*w1 + v1*w2 + v1*v2);
    }
    matrix toMatrix(){
        matrix m;
        m.a[0][0]=p[0]*p[0] + p[1]*p[1] - p[2]*p[2] - p[3]*p[3];
        m.a[1][0]=2*p[1]*p[2] - 2*p[0]*p[3];
        m.a[2][0]=2*p[1]*p[3] + 2*p[0]*p[2];
        m.a[3][0]=0;
    
        m.a[0][1]=2*p[1]*p[2] + 2*p[0]*p[3];
        m.a[1][1]=p[0]*p[0] - p[1]*p[1] + p[2]*p[2] - p[3]*p[3];
        m.a[2][1]=2*p[2]*p[3] - 2*p[0]*p[1];
        m.a[3][1]=0;

        m.a[0][2]=2*p[1]*p[3] - 2*p[0]*p[2];
        m.a[1][2]=2*p[2]*p[3] + 2*p[0]*p[1];
        m.a[2][2]=p[0]*p[0] - p[1]*p[1] - p[2]*p[2] + p[3]*p[3];
        m.a[3][2]=0;

        m.a[0][3]=0;
        m.a[1][3]=0;
        m.a[2][3]=0;
        m.a[3][3]=1;
        return m;
    }
    float getTheta(){
        float t = atan2(sqrt(1-p[0]*p[0]),p[0]) * 2;
        return t;
    }
    V3 getVec(){
        float t = sin(getTheta()/2);
        V3 p2 = V3(p[1]/t, p[2]/t, p[3]/t);
        return p2;
    }
};

quater G(float t, quater q, V3 u){
    return quater(cos(t),u*sin(t)) * q;
}
quater geodesic(quater qs, quater  q0, V3 u = V3(0,1,0)){
    cout << "@ Geodesic\n";
    cout << "qs = "; qs.print();
    cout << "q0 = "; q0.print();
    float ws = qs.p[0];
    V3 vs = V3(qs.p[1],qs.p[2],qs.p[3]);
    float w0 = q0.p[0];
    V3 v0 = V3(q0.p[1],q0.p[2],q0.p[3]);
    float a = ws*w0 + vs.dot(v0);
    float b = ws*(u.dot(v0)) + w0*(vs.dot(u))+vs.dot(u.cross(v0)); cout << "a/b = " << a << "/" << b << endl;
    float alpha = atan2(a,b); cout << "alpha = " << alpha << endl;

    quater plus = G(-alpha+PI/2.0, q0, u); cout << "plus = "; plus.print();
    quater minus = G(-alpha-PI/2.0, q0, u); cout << "minus = "; minus.print();
    if (qs.dot(plus) > qs.dot(minus)) return plus;
    return minus;
}

quater make_quater(float angle, position axis){
	return quater(cos(angle/2), axis * sin(angle/2));
}
// position calc_rotate(quater Q, position _P){
//     quater P = quater(0, _P);
//     P=(Q*P)*Q.inverse();
//     return position(P.p[1], P.p[2], P.p[3]);
// }
V3 calc_rotate(quater Q, V3 _P){
    quater P = quater(0, _P);
    P=(Q*P)*Q.inverse();
    return V3(P.p[1],P.p[2],P.p[3]);
}
V3 LOG(quater q){
    if (q.getTheta()<1e-6) return V3(0,0,0);
    return V3(q.getVec() * q.getTheta());
}
quater EXP(V3 p){
    float angle=p.norm();
    if (p.norm()<1e-6) return quater();
    return quater(cos(angle/2),sin(angle/2)*(p/p.norm()));
}