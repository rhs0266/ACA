
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

float angle(position &pv, position &v){ // return value is radian
    position cross = pv*v;
    return atan2(norm(cross)/norm(pv)/norm(v), (pv%v)/norm(pv)/norm(v));
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
    position getVec(){
        float t = sin(getTheta()/2);
        position p2 = position(p[1]/t, p[2]/t, p[3]/t);
        return p2;
    }
};

position calc_rotate(quater Q, position _P){
    quater P = quater(0, _P);
    P=(Q*P)*Q.inverse();
    return position(P.p[1], P.p[2], P.p[3]);
}
