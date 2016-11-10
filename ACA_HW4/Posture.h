#ifndef __POSTURE_H__
#define __POSTURE_H__


struct Posture{
    int motionIdx;
    V3 p;
    vector<quater> q;
    Posture(){};
    Posture(V3 _p, vector<quater> _q):p(_p),q(_q){};
    Posture operator+(const Displace &rhs){
        assert(q.size()!=rhs.q.size());
        
        addTranslation(calc_rotate(q[0], rhs.p));
        for (int i=0;i<q.size();i++) addRotation(i, rhs.q[i]);
    }
    Displace operator-(const Posture &rhs){
        assert(q.size()!=rhs.q.size());

        V3 _p = calc_rotate(rhs.q[0].inverse(),p-rhs.p);
        vector<V3> _q;
        for (int i=0;i<q.size();i++){
            _q.push_back(LOG(rhs.q[i].inverse() * q[i])); // TODO LOG()
        }
        return Displace(_p, _q);
    }
    void addTranslation(const V3& d){
        p += d;
    }
    void addRotation(int i, const V3& v){
        assert(0<=i && i<q.size());
        
        q[i]=q[i]*EXP(v); // TODO EXP()
    }
};

void Posture_to_Hierarchy(JOINT *joint, Posture posture, int *idx) {
	if (joint->parent == NULL) { // root joint
		joint->offset = posture.p;
	}
	joint->q = posture.q[*idx];
	(*idx)++;
	for (auto child : joint->children) {
		Posture_to_Hierarchy(child, posture, idx);
	}
}
void Hierarchy_to_Posture(JOINT *joint, Posture posture, int *idx) {
	if (joint->parent == NULL) {
		posture.p = joint->offset;
	}
	posture.q[*idx] = joint->q;
	(*idx)++;
	for (auto child : joint->children) {
		Hierarchy_to_Posture(child, posture, idx);
	}
}
void readFrame(JOINT *joint, float *motionData, int *idx) {
	joint->q = quater();
	for (int i = 0; i<joint->num_channels; i++) {
		int channel = (int)joint->channels_order[i];
		if (channel == Xposition) joint->offset(0) = motionData[(*idx) + i];
		if (channel == Yposition) joint->offset(1) = motionData[(*idx) + i];
		if (channel == Zposition) joint->offset(2) = motionData[(*idx) + i];
		if (channel == Xrotation) {
			float t = motionData[(*idx) + i];
			joint->q = quater(cos(t / 2), V3(1, 0, 0)*sin(t / 2)) * joint->q;
		}
		if (channel == Yrotation) {
			float t = motionData[(*idx) + i];
			joint->q = quater(cos(t / 2), V3(0, 1, 0)*sin(t / 2)) * joint->q;
		}
		if (channel == Zrotation) {
			float t = motionData[(*idx) + i];
			joint->q = quater(cos(t / 2), V3(0, 0, 1)*sin(t / 2)) * joint->q;
		}
	}
	(*idx) += joint->num_channels;
	for (auto &child : joint->children) {
		readFrame(child, motionData, idx);
	}
}
#endif