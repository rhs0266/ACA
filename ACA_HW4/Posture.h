#ifndef __POSTURE_H__
#define __POSTURE_H__


struct Posture{
    int motionIdx;
    V3 p;
    vector<quater> q;
    Posture(){};
    Posture(V3 _p, vector<quater> _q):p(_p),q(_q){};
    Posture operator+(const Displace &rhs){
        assert(q.size()==rhs.q.size());

        Posture ret = Posture(p,q);
        
        ret.addTranslation(calc_rotate(q[0], rhs.p)); // Rigid Transformation
        // ret.p += rhs.p;
        for (int i=0;i<q.size();i++){
        	ret.addRotation(i, rhs.q[i]);
        }
        return ret;
    }
    Displace operator-(const Posture &rhs){
        assert(q.size()==rhs.q.size());

        Displace ret= Displace();

        quater temp = rhs.q[0];
        ret.p = calc_rotate(temp.inverse(),p-rhs.p); // Rigid Transformation
        // ret.p = p - rhs.p; // Independent translation and rotation

        for (int i=0;i<q.size();i++){
        	quater temp = rhs.q[i];
            ret.q.push_back(LOG(temp.inverse() * q[i]));
        }
        return ret;
    }
    void addTranslation(const V3& d){
        p += d;
    }
    void addRotation(int i, const V3& v){
        assert(0<=i && i<q.size());
        
        q[i]=q[i]*EXP(v);
        // q[i]=EXP(v)*q[i];
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

float warpingFunction(float t){
	return 2*t*t*t-3*t*t+1;
}

vector<Posture> MotionAlignment(vector<Posture> motion, Posture lastPosture){
	vector<Posture> res = motion;
	// cout << motion.size() << endl;
	for (int i=res.size()-1;i>=0;i--) res[i].p-=res[0].p;
	Displace D = lastPosture-res[0];
	for (int i=1;i<D.q.size();i++) D.q[i] = V3(0,0,0);
	D.p(1) = 0; // In-plane translation

	float angle = D.q[0].norm();
	// D.q[0] = V3(0,D.q[0](1),0);
	quater dQ = geodesic(EXP(D.q[0]),quater(1,0,0,0));

	for (int i=0;i<res.size();i++){
		res[i].p = calc_rotate(dQ, res[i].p) + lastPosture.p;
		// TL[i].q[0] = TL[i].q[0] * EXP(D.q[0]);
		res[i].q[0] = dQ * res[i].q[0];
	}
	return res;
}

vector<Posture> MotionWarping(vector<Posture> motion, Posture lastPosture){
	vector<Posture> res;
	motion = MotionAlignment(motion, lastPosture);
	Displace D = lastPosture - motion[0];
	for (int i=1;i<motion.size();i++){
		res.push_back(motion[i] + (D * warpingFunction((float)i/(float)(motion.size()-1))));
	}
	return res;
}

#endif