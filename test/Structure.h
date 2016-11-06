#pragma once
#include <vector>
#include <string.h>
#include <iostream>
#include "Bvh.h"
#include "library_set.h"


#define V3(x,y,z) glVertex3f(x,y,z)
#define V3P(P) glVertex3f(P(0), P(1), P(2))
Bvh *bvh = NULL;
GLuint bvhVAO;
GLuint bvhVBO;
void bvh_load_upload(char *bvhFileName, int frame=1){
	if (bvh==NULL){
		bvh=new Bvh;
		bvh->load(bvhFileName);
	}
}

int motionDataIndex;

float ABS(float x){ return x>0?x:-x; }

int tempFlag=0;

void drawCube(Vector3f p1, Vector3f p2){
    float w=3, h=(p2-p1).norm();
    //cout << p1.transpose() << ", " << p2.transpose() << endl;
    vector<Vector3f> v;
    v.push_back(Vector3f(-w,-w,0)); v.push_back(Vector3f(-w,w,0)); v.push_back(Vector3f(w,w,0)); v.push_back(Vector3f(w,-w,0));
    v.push_back(Vector3f(-w,-w,h)); v.push_back(Vector3f(-w,w,h)); v.push_back(Vector3f(w,w,h)); v.push_back(Vector3f(w,-w,h));
    Vector3f t = p2 - p1; t /= t.norm();
    Vector3f t2 = t - Vector3f(0,0,1);
    Vector3f axis;
    if (Vector3f(0,0,1).cross(t2).norm()>1e-3) axis=Vector3f(0,0,1).cross(t2);
    else axis=Vector3f(0,0,1);
    float angle = acos(Vector3f(0,0,1).dot(t));
    // cout << t.transpose() << ", " << t2.transpose() << ", " << (axis/axis.norm()).transpose() << endl;
    // cout << angle << endl;
    quater Q = quater(cos(angle/2), axis/axis.norm()*sin(angle/2));
    for (int i=0;i<8;i++){
        v[i]=calc_rotate(Q,v[i]);
    }

    glPushMatrix();
    glTranslatef(p1(0),p1(1),p1(2));
    FOR (i,0,3){
        glBegin(GL_POLYGON);
        int i2=(i+1)%4;
        V3P(v[i]); V3P(v[i2]); V3P(v[i2+4]); V3P(v[i+4]);
        glEnd();
    }
    glBegin(GL_POLYGON); FOR (i,0,3) V3P(v[i]); glEnd();
    glBegin(GL_POLYGON); FOR (i,4,7) V3P(v[i]); glEnd();

    glColor3f(0,0,1);
    FOR (i,0,3){
        glBegin(GL_LINE_STRIP);
        V3P(v[i]); V3P(v[i+4]);
        glEnd();
    }
    glBegin(GL_LINE_STRIP);
    FOR (i,0,4){
        V3P(v[i%4]);
    }
    glEnd();
    glBegin(GL_LINE_STRIP);
    FOR (i,0,4){
        V3P(v[i%4+4]);
    }
    glEnd();
    glColor3f(1,1,1);
    glPopMatrix();
}

void drawing(JOINT* joint){
	if (joint != bvh->getRootJoint()){
        if (drawType==1){ // line
    		glBegin(GL_LINE_STRIP);
    		V3P(joint->parent->coord);
    		V3P(joint->coord);
    		glEnd();
        }
        if (drawType==2){ // volume
            //if (strcmp(joint->name,"thorax")==0)
            drawCube(joint->parent->coord, joint->coord);
        }
	}
	for (auto &child : joint->children){
		drawing(child);
	}
}

#include <stack>
vector<JOINT *> S;
void JacobianPseudoInv(JOINT *joint, const char *target_name, const Vector3f &dP, JOINT *from){
    if (strcmp(joint->name, target_name)==0){
        // Find target, current stack S are movable joints
        MatrixXf Jv(3,S.size());
        Vector3f v_end = Vector3f(dP);

        int i=0;
        for (auto &j : S){
            Vector3f p = joint->coord - j->coord;
            Vector3f w = p.cross(dP);
            w = w / w.norm();
            Jv(0,i) = (w.cross(p))(0);
            Jv(1,i) = (w.cross(p))(1);
            Jv(2,i) = (w.cross(p))(2);
            i++;
        }


        MatrixXf Jvt = Jv.transpose();
        VectorXf dTheta;
        if (Jv.rows() <= Jv.cols()){
            MatrixXf temp = Jv*Jvt;
            if (temp.determinant() < 1e-5){
                return;
            }
            MatrixXf JvInv = Jvt * temp.inverse();
            dTheta = JvInv * dP;
        }else{
            MatrixXf temp = Jvt*Jv;
            if (temp.determinant() < 1e-5){
                return;
            }
            MatrixXf JvInv = temp.inverse() * Jvt;
            dTheta = JvInv * dP;
        }
        i=0;
        for (auto &j : S){
            Vector3f p = joint->coord - j->coord;
            Vector3f w = p.cross(dP);
            float dT = dTheta(i);
            if (dT>0.05) dT=0.05;
            if (dT<-0.05) dT=-0.05; // limit rotating angle
            if (w.norm()>1e-3){
            	w = w / w.norm();
            	j->q = quater(cos(dT/2.0), w*sin(dT/2.0)) * j->q;
            }
            i++;
        }
        return;
    }
    else{
    	S.push_back(joint);
        // from rooted tree to un-rooted tree
    	if (joint->parent!=NULL && joint->parent!=from) JacobianPseudoInv(joint->parent, target_name, dP, joint);
        for (auto &child : joint->children){
            if (child==from) continue;
            JacobianPseudoInv(child, target_name, dP, joint);
        }
        S.pop_back();
    }
}

void reCalculateCoord(JOINT* joint, quater Q){
    quater Q2 = joint -> q;
    Q2 = Q2 * Q;
    Vector3f afterRot = calc_rotate(Q, joint->offset);
    if (joint->parent != NULL) joint -> coord = joint -> parent -> coord + afterRot;
    for (auto &child : joint->children){
        reCalculateCoord(child, Q2);
    }
}

JOINT *findJoint(JOINT *joint, const char* target_name){
	if (strcmp(joint->name, target_name)==0) return joint;
	JOINT *temp = NULL;
	for (auto &child : joint->children){
		temp = findJoint(child, target_name);
		if (temp!=NULL) break;
	}
	return temp;
}
	
void moveTarget(const char *constraint_joint_name, const char *target_joint_name, Vector3f goalP){
	Vector3f dP = goalP - findJoint(bvh->getRootJoint(), target_joint_name)->coord;
    if (dP.norm() < 1) return;
	// dP /= 10.0;
 //    if (dP.norm()<0.1) dP = dP * 0.1 / dP.norm(); // delta movement

    JacobianPseudoInv(findJoint(bvh->getRootJoint(),constraint_joint_name), target_joint_name, dP, NULL);

    reCalculateCoord(bvh->getRootJoint(), quater());
}

void jointRotationInitiation(JOINT *joint){
    joint -> q = quater();
    if (joint == bvh->getRootJoint()) joint->coord = Vector3f(0,0,0);
    else joint->coord = joint->parent->coord + joint->offset;
    for (auto &child : joint->children){
        jointRotationInitiation(child);
    }
}
void setting(){
    motionDataIndex = 0;
	jointRotationInitiation(bvh->getRootJoint());
}	
void draw(int idx){ // idx is only need for "BVH LOADING"
	if (bvh!=NULL){
		drawing(bvh->getRootJoint());
	}
}
position getEyePosition(){
	return position(bvh->motionData.data[0],bvh->motionData.data[1],bvh->motionData.data[2]+500.0);
}
