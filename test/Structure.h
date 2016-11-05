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
void drawing(JOINT* joint){
	//printf("%s ",joint->name);
	//cout << joint->coord.transpose() << endl;
	if (joint != bvh->getRootJoint()){
		glBegin(GL_LINE_STRIP);
		V3P(joint->parent->coord);
		V3P(joint->coord);
		glEnd();
	}
	for (auto &child : joint->children){
		drawing(child);
	}
}

#include <stack>
vector<JOINT *> S;
bool StackFlag;
void findAndStacking(JOINT *joint, const char *target_name, Vector3f dP, JOINT *from){
    //joint->q = quater();
    if (strcmp(joint->name, target_name)==0){
        // Find target, current stack S are movable joints
        MatrixXf Jv(3,S.size());
        Vector3f v_end = Vector3f(dP);

        int i=0;
        printf("JOINTS LIST : ");
        for (auto &j : S){
            printf(" -> %s : ",j->name);
            Vector3f p = joint->coord - j->coord;
            Vector3f w = p.cross(dP);
            cout << "w = " << w.transpose() << "    ";
            //if (w.norm()>1e-3) w = w / w.norm();
            w = w / w.norm();
            cout << "w.norm = " << w.transpose() << endl;
            Jv(0,i) = (w.cross(p))(0);
            Jv(1,i) = (w.cross(p))(1);
            Jv(2,i) = (w.cross(p))(2);
            i++;
        }
        printf("\n");

        cout << "Jv = \n" << Jv << endl;

        MatrixXf Jvt = Jv.transpose();
        VectorXf dTheta;
        if (Jv.rows() <= Jv.cols()){
            MatrixXf temp = Jv*Jvt;
            if (temp.determinant() < 1e-5){
                StackFlag = false;
                return;
            }
            MatrixXf JvInv = Jvt * temp.inverse();
            dTheta = JvInv * dP;
            double relative_error = (Jv*dTheta - dP).norm() / dP.norm(); // norm() is L2 norm
            if (relative_error > 1e-4){
                printf("			@WTF, relative error is %f\n",relative_error);
                //return;
            }
        }else{
            MatrixXf temp = Jvt*Jv;
            if (temp.determinant() < 1e-5){
                StackFlag = false;
                return;
            }
            MatrixXf JvInv = temp.inverse() * Jvt;
            dTheta = JvInv * dP;
            double relative_error = (Jv*dTheta - dP).norm() / dP.norm(); // norm() is L2 norm
            if (relative_error > 1e-4){
                printf("			@WTF2, relative error is %f\n",relative_error);
                //return;
            }
        }
        i=0;
        for (auto &j : S){
            Vector3f p = joint->coord - j->coord;
            Vector3f w = p.cross(dP);
            if (w.norm()>1e-5 && dTheta(i)>1e-3){
            	w = w / w.norm();
            	j->q = quater(cos(dTheta(i)/2.0), w*sin(dTheta(i)/2.0)) * j->q;
            }
            //else j->q = quater();
            i++;
        }
        StackFlag=true;
        return;
    }
    else{
    	S.push_back(joint);
    	if (joint->parent!=NULL && joint->parent!=from) findAndStacking(joint->parent, target_name, dP, joint);
        for (auto &child : joint->children){
            if (child==from) continue;
            findAndStacking(child, target_name, dP, joint);
        }
        S.pop_back();
    }
}

void reCalculateCoord(JOINT* joint, quater Q){
    quater Q2 = joint -> q;
    Q2 = Q2 * Q;
    Vector3f afterRot = calc_rotate(Q2, joint->offset);
    if (joint->parent != NULL) joint -> coord = joint -> parent -> coord + afterRot;
    //printf("%s %.5f %.5f %.5f\n",joint->name, joint->coord(0), joint->coord(1), joint->coord(2));
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
	
int Count=0;
bool moveTarget(const char *constraint_joint_name, const char *target_joint_name, Vector3f goalP){
	Vector3f dP = goalP - findJoint(bvh->getRootJoint(), target_joint_name)->coord;
	dP /= 50.0;
	cout << dP.transpose() << endl;
    if (dP.norm() < 1e-5) return false;
    StackFlag = false;
    //if (Count > 10) return false;
    printf("Iteration #%d\n", ++Count);
    printf("\nFind And Stacking-----------------------------\n");
    findAndStacking(findJoint(bvh->getRootJoint(),constraint_joint_name), target_joint_name, dP, NULL);

    printf("\nRe-Calculate Coordinates----------------------\n");
    reCalculateCoord(bvh->getRootJoint(), quater());
    //printf("MOVE TARGET FINISH\n\n");
    return StackFlag;
}

void jointRotationInitiation(JOINT *joint){
    joint -> q = quater();
    if (joint == bvh->getRootJoint()) joint->coord = Vector3f(0,0,0);
    else joint->coord = joint->parent->coord + joint->offset;
    printf("%s %.5f %.5f %.5f\n",joint->name, joint->coord(0), joint->coord(1), joint->coord(2));
    for (auto &child : joint->children){
        jointRotationInitiation(child);
    }
}
void setting(){
    motionDataIndex = 0;
    printf("@Setting\n");
	jointRotationInitiation(bvh->getRootJoint());
}	
void draw(int idx){
	//motionDataIndex = (idx%bvh->motionData.num_frames) * bvh->motionData.num_motion_channels;
	//jointRotationInitiation(bvh->getRootJoint());

	if (bvh!=NULL){
		// printf("@draw\n");
		motionDataIndex = (idx%bvh->motionData.num_frames) * bvh->motionData.num_motion_channels;
		//printf("%d ",motionDataIndex);
		drawing(bvh->getRootJoint());
	}
}
position getEyePosition(){
	return position(bvh->motionData.data[0],bvh->motionData.data[1],bvh->motionData.data[2]+500.0);
}
