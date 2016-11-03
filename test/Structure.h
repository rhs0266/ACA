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

void drawing(JOINT* joint, quater Q, Vector3f origin){
    quater Q2 = joint -> q;
    Q2 = Q2 * Q;
    Vector3f afterRot = calc_rotate(Q, joint->offset);
    Vector3f nextOrigin = afterRot + origin;
	if (joint != bvh->getRootJoint()){
		glBegin(GL_LINE_STRIP);
		V3P(origin);
		V3P(nextOrigin);
		glEnd();
	}
	for (auto &child : joint->children){
		drawing(child, Q2, nextOrigin);
	}
}

#include <stack>
stack<JOINT *> S;
void findAndStacking(JOINT *joint, const char *target_name, Vector3f goalP, Vector3f goalQ){
    joint->q = quater();
    if (strcmp(joint->name, target_name)==0){
        // Find target, current stack S are movable joints
        MatrixXf J1(3,S.size());
        Vector3f v_end = Vector3f(goalP - joint->coord);
        return;
    }
    else{
    	S.push(joint);
        for (auto &child : joint->children){
            findAndStacking(child, target_name, goal);
        }
        S.pop();
    }
}
void moveTarget(const char *joint_name, Vector3f goalP, quater goalQ){
    
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
	jointRotationInitiation(bvh->getRootJoint());
}	
void draw(int idx){
	//motionDataIndex = (idx%bvh->motionData.num_frames) * bvh->motionData.num_motion_channels;
	//jointRotationInitiation(bvh->getRootJoint());

	if (bvh!=NULL){
		motionDataIndex = (idx%bvh->motionData.num_frames) * bvh->motionData.num_motion_channels;
		//printf("%d ",motionDataIndex);
		drawing(bvh->getRootJoint(),quater(),OFFSET());
	}
}
position getEyePosition(){
	return position(bvh->motionData.data[0],bvh->motionData.data[1],bvh->motionData.data[2]+500.0);
}
