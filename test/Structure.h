#pragma once
#include <vector>
#include <string.h>
#include <iostream>
#include "Bvh.h"
#include "library_set.h"


#define V3(x,y,z) glVertex3f(x,y,z)
#define V3P(P) glVertex3f(P.x, P.y, P.z)
Bvh *bvh = NULL;
GLuint bvhVAO;
GLuint bvhVBO;
void bvh_load_upload(char *bvhFileName, int frame=1){
	if (bvh==NULL){
		bvh=new Bvh;
		bvh->load(bvhFileName);
	}
}
void drawCube(OFFSET size){
	float x=size.x/2.0, y=size.y, z=size.z/2.0;
	//float x=2.0, y=5.0, z=2.0;
	x-=0.2, y-=0.2, z-=0.2;

	glColor3f(0,0,1);
	glBegin(GL_LINE_STRIP);
		V3(-x,0,-z); V3(x,0,-z); V3(x,y,-z); V3(-x,y,-z); V3(-x,0,-z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(-x,0,z); V3(-x,y,z); V3(x,y,z); V3(x,0,z); V3(-x,0,z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(-x,0,-z); V3(-x,0,z); V3(x,0,z); V3(x,0,-z); V3(-x,0,-z);
	glEnd();	
	glBegin(GL_LINE_STRIP);
		V3(-x,y,-z); V3(x,y,-z); V3(x,y,z); V3(-x,y,z); V3(-x,y,-z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(-x,0,-z); V3(-x,y,-z); V3(-x,y,z); V3(-x,0,z); V3(-x,0,-z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(x,0,-z); V3(x,0,z); V3(x,y,z); V3(x,y,-z); V3(x,0,-z);
	glEnd();



	glColor3f(1,1,1);
	glBegin(GL_POLYGON);
		V3(-x,0,-z); V3(x,0,-z); V3(x,y,-z); V3(-x,y,-z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(-x,0,z); V3(-x,y,z); V3(x,y,z); V3(x,0,z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(-x,0,-z), V3(-x,0,z); V3(x,0,z); V3(x,0,-z);
	glEnd();	
	glBegin(GL_POLYGON);
		V3(-x,y,-z), V3(x,y,-z); V3(x,y,z); V3(-x,y,z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(-x,0,-z); V3(-x,y,-z); V3(-x,y,z); V3(-x,0,z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(x,0,-z); V3(x,0,z); V3(x,y,z); V3(x,y,-z);
	glEnd();
}

void drawLink(OFFSET size){
    float x=size.x, y=size.y, z=size.z;

	glColor3f(0,0,1);
	glBegin(GL_LINE_STRIP);
		V3(-x,-y,0); V3(x,-y,0); V3(x,y,0); V3(-x,y,0);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(-x,-y,z); V3(-x,y,z); V3(x,y,z); V3(x,-y,z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(-x,-y,0), V3(-x,-y,z); V3(x,-y,z); V3(x,-y,0);
	glEnd();	
	glBegin(GL_LINE_STRIP);
		V3(-x,y,0), V3(x,y,0); V3(x,y,z); V3(-x,y,z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(-x,-y,0); V3(-x,y,0); V3(-x,y,z); V3(-x,-y,z);
	glEnd();
	glBegin(GL_LINE_STRIP);
		V3(x,-y,0); V3(x,-y,z); V3(x,y,z); V3(x,y,0);
	glEnd();

	glColor3f(1,1,1);
	glBegin(GL_POLYGON);
		V3(-x,-y,0); V3(x,-y,0); V3(x,y,0); V3(-x,y,0);
	glEnd();
	glBegin(GL_POLYGON);
		V3(-x,-y,z); V3(-x,y,z); V3(x,y,z); V3(x,-y,z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(-x,-y,0), V3(-x,-y,z); V3(x,-y,z); V3(x,-y,0);
	glEnd();	
	glBegin(GL_POLYGON);
		V3(-x,y,0), V3(x,y,0); V3(x,y,z); V3(-x,y,z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(-x,-y,0); V3(-x,y,0); V3(-x,y,z); V3(-x,-y,z);
	glEnd();
	glBegin(GL_POLYGON);
		V3(x,-y,0); V3(x,-y,z); V3(x,y,z); V3(x,y,0);
	glEnd();
}

OFFSET LinearSpline(OFFSET s, OFFSET e, float dt){
	return OFFSET(s.x*(1.0-dt)+e.x*dt,s.y*(1.0-dt)+e.y*dt,s.z*(1.0-dt)+e.z*dt);
}

OFFSET TrifuncSpline(OFFSET s,float dt){
	float angle=dt*2*PI;
	//return OFFSET(0.0,e.y*dt+s.y*(1.0-dt),s.z*dt+e.z*(1.0-dt));
}
int motionDataIndex;

Matrix4f getTranslateMatrix(float x, float y, float z){
	Matrix4f X;
	X << 1, 0, 0, x,
		 0, 1, 0, y,
		 0, 0, 1, z,
		 0, 0, 0, 1;
	return X;
}

Matrix4f getRotateMatrix(float x, int axis){
	Matrix4f X;
	x = - x * PI / 180.0;
	if (axis==0){
		X << 1, 0, 0, 0,
			 0, cos(x), -sin(x), 0,
			 0, sin(x), cos(x), 0,
			 0, 0, 0, 1;
	}
	if (axis==1){
		X << cos(x), 0, sin(x), 0,
			 0, 1, 0, 0,
			 -sin(x), 0, cos(x), 0,
			 0, 0, 0, 1;
	}
	if (axis==2){
		X << cos(x), -sin(x), 0, 0,
			 sin(x), cos(x), 0, 0,
		 	 0, 0, 1 ,0,
			 0, 0, 0, 1;
	}
	return X;
}

float ABS(float x){ return x>0?x:-x; }



void drawing(JOINT* joint, quater Q, OFFSET origin){
    quater Q2 = joint -> q;
    Q2 = Q2 * Q;
    position afterRot = calc_rotate(Q, position(joint->offset.x, joint->offset.y, joint->offset.z));
    OFFSET nextOrigin = OFFSET(afterRot.p[0], afterRot.p[1], afterRot.p[2]) + origin;
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
void findAndStacking(JOINT *joint, const char *target_name, OFFSET goal){
    joint->q = quater();
    if (strcmp(joint->name, target_name)==0){
        // Find target, current stack S are movable joints
        MatrixXf m(6,S.size());
        VECTOR v_end = VECTOR(goal - joint->coord);
        return;
    }
    else{
        for (auto &child : joint->children){
            S.push(child);
            findAndStacking(child, target_name, goal);
            S.pop();
        }
    }
}
void moveTarget(const char *joint_name, OFFSET goal){
    
}

void jointRotationInitiation(JOINT *joint){
    joint -> q = quater();
    if (joint == bvh->getRootJoint()) joint->coord= position(0,0,0);
    else joint->coord = joint->parent->coord + position(joint->offset.x, joint->offset.y, joint->offset.z);
    printf("%s %.5f %.5f %.5f\n",joint->name, joint->coord.p[0], joint->coord.p[1], joint->coord.p[2]);
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
