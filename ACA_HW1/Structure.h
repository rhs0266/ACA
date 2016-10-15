#include <vector>
#include <string.h>
#include "library_set.h"

#define Xposition 0x01
#define Yposition 0x02
#define Zposition 0x04
#define Zrotation 0x10
#define Xrotation 0x20
#define Yrotation 0x40
    
struct OFFSET
{
    float x, y, z;
	OFFSET(){};
	OFFSET(float _x, float _y, float _z):x(_x),y(_y),z(_z){};
};

typedef struct JOINT JOINT;

struct JOINT
{
    const char* name = NULL;        // joint name
    JOINT* parent = NULL;           // joint parent
    OFFSET offset;                  // offset data
	OFFSET size;					// volume size
    unsigned int num_channels = 0;  // num of channels joint has
    short* channels_order = NULL;   // ordered list of channels
    std::vector<JOINT*> children;   // joint's children
    Matrix4d matrix;               // local transofrmation matrix (premultiplied with parents'
    unsigned int channel_start = 0; // index of joint's channel data in motion array

	OFFSET rotation;				// rotation information for x, y, z-axis
};

typedef struct
{
    JOINT* rootJoint;
    int num_channels;
} HIERARCHY;

typedef struct
{
    unsigned int num_frames;              // number of frames
    unsigned int num_motion_channels = 0; // number of motion channels 
    float* data = NULL;                   // motion float data array
    unsigned* joint_channel_offsets;      // number of channels from beggining of hierarchy for i-th joint
} MOTION;

Matrix4d makeMatrix(double x_rot, double y_rot, double z_rot){
	Matrix4d T1; T1 << 1, 0, 0, 0,
				   0, cos(x_rot), -sin(x_rot), 0,
				   0, sin(x_rot), cos(x_rot), 0,
				   0, 0, 0, 1;
	Matrix4d T2; T2 << cos(y_rot), 0, sin(y_rot), 0,
					0, 1, 0, 0,
					-sin(y_rot), 0, cos(y_rot), 0,
					0, 0, 0, 1;
	Matrix4d T3; T3 << cos(z_rot), -sin(z_rot), 0, 0,
					sin(z_rot), cos(z_rot), 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
	return T1*T2*T3;
}

Matrix4d idMatrix4d;
HIERARCHY hierarchy;

void setting(){
	float width=10.0;
	JOINT* set;
	hierarchy.rootJoint = new JOINT;
	JOINT* hip=hierarchy.rootJoint;
	set=hip;
	set->name="hip";
	set->parent=NULL;
	set->offset=OFFSET(0,0,0);
	set->size=OFFSET(0,0,0);
	set->rotation=OFFSET(0,0,0);
	{
		JOINT* backbone=new JOINT; hip->children.push_back(backbone);
		set=backbone;
		set->name="backbone";
		set->parent=hip;
		set->offset=OFFSET(0,0,0);
		set->size=OFFSET(20,10,width);
		set->rotation=OFFSET(0,0,0);
		{
		    JOINT* body=new JOINT; backbone->children.push_back(body);
		    set=body;
		    set->name="body";
		    set->parent=backbone;
		    set->offset=OFFSET(0,10,0);
		    set->size=OFFSET(20,30,width);
		    set->rotation=OFFSET(0,0,0);
		    {
		        JOINT* leftShoulder=new JOINT; body->children.push_back(leftShoulder);
		        set=leftShoulder;
		        set->name="leftShoulder";
		        set->parent=body;
		        set->offset=OFFSET(-10,30-3.5,0);
		        set->size=OFFSET(7,15,width);
		        set->rotation=OFFSET(0,0,90);
		        {
		            JOINT* leftElbow=new JOINT; leftShoulder->children.push_back(leftElbow);
		            set=leftElbow;
		            set->name="leftElbow";
		            set->parent=leftShoulder;
		            set->offset=OFFSET(0,15,0);
		            set->size=OFFSET(7,12,width);
		            set->rotation=OFFSET(0,0,0);
		            {
	                    JOINT* leftHand=new JOINT; leftElbow->children.push_back(leftHand);
	                    set=leftHand;
	                    set->name="leftHand";
	                    set->parent=leftElbow;
	                    set->offset=OFFSET(0,12,0);
	                    set->size=OFFSET(8,5,width);
	                    set->rotation=OFFSET(0,0,0);
		            }
		        }
		    }
		    {
		        JOINT* rightShoulder=new JOINT; body->children.push_back(rightShoulder);
		        set=rightShoulder;
		        set->name="rightShoulder";
		        set->parent=body;
		        set->offset=OFFSET(10,30-3.5,0);
		        set->size=OFFSET(7,15,width);
		        set->rotation=OFFSET(0,0,-90);
		        {
		            JOINT* rightElbow=new JOINT; rightShoulder->children.push_back(rightElbow);
		            set=rightElbow;
		            set->name="rightElbow";
		            set->parent=rightShoulder;
		            set->offset=OFFSET(0,15,0);
		            set->size=OFFSET(7,12,width);
		            set->rotation=OFFSET(0,0,0);
		            {
	                    JOINT* rightHand=new JOINT; rightElbow->children.push_back(rightHand);
	                    set=rightHand;
	                    set->name="rightHand";
	                    set->parent=rightElbow;
	                    set->offset=OFFSET(0,12,0);
	                    set->size=OFFSET(8,5,width);
	                    set->rotation=OFFSET(0,0,0);
		            }
		        }
		    }
		    {
		        JOINT* head=new JOINT; body->children.push_back(head);
		        set=head;
		        set->name="head";
		        set->parent=body;
		        set->offset=OFFSET(0,30,0);
		        set->size=OFFSET(10,14,width);
		        set->rotation=OFFSET(0,0,0);
		    }
		}
	}
	{
		JOINT* leftPelvis=new JOINT; hip->children.push_back(leftPelvis);
		set=leftPelvis;
		set->name="leftPelvis";
		set->parent=hip;
		set->offset=OFFSET(-5.5,0,0);
		set->size=OFFSET(9,16,width);
		set->rotation=OFFSET(0,0,180);
		{
			JOINT* leftKnee=new JOINT; leftPelvis->children.push_back(leftKnee);
			set=leftKnee;
			set->name="leftKnee";
			set->parent=leftPelvis;
			set->offset=OFFSET(0,16,0);
			set->size=OFFSET(9,16,width);
			set->rotation=OFFSET(0,0,0);
			{
				JOINT* leftFoot=new JOINT; leftKnee->children.push_back(leftFoot);
				set=leftFoot;
				set->name="leftFoot";
				set->parent=leftKnee;
				set->offset=OFFSET(0,16,width*1.5/2.0 - width/2.0);
				set->size=OFFSET(9,6,width*1.5);
				set->rotation=OFFSET(0,0,0);
			}
		}
	}
	{
		JOINT* rightPelvis=new JOINT; hip->children.push_back(rightPelvis);
		set=rightPelvis;
		set->name="rightPelvis";
		set->parent=hip;
		set->offset=OFFSET(5.5,0,0);
		set->size=OFFSET(9,16,width);
		set->rotation=OFFSET(0,0,180);
		{
			JOINT* rightKnee=new JOINT; rightPelvis->children.push_back(rightKnee);
			set=rightKnee;
			set->name="rightKnee";
			set->parent=rightPelvis;
			set->offset=OFFSET(0,16,0);
			set->size=OFFSET(9,16,width);
			set->rotation=OFFSET(0,0,0);
			{
				JOINT* rightFoot=new JOINT; rightKnee->children.push_back(rightFoot);
				set=rightFoot;
				set->name="rightFoot";
				set->parent=rightKnee;
				set->offset=OFFSET(0,16,width*1.5/2.0 - width/2.0);
				set->size=OFFSET(9,6,width*1.5);
				set->rotation=OFFSET(0,0,0);
			}
		}
	}
}

#define V3(x,y,z) glVertex3f(x,y,z)
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

OFFSET LinearSpline(OFFSET s, OFFSET e, float dt){
	return OFFSET(s.x*(1.0-dt)+e.x*dt,s.y*(1.0-dt)+e.y*dt,s.z*(1.0-dt)+e.z*dt);
}

OFFSET TrifuncSpline(OFFSET s,float dt){
	float angle=dt*2*PI;
	//return OFFSET(0.0,e.y*dt+s.y*(1.0-dt),s.z*dt+e.z*(1.0-dt));
}

void drawing(JOINT* joint){
	glPushMatrix();
	if (type==1){
		if (strcmp(joint->name,"body")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,0),OFFSET(40,0,0),dt);
		if (strcmp(joint->name,"leftElbow")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,0),OFFSET(40,0,0),dt);
		if (strcmp(joint->name,"rightElbow")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,0),OFFSET(40,0,0),dt);
		if (strcmp(joint->name,"leftHand")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,0),OFFSET(40,0,0),dt);
		if (strcmp(joint->name,"rightHand")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,0),OFFSET(40,0,0),dt);
		if (strcmp(joint->name,"leftShoulder")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,90),OFFSET(0,20,90),dt);
		if (strcmp(joint->name,"rightShoulder")==0)
			joint->rotation=LinearSpline(OFFSET(0,0,-90),OFFSET(0,-20,-90),dt);
	}
	if (type==2){
		if (strcmp(joint->name,"leftShoulder")==0)
			joint->rotation=OFFSET(0,cos(dt*2*PI)*40,sin(dt*2*PI)*40+90);
		if (strcmp(joint->name,"rightShoulder")==0)
			joint->rotation=OFFSET(0,-cos(dt*2*PI)*40,sin(-dt*2*PI)*40-90);
	}
	if (type==3){
		if (strcmp(joint->name,"leftPelvis")==0)
			joint->rotation=LinearSpline(OFFSET(-20,0,-180),OFFSET(20,0,-180),dt);
		if (strcmp(joint->name,"rightPelvis")==0)
			joint->rotation=LinearSpline(OFFSET(20,0,-180),OFFSET(-20,0,-180),dt);
		if (strcmp(joint->name,"leftShoulder")==0)
			joint->rotation=LinearSpline(OFFSET(210,0,20),OFFSET(150,0,20),dt);
		if (strcmp(joint->name,"rightShoulder")==0)
			joint->rotation=LinearSpline(OFFSET(150,0,-20),OFFSET(210,0,-20),dt);
	}
		
	glTranslatef(joint->offset.x, joint->offset.y, joint->offset.z);
	glRotatef(joint->rotation.x, 1.0, 0.0, 0.0);
	glRotatef(joint->rotation.y, 0.0, 1.0, 0.0);
	glRotatef(joint->rotation.z, 0.0, 0.0, 1.0);

	//printf("%s\n",joint->name);
	//if (strcmp(joint->name,"head")==0) glColor3f(1.0,0,0);
	//else glColor3f(1,1,1);
	drawCube(joint->size);
	for (auto &child : joint->children){
		drawing(child);
	}
	glPopMatrix();
}

void angleFixation(){
	JOINT* hip = hierarchy.rootJoint;
	{
		JOINT* backbone = hip->children[0];
		backbone->rotation.x = -10.0*dt, backbone->rotation.y = 0.0, backbone->rotation.z = 0.0;
	}
}
void draw(){
	//angleFixation();
	drawing(hierarchy.rootJoint);
}

