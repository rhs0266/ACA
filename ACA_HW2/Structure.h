#include <vector>
#include <string.h>
#include <iostream>
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
    vector<JOINT*> children;   // joint's children
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

class Bvh{
	JOINT* loadJoint(istream& stream, JOINT* parent = NULL);
	void loadHierarchy(istream& stream);
	void loadMotion(istream& stream);
public:
	Bvh();
	~Bvh();

	//loading
	void load(const std:string& filename);

	/** Loads motion data from a frame into local matrices **/
	void moveTo(unsigned frame);

	const JOINT* getRootJoint() const { return rootJoint; }
	unsigned getNumFrames() const { return motionData.numframes; }
private:
	JOINT* rootJoint;
	MOTION motionData;
};

void Bvh::load(const string& filename){
	fstream file;
	file.open(filename.c_str(), ios_base::in);

	if (file.is_open()){
		string line;
		while (file.good()){
			file >> line;
			if (trim(line) == "Hierarchy")
				loadHierarchy(file);
			break;
		}
		file.close();
	}
}

void Bvh::loadHierarchy(istream& stream){
	string tmp;
	while (stream.good()){
		stream >> tmp;
		if (trim(tmp)=="ROOT")
			rootJoint = loadJoint(stream);
		else if (trim(tmp)=="MOTION")
			loadMotion(stream);
	}
}

JOINT* Bvh::loadJoint(istream& stream, JOINT* parent){
	JOINT* joint = new JOINT;
	joint->parent = parent;

	// load joint name
	string* name = new string;
	stream >> *name;
	joint->name = name->c_str();

	string tmp;
	//setting local matrix to identity
	joint->matrix = glm::mat4(1.0);

	static int _channel_start=0;
	unsigned channel_order_index = 0;

	while (stream.good()){
		stream >> tmp;
		tmp=trim(tmp);

		//loading channels
		char c = tmp.at(0);
		if (c=='X' || c=='Y' || c=='Z'){
			if (tmp=="Xposition")
				joint->channels_order[channel_order_index++]=Xposition;
			if (tmp=="Yposition")
				joint->channels_order[channel_order_index++]=Yposition;
			if (tmp=="Zposition")
				joint->channels_order[channel_order_index++]=Zposition;

			if (tmp=="Xrotation")
				joint->channels_order[channel_order_index++]=Xrotation;
			if (tmp=="Yrotation")
				joint->channels_order[channel_order_index++]=Yrotation;
			if (tmp=="Zrotation")
				joint->channels_order[channel_order_index++]=Zrotation;
		}

		if (tmp=="OFFSET"){
			stream >> joint->offset.x >> joint->offset.y >> joint->offset.z;
		}
		else if (tmp=="CHANNELS"){
			stream >> joint->num_channels;
			motionData.num_motion_channels += joint->num_channels;

			joint->channel_start = _channel_start;
			_channel_start += joint->num_channels;

			joint->channels_order = new short[joint->num_channels];

		}
		else if (tmp=="JOINT"){
			JOINT* tmp_joint = loadJoint(stream, joint);

			tmp_joint->parent = joint;
			joint->children.push_back(tmp_joint);
		}
		else if (tmp=="End"){
			stream >> tmp >> tmp; // Site{
		
			Joint* tmp_joint = new JOINT;

			tmp_joint->parent = joint;
			tmp_joint->num_channels = 0;
			tmp_joint->name = "EndSite";
			joint->children.push_back(tmp_joint);
			stream >> tmp;
			if (tmp=="OFFSET")
				stream >> tmp_joint->offset.x >> tmp_joint->offset.y >> tmp_joint->offset.z;
				stream >> tmp;
		}
		else if (tmp=="}")
			return joint;
	}
}

void Bvh::loadMotion(istream& stream){
	string tmp;
	while (stream.good()){
		stream>>tmp;
		if (trim(tmp)=="Frames:")
			stream >> motionData.num_frames;
		else if (trim(tmp) == "Frame"){
			float frame_time;
			stream >> tmp >> frame_time;

			int num_frames = motionData.num_frames;
			int num_channels = motionData.num_motion_channels;

			//creating motion data array
			motionData.data = new float[num_frames * num_channels];

			for (int frame=0;frame<num_frames;frame++){
				for (int channel=0;channel<num_channels;channel++){
					float x;
					stringstream ss;
					stream tmp; ss<<tmp; ss>>x;
					int index = frame+num_channels + channel;
					motionData.data[index]=x;
				}
			}
		}
	}
}

static void moveJoint(Joint* joint, MOTION* motionData, int frame_starts_index){
	int start_index = frame_starts_index + joint->channel_start;
	joint->matrix = glm::stranslate(glm::mat4(1.0), glm::vec3(joint->offset.x, joint->offset.y, joint->offset.z));

	for (int i=0;i<joint->num_channels;i++){
		const short& channel = joint->channels_order[i];
		float value = motionData->data[start_index+i];
		if (channel&Xposition)
			joint->matrix=glm::translate(joint->matrix, glm::vec3(value,0,0));
		if (channel&Yposition)
			joint->matrix=glm::translate(joint->matrix, glm::vec3(0,value,0));
		if (channel&Zposition)
			joint->matrix=glm::translate(joint->matrix, glm::vec3(0,0,value));

		if (channel&Xrotation)
			joint->matrix=glm::rotate(joint->matrix, value, glm::vec3(1,0,0));
		if (channel&Yrotation)
			joint->matrix=glm::rotate(joint->matrix, value, glm::vec3(0,1,0));
		if (channel&Zrotation)
			joint->matrix=glm::rotate(joint->matrix, value, glm::vec3(0,0,1));

		if (joint->parent!=NULL)
			joint->matrix = joint->parent->matrix * joint->matrix;

		for (auto& child : joint->children)
			moveJoint(child,motionData,frame_starts_index);
}

void Bvh::moveTo(unsigned frame){
	unsigned start_index = frame * motionData.num_motion_channels;
	moveJoint(rootJoint, &motionData, start_index);
}

vector<glm::vec4> vertices;
vector<GLshort> indices;

GLuint bvhVAO;
GLuint bvhVBO;
Bvh *bvh = NULL;

void bvh_to_vertices(JOINT*					joint,
					 vector<glm::vec4>&		vertices,
					 vector<GLshort>&		indices,
					 GLshort				parentIndex = 0){
	glm::vec4 translatedVertex = joint->matrix[3];
	vertices.push_back(translatedVertex);
	GLshort myindex = vertices.size()-1;
	if (parentIndex != myindex){
		indices.push_back(parentIndex);
		indices.push_back(myindex);
	}
	for (auto& child: joint->children)
		bvh_to_vertices(child, vertices, indices, myindex);
}

void bvh_load_upload(int frame=1){
	if (bvh==NULL){
		bvh=new Bvh;
		bvh->load("file.bvh");
	}
	bvh->moveTo(frame);
	JOINT* rootJoint = (JOINT*) bvh->getRootJoint();
	bvh_to_vertices(rootJoint, vertices, indices);
}

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

