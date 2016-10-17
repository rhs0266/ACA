#include <vector>
#include <string.h>
#include <iostream>
#include "library_set.h"

#define Xposition (1<<0)
#define Yposition (1<<1)
#define Zposition (1<<2)
#define Zrotation (1<<3)
#define Xrotation (1<<4)
#define Yrotation (1<<5)
    
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
    Matrix4f matrix;               // local transofrmation matrix (premultiplied with parents'
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
	Bvh(){};
	~Bvh(){};

	//loading
	void load(const string& filename);

	/** Loads motion data from a frame into local matrices **/
	void moveTo(unsigned frame);

	const JOINT* getRootJoint() const { return rootJoint; }
	unsigned getNumFrames() const { return motionData.num_frames; }
//private:
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
			if (trim(line) == "HIERARCHY")
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

	string* name = new string;
	stream >> *name;
	joint->name = name->c_str();

	string tmp;
	joint->matrix = Matrix4f::Identity(4,4);

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
		
			JOINT* tmp_joint = new JOINT;

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

			cout << num_frames*num_channels << endl;

			for (int frame=0;frame<num_frames;frame++){
				for (int channel=0;channel<num_channels;channel++){
					float x;
					stringstream ss;
					stream >> tmp; ss<<tmp; ss>>x;
					int index = frame * num_channels + channel;
					motionData.data[index]=x;
				}
			}
		}
	}
}
Bvh *bvh = NULL;
GLuint bvhVAO;
GLuint bvhVBO;
void bvh_load_upload(char *bvhFileName, int frame=1){
	if (bvh==NULL){
		bvh=new Bvh;
		bvh->load(bvhFileName);
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
int motionDataIndex;

void drawing(const JOINT* joint){
	glPushMatrix();
	if (joint != bvh->getRootJoint()){
		glBegin(GL_LINE_STRIP);
		V3(0,0,0);
		V3(joint->offset.x, joint->offset.y, joint->offset.z);
		glEnd(); 
	}


	glTranslatef(joint->offset.x, joint->offset.y, joint->offset.z);

	for (int i=0;i<joint->num_channels;i++){
		int channel = (int)joint->channels_order[i];
		if (channel == Xposition){
			glTranslatef(bvh->motionData.data[motionDataIndex + i],0,0);
		}
		if (channel == Yposition){
			glTranslatef(0,bvh->motionData.data[motionDataIndex + i],0);
		}
		if (channel == Zposition){
			glTranslatef(0,0,bvh->motionData.data[motionDataIndex + i]);
		}

		if (channel == Xrotation){
			glRotatef(bvh->motionData.data[motionDataIndex + i], 1.0, 0.0, 0.0);
		}
		if (channel == Yrotation){
			glRotatef(bvh->motionData.data[motionDataIndex + i], 0.0, 1.0, 0.0);
		}
		if (channel == Zrotation){
			glRotatef(bvh->motionData.data[motionDataIndex + i], 0.0, 0.0, 1.0);
		}
	}
	{	
		drawCube(OFFSET(2.0,2.0,2.0));
	}
	motionDataIndex += joint->num_channels;
	for (auto &child : joint->children){
		drawing(child);
	}
	glPopMatrix();
}
void draw(int idx){
	if (bvh!=NULL){
		motionDataIndex = (idx%bvh->motionData.num_frames) * bvh->motionData.num_motion_channels;
		drawing(bvh->getRootJoint());
	}
}
position getEyePosition(){
	return position(bvh->motionData.data[0],bvh->motionData.data[1],bvh->motionData.data[2]+500.0);
}
