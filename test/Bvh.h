#pragma once
#include "rhs_math.h"
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
    OFFSET(position pos):x(pos.p[0]),y(pos.p[1]),z(pos.p[2]){};
    OFFSET operator+(const OFFSET& rhs){
        return OFFSET(x+rhs.x, y+rhs.y, z+rhs.z);
    }
    OFFSET operator-(const OFFSET& rhs){
        return OFFSET(x-rhs.x, y-rhs.y, z-rhs.z);
    }
};

typedef struct JOINT JOINT;
typedef struct OFFSET POINT;
typedef struct OFFSET AXIS;
typedef struct OFFSET VECTOR;

struct JOINT
{
    const char* name = NULL;        // joint name
    JOINT* parent = NULL;           // joint parent
    Vector3f offset;                  // offset data
	Vector3f size;					// volume size
    unsigned int num_channels = 0;  // num of channels joint has
    short* channels_order = NULL;   // ordered list of channels
    vector<JOINT*> children;        // joint's children
    Matrix4f matrix;                // local transofrmation matrix (premultiplied with parents'
    unsigned int channel_start = 0; // index of joint's channel data in motion array

    Vector3f coord;
    float angle;
    quater q;

	//OFFSET rotation;				// rotation information for x, y, z-axis
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

	JOINT* getRootJoint() const { return rootJoint; }
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

	// load joint name
	string* name = new string;
	stream >> *name;
	joint->name = name->c_str();

	string tmp;
	//setting local matrix to identity
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
			stream >> joint->offset(0) >> joint->offset(1) >> joint->offset(2);
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
				stream >> tmp_joint->offset(0) >> tmp_joint->offset(1) >> tmp_joint->offset(2);
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
