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
/*
static void moveJoint(JOINT* joint, MOTION* motionData, int frame_starts_index){
	int start_index = frame_starts_index + joint->channel_start;
	joint->matrix = glm::translate(glm::mat4(1.0), glm::vec3(joint->offset.x, joint->offset.y, joint->offset.z));

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
}*/

/*void Bvh::moveTo(unsigned frame){
	unsigned start_index = frame * motionData.num_motion_channels;
	moveJoint(rootJoint, &motionData, start_index);
}*/
/*
vector<glm::vec4> vertices;
vector<GLshort> indices;


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
*/
Bvh *bvh = NULL;
GLuint bvhVAO;
GLuint bvhVBO;
void bvh_load_upload(char *bvhFileName, int frame=1){
	if (bvh==NULL){
		bvh=new Bvh;
		bvh->load(bvhFileName);
	}
	/*bvh->moveTo(frame);
	JOINT* rootJoint = (JOINT*) bvh->getRootJoint();
	bvh_to_vertices(rootJoint, vertices, indices);*/
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

void drawing(const JOINT* joint){
	glPushMatrix();
	if (joint != bvh->getRootJoint()){
		if (drawType==0){
			glBegin(GL_LINE_STRIP);
			V3(0,0,0);
			V3(joint->offset.x, joint->offset.y, joint->offset.z);
			glEnd();
		} 

		else{
		    if (strcmp(joint->name,"EndSite")==0 && (strcmp(joint->parent->name,"head")==0 || strcmp(joint->parent->name,"Head")==0)){
		        glPushMatrix();    
		    	glTranslatef(0, 6, 0);
		        drawLink(OFFSET(5,5,13));
		        glPopMatrix();
		    }
		    else if (strcmp(joint->name,"lhumerus")==0 || strcmp(joint->name,"LeftArm")==0){
		        glPushMatrix();    
		    	glTranslatef(0, joint->offset.y/1.8, -3);
		        drawLink(OFFSET(5,joint->offset.y/2,5));
		        glPopMatrix();
		    }
		    else if (strcmp(joint->name,"rhumerus")==0 || strcmp(joint->name,"RightArm")==0){
		        glPushMatrix();    
		    	glTranslatef(0, joint->offset.y/1.8, -3);
		        drawLink(OFFSET(5,joint->offset.y/2,5));
		        glPopMatrix();
		    }
		    else if (ABS(joint->offset.z) > 0.1){
		        //printf("%.5f %.5f %.5f\n",joint->offset.x,joint->offset.y,joint->offset.z);
		        //drawLink(OFFSET(10,10,max(joint->offset.y,joint->offset.z)));
		        float len=joint->offset.z;
	//            len=len<ABS(joint->offset.x)?ABS(joint->offset.x):len;
	//            len=len<ABS(joint->offset.y)?ABS(joint->offset.y):len;
		        if (strcmp(joint->name,"head")==0 || strcmp(joint->name,"Head")==0){
		            glPushMatrix();    
		        	glTranslatef(0, 3, 0);
		            drawLink(OFFSET(5,8,len));
		            glPopMatrix();
		        }
		        else drawLink(OFFSET(5,5,len));
		    }else if (ABS(joint->offset.x)>0.1){
		        float len=joint->offset.x;
		        drawLink(OFFSET(len/2,5,5));
		    }else if (ABS(joint->offset.y)>0.1){
		        float len=joint->offset.y;
		        drawLink(OFFSET(5,len/2,5));
		    }
		}
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
	motionDataIndex += joint->num_channels;
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
void draw(int idx){
	//angleFixation();
	if (bvh!=NULL){
		motionDataIndex = (idx%bvh->motionData.num_frames) * bvh->motionData.num_motion_channels;
		//printf("%d ",motionDataIndex);
		drawing(bvh->getRootJoint());
	}
}
position getEyePosition(){
	return position(bvh->motionData.data[0],bvh->motionData.data[1],bvh->motionData.data[2]+500.0);
}
