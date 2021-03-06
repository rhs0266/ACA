#include "library_set.h"
#include "rhs_math.h"
#include "Structure.h"
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <algorithm>
FILE *out=stdout;

GLdouble rotMatrix[16] =
{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};

/* set global variables */
int width=700, height=700;
int curMouseX, curMouseY;
int dist[6]={0,20,30,40,40,45};
bool leftButton = false;
bool rightButton = false;
bool seekFlag;
GLfloat mousePosX, mousePosY;
float r;
float Near = 5.0f, Far = 1000.0f;
float trackballRadius;// = 80.0f;
float buf[701][701];
double fov = 45.0;
char *bvhFileName;
int frame_idx;

position translate;
position pv,v;
matrix rot_mat;
quater Tot;

/* vectors that makes the rotation and translation of the cube */
position eye = position(0.0f, 0.0f, 500.0f);
position ori = position(0.0f, 0.0f, 0.0f);
float rot[3] = { 0.0f, 0.0f, 0.0f };

void loadGlobalCoord()
{
    glLoadIdentity();
    position eye_new, ori_new, up_new;
    eye_new = calc_rotate(Tot,eye) + translate;
    ori_new = ori + translate;
    up_new = calc_rotate(Tot, position(0,1,0));
    gluLookAt(eye_new.p[0], eye_new.p[1], eye_new.p[2], ori_new.p[0], ori_new.p[1], ori_new.p[2], up_new.p[0], up_new.p[1], up_new.p[2]);

    glMultMatrixd(rotMatrix);
}


//------------------------------------------------------------------------
// Moves the screen based on mouse pressed button
//------------------------------------------------------------------------
position ray(position A, position B, position C, float r){ // A+kB on sphere
    position a = B;
    position b = C-A;
    position p = a * (a%b);
    // a is unit vector
    // p is projection vector of b to a

    float dist_p = norm(p);
    position e = b - p; // p + e = b
    float dist_e = norm(e);
    
    if (dist_e >= 0.9999f * r){
        float max_angle = atan2(r, sqrt(norm(b)*norm(b) - r*r));
        position axis = b * p;
        axis = axis / norm(axis);
        quater Q = quater(cos(max_angle/2), axis.p[0]*sin(max_angle/2), axis.p[1]*sin(max_angle/2), axis.p[2]*sin(max_angle/2));
        p = calc_rotate(Q,b);
        p = p / norm(p) * sqrt(norm(b)*norm(b) - r*r);
    }else{
        p = p / norm(p) * (norm(p) - sqrt(r*r - norm(e)*norm(e)));
    }
    return A+p;
}

void print(position x, string name){
    cout << name;
    fprintf(out," = %.3lf, %.3lf, %.3lf\n",x.p[0],x.p[1],x.p[2]);
}

position D2toD3(int x,int y){ // return position of intercept b/w cam & cos(center of sphere)
    x-=width/2; y=height/2-y;
    float camDist = height/2.0 / tan((fov/2.0)*PI/180.0);
    position vec = position((float)x,(float)y,0.0) - position(0.0,0.0,camDist);
    vec=vec/norm(vec);
    vec = calc_rotate(Tot, vec);

    position eye_new, ori_new; 
    eye_new = calc_rotate(Tot,eye) + translate;
    ori_new = translate;
    return ray(eye_new, vec, ori_new, trackballRadius);
}

void glutMotion(int x, int y)
{
    trackballRadius = eye.p[2] * 0.4 * (fov/45);
    pv=D2toD3(mousePosX, mousePosY);
    v =D2toD3(x,y);
    mousePosX = x;
    mousePosY = y;
    curMouseX = x, curMouseY = y;
    
    float theta = angle(v, pv);
    if (leftButton){
        position cross = pv*v;
        if (norm(cross)<=eps) return;
        cross = cross/norm(cross);
        quater Q = quater(cos(theta/2), cross.p[0]*sin(theta/2), cross.p[1]*sin(theta/2), cross.p[2]*sin(theta/2));
        Q = Q.inverse();
        Tot = Q * Tot;
    }
    return;
}

position getPointOnPlane(int x,int y){
    double proj[16],model[16];
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    float winX,winY;
    GLfloat winZ;
    winX = (float)x;
    winY = height - (float)y;
    glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
    winZ/=0.00390619;
    if (winZ>=1-1e-4){ // Pointer doesn't meet object's surface
        return position(9999,9999,9999);
    }
    winZ = 2.0*winZ - 1.0;
    winZ = ((-2.0*Far*Near)/(Far-Near)) / (winZ - (Far + Near) / (Far - Near));
    
    int x2=x-width/2, y2=height/2-y;
    float camDist = height/2.0 / tan((fov/2.0)*PI/180.0);
    position vec = position((float)x2,(float)y2,0.0) - position(0.0,0.0,camDist);
    vec=vec/norm(vec);
    vec = vec * (winZ / abs(vec.p[2]));
    return eye + vec;
}

//------------------------------------------------------------------------
// Function that handles mouse input
//------------------------------------------------------------------------
void glutMouse(int button, int state, int x, int y)
{
    curMouseX = x, curMouseY = y;
    switch ( button )
    {
        case GLUT_LEFT_BUTTON:

            if ( state == GLUT_DOWN )
            {
                mousePosX = x;
                mousePosY = y;
                leftButton = true;
                if (seekFlag == true){
                    seekFlag = false;
                    position newCenter = getPointOnPlane(x,y);
                    if (9999-1e-4<=newCenter.p[0] && newCenter.p[0]<=9999+1e-4){
                    }else{
                        translate = translate + calc_rotate(Tot, (newCenter - ori));
                    }
                } 
            }
            else if ( state == GLUT_UP )
            {
                leftButton = false;
            }
            break;
        case GLUT_RIGHT_BUTTON:
            if ( state == GLUT_DOWN ){
                mousePosX = x;
                mousePosY = y;
                rightButton = true;
            }
            else if (state == GLUT_UP){
                rightButton = false;
            }
            break;
        case 3:break;
        default:break;
    }
    return;
}

typedef float F;

void display() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)width / (GLfloat)height, Near, Far);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_BACK,GL_FILL);
    glPolygonMode(GL_FRONT,GL_LINE);
    loadGlobalCoord();


	draw(frame_idx);

    
    glutSwapBuffers();
}

void resize(int w, int h) {
    width = w;
    height = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)w / (GLfloat)h, .1f, 500.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        fprintf(out,"Thank you.\n");
        exit(0);
        break;
    case ' ': // spacebar for re//setting view
        eye=position(0.0f,0.0f,100.0f);
        ori=position(0.0f,0.0f,0.0f);
        rot[0]=0.0f; rot[1]=0.0f; rot[2]=0.0f;
        translate = position(0,0,0);
        fov=45;
        Tot=quater();
        break;
	case '1': // body, arm bending
		drawType=1-drawType;
		break;
    case 'w': // 'w' view up translate
        translate = translate + calc_rotate(Tot, position(0,-0.5,0));
        break;
    case 's': // 's' view down translate
        translate = translate + calc_rotate(Tot, position(0,+0.5,0));
        break;
    case 'a': // 'a' view left translate
        translate = translate + calc_rotate(Tot, position(+0.5,0,0));
        break;
    case 'd': // 'd' view right translate
        translate = translate + calc_rotate(Tot, position(-0.5,0,0));
        break;
    case 'f': // 'f' ready for getting mousepoint for 'seek'
        seekFlag = true;
        break;
    case 'b': // 'b' move camera backward to show all
        translate = position(0,0,0);
        eye=position(0.0f,0.0f,30.0f / tan((fov/2)*PI/180.0));
        //cout << 30.0f / tan((fov/2)*PI/180.0) << "\n";
        break;
    case '[': // '[' view dolly in
        if (eye.p[2]<=5) break;
        eye.p[2]-=1;
        break;
    case ']': // ']' view dolly out
        if (eye.p[2]>=300) break;
        eye.p[2]+=1;
        break;
    case ';': // ';' view zoom in
        if (fov<=5) break;
        fov -= 1.0;
        break;
    case '\'': // ''' view zoom out
        if (fov>=90) break;
        fov += 1.0;
        break;
    default:
        break;
    }
}


unsigned timeStep = 30;
void Timer(int unused)
{
	if (type==1)
		if (0.0f<=dt && dt<=0.995f) dt+=alpha;
		else alpha=-alpha, dt+=alpha;
	else if (type==2){
		dt+=alpha;
		if (dt>=1.0f) dt=0.0f;
	}
	else if (type==3){
		if (0.0f<=dt && dt<=0.995f) dt+=alpha;
		else alpha=-alpha, dt+=alpha;
	}
	frame_idx++;
    glutPostRedisplay();
    glutTimerFunc(timeStep, Timer, 0);
}

void ManualPrint(){
    fprintf(out,"MANUAL FOR PROGRAM, 2014-16371 Ryu Ho Seok\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ 1 ] : shift draw type(volume version, link only version)\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ a ] : move left\n");
    fprintf(out,"[ s ] : move down\n");
    fprintf(out,"[ d ] : move right\n");
    fprintf(out,"[ w ] : move up\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ [ ] : dolly in,  exists maximum range\n");
    fprintf(out,"[ ] ] : dolly out, exists maximum range\n");
    fprintf(out,"[ ; ] : zoom in,   exists maximum range\n");
    fprintf(out,"[ ' ] : zoom out,  exists maximum range\n");

    fprintf(out,"[ b ] : show all,  if cannot see anything, press 'space bar'\n");
    fprintf(out,"[ f ] : seek, after press f, click mouse on the surface. if not, nothing happens\n");
    fprintf(out,"[ l ] : show or hide standard line for checking the center of rotation\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ space bar ] : reset fov, cam dist, translate, rotation\n");
    fprintf(out,"[ esc ] : exit program.\n");
}

int main(int argc, char **argv) {
    ManualPrint();
	if (argc>=2){
    	bvh_load_upload(argv[1], 1);
		eye=getEyePosition();
		ori=eye-position(0,0,500);
    }
	//setting();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width , height);
    glutInitWindowPosition( 50, 0 );
    glutCreateWindow("Platonic Solid");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_DEPTH_BUFFER_BIT);
    glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutTimerFunc(timeStep, Timer, 0);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(glutMouse);
    glutMotionFunc(glutMotion);

    glutMainLoop();
    return 0;
}
