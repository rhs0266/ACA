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

V3 translate;
V3 pv,v;
matrix rot_mat;
quater Tot;

/* vectors that makes the rotation and translation of the cube */
V3 eye = V3(0.0f, 100.0f, 500.0f);
V3 ori = V3(0.0f, 0.0f, 0.0f);
float rot[3] = { 0.0f, 0.0f, 0.0f };
Posture current_posture;
vector<Posture> currentMotion, nextMotion;

#define CAM_FRONT 0
#define CAM_UP 1
#define CAM_FIXED 0
#define CAM_FREE 1
int viewDirection = CAM_UP, camera = CAM_FIXED;

void loadGlobalCoord(){
    glLoadIdentity();
    V3 eye_new, ori_new, up_new;
    
    if (camera == CAM_FIXED){
        if (viewDirection == CAM_UP) eye = current_posture.p + V3(0,800,10);
        else eye = current_posture.p + V3(0,100,500);
        ori = current_posture.p;
        translate = V3(0,0,0);
    }
    else{
        eye = eye + translate;
        ori = ori + translate;
        translate = V3(0,0,0);
    }
    eye_new = eye;
    ori_new = ori;
    up_new = V3(0,1,0);
    gluLookAt(eye_new(0), eye_new(1), eye_new(2), ori_new(0), ori_new(1), ori_new(2), up_new(0), up_new(1), up_new(2));

    glMultMatrixd(rotMatrix);}


//------------------------------------------------------------------------
// Moves the screen based on mouse pressed button
//------------------------------------------------------------------------
V3 ray(V3 A, V3 B, V3 C, float r){ // A+kB on sphere
    V3 a = B;
    V3 b = C-A;
    V3 p = a*a.dot(b);
    // a is unit vector
    // p is projection vector of b to a

    float dist_p = p.norm();
    V3 e = b - p; // p + e = b
    float dist_e = e.norm();
    
    if (dist_e >= 0.9999f * r){
        float max_angle = atan2(r, sqrt(b.norm()*b.norm() - r*r));
        V3 axis = b.cross(p);
        axis = axis / axis.norm();
        quater Q = quater(cos(max_angle/2), axis*sin(max_angle/2));
        p = calc_rotate(Q,b);
        p = p / p.norm() * sqrt(b.norm()*b.norm() - r*r);
    }else{
        p = p / p.norm() * (p.norm() - sqrt(r*r - e.norm()*e.norm()));
    }
    return A+p;}

void print(V3 x, string name){
    cout << name;
    fprintf(out," = %.3lf, %.3lf, %.3lf\n",x(0),x(1),x(2));
}

V3 D2toD3(int x,int y){ // return V3 of intercept b/w cam & cos(center of sphere)
    x-=width/2; y=height/2-y;
    float camDist = height/2.0 / tan((fov/2.0)*PI/180.0);
    V3 vec = V3((float)x,(float)y,0.0) - V3(0.0,0.0,camDist);
    vec=vec/vec.norm();
    vec = calc_rotate(Tot, vec);

    V3 eye_new, ori_new; 
    eye_new = calc_rotate(Tot,eye) + translate;
    ori_new = translate;
    return ray(eye_new, vec, ori_new, trackballRadius);
}

void glutMotion(int x, int y){
    trackballRadius = eye(2) * 0.4 * (fov/45);
    pv=D2toD3(mousePosX, mousePosY);
    v =D2toD3(x,y);
    mousePosX = x;
    mousePosY = y;
    curMouseX = x, curMouseY = y;
    
    float theta = angle(v, pv);
    if (leftButton){
        V3 cross = pv.cross(v);
        if (cross.norm()<=eps) return;
        cross = cross/cross.norm();
        quater Q = quater(cos(theta/2), cross*sin(theta/2));
        Q = Q.inverse();
        Tot = Q * Tot;
    }
    return;
}

V3 getPointOnPlane(int x,int y){
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
        return V3(9999,9999,9999);
    }
    winZ = 2.0*winZ - 1.0;
    winZ = ((-2.0*Far*Near)/(Far-Near)) / (winZ - (Far + Near) / (Far - Near));
    
    int x2=x-width/2, y2=height/2-y;
    float camDist = height/2.0 / tan((fov/2.0)*PI/180.0);
    V3 vec = V3((float)x2,(float)y2,0.0) - V3(0.0,0.0,camDist);
    vec=vec/vec.norm();
    vec = vec * (winZ / abs(vec(2)));
    return eye + vec;
}

//------------------------------------------------------------------------
// Function that handles mouse input
//------------------------------------------------------------------------
void glutMouse(int button, int state, int x, int y){
    curMouseX = x, curMouseY = y;
    switch ( button )
    {
        case GLUT_LEFT_BUTTON:

            if ( state == GLUT_DOWN )
            {
                mousePosX = x;
                mousePosY = y;
                leftButton = true;
                if (seekFlag){
                    seekFlag = false;
                    V3 newCenter = getPointOnPlane(x,y);
                    if (9999-1e-4<=newCenter(0) && newCenter(0)<=9999+1e-4){
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
    return;}

#define STOP 0
#define RF 1
#define ShortStep 2
#define AVG 0
#define SLOW 1
vector<Posture> StoW, WtoS, Walk, Stop;
vector<Posture> slowStoW, slowWtoS, slowWalk;
vector<Posture> TL_45, TL_90, TL_135, TL_180;
vector<Posture> TR_45, TR_90, TR_135, TR_180;
vector<Posture> Jump, HighJump;
int state = 0, speed = 0;

void makeGround(){
    drawCube(V3(0,-100,-15),V3(0,-100,15),15);
}

void display() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)width / (GLfloat)height, Near, Far);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_BACK,GL_FILL);
    glPolygonMode(GL_FRONT,GL_FILL);
    loadGlobalCoord();

    makeGround();

    assert(currentMotion.size()>0);

    int idx = frame_idx;
    if (idx == currentMotion.size()){
    	currentMotion = MotionWarping(nextMotion, currentMotion[idx-1]);
        if (state == STOP) nextMotion = Stop;
        else if (state == RF)
            if (speed == AVG) nextMotion = Walk;
            else nextMotion = slowWalk;
        else if (state == ShortStep)
            nextMotion = WtoS, state = STOP;
    	frame_idx = idx = 0;
    }
    // idx = currentMotion.size()-1;
    current_posture = currentMotion[idx];

    drawPosture(&current_posture);
    // drawPosture(&TL[frame_idx % TL.size()]);

    glutSwapBuffers();
}

int moveFlag=1;
void resize(int w, int h) {
    width = w;
    height = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)w / (GLfloat)h, .1f, 500.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        fprintf(out,"Thank you.\n");
        exit(0);
        break;

    case '1':
        drawType = 3 - drawType;
        break;
    case ' ':
        moveFlag = 1 - moveFlag;
        break;

    case '2':
        if (state == RF) nextMotion = TL_45;
        break;
    case '3':
        if (state == RF) nextMotion = TL_90;
        break;
    case '4':
        if (state == RF) nextMotion = TL_135;
        break;
    case '5':
        if (state == RF) nextMotion = TL_180;
        break;
    case '6':
        if (state == RF) nextMotion = TR_45;
        break;
    case '7':
        if (state == RF) nextMotion = TR_90;
        break;
    case '8':
        if (state == RF) nextMotion = TR_135;
        break;
    case '9':
        if (state == RF) nextMotion = TR_180;
        break;
    case '0':
        speed = 1 - speed;
        break;
    case '\'':
        if (state == RF){
            if (speed == AVG) nextMotion = WtoS;
            else nextMotion = slowWtoS;
            state = STOP;
        }else{
            if (speed == AVG) nextMotion = StoW;
            else nextMotion = slowStoW;
            state = RF;
        }
        break;

    case 'v':
        viewDirection = 1 - viewDirection;
        break;
    case 'c':
        camera = 1 - camera;
        break;

    case 'j':
        if (state == STOP) nextMotion = Jump;
        break;
    case 'k':
        if (state == STOP) nextMotion = HighJump;
        break;
    case 'l':
        if (state==STOP){
            state = ShortStep;
            nextMotion = StoW;
        }
        break;
    case 'w': // 'w' view up translate
        translate = translate + calc_rotate(Tot, V3(0,-1,0));
        break;
    case 's': // 's' view down translate
        translate = translate + calc_rotate(Tot, V3(0,+1,0));
        break;
    case 'a': // 'a' view left translate
        translate = translate + calc_rotate(Tot, V3(+1,0,0));
        break;
    case 'd': // 'd' view right translate
        translate = translate + calc_rotate(Tot, V3(-1,0,0));
        break;

    case '[': // '[' view dolly in
        if (eye(2)<=5) break;
        eye(2)-=1;
        break;
    case ']': // ']' view dolly out
        if (eye(2)>=300) break;
        eye(2)+=1;
        break;
    default:
        break;
    }
}


unsigned timeStep = 30;
void Timer(int unused)
{
    if (moveFlag==1) frame_idx++;
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
    fprintf(out,"[2 | 3 | 4 | 5 ] : turn left 45, 90, 135, 180 degrees. works if it is walking\n");
    fprintf(out,"[6 | 7 | 8 | 9 ] : turn right 45, 90, 135, 180 degrees. works if it is walking\n");
    fprintf(out,"[0] : convert speed type between average and slow walk\n");
    fprintf(out,"[j | k] : normal and high jump. works if it stops\n");
    fprintf(out,"[l] : short step. works if it stops");
    fprintf(out,"['] : convert stop or walk\n");
    fprintf(out,"[v] : convert view direction\n");
    fprintf(out,"[c] : convert camera fixation\n");
    fprintf(out,"[space bar] : pause or resume\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ [ ] : dolly in,  exists maximum range\n");
    fprintf(out,"[ ] ] : dolly out, exists maximum range\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ esc ] : exit program.\n");
}

void MotionClipSetting(){
    //stop
    bvh_load_upload("walk_fast_stright.bvh");
    StoW = readMultiFrames(68, 108);
    Stop = readMultiFrames(52, 58);
    for (int i=Stop.size()-1;i>=0;i--) Stop.push_back(Stop[i]);

    // turn left
    bvh_load_upload("walk_normal_left_45.bvh");
    TL_45 = readMultiFrames(399, 438);
    bvh_load_upload("walk_normal_left_90.bvh");
    TL_90 = readMultiFrames(116, 163);
    bvh_load_upload("walk_normal_left_135.bvh");
    TL_135 = readMultiFrames(795, 878);
    bvh_load_upload("walk_normal_left_180.bvh");
    TL_180 = readMultiFrames(108, 186);

    // turn right
    bvh_load_upload("walk_normal_right_45.bvh");
    TR_45 = readMultiFrames(66, 141);
    bvh_load_upload("walk_normal_right_90.bvh");
    TR_90 = readMultiFrames(75, 153);
    bvh_load_upload("walk_normal_right_135.bvh");
    TR_135 = readMultiFrames(79, 159);
    bvh_load_upload("walk_normal_right_180.bvh");
    TR_180 = readMultiFrames(59, 174);

    // Walk, WtoS
    bvh_load_upload("16_15_walk.bvh");
    Walk = readMultiFrames(18,18+33);
    bvh_load_upload("16_33_slow_walk,_stop.bvh");
    WtoS = readMultiFrames(38, 60);

    // jump
    bvh_load_upload("16_02_jump.bvh");
    Jump = readMultiFrames(31, 83);
    bvh_load_upload("16_03_high_jump.bvh");
    HighJump = readMultiFrames(18, 98);
    
    // slow walk
    bvh_load_upload("walk_slow_stright.bvh");
    slowStoW = readMultiFrames(320, 344);
    slowWalk = readMultiFrames(344, 396);
    slowWtoS = readMultiFrames(446, 496);
}

int main(int argc, char **argv) {
    ManualPrint();
    {
        MotionClipSetting();

        state = STOP;
        nextMotion = Stop;
        currentMotion = Stop;
    }
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width , height);
    glutInitWindowPosition( 50, 0 );
    glutCreateWindow("Motion Warping");

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
