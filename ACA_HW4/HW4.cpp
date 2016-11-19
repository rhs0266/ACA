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
V3 eye = V3(0.0f, 0.0f, 500.0f);
V3 ori = V3(0.0f, 0.0f, 0.0f);
float rot[3] = { 0.0f, 0.0f, 0.0f };
Posture current_posture;
vector<Posture> currentMotion, nextMotion;

V3 getEyePosition(){
	return current_posture.p + V3(0,100,500);
}

void loadGlobalCoord(){
    glLoadIdentity();
    V3 eye_new, ori_new, up_new;
    // eye_new = calc_rotate(Tot,eye) + translate;
    eye_new = getEyePosition();
    // ori_new = ori + translate;
    ori_new = current_posture.p;
    // up_new = calc_rotate(Tot, V3(0,1,0));
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

vector<Posture> StoW, WtoS, Walk, TL, TR;

void makeGround(){
    drawCube(V3(0,-150,-100),V3(0,-150,-700),60);
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

    // drawBvh(frame_idx);


    // int idx = frame_idx % (Walk.size() + TL.size());
    // if (idx<Walk.size()){
    //     current_posture = Walk[idx];
    // }else{
    //     current_posture = TL[idx - Walk.size()];
    // }

    assert(currentMotion.size()>0);

    int idx = frame_idx;
    if (idx == currentMotion.size()){
    	cout << idx << " out of " << currentMotion.size() << endl;
    	currentMotion = MotionWarping(nextMotion, currentMotion[idx-1]);
    	nextMotion = Walk;
    	frame_idx = idx = 0;
    }
    // idx = currentMotion.size()-1;
    current_posture = currentMotion[idx];

    drawPosture(&current_posture);
    // drawPosture(&TL[frame_idx % TL.size()]);

    glutSwapBuffers();
}

int tempFlag=0;
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
    case ' ': // spacebar for re//setting view
        eye=V3(0.0f,0.0f,100.0f);
        ori=V3(0.0f,0.0f,0.0f);
        rot[0]=0.0f; rot[1]=0.0f; rot[2]=0.0f;
        translate = V3(0,0,0);
        fov=45;
        Tot=quater();
        break;

    case '1':
        drawType = 3 - drawType;
        break;
    case '2':
        tempFlag = 1 - tempFlag;
        break;
    case 'i':
        int target_frame; scanf("%d",&target_frame);
        readSingleFrame(target_frame, &current_posture);
        break;
    case 'h':
        frame_idx--;
        break;
    case 'l':
        frame_idx++;
        break;
    case 'p':
        printf("current_frame = %d\n",frame_idx);
        break;
    case 'n':
    	// currentMotion = MotionWarping(Walk, current_posture);
    	// frame_idx = 0;
    	nextMotion = Walk;
    	break;
	case 'm':
    	// currentMotion = MotionWarping(TL, current_posture);
    	// frame_idx = 0;
    	nextMotion = TL;
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
    if (tempFlag==1) frame_idx++;
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
    fprintf(out,"[m | , | . | / ] : move left hand of human object. each command corresponds to goal positions\n");
    fprintf(out,"[c | v | b | n ] : move right hand of human object. each command corresponds to goal positions\n");
    fprintf(out,"[k | l | ; | ' ] : move left toes of human object. each command corresponds to goal positions\n");
    fprintf(out,"[f | g | h | j ] : move right toes of human object. each command corresponds to goal positions\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ [ ] : dolly in,  exists maximum range\n");
    fprintf(out,"[ ] ] : dolly out, exists maximum range\n");
    fprintf(out,"------------------\n");
    fprintf(out,"[ esc ] : exit program.\n");
}

int main(int argc, char **argv) {
    ManualPrint();
    if (argc>=3){
        //InitialPosutre(&current_posture);
        //setting();

        bvh_load_upload(argv[1]);
        Walk = readMultiFrames(18,18+33);
        bvh_load_upload(argv[2]);
        TL = readMultiFrames(0,102);

        // cout << TL.size() << endl;
        nextMotion = Walk;

        currentMotion = Walk;

    }
// -0.99416 -0.04533 -0.08642 -0.04614
//      res[0] : 0.99934 -0.00660 0.02221 0.02791

    // quater q1 = quater(0.99934,-0.00660,0.02221,0.02791);
    // // quater q2 = quater(-0.99416,-0.04533,-0.08642,-0.04614);
    // quater q2 = quater(0.99416,0.04533,0.08642,0.04614);
    // V3 minus = LOG(q1.inverse() * q2); cout << "q2 - q1 = " << minus.transpose() << endl;
    // cout << "EXP(q2-q1) = "; (EXP(minus)).print();
    // quater MINUS = geodesic(EXP(minus),quater(1,0,0,0)); MINUS.print();
    // quater q3 = q1 * MINUS; q3.print();

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
