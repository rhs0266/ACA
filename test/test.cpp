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

Vector3f translate;
Vector3f pv,v;
matrix rot_mat;
quater Tot;

/* vectors that makes the rotation and translation of the cube */
Vector3f eye = Vector3f(0.0f, 0.0f, 500.0f);
Vector3f ori = Vector3f(0.0f, 0.0f, 0.0f);
float rot[3] = { 0.0f, 0.0f, 0.0f };

void loadGlobalCoord()
{
    glLoadIdentity();
    Vector3f eye_new, ori_new, up_new;
    eye_new = calc_rotate(Tot,eye) + translate;
    ori_new = ori + translate;
    up_new = calc_rotate(Tot, Vector3f(0,1,0));
    gluLookAt(eye_new(0), eye_new(1), eye_new(2), ori_new(0), ori_new(1), ori_new(2), up_new(0), up_new(1), up_new(2));

    glMultMatrixd(rotMatrix);
}


//------------------------------------------------------------------------
// Moves the screen based on mouse pressed button
//------------------------------------------------------------------------
Vector3f ray(Vector3f A, Vector3f B, Vector3f C, float r){ // A+kB on sphere
    Vector3f a = B;
    Vector3f b = C-A;
    Vector3f p = a*a.dot(b);
    // a is unit vector
    // p is projection vector of b to a

    float dist_p = p.norm();
    Vector3f e = b - p; // p + e = b
    float dist_e = e.norm();
    
    if (dist_e >= 0.9999f * r){
        float max_angle = atan2(r, sqrt(b.norm()*b.norm() - r*r));
        Vector3f axis = b.cross(p);
        axis = axis / axis.norm();
        quater Q = quater(cos(max_angle/2), axis*sin(max_angle/2));
        p = calc_rotate(Q,b);
        p = p / p.norm() * sqrt(b.norm()*b.norm() - r*r);
    }else{
        p = p / p.norm() * (p.norm() - sqrt(r*r - e.norm()*e.norm()));
    }
    return A+p;
}

void print(Vector3f x, string name){
    cout << name;
    fprintf(out," = %.3lf, %.3lf, %.3lf\n",x(0),x(1),x(2));
}

Vector3f D2toD3(int x,int y){ // return Vector3f of intercept b/w cam & cos(center of sphere)
    x-=width/2; y=height/2-y;
    float camDist = height/2.0 / tan((fov/2.0)*PI/180.0);
    Vector3f vec = Vector3f((float)x,(float)y,0.0) - Vector3f(0.0,0.0,camDist);
    vec=vec/vec.norm();
    vec = calc_rotate(Tot, vec);

    Vector3f eye_new, ori_new; 
    eye_new = calc_rotate(Tot,eye) + translate;
    ori_new = translate;
    return ray(eye_new, vec, ori_new, trackballRadius);
}

void glutMotion(int x, int y)
{
    trackballRadius = eye(2) * 0.4 * (fov/45);
    pv=D2toD3(mousePosX, mousePosY);
    v =D2toD3(x,y);
    mousePosX = x;
    mousePosY = y;
    curMouseX = x, curMouseY = y;
    
    float theta = angle(v, pv);
    if (leftButton){
        Vector3f cross = pv.cross(v);
        if (cross.norm()<=eps) return;
        cross = cross/cross.norm();
        quater Q = quater(cos(theta/2), cross*sin(theta/2));
        Q = Q.inverse();
        Tot = Q * Tot;
    }
    return;
}

Vector3f getPointOnPlane(int x,int y){
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
        return Vector3f(9999,9999,9999);
    }
    winZ = 2.0*winZ - 1.0;
    winZ = ((-2.0*Far*Near)/(Far-Near)) / (winZ - (Far + Near) / (Far - Near));
    
    int x2=x-width/2, y2=height/2-y;
    float camDist = height/2.0 / tan((fov/2.0)*PI/180.0);
    Vector3f vec = Vector3f((float)x2,(float)y2,0.0) - Vector3f(0.0,0.0,camDist);
    vec=vec/vec.norm();
    vec = vec * (winZ / abs(vec(2)));
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
                if (seekFlag){
                    seekFlag = false;
                    Vector3f newCenter = getPointOnPlane(x,y);
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
        eye=Vector3f(0.0f,0.0f,100.0f);
        ori=Vector3f(0.0f,0.0f,0.0f);
        rot[0]=0.0f; rot[1]=0.0f; rot[2]=0.0f;
        translate = Vector3f(0,0,0);
        fov=45;
        Tot=quater();
        break;
	case '1': // body, arm bending
		drawType=1-drawType;
		break;
    case 'm':
        //moveTarget("pelvis", "ltoes", Vector3f(20.0,12,-60));
        moveTarget("lhumerus", "lhand", Vector3f(20.0,12,20));
        break;
    case 'w': // 'w' view up translate
        translate = translate + calc_rotate(Tot, Vector3f(0,-1,0));
        break;
    case 's': // 's' view down translate
        translate = translate + calc_rotate(Tot, Vector3f(0,+1,0));
        break;
    case 'a': // 'a' view left translate
        translate = translate + calc_rotate(Tot, Vector3f(+1,0,0));
        break;
    case 'd': // 'd' view right translate
        translate = translate + calc_rotate(Tot, Vector3f(-1,0,0));
        break;
    case 'f': // 'f' ready for getting mousepoint for 'seek'
        seekFlag = true;
        break;
    case 'b': // 'b' move camera backward to show all
        translate = Vector3f(0,0,0);
        eye=Vector3f(0.0f,0.0f,30.0f / tan((fov/2)*PI/180.0));
        //cout << 30.0f / tan((fov/2)*PI/180.0) << "\n";
        break;
    case '[': // '[' view dolly in
        if (eye(2)<=5) break;
        eye(2)-=1;
        break;
    case ']': // ']' view dolly out
        if (eye(2)>=300) break;
        eye(2)+=1;
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
		//eye=getEyeVector3f();
		//ori=eye-Vector3f(0,0,500);
    }
	setting();
    // moveTarget("pelvis", "ltoes", Vector3f(1.0,0,1.0));
    // moveTarget("pelvis", "lhand", Vector3f(1.0,0,1.0));
    // moveTarget("pelvis", "lhand", Vector3f(1.0,0,1.0));
    // moveTarget("pelvis", "lhand", Vector3f(1.0,0,1.0));
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
