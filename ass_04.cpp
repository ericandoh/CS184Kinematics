#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <limits>

#define PI 3.14159265  // Should be used from mathlib

inline float sqr(float x) { return x*x; }


using namespace std;

/*
//****************************************************

Some side notes

OPENGL 3d rendering
https://www3.ntu.edu.sg/home/ehchua/programming/opengl/CG_Examples.html

IO Stuff
http://www.cplusplus.com/doc/tutorial/files/

Cylinder btw 2 points
http://lifeofaprogrammergeek.blogspot.com/2008/07/rendering-cylinder-between-two-points.html

//****************************************************
*/

//****************************************************
// Some Classes
//****************************************************

class Viewport;

class Viewport {
  public:
    int w, h; // width and height
};


//in classes
struct vec3 {
  float x, y, z;
};
typedef struct vec3 vec3;

struct color {
  float r, g, b;
};
typedef struct color color;



//****************************************************
// Global Variables
//****************************************************

Viewport  viewport;

int joint_count = 4;

int frame_count = 1000;

// array of rotations; size joint_count
vec3* rotations;

// array of joint positions + endpoint; size joint_count + 1
vec3* points;

//all points
vec3** allpoints;

//our goal point
vec3 goal;


void myDisplay();

//****************************************************
// Simple init function
//****************************************************
void initScene() {
  // 3D setup - shamelessly stolen from https://www3.ntu.edu.sg/home/ehchua/programming/opengl/CG_Examples.html
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClearDepth(1.0f);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glShadeModel(GL_SMOOTH);
  
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  
  //setup materials
  GLfloat mat_ambient[] = { 0.3, 0.3, 0.3 };
  GLfloat mat_diffuse[] = { 0.0, 0.0, 0.0 };

  srand ( time(NULL) );
  int choice = rand() % 3;
  mat_ambient[choice] = 0.8;
  
  choice = rand() % 3;
  cout << choice;
  mat_diffuse[choice] = 1.0;

  GLfloat mat_specular[] = { 1.0, 1.0, 1.0 };
  GLfloat mat_shininess[] = { 50.0 };
  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

  //setup lights
  GLfloat ambientLight[] = {0.1, 0.1, 0.1};
  GLfloat diffuseLight[] = {1.0, 1.0, 1.0};
  GLfloat specularLight[] = {1.0, 1.0, 1.0};
  GLfloat light_position[] = { -3.0, 3.0, 3.0, 0.0 };

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);

  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}


void cleanup() {
  //clean up
}

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  if (h == 0) {
    h = 1;
  }
  GLfloat aspect = (GLfloat)w / (GLfloat)h;

  glViewport (0,0,viewport.w,viewport.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}

void myKeyPressed(unsigned char key, int x, int y) {
  if(key == 32) {
    cleanup();
    exit(0);
  }
  else if (key == 's') {
    //key
  }
  //myDisplay();
}

/*
void mySpecialInput(int key, int x, int y) {
  int mod = glutGetModifiers();
  if(mod == GLUT_ACTIVE_SHIFT) {
    switch(key) {
      case GLUT_KEY_UP:
        yshift -= 1.0f;
        break;
      case GLUT_KEY_DOWN:
        yshift += 1.0f;
        break;
      case GLUT_KEY_LEFT:
        xshift += 1.0f;
        break;
      case GLUT_KEY_RIGHT:
        xshift -= 1.0f;
        break;
    }
  } else {
    switch(key) {
      case GLUT_KEY_UP:
        z_angle = fmod((z_angle + angle_change), 2*PI);
        break;
      case GLUT_KEY_DOWN:
        z_angle = fmod((z_angle - angle_change), 2*PI);
        break;
      case GLUT_KEY_LEFT:
        angle = fmod((angle + angle_change), 2*PI);
        break;
      case GLUT_KEY_RIGHT:
        angle = fmod((angle - angle_change), 2*PI);
        break;
    }
  }
  myDisplay();
}*/

//****************************************************
// A routine to set a pixel by drawing a GL point.  This is not a
// general purpose routine as it assumes a lot of stuff specific to
// this example.
//****************************************************

void setPixel(int x, int y, GLfloat r, GLfloat g, GLfloat b) {
  glColor3f(r, g, b);
  glVertex2f(x + 0.5, y + 0.5);   // The 0.5 is to target pixel
  // centers 
  // Note: Need to check for gap
  // bug on inst machines.
}

//****************************************************
// Vector Manipulation Methods
//****************************************************

float dotProduct(vec3 *a, vec3 *b) {
  return a->x * b->x + a->y * b->y + a->z*b->z;
}

void normalize(vec3* vec) {
  float mag = sqrt(pow(vec->x, 2) + pow(vec->y, 2) + pow(vec->z, 2));
  vec->x = vec->x / mag;
  vec->y = vec->y / mag;
  vec->z = vec->z / mag;
}

void scale(vec3 *dest, vec3 *vec, float scale) {
  dest->x = vec->x * scale;
  dest->y = vec->y * scale;
  dest->z = vec->z * scale;
}

void subtract(vec3 *dest, vec3 *a, vec3 *b) {
  dest->x = a->x - b->x;
  dest->y = a->y - b->y;
  dest->z = a->z - b->z;
}

void add(vec3 *dest, vec3 *a, vec3 *b) {
  dest->x = a->x + b->x;
  dest->y = a->y + b->y;
  dest->z = a->z + b->z;
}

void set(vec3* dest, vec3* src) {
  dest->x = src->x;
  dest->y = src->y;
  dest->z = src->z;
}

void crossProduct(vec3* dest, vec3* first, vec3* second) {
  dest->x = first->y * second->z - first->z * second->y;
  dest->y = first->z * second->x - first->x * second->z;
  dest->z = first->x * second->y - first->y * second->x;
}

float magnitude(vec3* a) {
  return sqrt(a->x * a->x + a->y * a->y + a->z * a->z);
}

float** penroseInverse() {
  //return pernose inverse
}

//****************************************************
// Meat of the assignment
//****************************************************

void calculateRotations() {
  //initialize rotations + setup stuff relevant to this assignemtn in particular


  //while loop...
    //updateJoint()
    //save points from updateJoint to allpoints
}

//one iteration of the joint algorithm
//should be called by my myDisplay method
void updateJoint() {
  //find jacobian
  //find dr
  //update rotations + return rotations
}

float** calculateJacobian() {

}







//****************************************************
// Draw a random ass triangle 
//****************************************************

/*
void drawTriangle(triangle *triangle) {
  glBegin(GL_TRIANGLES);

  //glColor3f(triangle->color->r, triangle->color->g, triangle->color->b);

  glNormal3f( triangle->a.norm.x, triangle->a.norm.y, triangle->a.norm.z );
  glVertex3f( triangle->a.pos.x, triangle->a.pos.y, triangle->a.pos.z );
  glNormal3f( triangle->b.norm.x, triangle->b.norm.y, triangle->b.norm.z );
  glVertex3f( triangle->b.pos.x, triangle->b.pos.y, triangle->b.pos.z );
  glNormal3f( triangle->c.norm.x, triangle->c.norm.y, triangle->c.norm.z );
  glVertex3f( triangle->c.pos.x, triangle->c.pos.y, triangle->c.pos.z );

  glEnd();
}*/

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);             // indicate we are specifying camera transformations
  glLoadIdentity();               // make sure transformation is "zero'd"

  /*float xpos = vdistance * cos(angle) * cos(z_angle);
  float ypos = vdistance * sin(angle) * cos(z_angle);
  float zpos = vdistance * sin(z_angle);*/
  float xpos = 0;
  float ypos = 5;
  float zpos = 0;
  
  gluLookAt(xpos, ypos, zpos,     // eye position
            0.0f, 0.0f, 0.0f,     // where to look at
            0.0f, 0.0f, 1.0f);    // up vector*/

  //drawTriangle(*triangle)
  glPushMatrix();
      glTranslated(0.0,0.0,0.0);
      glutSolidSphere(1,50,50);
  glPopMatrix();

  //go through allpoints, render balls + joints

  glFlush();
  glutSwapBuffers();          // swap buffers (we earlier set double buffer)
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {

  //parse in command line arguments
  /*if (argc < 3) {
    cout << "prgm must take in at least 2 arguments\n";
    cout << "1st argument: .bez file\n";
    cout << "2nd argument: param constant\n";
    cout << "(optional) 3rd argument: -a (for adaptive)";
    exit(0);
  }

  //ignore first arg - it is the name of this program
  int argI = 3;
  char *argument;
  while(argI < argc) {
    argument = argv[argI++];
    if (strcmp(argument, "-a") == 0) {
      //do stuff
    }
    else {
      printf("Unknown argument %s\n", argv[argI++]);
    }
  }*/

  //do processing here

  //This initializes glut
  glutInit(&argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initalize theviewport size
  viewport.w = 700;
  viewport.h = 700;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0,0);
  glutCreateWindow(argv[0]);

  initScene();              // quick function to set up scene

  glutDisplayFunc(myDisplay);       // function to run when its time to draw something
  glutReshapeFunc(myReshape);       // function to run when the window gets resized
  glutKeyboardFunc(myKeyPressed);
  //glutSpecialFunc(mySpecialInput);

  glutMainLoop();             // infinite loop that will keep drawing and resizing
  // and whatever else

  return 0;
}