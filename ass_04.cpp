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

#include <Eigen/SVD>

using Eigen::Vector3f;
//using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

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

//****************************************************
// Global Variables
//****************************************************

Viewport  viewport;

//used for rendering cylinderes;
GLUquadricObj *quadric=gluNewQuadric();

int checkpoint_counter = 0;

float increment_amount = 0.05f;
float current_increment = 0.0f;

float* lengths;

const int joint_count = 4;

const int goal_count = 20;

bool paused = false;

std::vector<vector<vec3>> rotation_frames;

void myDisplay();

void calculateRotations(vec3 rotations[], vec3* goal, int n);

void updateCalculations(float rotation_matrices[][3][3], vec3 pi_vectors[], 
  float transformation_matrices[][4][4], vec3 rotations[], float lengths[], int n);

void findEndEffector(vec3* end_effector, float rotation_matrices[][3][3], float transformation_matrices[][4][4], vec3* pn, int n);

bool reachedGoal(vec3* end_effector, vec3* goal, float lengths[], int n, float epsilon);

void calculateJacobian(float** jac, float rotation_matrices[][3][3], float transformation_matrices[][4][4], vec3* pn, int n);

void calculateRi(float dest[][3], vec3* rotation);

void findRealCoordinates(vec3 positions[], vec3 rotations[], float lengths[], int n);

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
  else if (key == 'p') {
    paused = !paused;
  }
  //myDisplay();
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

void setIdentity(float matrix[][3]) {
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      matrix[x][y] = (x == y) ? 1.0f : 0.0f;
    }
  }
}

void setIdentity4(float matrix[][4]) {
  for (int x = 0; x < 4; x++) {
    for (int y = 0; y < 4; y++) {
      matrix[x][y] = (x == y) ? 1.0f : 0.0f;
    }
  }
}

void addMatrix(float dest[][3], float a[][3], float b[][3]) {
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      dest[x][y] = a[x][y] + b[x][y];
    }
  }
}

void scaleMatrix(float dest[][3], float src[][3], float scl) {
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      dest[x][y] = src[x][y] * scl;
    }
  }
}

void mulMatrixVector(vec3* dest, float a[][3], vec3* b) {
  //a = i,j (3x3)
  //b = j,1 (3x1)
  //result = i,1 (3x1)
  vec3 temp;
  float val;
  for (int i = 0; i < 3; i++) {
    val = 0;
    //iterate j from 0 to 2
    val += a[i][0] * b->x;
    val += a[i][1] * b->y;
    val += a[i][2] * b->z;
    if (i == 0) {
      temp.x = val;
    }
    else if (i == 1) {
      temp.y = val;
    }
    else {
      temp.z = val;
    }
  }
  set(dest, &temp);
}

void mulMatrix3(float dest[][3], float a[][3], float b[][3]) {
  float temp[3][3];
  for (int n = 0; n < 3; n++) {
    for (int p = 0; p < 3; p++) {
      temp[n][p] = 0;
      for (int j = 0; j < 3; j++) {
        temp[n][p] += a[n][j] * b[j][p];
      }
    }
  }
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      dest[x][y] = temp[x][y];
    }
  }
}

void mulMatrix4(float dest[][4], float a[][4], float b[][4]) {
  float temp[4][4];
  for (int n = 0; n < 4; n++) {
    for (int p = 0; p < 4; p++) {
      temp[n][p] = 0;
      for (int j = 0; j < 4; j++) {
        temp[n][p] += a[n][j] * b[j][p];
      }
    }
  }
  for (int x = 0; x < 4; x++) {
    for (int y = 0; y < 4; y++) {
      dest[x][y] = temp[x][y];
    }
  }
}

//calculates the homogenized (4x4) transform matrix
//given a rotation and a length
void getHomogenized(float dest[][4], float rotation[][3], float length) {
  //calculate pi
  vec3 pi;
  vec3 temp_length;
  temp_length.x = length;
  temp_length.y = 0;
  temp_length.z = 0;

  mulMatrixVector(&pi, rotation, &temp_length);

  //set rotation parts of Xi
  for (int x = 0; x < 3; x++){
    for (int y = 0; y < 3; y++) {
      dest[x][y] = rotation[x][y];
    }
  }
  //set translation (pi) parts of Xi
  dest[0][3] = pi.x;
  dest[1][3] = pi.y;
  dest[2][3] = pi.z;
  //set corner to 1 - we apply this to points not directions
  dest[3][3] = 1;

  //zero bottom parts
  dest[3][0] = 0;
  dest[3][1] = 0;
  dest[3][2] = 0;
}

void applyTransform(vec3* dest, float a[][4], vec3* b) {
  //a = i,j (4x4)
  //b = j,1 (4x1)
  //result = i,1 (4x1)=>cut to (3x1)
  vec3 temp;
  float val;
  float scl = 1.0f;
  for (int i = 0; i < 4; i++) {
    val = 0;
    //iterate j from 0 to 3
    val += a[i][0] * b->x;
    val += a[i][1] * b->y;
    val += a[i][2] * b->z;
    val += a[i][3];
    if (i == 0) {
      temp.x = val;
    }
    else if (i == 1) {
      temp.y = val;
    }
    else if (i == 2) {
      temp.z = val;
    }
    else {
      //should be 1
      scl = val;
    }
  }
  scale(dest, &temp, 1 / scl);
}

void crossProductMatrix(float dest[][3], vec3* a) {
  dest[0][0] = 0;
  dest[0][1] = -a->z;
  dest[0][2] = a->y;

  dest[1][0] = a->z;
  dest[1][1] = 0;
  dest[1][2] = -a->x;

  dest[2][0] = -a->y;
  dest[2][1] = a->x;
  dest[2][2] = 0;
}

void visualizeVector(vec3* a) {
  cout << a->x << "," << a->y << "," << a->z << "\n";
}

void visualize(float matrix[][3]) {
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      cout << matrix[x][y] << ",";
    }
    cout << "\n";
  }
}

void solveForDP(float dr[], float** jac, vec3* dp, int n) {
  
  //first convert to MatrixXd form
  Eigen::MatrixXd jacobian(3, 3*n);
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3*n; y++) {
      jacobian(x, y) = jac[x][y];
    }
  }

  Eigen::MatrixXd dp_mat(3, 1);
  dp_mat(0, 0) = dp->x;
  dp_mat(1, 0) = dp->y;
  dp_mat(2, 0) = dp->z;

  //find U, Sigma, V (JAC = U Sigma V*)
  Eigen::MatrixXd dr_mat = jacobian.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(dp_mat);
  //JacobiSVD<Eigen::MatrixXd> svd(m, ComputeThinU | ComputeThinV);

  //cout << "jacobian:" << endl << jacobian << endl;
  //cout << "answer:" << endl << dr_mat << endl;

  for (int i = 0; i < 3 * n; i++) {
    dr[i] = dr_mat(i, 0);
  }
}

//****************************************************
// Meat of the assignment
//****************************************************

//one iteration of the joint algorithm
//should be called by my myDisplay method
//updates rotations[]
void updateJoint(vec3 rotations[], vec3* end_effector, vec3* goal, float rotation_matrices[][3][3], float transformation_matrices[][4][4], vec3* pn, int n, float lambda) {
  //find jacobian

  float** jac;
  jac = new float*[3];
  for (int i = 0; i < 3; i++) {
    jac[i] = new float[3*n];
  }

  calculateJacobian(jac, rotation_matrices, transformation_matrices, pn, n);

  // dp = lambda * (g - pe)
  vec3 dp;
  subtract(&dp, goal, end_effector);
  scale(&dp, &dp, lambda);

  //we will solve for this
  float dr[3*joint_count];

  //get pseudoinverse
  solveForDP(dr, jac, &dp, n); 
  
  /*
  //find dr
  //dr is stored in an arraylist called dr_lst
  finddr(pseudo_inverse, &temp, n);*/
  
  //update rotations
  for(int i = 0; i < 3*n; i++) {
    if(i % 3 == 0) {
      rotations[i / 3].x += dr[i];
    }
    else if(i % 3 == 1) {
      rotations[i / 3].y += dr[i];
    }
    else {
      rotations[i / 3].z += dr[i];
    }
  }

  for (int i = 0; i < 3; i++) {
    delete[] jac[i];
  }
  delete[] jac;
  
  //find pi (the positions of each joint)
  //render in openGL
}

//given an array of rotations (exp maps in vec3), and an array of lengths
//calculates rotation_matrices (rodriguez matrices for rotation)
//calculates pi_vectors (individual joint displacements)

void updateCalculations(float rotation_matrices[][3][3], vec3 pi_vectors[], 
  float transformation_matrices[][4][4], vec3 rotations[], float lengths[], int n) {

  //calculate all Ri rotation matrices
  for (int i = 0; i < n; i++) {
    calculateRi(rotation_matrices[i], &(rotations[i]));
  }

  //calculates all pi vectors
  vec3 temp_length;
  for (int i = 0; i < n; i++) {
    temp_length.x = lengths[i];
    temp_length.y = 0;
    temp_length.z = 0;
    mulMatrixVector(&(pi_vectors[i]), rotation_matrices[i], &temp_length);
  }

  //calculate all X_i's
  for (int i = 0; i < n; i++) {
    getHomogenized(transformation_matrices[i], rotation_matrices[i], lengths[i]);
  }
}

void initializeParameters() {

  //initialize rotations + setup stuff relevant to this assignment in particular
  //replace the below with some arg passing
  //these are default system params
  const int n = joint_count;
  lengths = (float*)malloc(sizeof(float) * joint_count);
  lengths[0] = 1;
  lengths[1] = 2;
  lengths[2] = 2;
  lengths[3] = 1;

  // array of rotations; size n
  vec3 rotations[joint_count];
  vec3 zero;
  zero.x = 0;
  zero.y = 0;
  zero.z = 0;

  for (int i = 0; i < n; i++) {
    set(&(rotations[i]), &zero);
  }

  vec3 goals[goal_count];
  for (int i = 0; i < goal_count; i++) {
	  goals[i] = {(float)(7*sin(2*i*PI/goal_count)), (float)(4*cos(2*i*PI/goal_count)), 0};
  }


  rotation_frames.clear();

  /*vector<vec3> temp_initial;
  for (int f = 0; f < n; f++) {
    temp_initial.push_back(rotations[f]);
  }
  rotation_frames.push_back(temp_initial);*/

  for (int i = 0; i < goal_count; i++) {
    vector<vec3> temp;
    calculateRotations(rotations, &(goals[i]), n);
    //copy into our frames
    for (int f = 0; f < n; f++) {
      temp.push_back(rotations[f]);
    }
    rotation_frames.push_back(temp);
  }
}

void calculateRotations(vec3 rotations[], vec3* goal, int n) {

  float epsilon = 0.01f;
  float lambda = 0.001f;

  int max_iter = 7000;

  //------below this line is all calculated from above stuff

  //variable to hold all Rodriguez rotation matrices
  float rotation_matrices[joint_count][3][3];

  //variable to hold all pi displacement vectors
  vec3 pi_vectors[joint_count];

  //variable to hold all Xi transformation matrices
  float transformation_matrices[joint_count][4][4];

  updateCalculations(rotation_matrices, pi_vectors, transformation_matrices, rotations, lengths, n);

  vec3 end_effector;
  findEndEffector(&end_effector, rotation_matrices, transformation_matrices, &(pi_vectors[n-1]), n);
  //cout << "initial" << endl;
  //visualizeVector(&end_effector);

  int iter_count = 0;
  while((!reachedGoal(&end_effector, goal, lengths, n, epsilon)) && iter_count < max_iter) {
    updateJoint(rotations, &end_effector, goal, rotation_matrices, transformation_matrices, &(pi_vectors[n-1]), n, lambda);
    updateCalculations(rotation_matrices, pi_vectors, transformation_matrices, rotations, lengths, n);
    //save points from updateCalculations to allpoints
    
    findEndEffector(&end_effector, rotation_matrices, transformation_matrices, &(pi_vectors[n-1]), n);

    //cout << "next" << endl;
    //visualizeVector(&end_effector);

    iter_count += 1;
  }
  //cout << "end" << endl;
  //visualizeVector(&end_effector);
  //cout << "iters " << iter_count << endl;
}

bool reachedGoal(vec3* end_effector, vec3* goal, float lengths[], int n, float epsilon) {
  vec3 temp;
  subtract(&temp, end_effector, goal);
  float dist = magnitude(&temp);
  
  //check to see if goal is too far
  float maxreach = 0.0f;
  for(int i = 0; i < n; i++) {
    maxreach += lengths[i];
  }
  float goalreach = magnitude(goal);

  return dist < epsilon;
}

void findRealCoordinates(vec3 positions[], vec3 rotations[], float lengths[], int n) {
  //used in real time to calculate joint position based on current rotations
  float rotation_matrices[joint_count][3][3];

  //variable to hold all pi displacement vectors
  vec3 pi_vectors[joint_count];

  //variable to hold all Xi transformation matrices
  float transformation_matrices[joint_count][4][4];

  updateCalculations(rotation_matrices, pi_vectors, transformation_matrices, rotations, lengths, n);

  float x_n_to_zero[4][4];
  for (int i = 0; i < n; i++) {
    setIdentity4(x_n_to_zero);
    for (int k = 0; k < i; k++) {
      mulMatrix4(x_n_to_zero, x_n_to_zero, transformation_matrices[k]);
    }
    applyTransform(&(positions[i]), x_n_to_zero, &(pi_vectors[i]));
  }
}

void findEndEffector(vec3* end_effector, float rotation_matrices[][3][3], float transformation_matrices[][4][4], vec3* pn, int n) { 
  float x_n_to_zero[4][4];
  //location of end effector given by X_n->0 * p_n
  setIdentity4(x_n_to_zero);
  for (int k = 0; k < n-1; k++) {
    mulMatrix4(x_n_to_zero, x_n_to_zero, transformation_matrices[k]);
  }
  // X_n->i * p_n
  applyTransform(end_effector, x_n_to_zero, pn);
}

//calculates the rotation matrix 3x3 from an exponential map vector
//via the Rodriguez formula
void calculateRi(float dest[][3], vec3* rotation) {

  //R = ((r rt)+sin(theta)(rx)-cos(theta)(rx)(rx))
  //OR: R = I + (sin0)K+(1-cos0)K^2

  vec3 axis;
  set(&axis, rotation);

  float theta = fmod(magnitude(rotation), 2*PI);
  if (theta != 0) {
    normalize(&axis);
  }
  else {
    axis.x = 1.0f;
    axis.y = 0.0f;
    axis.z = 0.0f;
  }

  float temp[3][3];

  float rax[3][3];
  rax[0][0] = 0.0f;
  rax[0][1] = -axis.z;
  rax[0][2] = axis.y;
  rax[1][0] = axis.z;
  rax[1][1] = 0.0f;
  rax[1][2] = -axis.x;
  rax[2][0] = -axis.y;
  rax[2][1] = axis.x;
  rax[2][2] = 0.0f;

  float sint = sin(theta);
  float oneMinusCost = 1.0f - cos(theta);

  //dest = I
  setIdentity(dest);
  //temp = sint*(Rx)
  scaleMatrix(temp, rax, sint);
  //dest = I + sint*(Rx)
  addMatrix(dest, dest, temp);
  //temp = (Rx)(Rx)
  mulMatrix3(temp, rax, rax);

  //temp = (1-cost)*(Rx)^2
  scaleMatrix(temp, temp, oneMinusCost);
  //dest = I + sint*(Rx) + (1-cost)*(Rx)^2
  addMatrix(dest, dest, temp);
}

void calculateJacobian(float** jac, float rotation_matrices[][3][3], float transformation_matrices[][4][4], vec3* pn, int n) {
  //temp J_i var used to hold part of Jacobian at each part
  float ji[3][3];
  
  float r_i_to_zero[3][3];
  float x_n_to_i[4][4];
  vec3 crossMeAndYouDie;
  float cpMatrix[3][3];
  for (int i = 0; i < n; i++) {
    //ji = -R_(i->0) cross[X_(n->i) p_n]
    //calculate R_(i->0)
    setIdentity(r_i_to_zero);
    for (int k = i - 1; k >= 0; k--) {
      mulMatrix3(r_i_to_zero, r_i_to_zero, rotation_matrices[k]);
    }

    setIdentity4(x_n_to_i);
    for (int k = i; k < n-1; k++) {
      mulMatrix4(x_n_to_i, x_n_to_i, transformation_matrices[k]);
    }
    // X_n->i * p_n
    applyTransform(&crossMeAndYouDie, x_n_to_i, pn);

    // cross[ X_n->i * p_n ]
    crossProductMatrix(cpMatrix, &crossMeAndYouDie);

    // R_(i->0) cross[X_(n->i) p_n]
    mulMatrix3(cpMatrix, r_i_to_zero, cpMatrix);

    //add a minus
    scaleMatrix(cpMatrix, cpMatrix, -1.0f);

    //add result to our jacobian
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        jac[x][i * 3 + y] = cpMatrix[x][y];
      }
    }
  }
}


//****************************************************
// function that does the actual drawing of stuff
//***************************************************

void drawSphere(vec3* pos) {
  glPushMatrix();
      glTranslated(pos->x,pos->y,pos->z);
      glutSolidSphere(0.1,30,30); //radius,slices,stacks
  glPopMatrix();
}

//taken shamelessly from http://lifeofaprogrammergeek.blogspot.com/2008/07/rendering-cylinder-between-two-points.html
void renderCylinder(vec3* a, vec3* b, float radius, int subdivisions)
{
  float x1 = a->x;
  float y1 = a->y;
  float z1 = a->z;

  float x2 = b->x;
  float y2 = b->y;
  float z2 = b->z;

  float vx = x2-x1;
  float vy = y2-y1;
  float vz = z2-z1;

  //handle the degenerate case of z1 == z2 with an approximation
  if(vz == 0)
      vz = .0001;

  float v = sqrt( vx*vx + vy*vy + vz*vz );
  float ax = 57.2957795*acos( vz/v );
  if ( vz < 0.0 )
      ax = -ax;
  float rx = -vy*vz;
  float ry = vx*vz;
  glPushMatrix();

  //draw the cylinder body
  glTranslatef( x1,y1,z1 );
  glRotatef(ax, rx, ry, 0.0);
  gluQuadricOrientation(quadric,GLU_OUTSIDE);
  gluCylinder(quadric, radius, radius, v, subdivisions, 1);

  //draw the first cap
  gluQuadricOrientation(quadric,GLU_INSIDE);
  gluDisk( quadric, 0.0, radius, subdivisions, 1);
  glTranslatef( 0,0,v );

  //draw the second cap
  gluQuadricOrientation(quadric,GLU_OUTSIDE);
  gluDisk( quadric, 0.0, radius, subdivisions, 1);
  glPopMatrix();
}

void drawSphereJoints() {
  //temporary to hold
  vec3 rotations[joint_count];
  vec3 current, next;

  vec3 one;
  one.x = 1;
  one.y = 0;
  one.z = 0;

  float real_increment = current_increment;
  if (real_increment < 0)
    real_increment = 0;

  //calculate rotation vectors for this frame
  float cur_mag, next_mag, new_mag;
  float angle_rot;
  float dot_prod;
  for (int i = 0; i < joint_count; i++) {
    //rotation from previous frame
    set(&current, &(rotation_frames.at(checkpoint_counter % goal_count).at(i)));
    cur_mag = magnitude(&current);
    if (cur_mag != 0)
      normalize(&current);
    else
      set(&current, &one);

    set(&next, &(rotation_frames.at( (checkpoint_counter + 1 ) % goal_count).at(i)));
    next_mag = magnitude(&next);
    if (next_mag != 0)
      normalize(&next);
    else
      set(&next, &one);

    dot_prod = dotProduct(&next, &current);
    if (dot_prod < 0) {
      scale(&next, &next, -1);
      next_mag = fmod(2*PI-next_mag, 2*PI);
    }

    angle_rot = fmod(next_mag - cur_mag + 2*PI, 2*PI);
    if (angle_rot > PI) {
      angle_rot = angle_rot - (2*PI);
    }

    new_mag = fmod((cur_mag + real_increment * (angle_rot)), (2*PI));

    //difference in rotation vector
    subtract(&next, &next, &current);
    //interpolate difference by current icnrement
    scale(&next, &next, real_increment);
    //add to current
    add(&(rotations[i]), &current, &next);
    normalize(&(rotations[i]));
    scale(&(rotations[i]), &(rotations[i]), new_mag);
  }

  vec3 zero = {0, 0, 0};

  drawSphere(&zero);

  vec3 positions[joint_count];
  findRealCoordinates(positions, rotations, lengths, joint_count);
  for (int i = 0; i < joint_count; i++) {
    drawSphere(&(positions[i]));
    if (i != 0) {
      renderCylinder(&(positions[i]), &(positions[i-1]), 0.05, 30);
    }
    else {
      renderCylinder(&(positions[i]), &zero, 0.05, 30);
    }
  }
}

void timerFunc(int v) {
  if (!paused) {
    glutPostRedisplay();
  }
  glutTimerFunc(50,timerFunc, v + 1);
}

void myDisplay() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);             // indicate we are specifying camera transformations
  glLoadIdentity();               // make sure transformation is "zero'd"

  /*float xpos = vdistance * cos(angle) * cos(z_angle);
  float ypos = vdistance * sin(angle) * cos(z_angle);
  float zpos = vdistance * sin(z_angle);*/
  float xpos = 10;
  float ypos = 10;
  float zpos = 10;
  
  gluLookAt(xpos, ypos, zpos,     // eye position
            0.0f, 0.0f, 0.0f,     // where to look at
            0.0f, 0.0f, 1.0f);    // up vector*/


  current_increment += increment_amount;
  if (current_increment >= 1.0f) {
    //move to next
    checkpoint_counter += 1;
    //checkpoint_counter = checkpoint_counter % goal_count;
    current_increment = -0.1f;
  }
  drawSphereJoints();

  //go through allpoints, render balls + joints

  glFlush();
  glutSwapBuffers();          // swap buffers (we earlier set double buffer)
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {

  //testFunction();

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

  initializeParameters();

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

  gluQuadricNormals(quadric, GLU_SMOOTH);

  glutTimerFunc(50, timerFunc, 1);

  glutMainLoop();             // infinite loop that will keep drawing and resizing
  // and whatever else

  return 0;
}