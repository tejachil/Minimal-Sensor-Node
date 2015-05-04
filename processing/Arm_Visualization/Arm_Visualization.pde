import processing.serial.*;
final String SERIAL_DEV = "/dev/ttyUSB0"; 

Serial serialPort;

float[] front_arm = {250,150,200,100};
float[] front_forearm = {front_arm[0],front_arm[1],200,0};

float[] side_arm = {600,150,200,100};
float[] side_forearm = {side_arm[0],side_arm[1],200,0};
float size = 100;
float arm_forarm_ratio = 1.4;

float [][] q = new float [3][4];
float [][] Euler = new float [3][3]; // psi, theta, phi

float [] qArm = new float [4];
float [] qForeArm = new float [4];

float [] eulerArm = new float [3];
float [] eulerForeArm = new float [3];
 
void setup(){
  size(800,400);
  stroke(255);
  
  serialPort = new Serial(this, SERIAL_DEV, 115200);
}

void serialEvent(Serial p) {
  if(p.available() >= 100) {
    
    String inputString = p.readStringUntil('\n');
    
    if (inputString != null && inputString.length() > 0) {
      String [] eachNodeStringArray = split(inputString, ", ");
      for(int i = 0; i < eachNodeStringArray.length; ++i){
        String [] inputStringArr = split(eachNodeStringArray[i], " ");
        if(inputStringArr.length >= 4) { // q1,q2,q3,q4,\r\n so we have 5 elements
          q[i][0] = Float.parseFloat(inputStringArr[0]);
          q[i][1] = Float.parseFloat(inputStringArr[1]);
          q[i][2] = Float.parseFloat(inputStringArr[2]);
          q[i][3] = Float.parseFloat(inputStringArr[3]);
        }
      }
      println("q0: " + q[0][0] + " q1: " + q[0][1] + " q2: " + q[0][2] + " q3: " + q[0][3]);
      qArm = quatProd(quatConjugate(q[0]), q[1]);
      qForeArm = quatProd(quatConjugate(q[0]), q[2]);
      quaternionToEuler(qArm, eulerArm);
      quaternionToEuler(qForeArm, eulerForeArm);
    }
  }
}

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));//atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = asin(2*(q[0]*q[2]-q[3]*q[1]));//-asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));//atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}
 
void draw(){
  background(51);
  updateSideView();
  
  updateFrontView();
}

void updateFrontView(){
  fill(0);
  noStroke();
  rect(front_arm[0]-10, front_arm[1]-10, 100, 200, 30, 30, 0, 0);
  fill(150);
  ellipse(front_arm[0]+45, 45, 70, 70);
  
  updateFrontArm();
  updateFrontForearm();
  // draw Arm
  strokeWeight(4);
  stroke(200, 0, 0);
  line(front_arm[0],front_arm[1],front_arm[2],front_arm[3]);
  // draw elbow
  ellipse(front_forearm[0],front_forearm[1],10,10);
  
  // draw Forearm
  stroke(0, 0, 200);
  line(front_forearm[0],front_forearm[1],front_forearm[2],front_forearm[3]);
  // draw hand
  ellipse(front_forearm[2],front_forearm[3],10,10);
}

void updateSideView(){
  fill(0);
  noStroke();
  rect(side_arm[0]-20, side_arm[1]-20, 40, 200, 10, 10, 0, 0);
  fill(150);
  ellipse(side_arm[0], 45, 70, 70);
  
  updateSideArm();
  updateSideForearm();
  // draw Arm
  strokeWeight(4);
  stroke(200, 0, 0);
  line(side_arm[0],side_arm[1],side_arm[2],side_arm[3]);
  // draw elbow
  ellipse(side_forearm[0],side_forearm[1],10,10);
  
  // draw Forearm
  stroke(0, 0, 200);
  line(side_forearm[0],side_forearm[1],side_forearm[2],side_forearm[3]);
  // draw hand
  ellipse(side_forearm[2],side_forearm[3],10,10);
}

void updateFrontForearm(){
 float x2 = (-cos(eulerForeArm[2]) * size*arm_forarm_ratio) + front_arm[2];
 float y2 = (-sin(eulerForeArm[2]) * size*arm_forarm_ratio) + front_arm[3];
 front_forearm[0] = front_arm[2];
 front_forearm[1] = front_arm[3];
 front_forearm[2] = x2;
 front_forearm[3] = y2;
}
 
void updateFrontArm(){
 float x = (-cos(eulerArm[2]) * size) + front_arm[0];
 float y = (-sin(eulerArm[2]) * size) + front_arm[1];
 front_arm[2] = x;
 front_arm[3] = y;
}
 
void updateSideForearm(){
 float x2 = (cos(-eulerForeArm[0]) * size*arm_forarm_ratio) + side_arm[2];
 float y2 = (sin(-eulerForeArm[0]) * size*arm_forarm_ratio) + side_arm[3];
 side_forearm[0] = side_arm[2];
 side_forearm[1] = side_arm[3];
 side_forearm[2] = x2;
 side_forearm[3] = y2;
}
 
void updateSideArm(){
 float x = ( cos(-eulerArm[0]) * size) + side_arm[0];
 float y = ( sin(-eulerArm[0]) * size) + side_arm[1];
 side_arm[2] = x;
 side_arm[3] = y;
}

float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];
  
  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];
  
  return conj;
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];
  
  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  
  return q;
}


