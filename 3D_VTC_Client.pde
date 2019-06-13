
// 3D VTC by Don Moloney
// Adapted from: Daniel Shiffman
//               Kinect Point Cloud example

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

//trying to get the basic "render their head" function working.

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

//for networking  https://processing.org/reference/libraries/net/clientEvent_.html
import processing.net.*;
int dataIn;
Client c;
int lineCount=0;
KinectTracker tracker; //HT
// Kinect Library object
Kinect kinect;
float scale = 6.6; //HT

float CameraStartRadius = 300; //HT
float radius = CameraStartRadius; //HT
boolean mirror = true;
int minThresh = 100; //100 is good
int maxThresh = 830; //830 is good
boolean ir = false;
boolean isColour = true;
// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
// values calibrated 10.04.19. Could do with depth adjustment (parallax error).
int x_offset = 48;
float x_scale = 0.8577083;
int y_offset = 35;
float y_scale=0.8732691;
int x_colour;
int y_colour;
int offsetColour;
float cameraZ = 0;
float factor = 400;
int skip = 2; //.................................................................................................
int stroke_weight = 2*skip;
int[] depth = new int[307200/skip/skip+1];
int[] kColour = new int[307200/skip/skip+1];
int sceneCentre = floor(minThresh + 0.2*(maxThresh-minThresh));
int kinectwidth=640;
int kinectheight=480;
int offset=0;
int linesIn=0;
int oldmillis = 0;
float FPS = 0;

void setup() {
  for (int i=0;i<depth.length;i++){
  depth[i]=0;
  kColour[i]=0;
}
  size(800, 600, P3D);
  kinect = new Kinect(this); //HT (keep this.. ??)
  tracker = new KinectTracker(); //HT - this is causing problems. 

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  c = new Client(this, "192.168.0.95", 12345); // Replace with your server’s IP and port

c.write(1); //tells server we are ready to receive data
}

void draw() {
  background(100);
  
  tracker.track(); //HT
  tracker.display(); //HT
  PVector v1 = tracker.getPos(); //HT
  int lowlim = floor(270 - 150*scale); //HT
  int highlim = floor(270 + 150*scale); //HT
  float theta = map(v1.x, lowlim,highlim,-1,1); //HT 
  float thetaY = map(v1.y, lowlim,highlim,-1,1); //HT 
  //float theta = map(mouseX, 0,width,0,PI); // mouse control
  camera(-radius*theta, radius*thetaY, radius-400, 0, 0, sceneCentre, 0, 1, 0); //HT
  //camera(mouseX-width/2, -mouseY+height/2, cameraZ, 0, 0,sceneCentre, 0, 1, 0); //mouse control

int i_temp=0;
int k_temp=0;

  for (int x = 0; x < kinectwidth; x += skip) {
    for (int y = 0; y < kinectheight; y += skip) {
      offset = x + y*kinectwidth;
      //int rawDepth = depth[offset];
      int rawDepth = depth[i_temp];
      //int rawDepth= minThresh + int(random(0,400));
      //int rawColour = kColour[offset];
      
      i_temp++;
      int rawColour = kColour[k_temp];
        k_temp++;
      //int rawColour = int(random(0,255));
      if (rawDepth > minThresh && rawDepth < maxThresh ){
        
        PVector v = depthToWorld(x, y, rawDepth);
        stroke(255);
        pushMatrix();
        translate(v.x*factor, v.y*factor, v.z*factor);
        strokeWeight(stroke_weight);
        if (isColour){
          stroke(rawColour);
          
        } else{
          stroke(255);
        }
        point(0, 0);
        popMatrix();
      }
      
    }
  }
  if (frameCount%50==0){
    println("linesIn:"+linesIn);
    println("depth.length:" + depth.length);
    println("FPS: ",FPS);
  }
}


void clientEvent(Client c) {
  String input = c.readStringUntil('\n');
  
  if (input!=null){
    if (input.indexOf("depth")>-1){
      depth = int(split(input.substring(5,input.length()),' '));
      int newmillis = millis();
      FPS = 1000.0/(newmillis-oldmillis);
      oldmillis = millis();
    }
    
    if (input.indexOf("colour")>-1){
      kColour = int(split(input.substring(6,input.length()),' '));
    }
    
    c.write(1); //tells server we are ready for more data
    linesIn++;

  }
}
  



// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
  
  //send array... 
  //clear array/buffer
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}


void keyPressed() {
  if (key == 'i') {
    ir = !ir;
    kinect.enableIR(ir);
  }
  if (key == 'c') {
    isColour = !isColour;
  }
    if (key == 'w') {
    cameraZ += 10;
    println("cameraZ: ", cameraZ);
  }
    if (key == 's') {
    cameraZ += -10;
    println("cameraZ: ", cameraZ);
  }
    if (key == 'a') {
    x_offset += skip;
    println("X_offset: ", x_offset);
  }
    if (key == 'd') {
    x_offset += -skip;
    println("X_offset: ", x_offset);
  }
    if (key == '8') {
    y_scale *= 0.99;
    println("Y_scale: ", y_scale);
  }
    if (key == '5') {
    y_scale *= 1.01;
    println("Y_scale: ", y_scale);
  }
    if (key == '4') {
    stroke_weight+=-1;
    println("stroke_weight: ", stroke_weight);
  }
    if (key == '6') {
    stroke_weight++;
    println("stroke_weight: ", stroke_weight);
  }
  if(key == 'm'){
    mirror = !mirror;
    kinect.enableMirror(mirror);
  }
  if (key == 'o'){
      factor *= 2f;
      println(factor);
    } else if (key == 'p'){
      factor *= 0.5f;
      println(factor);
    } else{
      factor += 0;      
    }
}
