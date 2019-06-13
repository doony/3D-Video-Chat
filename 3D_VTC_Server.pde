
// 3D VTC by Don Moloney
// Adapted from: Daniel Shiffman
//               Kinect Point Cloud example

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

// this will send the data to the client... seems to be able to render at skip=8 pretty well
// skip = 8: 6Mbit
// skip=4: 2Mbit because the frame rate slows down

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

//for networking 
import processing.net.*;
Server s; 
Client c;
// Kinect Library object
Kinect kinect;
boolean mirror = true;
float minThresh = 100; //100 is good
float maxThresh = 830; //830 is good
boolean ir = false;
boolean isColour = true;
// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
// values calibrated 10.04.19. Could do with depth adjustment (parallax error).
int x_offset = 48;
float x_scale = 0.8577083;
int y_offset = 35;
float y_scale=0.89865094;
int linesOut=0;
boolean sendReady = true;
int oldmillis=0;

int x_colour;
int y_colour;
int offsetColour;
float cameraZ = 0;
// Scale up by 200
float factor = 400;
int skip = 2; // skip=1: 640x480 (max resolution of the kinect1). increase skip to decrease resolution. skip skip skip skip 
int stroke_weight = 2*skip;
int sceneCentre = floor(minThresh + 0.2*(maxThresh-minThresh));
boolean writeString = false;

String[] longDepthStringArray;
String[] longColourStringArray;

void setup() {
  // Rendering in P3D
  size(800, 600, P3D);
    
  kinect = new Kinect(this);
  kinect.initDepth();
  kinect.enableIR(!ir);
  kinect.enableIR(ir); // I don't know... this gets it going in colour immediately instead of having to hit 'i'
  
  longDepthStringArray = new String[int(kinect.width*kinect.height/skip/skip)+2];
  longColourStringArray= new String[int(kinect.width*kinect.height/skip/skip)+2];
   
  
  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  
  //networking:
 // frameRate(5); // Slow it down a little (the USB camera on the kinect doesn't like this line..... returns "USB Camera Marked Dead"
  s = new Server(this, 12345);  // Start a simple server on a port
  linesOut = 0;
}

void draw() {
  
  background(100);
  int i_step = 1;
  longDepthStringArray[0]="depth";
  longColourStringArray[0]="colour";
  
String bigStringColour = "";
String bigStringDepth = "";
  float theta = map(mouseX, 0,width,0,PI);
  camera(mouseX-width/2, -mouseY+height/2, cameraZ, 0, 0,sceneCentre, 0, 1, 0);
  // camera(mouseX-width/2, mouseY-height/2,  0, 0, 0, 35+minThresh*factor/200, 0, 1, 0); //this works. Pans left/right/up/down keeping eye on digidon's nose.
  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();
  PImage Kcolour = kinect.getVideoImage();

  //println("MouseX: ", mouseX, "MouseY: ", mouseY);
  //println("Kinect width: ", kinect.width, ", Kinect height: ",kinect.height);
if (sendReady){writeString = true;} // if the client has said, since we sent the last frame, that they're ready, then we'll actually start putting together another package for them this frame. 
                                    //But we don't want to get the 'sendReady' message half way through the frame and send a bad package.
  for (int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;
      x_colour = floor(x*x_scale + x_offset);
      y_colour = floor(y*y_scale + y_offset);
      if (x_colour > kinect.width-10 || x_colour < 10 ){
        offsetColour = 1;
      } else if (y_colour > kinect.height -10 || y_colour <10){
        offsetColour = 1;
      } else {
        offsetColour =  x_colour + y_colour*kinect.width;
      }
      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      int rawColour = Kcolour.pixels[offsetColour];
      if (writeString){ // we enable writeString at the start of the frame... basically if we haven't been asked for data *this* frame, we don't want to bog down with writing strings.
        
        
      }
      if (rawDepth > minThresh && rawDepth < maxThresh ){
        if (writeString){ // we enable writeString at the start of the frame... basically if we haven't been asked for data *this* frame, we don't want to bog down with writing strings.
        //bigStringColour += rawColour + " "; 
        //bigStringDepth += rawDepth + " ";
        longDepthStringArray[i_step] = str(rawDepth);
        longColourStringArray[i_step] = str(rawColour);
        i_step++;
      }
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
      } else {if (writeString){
        //bigStringDepth += 0 + " ";
        longDepthStringArray[i_step] = str(0);
        longColourStringArray[i_step] = str(0);
        i_step++;
      }} 
    }
  }

  if(sendReady && writeString){
    int newmillis = millis();
    longDepthStringArray[i_step] = "\n";
    longColourStringArray[i_step] = "\n";
    String longDepthStringFromArray = String.join(" ",longDepthStringArray); 
    String longColourStringFromArray = String.join(" ",longColourStringArray); 
    s.write(longDepthStringFromArray);
    delay(50);
    s.write(longColourStringFromArray);
    
    
    println(linesOut);
    println("FPS: ",1000.0/(newmillis-oldmillis));
    println("Depth.length: " + depth.length + " kColour.length: " + Kcolour.pixels.length);
    println("Depth String length: " + longDepthStringFromArray.length() + " Colour String Length: "+longColourStringFromArray.length());
    println("depth: ",bigStringDepth.length());
    println("colour: ",bigStringColour.length());
    
    sendReady = false;
    writeString = false;
    linesOut++;
    oldmillis = millis();
  } else {
  }
  

}

void clientEvent(Client client) {
  int msg = client.read();
  if (msg ==1) {
    sendReady=true;
  }
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
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
    stroke_weight+= -1;
  }
    if (key == '6') {
      stroke_weight++;
    
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
