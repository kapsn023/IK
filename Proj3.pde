//Root
Vec2 root1 = new Vec2(200,200);
Vec2 root2 = new Vec2(500, 300);
float radius = 25.0;

int numObs = 3;
Vec2[] locations = new Vec2[numObs];
float[] radii = new float[numObs];

//Upper Arm
float l0 = 75; 
float a0 = 0.3; //Shoulder joint
float limit0 = 3.2;

//Lower Arm
float l1 = 75;
float a1 = 0.3; //Elbow joint
float limit1 = 1.58;

//Hand
float l2 = 75;
float a2 = 0.3; //Wrist joint
float limit2 = 1.58;

float l3 = 75;
float a3 = 0.3;
float limit3 = 1.58;

float l4 = 75;
float a4 = 0.3;
float limit4 = 3.2;

Vec2 start_l1,start_l2,start_l3,endPoint1, endPoint2;


boolean isTarget1 = false;

void setup(){
  size(1280,920);
  surface.setTitle("Inverse Kinematics");
  
  endPoint1 = root1;
  endPoint2 = root2;
  
  locations[0] = new Vec2 (700, 300);
  locations[1] = new Vec2 (300, 100);
  locations[2] = new Vec2 (300, 500);
  
  radii[0] = 100;
  radii[1] = 100;
  radii[2] = 100;
  
}

void mouseDragged(){
  Vec2 mouse_pos = new Vec2(mouseX, mouseY);
  Vec2 old_mouse_pos = new Vec2(pmouseX, pmouseY);
  
  if (mouse_pos.distanceTo(root1) < radius) {
    if (!isTarget1){
      root2 = endPoint2;
      a1 = -a1;
      a2 = -a2;
      a3 = -a3;
    }
    root1.add(mouse_pos.minus(old_mouse_pos));
    isTarget1 = true;
  }
  else if (mouse_pos.distanceTo(root2) < radius) {
    if (isTarget1){
      root1 = endPoint1;
      a1 = -a1;
      a2 = -a2;
      a3 = -a3;
    }
    root2.add(mouse_pos.minus(old_mouse_pos));
    isTarget1 = false;
  }
}

boolean isPointInsideCircle(Vec2 c, float r, Vec2 p) {
  float distance = c.distanceTo(p);
  return distance < r; //<>//
}

float calculateClosestPointX(Vec2 c, Vec2 p1, Vec2 p2) {
  float cx = c.x;
  float x1 = p1.x;
  float x2 = p2.x;
  return (cx < x1) ? x1 : (cx > x2) ? x2 : cx;
  }

float calculateClosestPointY(Vec2 c, Vec2 p1, Vec2 p2) {
  float y1 = p1.y;
  float y2 = p2.y;
  float x1 = p1.x;
  float x2 = p2.x;
  float m = (y2 - y1) / (x2 - x1);
  float b = y1 - m * x1;
  float closestX = calculateClosestPointX(c, p1, p2);
  return m * closestX + b;
  }

boolean armCircleCollision(){
  //return false;
  for (int i = 0; i < numObs; i++){
    float r = radii[i];
    Vec2 c = locations[i];
    Vec2 p1 = endPoint1;
    Vec2 p2 = start_l1;
    if (isPointInsideCircle(c, r, p1) || isPointInsideCircle(c, r, p2)) {
      println("Point collision");
      return true;
    }
    float closestX = calculateClosestPointX(c, p1, p2);
    float closestY = calculateClosestPointY(c, p1, p2);
    float distance = (float) Math.sqrt((closestX - c.x) * (closestX - c.x) + (closestY - c.y) * (closestY - c.y));
    if (distance < r){
      println("Line collision");
      return true;
    }
    
    p1 = start_l1;
    p2 = start_l2;
    if (isPointInsideCircle(c, r, p1) || isPointInsideCircle(c, r, p2)) {
      println("Point collision");
      return true;
    }
    closestX = calculateClosestPointX(c, p1, p2);
    closestY = calculateClosestPointY(c, p1, p2);
    distance = (float) Math.sqrt((closestX - c.x) * (closestX - c.x) + (closestY - c.y) * (closestY - c.y));
    if (distance < r){
      println("Line collision");
      return true;
    }
    
    p1 = start_l2;
    p2 = start_l3;
    if (isPointInsideCircle(c, r, p1) || isPointInsideCircle(c, r, p2)) {
      println("Point collision");
      return true;
    }
    closestX = calculateClosestPointX(c, p1, p2);
    closestY = calculateClosestPointY(c, p1, p2);
    distance = (float) Math.sqrt((closestX - c.x) * (closestX - c.x) + (closestY - c.y) * (closestY - c.y));
    if (distance < r){
      println("Line collision");
      return true;
    }
    
    p1 = start_l3;
    p2 = endPoint2;
    if (isPointInsideCircle(c, r, p1) || isPointInsideCircle(c, r, p2)) {
      println("Point collision");
      return true;
    }
    closestX = calculateClosestPointX(c, p1, p2);
    closestY = calculateClosestPointY(c, p1, p2);
    distance = (float) Math.sqrt((closestX - c.x) * (closestX - c.x) + (closestY - c.y) * (closestY - c.y));
    if (distance < r){
      println("Line collision");
      return true;
    }
    
  }
  return false;
}

void solve(){
  //root2 = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  boolean go = true;
  
  if (!isTarget1){
    
    startToGoal = root2.minus(start_l3);
    startToEndEffector = endPoint2.minus(start_l3);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a3 = a3;
    while (go || armCircleCollision()){
      go = false;
      a3 = old_a3;
      if (cross(startToGoal,startToEndEffector) < 0)
        a3 += angleDiff;
      else
        a3 -= angleDiff;
      /*TODO: Wrist joint limits here*/
      a3 = clamp(a3, -limit3, limit3);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
    go = true;
    
    //Update wrist joint
    startToGoal = root2.minus(start_l2);
    startToEndEffector = endPoint2.minus(start_l2);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a2 = a2;
    while (go || armCircleCollision()){
      go = false;
      a2 = old_a2;
      if (cross(startToGoal,startToEndEffector) < 0)
        a2 += angleDiff;
      else
        a2 -= angleDiff;
      /*TODO: Wrist joint limits here*/
      a2 = clamp(a2, -limit2, limit2);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
    
    go = true;
    //Update elbow joint
    startToGoal = root2.minus(start_l1);
    startToEndEffector = endPoint2.minus(start_l1);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a1 = a1;
    while (go || armCircleCollision()){
      go = false;
      a1 = old_a1;
      if (cross(startToGoal,startToEndEffector) < 0)
        a1 += angleDiff;
      else
        a1 -= angleDiff;
      a1 = clamp(a1, -limit1, limit1);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
    
    go = true;
    //Update shoulder joint
    startToGoal = root2.minus(root1);
    if (startToGoal.length() < .0001) return;
    startToEndEffector = endPoint2.minus(root1);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a0 = a0;
    while (go || armCircleCollision()){
      go = false;
      a0 = old_a0;
      if (cross(startToGoal,startToEndEffector) < 0)
        a0 += angleDiff;
      else
        a0 -= angleDiff;
      /*TODO: Shoulder joint limits here*/
      a0 = clamp(a0, -limit0, limit0);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
   
    println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2,"Angle 3:",a3);
  }
  else{
    go = true;
    //Update elbow joint
    startToGoal = root1.minus(start_l1);
    startToEndEffector = endPoint1.minus(start_l1);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a1 = a1;
    while (go || armCircleCollision()){
      go = false;
      a1 = old_a1;
      if (cross(startToGoal,startToEndEffector) < 0)
        a1 += angleDiff;
      else
        a1 -= angleDiff;
      a1 = clamp(a1, -limit1, limit1);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
    go = true;
    //Update wrist joint
    startToGoal = root1.minus(start_l2);
    startToEndEffector = endPoint1.minus(start_l2);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a2 = a2;
    while (go || armCircleCollision()){
      go = false;
      a2 = old_a2;
      if (cross(startToGoal,startToEndEffector) < 0)
        a2 += angleDiff;
      else
        a2 -= angleDiff;
      /*TODO: Wrist joint limits here*/
      a2 = clamp(a2, -limit2, limit2);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
    go = true;
    startToGoal = root1.minus(start_l3);
    startToEndEffector = endPoint1.minus(start_l3);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a3 = a3;
    while (go || armCircleCollision()){
      go = false;
      a3 = old_a3;
      if (cross(startToGoal,startToEndEffector) < 0)
        a3 += angleDiff;
      else
        a3 -= angleDiff;
      /*TODO: Wrist joint limits here*/
      a3 = clamp(a3, -limit3, limit3);
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= 0.5;
    }
    
    go = true;
    //Update shoulder joint
    startToGoal = root1.minus(root2);
    if (startToGoal.length() < .0001) return;
    startToEndEffector = endPoint1.minus(root2);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    float old_a4 = a4;
    while (go || armCircleCollision()){
      go = false;
      a4 = old_a4;
      if (cross(startToGoal,startToEndEffector) < 0)
        a4 += angleDiff;
      else
        a4 -= angleDiff;
      /*TODO: Shoulder joint limits here*/
      fk(); //Update link positions with fk (e.g. end effector changed)
      a0 = clamp(a0, -limit0, limit0);
      angleDiff *= 0.5;
    }
    
   
    println("Angle 4:",a4,"Angle 3:",a3,"Angle 2:",a2);
  }
}

void fk(){
  if (!isTarget1){
    start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root1);
    start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
    start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
    endPoint2 = new Vec2(cos(a0+a1+a2+a3)*l2,sin(a0+a1+a2+a3)*l2).plus(start_l3);
  }
  else{
    start_l3 = new Vec2(cos(a4)*l4,sin(a4)*l4).plus(root2);
    start_l2 = new Vec2(cos(a4+a3)*l3,sin(a4+a3)*l3).plus(start_l3);
    start_l1 = new Vec2(cos(a4+a3+a2)*l2,sin(a4+a3+a2)*l2).plus(start_l2);
    endPoint1 = new Vec2(cos(a3+a1+a2+a4)*l4,sin(a4+a1+a2+a3)*l4).plus(start_l1);
  }
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  
  fill(180, 50, 200);
  for (int i = 0; i < numObs; i++){
    pushMatrix();
  translate(locations[i].x, locations[i].y);
  circle(0.0, 0.0, radii[i]*2.0);
  popMatrix();
  }
  

  fill(200,100,150);
  if (!isTarget1){
    pushMatrix();
    translate(root1.x,root1.y);
    rotate(a0);
    rect(0, -armW/2, l0, armW);
    popMatrix();
    
    pushMatrix();
    translate(start_l1.x,start_l1.y);
    rotate(a0+a1);
    rect(0, -armW/2, l1, armW);
    popMatrix();
    
    pushMatrix();
    translate(start_l2.x,start_l2.y);
    rotate(a0+a1+a2);
    rect(0, -armW/2, l2, armW);
    popMatrix();
    
    pushMatrix();
    translate(start_l3.x,start_l3.y);
    rotate(a0+a1+a2+a3);
    rect(0, -armW/2, l3, armW);
    popMatrix();
  }
  else {
    pushMatrix();
    translate(root2.x,root2.y);
    rotate(a4);
    rect(0, -armW/2, l4, armW);
    popMatrix();
    
    pushMatrix();
    translate(start_l3.x,start_l3.y);
    rotate(a4+a3);
    //circle(0.0, 0.0, radius);
    rect(0, -armW/2, l3, armW);
    popMatrix();
    
    pushMatrix();
    translate(start_l2.x,start_l2.y);
    rotate(a4+a3+a2);
    //circle(0.0, 0.0, radius);
    rect(0, -armW/2, l2, armW);
    popMatrix();
    
    pushMatrix();
    translate(start_l1.x,start_l1.y);
    rotate(a4+a3+a2+a1);
    //circle(0.0, 0.0, radius);
    rect(0, -armW/2, l1, armW);
    popMatrix();
  }
  
  line(endPoint1.x, endPoint1.y, start_l1.x, start_l1.y);
  line(start_l1.x, start_l1.y, start_l2.x, start_l2.y);
  line(start_l2.x, start_l2.y, start_l3.x, start_l3.y);
  line(start_l3.x, start_l3.y, endPoint2.x, endPoint2.y);
  
  fill(200,190,180);
  pushMatrix();
  translate(root1.x,root1.y);
  circle(0.0, 0.0, radius);
  popMatrix();
  
  pushMatrix();
  translate(root2.x,root2.y);
  circle(0.0, 0.0, radius);
  popMatrix();
    
}



//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
