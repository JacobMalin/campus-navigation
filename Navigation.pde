//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

// Navigation simulation

//Change the below parameters to change the display bounding geometry & paths/goals/scenario, respectively
boolean debug = false;
boolean goals = false;
boolean scene = true;
boolean points = false;
boolean photo = false;
int numNodes  = 100;
int numAgents = scene ? 100 : 20;

// Speeds and accel
float agentRad = scene ? 5 : 25;
float agentPad = scene ? 2 : 2;
float maxAcc = 1500;
float maxVel = 100; // In pixels per second, i think
float maxAngVel = 0.7 * TAU; // In radians per second, i think

float goalAccMagnitude = 0.6*maxAcc;
float obstacleAccMagnitude = 2000*maxAcc;
float agentAvoidAccMagnitude = 1*maxAcc;
float strongAgentAvoidAccMagnitude = 1*maxAcc;

// Distances
float agentMinDist = 2*agentRad + 2*agentPad;
float TTCMinDist = 200 + agentMinDist;
float closeToGoalDist = 8;

Vec2 agentPos[] = new Vec2[numAgents];
Vec2 agentVel[] = new Vec2[numAgents];
float agentAng[] = new float[numAgents];
float agentAngVel[] = new float[numAgents];
int agentPerson[] = new int[numAgents];
Vec2 goalPos[] = new Vec2[numAgents];

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

// Used to draw polygons easier
ArrayList<Vec2> pointArr = new ArrayList<Vec2>();

// Colors
color backgroundColor = #C5DD92;
color buildingColor = #D7816A;
color strokeColor = #1D1E2C;
color carpetColor = #D7E3EA;

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyShape = pointInShapeList(obstaclePoints, randPos, agentPad + agentRad);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyShape){
      randPos = new Vec2(random(width),random(height));
      insideAnyShape = pointInShapeList(obstaclePoints, randPos, agentPad + agentRad);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

ArrayList<ArrayList<Integer>> agentPath = new ArrayList<ArrayList<Integer>>(numAgents);

// Images
int numPeople = 7;
int numStates = 3;
PImage person[][] = new PImage[numPeople][numStates];
PImage campus, apartment;

// Screen size
float campusMult = 1.5;
int campusWidth = int(600 * campusMult);
int campusHeight = int(488 * campusMult);
float apartmentMult = 1.4;
int apartmentWidth = int(1000 * apartmentMult);
int apartmentHeight = int(684 * apartmentMult);

PShape obstacles[];
Vec2[][] obstaclePoints;

// Set screen size dynamically
void settings() {
  if (scene) size(campusWidth, campusHeight);
  else size(apartmentWidth, apartmentHeight);
}

int strokeWidth = 2;
void setup(){
  // Choose obstacles
  if (scene) obstaclePoints = campusPoints;
  else obstaclePoints = apartmentPoints;
  
  // Generate roadmap
  testPRM();
  
  // Load people images
  for (int i = 0; i < numPeople; i++) {
    for (int j = 0; j < numStates; j++) {
      person[i][j] = loadImage("People/Person" + (i+1) + "-" + (j+1) + ".png");
    }
  }
  
  campus = loadImage("Reference/campus.jpg");
  apartment = loadImage("Reference/apartment.jpg");
  
  // Load obstacles
  obstacles = new PShape[obstaclePoints.length];
  for (int i = 0; i < obstaclePoints.length; i++){
    PShape s = createShape();
    boolean contour = false;
    
    s.beginShape();
    s.stroke(strokeColor);
    s.fill(buildingColor);
    for (int j = 0; j < obstaclePoints[i].length; j++) {
      if (obstaclePoints[i][j] == null) s.fill(backgroundColor);
      else s.vertex(obstaclePoints[i][j].x, obstaclePoints[i][j].y);
    }
    
    if (contour) s.endContour();
    
    s.endShape(CLOSE);
    
    obstacles[i] = s;
  }
}

// Choose an open space
Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyShape = pointInShapeList(obstaclePoints, randPos, agentPad + agentRad);
  while (insideAnyShape){
    randPos = new Vec2(random(width),random(height));
    insideAnyShape = pointInShapeList(obstaclePoints, randPos, agentPad + agentRad);
  }
  return randPos;
}

// Generate PRM
void testPRM(){
  // Set up agents & their goals
  for (int i = 0; i < numAgents; i++) {
    agentPos[i] = sampleFreePos();
    agentVel[i] = new Vec2(0, 0);
    agentPath.add(new ArrayList<Integer>());
    
    goalPos[i] = sampleFreePos();
    
    agentPerson[i] = int(random(numPeople));
  }
  
  generateRandomNodes(numNodes);
  connectNeighbors(obstaclePoints, agentPad + agentRad, nodePos, numNodes);
  
  // Agent initial paths
  for (int i = 0; i < numAgents; i++) {
    agentPath.set(i, planPath(agentPos[i], goalPos[i], obstaclePoints, agentPad + agentRad, nodePos, numNodes));
  }
}

// Returns rotation goal for agents
float calcGoalRotation(int id) {
  Vec2 dir;
  
  dir = agentVel[id];
  
  if (dir.x == 0 && dir.y == 0) return agentAng[id];
  else if (dir.x == 0) return dir.y > 0 ? 0 : PI;
  else if (dir.x < 0) return atan(dir.y / dir.x) - PI/2 + 2*PI;
  else return atan(dir.y / dir.x) + PI/2;
}

// Update motion
void update(float dt) {
  for (int i = 0; i < numAgents; i++) {
    Vec2 startDir;
    Vec2 agentAcc = new Vec2(0, 0);
    
    // Static collision avoidance
    Vec2 collisionAgentAcc = new Vec2(0, 0);
    for (int j = 0; j < obstacles.length; j++) {
      //agentPos[i] = new Vec2(mouseX, mouseY);
      if (pointInShape(obstaclePoints[j], agentPos[i], agentPad + agentRad)) {
        for (int k = 0, l = -1; k < obstaclePoints[j].length; l = k, k++) {
          Vec2 vk = obstaclePoints[j][mod(k, obstaclePoints[j].length)];
          Vec2 vl = obstaclePoints[j][mod(l, obstaclePoints[j].length)];
          if (vk != null && vl != null) {
              Vec2 dir = vk.minus(vl).normalized();
              if (rayCircleIntersect(agentPos[i], agentPad + agentRad, vl, dir, vl.distanceTo(vk)).hit) {
                Vec2 perpendicular = new Vec2(-dir.y, dir.x);
                Vec2 normalDir = projAB(agentPos[i].minus(vk), perpendicular).normalized();
                if (pointInShape(obstaclePoints[j], agentPos[i], 0)) collisionAgentAcc.subtract(normalDir);
                else collisionAgentAcc.add(normalDir);
              }
          }
        }
      }
    }
    if (collisionAgentAcc.length() > 0) agentAcc.add(collisionAgentAcc.normalized().times(obstacleAccMagnitude));
    
    // Agent strong collision avoidance
    Vec2 strongAvoidAgentAcc = new Vec2(0, 0);
    for (int j = 0; j < numAgents; j++) {
      if (j == i) continue;
      if (agentPos[i].distanceTo(agentPos[j]) < agentMinDist) {
        Vec2 normal = agentPos[i].minus(agentPos[j]).normalized();
        strongAvoidAgentAcc.add(normal);
      }
    }
    if (strongAvoidAgentAcc.length() > 0) agentAcc.add(strongAvoidAgentAcc.normalized().times(strongAgentAvoidAccMagnitude));
    
    // TTC collision avoidance
    Vec2 ttcAgentAcc = new Vec2(0, 0);
    for (int j = 0; j < numAgents; j++){
      if (j == i) continue;
      if (agentPos[i].distanceTo(agentPos[j]) > TTCMinDist) continue;
      float ttc = computeTTC(agentPos[i],agentVel[i], agentRad, agentPos[j],agentVel[j], agentRad);
      Vec2 futurePos_id = agentPos[i].plus(agentVel[i].times(ttc));
      Vec2 futurePos_j = agentPos[j].plus(agentVel[j].times(ttc));
      Vec2 avoidDir = futurePos_id.minus(futurePos_j).normalized();
      Vec2 avoidForce = avoidDir.times(1/ttc);
      //println(ttc, avoidForce.x, avoidForce.y);
      if (ttc > 0){
        ttcAgentAcc.add(avoidForce);
      }
    }
    if (ttcAgentAcc.length() > 0) agentAcc.add(ttcAgentAcc.normalized().times(agentAvoidAccMagnitude));
    
    // Goal Acc
    // If goal is not pathable
    if (agentPath.get(i).size() > 0 && agentPath.get(i).get(0) == -1) {
      goalPos[i] = sampleFreePos();
      agentPath.set(i, planPath(agentPos[i], goalPos[i], obstaclePoints, agentPad + agentRad, nodePos, numNodes));
    } else {
      // If path/goal is no longer visible
      if (agentPath.get(i).size() > 0 && agentPath.get(i).get(0) != -1 && 
          rayShapeListIntersect(obstaclePoints, nodePos[agentPath.get(i).get(0)], 
                                agentPos[i].minus(nodePos[agentPath.get(i).get(0)]).normalized(), 
                                agentPos[i].distanceTo(nodePos[agentPath.get(i).get(0)]), agentPad + agentRad) || // If the current node in the path is not visible
          agentPath.get(i).size() == 0 &&
          rayShapeListIntersect(obstaclePoints, goalPos[i], agentPos[i].minus(goalPos[i]).normalized(), agentPos[i].distanceTo(goalPos[i]), agentPad + agentRad)) { // If the goal is not visible
          agentPath.set(i, planPath(agentPos[i], goalPos[i], obstaclePoints, agentPad + agentRad, nodePos, numNodes));
      }
      
      // When to pop nodes from path
      while (agentPath.get(i).size() > 1 && agentPath.get(i).get(0) != -1 && 
             !rayShapeListIntersect(obstaclePoints, nodePos[agentPath.get(i).get(1)],  //<>//
                                     agentPos[i].minus(nodePos[agentPath.get(i).get(1)]).normalized(),
                                     agentPos[i].distanceTo(nodePos[agentPath.get(i).get(1)]), agentPad + agentRad) || // If the next node in the path is visible
             agentPath.get(i).size() > 0 && agentPath.get(i).get(0) != -1 &&
             !rayShapeListIntersect(obstaclePoints, goalPos[i], agentPos[i].minus(goalPos[i]).normalized(), agentPos[i].distanceTo(goalPos[i]), agentPad + agentRad)) { // If the goal is visible
        agentPath.get(i).remove(0);
      }
      
      // If a path exists
      if (agentPath.get(i).size() > 0 && agentPath.get(i).get(0) != -1) {
        startDir = nodePos[agentPath.get(i).get(0)].minus(agentPos[i]);
        agentAcc.add(startDir.normalized().times(goalAccMagnitude));
      
      // Else if the goal is visible
      } else if (!pointInShapeList(obstaclePoints, agentPos[i], agentPad + agentRad)) {
        startDir = goalPos[i].minus(agentPos[i]);
        agentAcc.add(startDir.normalized().times(goalAccMagnitude));
        
        // If the agent is at the goal
        if (goalPos[i].distanceTo(agentPos[i]) < closeToGoalDist) {
          agentVel[i] = new Vec2(0, 0);
          goalPos[i] = sampleFreePos();
          agentPath.set(i, planPath(agentPos[i], goalPos[i], obstaclePoints, agentPad + agentRad, nodePos, numNodes));
        }
      }
    }
    
    // Update vel and pos
    agentAcc.clampToLength(maxAcc);
    agentVel[i].add(agentAcc.times(dt));
    agentVel[i].clampToLength(maxVel);
    agentPos[i].add(agentVel[i].times(dt));
  }
  
  // Angle calculations
  for (int i = 0; i < numAgents; i++) {
    float goalAngle = calcGoalRotation(i);
    
    if (abs(agentAng[i] - goalAngle) > PI) {
      if (goalAngle > agentAng[i]) agentAngVel[i] = -((2*PI - goalAngle) + agentAng[i]);
      else agentAngVel[i] = (2*PI - agentAng[i]) + goalAngle;
    } else agentAngVel[i] = goalAngle - agentAng[i];
    
    if (agentAngVel[i] > 0 && agentAngVel[i] > maxAngVel * dt) agentAngVel[i] = maxAngVel * dt;
    if (agentAngVel[i] < 0 && agentAngVel[i] < -maxAngVel * dt) agentAngVel[i] = -maxAngVel * dt;
    agentAng[i] += agentAngVel[i];
    agentAng[i] %= 2*PI;
  }
  
}

void draw(){
  strokeWeight(1);
  background(backgroundColor); // Green
  stroke(0,0,0);
  fill(255,255,255);
  
  // Reference images
  imageMode(NORMAL);
  if (photo) {
    if (scene) image(campus, 0, 0, campusWidth, campusHeight);
    else image(apartment, 0, apartmentHeight*0.1, apartmentWidth*0.9, apartmentHeight*0.9);
  }
  
  update(1/frameRate);
  
  // Draw stuff to help draw obstacles
  if (points) {
    for (int i = 0; i < pointArr.size(); i++) {
      circle(pointArr.get(i).x,pointArr.get(i).y,2);
    }
  }
  
  // For the apartment scene, draw floor
  if (!scene && !photo) {
    fill(carpetColor);
    beginShape();
    vertex(1224, 819);
    vertex(1226, 324);
    vertex(1135, 320);
    vertex(1133, 124);
    vertex(46, 122);
    vertex(46, 523);
    vertex(151, 522);
    vertex(154, 817);
    endShape(CLOSE);
  }
  
  //Draw the obstacles
  for (int i = 0; i < obstacles.length; i++) {
    shape(obstacles[i]);
  }
  
  ////Draw PRM Nodes
  //fill(0);
  //for (int i = 0; i < numNodes; i++){
  //  circle(nodePos[i].x,nodePos[i].y,5);
  //}
  
  ////Draw graph
  //stroke(100,100,100);
  //strokeWeight(1);
  //for (int i = 0; i < numNodes; i++){
  //  for (int j : neighbors[i]){
  //    line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
  //  }
  //}
  
  //Draw Planned Path
  if (debug) {
    for (int i = 0; i < numAgents; i++) {
      if (!(agentPath.get(i).size() > 0 && agentPath.get(i).get(0) == -1)) {
        stroke(20,255,40, 100);
        strokeWeight(2);
        if (agentPath.get(i).size() == 0){
          line(agentPos[i].x,agentPos[i].y,goalPos[i].x,goalPos[i].y);
        } else {
          line(agentPos[i].x,agentPos[i].y,nodePos[agentPath.get(i).get(0)].x,nodePos[agentPath.get(i).get(0)].y);
          for (int j = 0; j < agentPath.get(i).size()-1; j++){
            int curNode = agentPath.get(i).get(j);
            int nextNode = agentPath.get(i).get(j+1);
            line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
          }
          line(goalPos[i].x,goalPos[i].y,nodePos[agentPath.get(i).get(agentPath.get(i).size()-1)].x,nodePos[agentPath.get(i).get(agentPath.get(i).size()-1)].y);
        }
      }
    }
  }
  
  // Draw goals
  stroke(0, 0, 0);
  strokeWeight(1);
  if (goals) {
    for (int i = 0; i < numAgents; i++) {
      fill(250,30,50);
      circle(goalPos[i].x,goalPos[i].y,20);
    }
  }
  
  // Draw agents
  for (int i = 0; i < numAgents; i++) {
    if (debug) {
      fill(20,60,250);
      circle(agentPos[i].x, agentPos[i].y, agentRad*2);
    }
    pushMatrix();
    translate(agentPos[i].x, agentPos[i].y);
    rotate(agentAng[i]);
    imageMode(CENTER);
    if (agentVel[i].length() < 10) image(person[agentPerson[i]][0], 0, 0, agentRad*2, agentRad*2);
    else image(person[agentPerson[i]][(frameCount / 20) % 2 + 1], 0, 0, agentRad*2, agentRad*2);
    popMatrix();
  }
}

void keyPressed(){
  // Reset
  if (key == 'r'){
    testPRM();
    if (points) pointArr.clear();
    return;
  }
}

void mousePressed(){
  // Draw points to reference for obstacles
  if (points) {
    pointArr.add(new Vec2(mouseX, mouseY));
    print("\n{\n");
    for (int i = 0; i < pointArr.size(); i++) {
      print("    new Vec2(" +  int(pointArr.get(i).x) + ", " + int(pointArr.get(i).y) + ")");
      println(", ");
    }
    println("  },");
  }
}
