//Compute collision tests.

// Java modulus is bad, this one works for negative numbers
int mod(int a, int b) {
  return (a % b + b) % b;
}

//Returns true if the point is inside a circle
//You must consider a point as colliding if it's distance is <= eps
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos, float eps){
  Vec2 dist = pointPos.minus(center);
  return dist.length() - r <= eps;
}

//Returns true if the point is inside a list of circle
//You must consider a point as colliding if it's distance is <= eps
boolean pointInCircleList(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  for (int i = 0; i < numObstacles; i++) {
      if (pointInCircle(centers[i], radii[i], pointPos, eps)) return true;
  }
  return false;
}

// I could not figure this out, so https://discourse.processing.org/t/checking-for-a-point-within-a-2d-shape-v3-5/26874/15, modified for eps
//Returns true if the point is inside a shape
//You must consider a point as colliding if it's distance is <= eps
boolean pointInShape(Vec2[] verticies, Vec2 pointPos, float eps) {
  boolean oddNodes = false;
  for (int i = 0, j = -1; i < verticies.length; j = i, i++) {
    Vec2 vi = verticies[mod(i, verticies.length)];
    Vec2 vj = verticies[mod(j, verticies.length)];
    if (vi != null && vj != null) {
        if (rayCircleIntersect(pointPos, eps, vi, vj.minus(vi).normalized(), vj.distanceTo(vi)).hit) return true; // if the point + eps collides with a line, then its in the shape
        if ((vi.y < pointPos.y && vj.y >= pointPos.y || vj.y < pointPos.y && vi.y >= pointPos.y) && 
            (vi.x + (pointPos.y - vi.y) / (vj.y - vi.y) * (vj.x - vi.x) < pointPos.x)) oddNodes = !oddNodes;
    }
  }
  return oddNodes;
}

//Returns true if the point is inside a list of shapes
//You must consider a point as colliding if it's distance is <= eps
boolean pointInShapeList(Vec2[][] shapes, Vec2 pointPos, float eps) {
  for (int i = 0; i < shapes.length; i++) {
      if (pointInShape(shapes[i], pointPos, eps)) return true;
  }
  return false;
}

class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

// If a ray collides with a circle
hitInfo rayCircleIntersect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  float eps = 1;
  if (pointInCircle(center, r, l_start, eps)) {
    hit.hit = true;
    hit.t = 0;
    return hit;
  }
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Lenght of l_dir (we noramlized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+strokeWidth)*(r+strokeWidth); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the lenth of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only take the first collision [is this safe?]
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      }
    }
    
  return hit;
}

// If a ray collides with a list of circles
hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii, int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.hit = false;
  hit.t = MAX_FLOAT;
  
  // Loop over all circles
  for (int i = 0 ; i < numObstacles; i++){
    hitInfo circleHit = rayCircleIntersect(centers[i], radii[i], l_start, l_dir, max_t);
    if (circleHit.hit && circleHit.t < hit.t ) {
      hit.hit = circleHit.hit;
      hit.t = circleHit.t;
    }
  }
  
  return hit;
}

// I could not for the life of me figure out when two line segments intersect, so this is taken from https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
boolean ccw(Vec2 A, Vec2 B, Vec2 C) {
  return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

// Return true if line segments AB and CD intersect
boolean intersect(Vec2 A, Vec2 B, Vec2 C, Vec2 D) {
  return ccw(A,C,D) != ccw(B,C,D) && ccw(A,B,C) != ccw(A,B,D);
}

// If a ray instersects a shape
boolean rayShapeIntersect(Vec2[] verticies, Vec2 l_start, Vec2 l_dir, float max_t, float eps) {
  if (pointInShape(verticies, l_start, eps)) return true;
  
  for (int i = 0; i < verticies.length; i++) {
      Vec2 wallStart = verticies[i];
      Vec2 wallEnd = verticies[mod(i-1, verticies.length)];
      
    if (wallStart != null && wallEnd != null) {
      // Add eps to wall
      Vec2 wallDir = wallEnd.minus(wallStart).normalized();
      Vec2 wallStartEps = wallStart.minus(wallDir.times(eps));
      Vec2 wallEndEps = wallEnd.plus(wallDir.times(eps));
      
      Vec2 l_end = l_start.plus(l_dir.times(max_t));
      if (intersect(wallStartEps, wallEndEps, l_start, l_end)) return true;
    }
  }
  
  return false;
}

// If a ray intersects a list of shapes
boolean rayShapeListIntersect(Vec2[][] shapes, Vec2 l_start, Vec2 l_dir, float max_t, float eps) {
  // Loop over all shapes
  for (int i = 0 ; i < shapes.length; i++) {
    boolean currentHit = rayShapeIntersect(shapes[i], l_start, l_dir, max_t, eps);
    if (currentHit) return true;
  }
  
  return false;
}




// TTC Stuff - taken from TTC_Working

//When will agents 1 and 2 collide if they keep their current velocities?
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  //return -1;
  float combinedRadius = radius1+radius2;
  Vec2 relativeVelocity = vel1.minus(vel2);
  float ttc = rayCircleIntersectTime(pos2, combinedRadius, pos1, relativeVelocity);
  return ttc;
}

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
  
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length()*l_dir.length(); 
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
  
  float d = b*b - 4*a*c; //discriminant 
  
  if (d >=0 ){ 
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision! 
    if (t >= 0) return t;
    return -1;
  }
  
  return -1; //We are not colliding, so there is no good t to return 
}
