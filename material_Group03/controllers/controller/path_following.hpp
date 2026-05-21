// IMPLEMENTED BY: Pol Rius (423483)

#pragma once

#include <Eigen/Geometry>
#include <Eigen/Dense>

typedef Eigen::Vector2d vec_2d;

#define NPOINTS (sizeof(path)/sizeof(path[0]))
#define NSEGMENTS (NPOINTS-1)

#define SIGN(x) ((x > 0.0) - (x < 0.0))
#define K_FORWARD 1.0
#define K_NORMAL  5.0

#define K_ANGLE  20   // In s^(-1)
#define K_V      2    // In m

#define R_BLEND 0.5  // In meters 
#define D_BIAS  0.05 // In meters

#define R_WHEEL 0.11 // In meters
#define L_AXIS  0.4  // In meters

#define R_FINISH 0.2 // Radius within last waypoint to consider the mission to be completed, in meters

#define MAX_WHEEL_SPEED 6.4

// Path to follow
vec_2d path[] = {
    vec_2d(-1,  0),
    vec_2d(6.1, 0),
    vec_2d(6.1, 1.3),
    vec_2d(-0.5,  1.3),
    vec_2d(-0.5,  2.5),
    vec_2d(6.1, 2.5),
    vec_2d(6.1, 3.9),
    vec_2d(-0.5,  3.9),
    vec_2d(-0.5,  6)
};

double last_e = 0;

double clipWheelSpeed(double speed) {
    return (speed > MAX_WHEEL_SPEED) ? MAX_WHEEL_SPEED : speed;
}

/// @brief Gets projected distance from point z to any given segment (no clamping)
/// @param segment_idx 
/// @param z 
/// @return Distance from z to segment_idx
double getDistanceToSegment(int segment_idx, vec_2d z) {
    vec_2d x1 = path[segment_idx];
    vec_2d x2 = path[segment_idx+1];
    vec_2d vline = x2-x1;
    vec_2d x1z = z-x1;
    vec_2d p = x1 + vline*(vline.dot(x1z)/vline.dot(vline));  
    return (p-z).norm();
}

/// @brief Gets closest segment to z
/// @param z 
/// @return Index of closest segment 
int getClosestSegment(vec_2d z) {
    double min_d = 10e3;
    int min_i = 0;
    double s, dx1, dx2, d; 
    vec_2d vline, x1z, p;

    for (unsigned int i = 0; i < NSEGMENTS; i++) {
        vec_2d x1 = path[i];
        vec_2d x2 = path[i+1];
        vline = x2-x1;
        x1z = z-x1;
        p = x1 + vline*(vline.dot(x1z)/vline.dot(vline));

        // 0 -> x1, 1 -> x2
        s = vline.dot(x1z)/vline.dot(vline);

        // If not perpendicular to segment (projection beyond ends)
        // then clamp to closest end
        if ((s > 1.0) || (s < 0.0)) {
            dx1 = (x1-z).norm();
            dx2 = (x2-z).norm();
            d = (dx1 < dx2) ? dx1 : dx2;
        } else {
            d = (p-z).norm();
        }

        // Important, if segment k and k+1 are clamping to an end, return k+1
        if (d <= min_d) {
            min_d = d;
            min_i = i;
        }
    }

    return min_i;
}

/// @brief Generates direction vector for segment i and position z
/// @param i Path number
/// @param z Position
/// @return Normalized direction vector
vec_2d getSingleVector(int i, vec_2d z) {
    vec_2d x1 = path[i];
    vec_2d x2 = path[i+1];
    vec_2d vline = x2-x1;
    vec_2d x1z = z-x1;
    vec_2d p = x1 + vline*(vline.dot(x1z)/vline.dot(vline));
    vec_2d vnorm = p - z;

    // Eigen already handles the case where vnorm is zero. The resulting vector is normalized.
    vec_2d direction_vector = (K_FORWARD*vline.normalized() + 
                               K_NORMAL*vnorm).normalized();

    return direction_vector;
}

/// @brief Calculates the distance from the projection of z on segment i to the endpoint of segment i
/// @param i Segment index
/// @param z Position
/// @return Distance from the projected point on the line to the end waypoint.
double getProjectedDistanceToEnd(int i, vec_2d z) {
    vec_2d x1 = path[i];
    vec_2d x2 = path[i+1];
    vec_2d vline = x2-x1;
    vec_2d x1z = z-x1;
    vec_2d p = x1 + vline*(vline.dot(x1z)/vline.dot(vline));

    return (p-x2).norm();
}

/// @brief Generates direction vector given any point z.
/// @param z 
/// @param wp Pointer to store current waypoint
/// @return Normalized direction vector
vec_2d getDirectionVector(vec_2d z, int* wp) {
    vec_2d x2, vr, v1, v2;
    double d1, d2;
    int idx_closest = getClosestSegment(z);
    x2 = path[idx_closest+1];
    *wp = idx_closest;

    if (((x2-z).norm() > R_BLEND) || (idx_closest == (NSEGMENTS-1))) {
        vr = getSingleVector(idx_closest, z);     
    } 
    else {
        v1 = getSingleVector(idx_closest, z);  
        v2 = getSingleVector(idx_closest+1, z);
        d1 = getProjectedDistanceToEnd(idx_closest, z);
        d2 = R_BLEND - d1;
        //d1 = getDistanceToSegment(idx_closest, z)+D_BIAS;
        //d2 = getDistanceToSegment(idx_closest+1, z)+D_BIAS;
        
        vr = (v1*d1 + v2*d2).normalized();
    }

    return vr;
}


/// @brief Returns wheel speeds (m/s) for path following
/// @param pos          Estimated position
/// @param left_speed   Left wheel speed (reference)
/// @param right_speed  Right wheel speed (reference)
/// @return Current waypoint
int getWheelSpeeds(double* pos, double &left_speed, double &right_speed) {
    vec_2d z, heading, vp;
    double angle, proj, w, vr;
    int wp;
    
    z = vec_2d(pos[0], pos[1]);
    heading = vec_2d(cos(pos[2]), sin(pos[2]));
    
    vp = getDirectionVector(z, &wp);
    
    // Project vp on current heading
    proj = vp.dot(heading);
    
    // Only advance if the projection is in front of the robot
    vr = (proj > 0) ? proj*K_V : 0.0;

    Eigen::Vector3d auxv1, auxv2;
    auxv1 = Eigen::Vector3d(heading.x(), heading.y(), 0);
    auxv2 = Eigen::Vector3d(vp.x(), vp.y(), 0);

    // Compute angle (signed)
    angle = SIGN(auxv1.cross(auxv2).z())*acos(proj/vp.norm());

    // Proportional controller to follow heading
    w = angle*K_ANGLE;

    // Translate forward speed, rotational speed to wheel speed
    left_speed =  vr-w*L_AXIS/2;
    right_speed = vr+w*L_AXIS/2;

    return wp;
}

bool checkIfPathIsFinished(double* pos) {
  vec_2d z; double d;
  
  z = vec_2d(pos[0], pos[1]);
  d = (z-path[NPOINTS-1]).norm();
  
  if (d < R_FINISH) return true;
  
  return false;
}

