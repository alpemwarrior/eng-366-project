#include <Eigen/MatrixFunctions>

typedef Eigen::Vector2d vec_2d;

#define NPOINTS (sizeof(path)/sizeof(path[0]))
#define NSEGMENTS (NPOINTS-1)
#define SIGN(x) ((x > 0) - (x < 0))
#define K_FORWARD 1.0
#define K_NORMAL  5.0

#define K_ANGLE 1    // In s^(-1)
#define K_V     1    // In m

#define R_BLEND 0.3  // In meters 
#define D_BIAS  0.05 // In meters

#define R_WHEEL 0.11 // In meters
#define L_AXIS  0.4  // In meters

// Path to follow
vec_2d path[] = {
    vec_2d(-1,  0),
    vec_2d(6.1, 0),
    vec_2d(6.1, 1.3),
    vec_2d(-1,  1.3),
    vec_2d(-1,  2.5),
    vec_2d(6.1, 2.5),
    vec_2d(6.1, 3.9),
    vec_2d(-1,  3.9),
    vec_2d(-1,  6)
};

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

    for (int i = 0; i < NSEGMENTS; i++) {
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

/// @brief Generates direction vector given any point z.
/// @param z 
/// @return Normalized direction vector
vec_2d getDirectionVector(vec_2d z) {
    vec_2d x2, vr, v1, v2;
    double d1, d2;
    int idx_closest = getClosestSegment(z);
    vec_2d x2 = path[idx_closest+1];

    if (((x2-z).norm() > R_BLEND) || (idx_closest == (NSEGMENTS-1))) {
        vr = getSingleVector(idx_closest, z);     
    } 
    else {
        v1 = getSingleVector(idx_closest, z);  
        v2 = getSingleVector(idx_closest+1, z);
        d1 = getDistanceToSegment(idx_closest, z)+D_BIAS;
        d2 = getDistanceToSegment(idx_closest+1, z)+D_BIAS;
        vr = (v1*d1 + v2*d2).normalized();
    }

    return vr;
}

/// @brief Calculates the wheel speed (m/s) for path following. 
/// @param z Current position
/// @param heading Current heading
/// @return vec_2d(vr, vl)
vec_2d getWheelSpeeds(vec_2d z, vec_2d heading) {
    vec_2d vp, vw;
    double angle, proj, w, vr;
    vp = getDirectionVector(z);

    
    // Project vp on current heading
    proj = vp.dot(heading);
    
    // Only advance if the projection is in front of the robot
    vr = (proj > 0) ? proj*K_V : 0.0;

    // Compute angle (signed)
    angle = SIGN(heading.cross(vp))*acos(proj/vp.norm());

    // Proportional controller to follow heading
    w = angle*K_ANGLE;

    // Translate forward speed, rotational speed to wheel speed
    vw = vec_2d(
        (vr-w*L_AXIS/2)/R_WHEEL,
        (vr+w*L_AXIS/2)/R_WHEEL
    );

    return vw;
}

