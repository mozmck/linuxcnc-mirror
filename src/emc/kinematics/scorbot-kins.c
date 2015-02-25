
//
// This is a kinematics module for the Scorbot ER 3.
//
// Copyright (C) 2015 Sebastian Kuzminsky <seb@highlab.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

//
// The origin of the G53 coordinate system is at the center of rotation of
// joint J0, and at the bottom of the base (part 5 in the parts diagram on
// page 7-11 of the SCORBOT-ER III User's Manual).
//
// Joint 0 is rotation around the Z axis.  It chooses the plane that
// the rest of the arm moves in.
//
// Joint 1 is the shoulder.
//
// Joint 2 is the elbow.
//


#include "kinematics.h"
#include "rtapi_math.h"
#include "gotypes.h"


// linkage constants, in mm & degrees
#define L0_LENGTH 355 // Link 0 connects the origin to J1 (shoulder)
#define L0_ANGLE  89  // Link 0 is angled up from horizontal

#define L1_LENGTH 220  // Link 1 connects J1 (shoulder) to J2 (elbow)
#define L2_LENGTH 220  // Link 2 connects J2 (shoulder) to the wrist


// Compute the cartesian coordinates of J1, given the J0 angle (and the
// fixed, known link L0 between J0 and J1).
static void compute_j1_cartesian_location(double j0, EmcPose *j1_cart) {
    double r;
    r = L0_LENGTH * cos(TO_RAD * L0_ANGLE);
    j1_cart->tran.x = r * cos(TO_RAD * j0);
    j1_cart->tran.y =  r * sin(TO_RAD * j0);
    j1_cart->tran.z = L0_LENGTH * sin(TO_RAD * L0_ANGLE);
    j1_cart->a = 0;
    j1_cart->b = 0;
    j1_cart->c = 0;
    j1_cart->u = 0;
    j1_cart->v = 0;
    j1_cart->w = 0;
}


// Forward kinematics takes the joint positions and computes the cartesian
// coordinates of the controlled point.
int kinematicsForward(
    const double *joints,
    EmcPose *pose,
    const KINEMATICS_FORWARD_FLAGS *fflags,
    KINEMATICS_INVERSE_FLAGS *iflags
) {
    EmcPose j1_vector;  // the vector from j0 ("base") to joint 1 ("shoulder", end of link 0)
    EmcPose j2_vector;  // the vector from j1 ("shoulder") to  joint 2 ("elbow", end of link 1)
    EmcPose j3_vector;  // the vector from j2 ("elbow") to joint 3 ("wrist", end of link 2)

    double r;

    compute_j1_cartesian_location(joints[0], &j1_vector);

    // Link 1 connects j1 (shoulder) to j2 (elbow).
    r = L1_LENGTH * cos(TO_RAD * joints[1]);
    j2_vector.tran.x = r * cos(TO_RAD * joints[0]);
    j2_vector.tran.y =  r * sin(TO_RAD * joints[0]);
    j2_vector.tran.z = L1_LENGTH * sin(TO_RAD * joints[1]);

    // Link 2 connectes j2 (elbow) to j3 (wrist).
    // J3 is the controlled point.
    r = L2_LENGTH * cos(TO_RAD * joints[2]);
    j3_vector.tran.x = r * cos(TO_RAD * joints[0]);
    j3_vector.tran.y =  r * sin(TO_RAD * joints[0]);
    j3_vector.tran.z = L2_LENGTH * sin(TO_RAD * joints[2]);

    // The end-effector location is the sum of the linkage vectors.
    pose->tran.x = j1_vector.tran.x + j2_vector.tran.x + j3_vector.tran.x;
    pose->tran.y = j1_vector.tran.y + j2_vector.tran.y + j3_vector.tran.y;
    pose->tran.z = j1_vector.tran.z + j2_vector.tran.z + j3_vector.tran.z;

    return 0;
}


// Inverse kinematics takes the cartesian coordinates of the controlled
// point and computes corresponding the joint positions.
int kinematicsInverse(
    const EmcPose *pose,
    double *joints,
    const KINEMATICS_INVERSE_FLAGS *iflags,
    KINEMATICS_FORWARD_FLAGS *fflags
) {
    EmcPose j1_cart;
    double delta, distance_between_centers;
    double r1, z1;  // (R1, Z1) is the location of J1
    double r2, z2;  // (R2, Z2) is the location of the controlled point

    // these are the two intersection points
    double ir1, iz1;
    double ir2, iz2;

    // the location of J2, this is what we're trying to find
    double j2_r, j2_z;

    // J0 is easy.  Project the (X, Y, Z) of the pose onto the Z=0 plane.
    // J0 points at the projected (X, Y) point.  tan(J0) = Y/X
    // J0 then defines the plane that the rest of the arm operates in.
    joints[0] = TO_DEG * atan2(pose->tran.y, pose->tran.x);

    compute_j1_cartesian_location(joints[0], &j1_cart);

    // FIXME: Until i figure the wrist differential out, the controlled
    //     point will be the location of the wrist joint, J3/J4.

    // The location of J1 (computed above) and the location of the
    // controlled point are separated by J1, L1, J2, and L2.  L1 and L2 are
    // known, but J1 and J2 are not.  J1 and L1 define a circle around the
    // location of J1.  J2 and L2 define a circle around the controlled
    // point.  These two circles intersect in two points.  Pick one of the
    // points and that determines the two angles.

    // (R1, Z1) is the location of J1, in the plane defined by the angle
    // of J0, with the origin at the location of J0.
    r1 = sqrt(pow(j1_cart.tran.x, 2) +  pow(j1_cart.tran.y, 2));
    z1 = j1_cart.tran.z;

    // (R2, Z2) is the location of J3 (the controlled point), again in the
    // plane defined by the angle of J0, with the origin of the machine.
    r2 = sqrt(pow(pose->tran.x, 2) +  pow(pose->tran.y, 2));
    z2 = pose->tran.z;

    // Distance between controlled point and the location of j1.  These two
    // points are separated by link 1, joint 1, and link 2.
    distance_between_centers = sqrt(pow((r2 - r1), 2) + pow((z2 - z1), 2));

    if (distance_between_centers > (L1_LENGTH + L2_LENGTH)) {
        // trying to reach too far
        return GO_RESULT_RANGE_ERROR;
    }

    if (distance_between_centers < fabs(L1_LENGTH - L2_LENGTH)) {
        // trying to reach too far into armpit
        return GO_RESULT_RANGE_ERROR;
    }

    delta = (1.0 / 4.0) * sqrt((distance_between_centers + L1_LENGTH + L2_LENGTH) * (distance_between_centers + L1_LENGTH - L2_LENGTH) * (distance_between_centers - L1_LENGTH + L2_LENGTH) * (L1_LENGTH + L2_LENGTH - distance_between_centers));

    ir1 = ((r1 + r2) / 2) + (((r2 - r1) * (pow(L1_LENGTH, 2) - pow(L2_LENGTH, 2)))/(2 * pow(distance_between_centers, 2))) + ((2 * (z1 - z2) * delta) / pow(distance_between_centers, 2));
    ir2 = ((r1 + r2) / 2) + (((r2 - r1) * (pow(L1_LENGTH, 2) - pow(L2_LENGTH, 2)))/(2 * pow(distance_between_centers, 2))) - ((2 * (z1 - z2) * delta) / pow(distance_between_centers, 2));

    iz1 = ((z1 + z2) / 2) + (((z2 - z1) * (pow(L1_LENGTH, 2) - pow(L2_LENGTH, 2)))/(2 * pow(distance_between_centers, 2))) - ((2 * (r1 - r2) * delta) / pow(distance_between_centers, 2));
    iz2 = ((z1 + z2) / 2) + (((z2 - z1) * (pow(L1_LENGTH, 2) - pow(L2_LENGTH, 2)))/(2 * pow(distance_between_centers, 2))) + ((2 * (r1 - r2) * delta) / pow(distance_between_centers, 2));


    // (ir1, iz1) is one intersection point, (ir2, iz2) is the other.
    // These are the possible locations of the J2 joint.
    // FIXME: For now we arbitrarily pick the one with the bigger Z.

    if (iz1 > iz2) {
        j2_r = ir1;
        j2_z = iz1;
    } else {
        j2_r = ir2;
        j2_z = iz2;
    }

    // Make J1 point at J2 (j2_r, j2_z).
    {
        double l1_r = j2_r - r1;
        joints[1] = TO_DEG * acos(l1_r / L1_LENGTH);
    }

    // Make J2 point at the controlled point.
    {
        double l2_r = r2 - j2_r;
        double j2;
        j2 = TO_DEG * acos(l2_r / L2_LENGTH);
        if (j2_z > pose->tran.z) {
            j2 *= -1;
        }
        joints[2] = j2;
    }

    joints[3] = 1.234;
    joints[4] = cos(M_PI/4);
    // joints[3] = pose->a;
    // joints[4] = pose->b;
    // joints[5] = pose->c;
    // joints[6] = pose->u;
    // joints[7] = pose->v;
    // joints[8] = pose->w;

    return 0;
}


KINEMATICS_TYPE kinematicsType(void) {
    return KINEMATICS_BOTH;
}


#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int comp_id;

int rtapi_app_main(void) {
    comp_id = hal_init("scorbot-kins");
    if (comp_id < 0) {
        return comp_id;
    }
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void) {
    hal_exit(comp_id);
}

