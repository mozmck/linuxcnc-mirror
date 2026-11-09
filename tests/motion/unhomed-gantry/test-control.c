/* Exercise the real mode transitions without starting realtime threads. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../src/emc/motion/control.c"

static emcmot_status_t status_data;
static emcmot_config_t config_data;
static emcmot_internal_t internal_data;
static emcmot_hal_data_t hal_data;
emcmot_status_t *emcmotStatus = &status_data;
emcmot_config_t *emcmotConfig = &config_data;
emcmot_internal_t *emcmotInternal = &internal_data;
emcmot_hal_data_t *emcmot_hal_data = &hal_data;
emcmot_joint_t joints[EMCMOT_MAX_JOINTS];
static bool homed[EMCMOT_MAX_JOINTS];
static bool homing;
static hal_bit_t eoffset_limited, eoffset_active;
static EmcPose planner_position;
static int count;

bool get_homed(int j) { return homed[j]; }
bool get_homing_is_active(void) { return homing; }
void do_cancel_homing(int j) { (void)j; }
void axis_initialize_external_offsets(void) {}
bool axis_jog_abort_all(bool immediate) { (void)immediate; return false; }
void axis_sync_teleop_tp_to_carte_pos(int factor, double *p[]) { (void)factor; (void)p; }
void axis_apply_ext_offsets_to_carte_pos(int factor, double *p[]) { (void)factor; (void)p; }
int tpSetPos(TP_STRUCT *tp, const EmcPose *pos) { (void)tp; planner_position = *pos; return 0; }
int tpClear(TP_STRUCT *tp) { (void)tp; return 0; }
void reportError(const char *fmt, ...) { (void)fmt; assert(!"unexpected motion error"); }
void rtapi_print(const char *fmt, ...) { (void)fmt; }
void rtapi_print_msg(msg_level_t level, const char *fmt, ...) { (void)level; (void)fmt; }
KINEMATICS_TYPE kinematicsType(void) { return config_data.kinType; }
int kinematicsForward(const double *j, EmcPose *p,
                      const KINEMATICS_FORWARD_FLAGS *f, KINEMATICS_INVERSE_FLAGS *i)
{ (void)f; (void)i; return mapped_joints_to_position(NO_OF_KINS_JOINTS, j, p); }
int kinematicsInverse(const EmcPose *p, double *j,
                      const KINEMATICS_INVERSE_FLAGS *i, KINEMATICS_FORWARD_FLAGS *f)
{ (void)f; (void)i; return position_to_mapped_joints(NO_OF_KINS_JOINTS, p, j); }

static void close_to(double actual, double expected)
{ assert(fabs(actual - expected) < 1e-12); }

static void setup(void)
{
    memset(&status_data, 0, sizeof(status_data));
    memset(&config_data, 0, sizeof(config_data));
    memset(&internal_data, 0, sizeof(internal_data));
    memset(joints, 0, sizeof(joints));
    memset(homed, 0, sizeof(homed));
    homing = false;
    coord_cubic_active = false;
    config_data.numJoints = 4;
    config_data.kinType = KINEMATICS_IDENTITY;
    internal_data.enabling = 1;
    hal_data.eoffset_limited = &eoffset_limited;
    hal_data.eoffset_active = &eoffset_active;
    SET_MOTION_ENABLE_FLAG(1);
    SET_MOTION_INPOS_FLAG(1);
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        joints[j].primary_joint = j;
        SET_JOINT_ACTIVE_FLAG(&joints[j], 1);
        joints[j].pos_cmd = joints[j].pos_fb = 10 + j;
        joints[j].coarse_pos = joints[j].free_tp.curr_pos = joints[j].pos_cmd;
        cubicInit(&joints[j].cubic);
        cubicSetSegmentTime(&joints[j].cubic, .001);
        cubicSetInterpolationRate(&joints[j].cubic, 1);
        cubicAddPoint(&joints[j].cubic, joints[j].pos_cmd);
    }
    joints[3].primary_joint = 1;
    joints[3].pos_cmd = 11.020;
    joints[3].pos_fb = 11.0199;
    joints[3].coarse_pos = joints[3].free_tp.curr_pos = 11.020;
    joints[3].motor_offset = 2;
    joints[3].backlash_filt = .001;
}

static void verify_transition(bool coordinated, bool reenable)
{
    const double primary_motor = joints[1].pos_cmd + joints[1].motor_offset;
    const double motor = joints[3].pos_cmd + joints[3].motor_offset + joints[3].backlash_filt;
    const double error = joints[3].pos_cmd - joints[3].pos_fb;
    if (coordinated) {
        internal_data.coordinating = 1;
        if (reenable) { SET_MOTION_COORD_FLAG(1); }
    } else {
        internal_data.teleoperating = 1;
        if (reenable) { SET_MOTION_TELEOP_FLAG(1); }
    }
    if (reenable) { SET_MOTION_ENABLE_FLAG(0); }
    set_operating_mode();
    assert(status_data.motion_state == (coordinated ? EMCMOT_MOTION_COORD : EMCMOT_MOTION_TELEOP));
    close_to(joints[3].pos_cmd, joints[1].pos_cmd);
    close_to(joints[1].pos_cmd + joints[1].motor_offset, primary_motor);
    close_to(joints[3].pos_cmd + joints[3].motor_offset + joints[3].backlash_filt, motor);
    close_to(joints[3].pos_cmd - joints[3].pos_fb, error);
    close_to(joints[3].coarse_pos, joints[1].pos_cmd);
    if (!homed[3]) {
        close_to(joints[3].free_tp.curr_pos, joints[1].pos_cmd);
        close_to(joints[3].free_tp.pos_cmd, joints[1].pos_cmd);
    }
    if (coordinated || reenable) close_to(planner_position.tran.y, joints[1].pos_cmd);
    double target[EMCMOT_MAX_JOINTS] = {0};
    kinematicsInverse(&status_data.carte_pos_cmd, target, &iflags, &fflags);
    cubicDrain(&joints[3].cubic);
    cubicAddPoint(&joints[3].cubic, target[3]);
    close_to(cubicInterpolate(&joints[3].cubic, 0, 0, 0, 0)
             + joints[3].motor_offset + joints[3].backlash_filt, motor);
    count++;
}

int main(void)
{
    int mapping[EMCMOT_MAX_JOINTS];
    assert(map_coordinates_to_jnumbers("XYZY", EMCMOT_MAX_JOINTS, 1, mapping) == 0);
    for (int both = 0; both < 2; both++) {
        for (int coord = 0; coord < 2; coord++) {
            for (int reenable = 0; reenable < 2; reenable++) {
                setup();
                config_data.kinType = both ? KINEMATICS_BOTH : KINEMATICS_IDENTITY;
                verify_transition(coord, reenable);
            }
        }
    }
    /* Abort after only the primary has reset its homing coordinate. */
    setup();
    joints[1].pos_cmd = joints[1].pos_fb = joints[1].coarse_pos = 0;
    joints[1].motor_offset = 11;
    verify_transition(false, false);
    setup();
    homed[1] = true;
    verify_transition(false, false);
    setup();
    homed[3] = true;
    verify_transition(false, false);
    /* Independent joints, completed homing, and extra joints retain their coordinates. */
    setup();
    homed[1] = homed[3] = true;
    sync_unhomed_gantry_joints();
    close_to(joints[3].pos_cmd, 11.020);
    setup();
    joints[3].primary_joint = 3;
    sync_unhomed_gantry_joints();
    close_to(joints[3].pos_cmd, 11.020);
    setup();
    config_data.numExtraJoints = 1;
    sync_unhomed_gantry_joints();
    close_to(joints[3].pos_cmd, 11.020);
    /* Neither motion nor an active homing sequence may be rebased mid-move. */
    setup();
    SET_MOTION_INPOS_FLAG(0);
    internal_data.teleoperating = 1;
    set_operating_mode();
    close_to(joints[3].pos_cmd, 11.020);
    assert(!GET_MOTION_TELEOP_FLAG());
    setup();
    homing = true;
    internal_data.coordinating = 1;
    set_operating_mode();
    close_to(joints[3].pos_cmd, 11.020);
    assert(!GET_MOTION_COORD_FLAG());
    printf("%d gantry transitions preserve motor position and following error; exclusion checks passed\n", count);
    return 0;
}
