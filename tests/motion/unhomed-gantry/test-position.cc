/* Exercise the real INI parsing, mapping transport, and position-file I/O. */
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "../../../src/emc/task/taskintf.cc"

char emc_inifile[LINELEN];
int emc_debug;
static int primary[EMCMOT_MAX_JOINTS], commands;
static double loaded[EMCMOT_MAX_JOINTS];
static bool fail_command;

/* Expected error cases must not put source-line-dependent text on stdout. */
int rcs_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int result = vfprintf(stderr, fmt, args);
    va_end(args);
    return result;
}
int set_print_rcs_error_info(const char *file, int line)
{ (void)file; (void)line; return 0; }
int print_rcs_error_new(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int result = vfprintf(stderr, fmt, args);
    va_end(args);
    return result;
}

int usrmotWriteEmcmotCommand(emcmot_command_t *cmd)
{
    if (fail_command) return -1;
    if (cmd->command == EMCMOT_SET_JOINT_PRIMARY) {
        assert(cmd->joint >= 0 && cmd->joint < EMCMOT_MAX_JOINTS);
        assert(cmd->primary_joint >= 0 && cmd->primary_joint <= cmd->joint);
        primary[cmd->joint] = cmd->primary_joint;
        commands++;
    } else {
        assert(cmd->command == EMCMOT_SET_JOINT_MOTOR_OFFSET);
        loaded[cmd->joint] = -cmd->motor_offset;
    }
    return 0;
}

static void config(const char *kins, const char *options, int joints)
{
    FILE *f = fopen(emc_inifile, "w");
    assert(f);
    fprintf(f, "[KINS]\nKINEMATICS = %s\nOPTIONS = %s\n"
               "[TRAJ]\nCOORDINATES = XYZ\nPOSITION_FILE = position.txt\n", kins, options);
    assert(fclose(f) == 0);
    TrajConfig.Joints = joints;
    commands = 0;
    fail_command = false;
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        primary[j] = -1;
        emcmotStatus.joint_status[j].pos_fb = 100 + j;
    }
}

static void save_and_load(int secondary, int main_joint)
{
    assert(emcPositionSave() == 0);
    assert(emcPositionLoad() == 0);
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        assert(loaded[j] == 100 + (j == secondary ? main_joint : j));
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2 && chdir(argv[1]) == 0);
    strcpy(emc_inifile, "machine.ini");
    config("trivkins coordinates=XYZY kinstype=1", "", 4);
    assert(find_axis_primary_joints() == 0 && commands == 4);
    assert(primary[3] == 1);
    save_and_load(3, 1);
    config("trivkins", "coordinates=yzyx kinstype=b", 4);
    assert(find_axis_primary_joints() == 0);
    assert(primary[2] == 0 && primary[3] == 3);
    save_and_load(2, 0);
    config("trivkins coordinates=XYZY", "coordinates=XYZX", 4);
    assert(find_axis_primary_joints() == 0 && primary[3] == 1);
    /* Extra and unmapped joints retain their own saved positions. */
    config("trivkins coordinates=XYZY", "", 6);
    assert(find_axis_primary_joints() == 0 && primary[4] == 4 && primary[5] == 5);
    save_and_load(3, 1);
    /* A kinematics type of BOTH alone does not imply trivkins mapping. */
    config("corexykins", "coordinates=XYZY", 4);
    emcmotConfig.kinType = KINEMATICS_BOTH;
    assert(find_axis_primary_joints() == 0);
    for (int j = 0; j < 4; j++) assert(primary[j] == j);
    save_and_load(-1, -1);
    config("trivkins", "", 4);
    assert(find_axis_primary_joints() == 0 && primary[3] == 3);
    save_and_load(-1, -1);
    config("trivkins coordinates=YYYYYYYYYYYYYYYY", "", 16);
    assert(find_axis_primary_joints() == 0 && commands == 16);
    for (int j = 0; j < 16; j++) assert(primary[j] == 0);
    assert(emcPositionSave() == 0 && emcPositionLoad() == 0);
    for (int j = 0; j < 16; j++) assert(loaded[j] == 100);
    config("trivkins coordinates=YYYYYYYYYYYYYYYYY", "", 16);
    assert(find_axis_primary_joints() == -1 && commands == 0);
    config("trivkins coordinates=XYZY", "", 4);
    fail_command = true;
    assert(find_axis_primary_joints() == -1);
    puts("position mapping, transport, save/load, unused joints and bounds checks passed");
    return 0;
}
