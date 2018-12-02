
////////////ROBOT-SPECIFIC////////////

const double WHEEL_DIAMETER = 4.0; //inches, for fps for auton
const double TICKS_PER_ROT = 1365.0; //about 3 encoder rotations for each actual rotation

const double MAX_Y_RPM_LOW = 550.0;
const double MAX_Y_RPM_HIGH = 1250.0;
const double MAX_Y_RPM_HD = 0; //HDrive

double MAX_FPS = 0;

const double ACTUAL_MAX_Y_RPM_LOW = 625.0;
const double ACTUAL_MAX_Y_RPM_HIGH = 1300.0;
const double ACTUAL_MAX_Y_RPM_HD = 0;

double DYN_MAX_Y_RPM = 625.0; //for field-centric
const double MAX_X_RPM = 400.0; // for HDrive

const double MAX_YAW_RATE_LOW = 12.0; //max angular velocity divided by the max rpm multiplied by set max rpm
const double MAX_YAW_RATE_HIGH = 28.0;
const double MAX_YAW_RATE_HD = 0.0;

const double MAX_KICK_FPS = ((MAX_X_RPM * WHEEL_DIAMETER * 3.14159) / 12.0) / 60.0; //kicker for HDrive
const int Kv_KICK = (1 / MAX_KICK_FPS);

const double UP_SHIFT_VEL = 375.0; // (24/14) * 9370 //RPM
const double DOWN_SHIFT_VEL = 200.0; //will be less than up shift vel (14/56) *9370 //RPM

/////////////////////////////////////////////////////

const double DRIVE_WAIT_TIME = 0.05; //seconds
const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm

double FF_SCALE = 0.7;

const double TICKS_PER_FOOT = 1315.0;

double l_last_current;

//Teleop
const double K_P_RIGHT_VEL_LOW = 0.001;
const double K_P_LEFT_VEL_LOW = 0.001;
const double K_D_RIGHT_VEL_LOW = 0.000;
const double K_D_LEFT_VEL_LOW = 0.000;
const double K_P_YAW_VEL_LOW = 30.0;
const double K_D_YAW_VEL_LOW = 0.0;

const double K_P_RIGHT_VEL_HIGH = 0.001;
const double K_P_LEFT_VEL_HIGH = 0.001;
const double K_D_RIGHT_VEL_HIGH = 0.00;
const double K_D_LEFT_VEL_HIGH = 0.0;
const double K_P_YAW_VEL_HIGH = 10.0;
const double K_D_YAW_VEL_HIGH = 0.000;

const double K_P_YAW_HEADING_POS_HD = 0.0;
const double K_D_VISION_POS_HD = 0.0;
const double K_P_YAW_HEADING_POS_WC = 0.01;
const double K_D_VISION_POS_WC = 0.0;

const double K_P_KICK_VEL = 0.00365;
const double K_D_KICK_VEL = 0.0;
const double K_F_KICK_VEL = 1.0 / 400.0;

//Auton
const double K_P_YAW_AU_HD = 0.0;
const double K_D_YAW_AU_HD = 0.0;
const double K_P_YAW_AU_WC = 0.0;
const double K_D_YAW_AU_WC = 0.0;

const double K_P_RIGHT_DIS = 0.15;
const double K_P_LEFT_DIS = 0.15;
const double K_P_KICKER_DIS = 0.280;

const double K_I_RIGHT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_I_KICKER_DIS = 0.0;

const double K_D_RIGHT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0;

double K_P_YAW_DIS = 0.5;
double K_I_YAW_DIS = 0.0;
double K_D_YAW_DIS = 0.0;
