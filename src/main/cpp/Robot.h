#pragma once

using namespace frc;
#include "WPILib.h"
#include "ctre/Phoenix.h"

namespace frc668 {

class Robot:
        public CoopMTRobot,
        public JoystickObserver
{
private:
    enum AutonomousRoutine {

    };

    const char *GetAutoName(AutonomousRoutine routine);

    enum Alliance{
      Red,
      Blue
    };

    enum DriveMode{

    };

    enum BumperMode{

    };

    LogSpreadsheet *m_logger;

    PowerDistributionPanel *m_pdp;

    /**
     * Inputs (joysticks, sensors, etc...)
     */
    ObservableJoystick		*m_driverJoystick;
    ObservableJoystick		*m_operatorJoystick;
    ObservableJoystick		*m_tuningJoystick;

    /**
     * Outputs (motors, solenoids, etc...)
     */
    TalonSRX		*m_leftDriveTalonA;
    TalonSRX		*m_leftDriveTalonB;
    TalonSRX		*m_rightDriveTalonA;
    TalonSRX		*m_rightDriveTalonB;
    TalonSRX        *m_leftAgitatorTalon;
    ADXRS450_Gyro        *m_austinGyro;
    Drive			*m_drive;

    /**
     * Auto
     */
    uint32_t 					m_teleopTimer;
    int						    m_speedSetpt;
    double						m_flailSetpt;
    Alliance          m_alliance;
    double						m_conveyorSetpt;
    int               m_kickerSetpt;
    bool              m_endMode;
    DriveMode         m_driveMode;
    BumperMode        m_bumperMode;

public:
    /**
     * Defined in Robot.cpp
     */
    Robot(void);
    ~Robot(void);
    void Initialize(void) override;

    /**
     * Defined in Disabled.h
     */
    void DisabledStart(void) override;
    void DisabledStop(void) override;
    void DisabledContinuous(void) override;

    /**
     * Defined in Autonomous.h
     */
    void AutonomousStart(void) override;
    void AutonomousStop(void) override;
    void AutonomousContinuous(void) override;

    void HopperThenShoot(void);
    void MadtownHopperThenShoot(void);
    void KpaAndGearAuto(void);
    void HaltAuto(void);
    void CitrusKpaAndGearAuto(void);
    void LiteralCitrusHopperAuto(void);
    void ModifiedCitrusHopperAuto(void);
    void SpartanHopperAuto(void);
    void KillerHopperAuto(void);
    void MidPegKpaAuto(void);
    /**
     * Defined in Teleop.h
     */
    void TeleopStart(void) override;
    void TeleopStop(void) override;
    void TeleopContinuous(void) override;

    /**
     * Function called by the observable joystick whenever a joystick
     * button is pressed or released
     */
    void ObserveJoystickStateChange(uint32_t port, uint32_t button,
            bool newState) override;
    void HandleTeleopButton(uint32_t port, uint32_t button,
            bool newState);
    void HandleDisabledButton(uint32_t port, uint32_t button,
            bool newState);

    /**
     * Defined in Test.h
     */
    void TestStart(void) override;
    void TestStop(void) override;
    void TestContinuous(void) override;

    /**
     * Defined in Robot.cpp
     */
    void AllStateContinuous(void) override;

    void PrintState();
};

}
