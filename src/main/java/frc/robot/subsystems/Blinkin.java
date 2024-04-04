package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.LoggingConstants;

public class Blinkin extends SubsystemBase {

    private double m_color = 0.0;
    private Spark m_blinkin = new Spark(BlinkinConstants.pwmPort);

    public Blinkin() {

    }

    public void withNote() {
        // green .77
        m_blinkin.set(0.77);
    }

    public void shooterOn() {
        // yellow flashing light - shooter motor on
        // STROBE_GOLD(-0.07),
        m_blinkin.set(-0.07);
    }

    public void shooterAtSetPoint() {
        // blue .87
        m_blinkin.set(0.87);
    }

    public void intakeOn() {
        // HEARTBEAT_RED(-0.25),
        m_blinkin.set(-.11);
    }

    public void driving() {
        // black .99
        m_blinkin.set(0.99);
    }

    public void climbed() {
        // RAINBOW_PARTY_PALETTE(-0.97),
        m_blinkin.set(-0.97);
    }

    @Override
    public void periodic() {
        log();

    }

    public void log() {
        if (LoggingConstants.kLogging) {
        }
    }

}