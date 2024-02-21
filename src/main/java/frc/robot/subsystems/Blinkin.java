package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.LoggingConstants;
import frc.utils.BlinkinLEDController;

public class Blinkin extends SubsystemBase {

    private double m_color = 0.0;
    private Spark m_blinkin = new Spark(BlinkinConstants.pwmPort);

    public Blinkin() {

    }

    public void withNote() {
        m_blinkin.set(.91);
    }

    public void shooterAtSetPoint() {
        m_blinkin.set(0.69);
    }

    public void driving() {
        m_blinkin.set(0.37);
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