
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningModeConstants;

public class FeederSubsystem extends SubsystemBase {

    private boolean TUNING_MODE = TuningModeConstants.kFeederTuning;

    DigitalInput lightSensor1 = new DigitalInput(ShooterConstants.kIRPort1);
    DigitalInput lightSensor2 = new DigitalInput(ShooterConstants.kIRPort2);

    private final CANSparkMax m_feederMotor = new CANSparkMax(
        CanIdConstants.kFeederCanId, MotorDefaultsConstants.Neo550MotorType);

    private double speed = FeederConstants.speed;

    /** Creates a new ExampleSubsystem. */
    public FeederSubsystem() {
        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_feederMotor.setSmartCurrentLimit(MotorDefaultsConstants.Neo550CurrentLimit);

        m_feederMotor.burnFlash();
    }

    // This method will be called once per scheduler run
    public void periodic() {

        if (LoggingConstants.kLogging) {
            log();
        }

    }

    public void log() {
        // SmartDashboard.putNumber("Arm Position", getArmPosition());
        // SmartDashboard.putNumber("Shooter Arm Position", getShooterArmPosition());
        SmartDashboard.putBoolean("Note detected", isNoteDetected());
        SmartDashboard.putBoolean("Sensor 1 note detected", lightSensor1.get());
        SmartDashboard.putBoolean("Sensor 2 note detected", lightSensor2.get());
    }

  
    // Stop the Feeder
    public void stop() {
        m_feederMotor.set(0);
    }

    public void run() {
        m_feederMotor.set(1);
    }

    public void runBackwards() {
        m_feederMotor.set(-.25);
    }

    public Boolean isNoteDetected() {
        return lightSensor1.get() || lightSensor2.get();
    }

    public boolean isNoteNotDetected() {
        return !lightSensor1.get() || !lightSensor2.get();
    }

}
