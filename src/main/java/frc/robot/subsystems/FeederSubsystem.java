
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
import frc.robot.Constants.TuningModeConstants;

public class FeederSubsystem extends SubsystemBase {

    private boolean TUNING_MODE = TuningModeConstants.kFeederTuning;
    private boolean PRACTICE_MODE = TuningModeConstants.kPracticeMode;

    DigitalInput lightSensor1 = new DigitalInput(FeederConstants.kIRPort1);
    DigitalInput lightSensor2 = new DigitalInput(FeederConstants.kIRPort2);

    private final CANSparkMax m_feederMotor = new CANSparkMax(
            CanIdConstants.kFeederCanId, MotorDefaultsConstants.Neo550MotorType);

    // Robot starts with Note
    private boolean noteTrue = true;

    // private RelativeEncoder m_feederEncoder;

    // private SparkPIDController m_feederPIDController =
    // m_feederMotor.getPIDController();

    // PID Constants for tuning
    // double kFeederP = FeederConstants.kFeederP;
    // double kFeederI = FeederConstants.kFeederI;
    // double kFeederD = FeederConstants.kFeederD;
    // double kFeederFF = FeederConstants.kFeederFF;

    // Feeder Set Points
    double kFeederSetPoint = FeederConstants.kFeederSetPoint;

    /** Creates a new ExampleSubsystem. */
    public FeederSubsystem() {
        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_feederMotor.setSmartCurrentLimit(MotorDefaultsConstants.Neo550CurrentLimit);

        // // Setup encoders and PID controllers for the Feeder and shooter Feeders.
        // m_feederEncoder = m_feederMotor.getEncoder();
        // m_feederPIDController = m_feederMotor.getPIDController();
        // m_feederPIDController.setFeedbackDevice(m_feederEncoder);

        // m_feederPIDController.setP(FeederConstants.kFeederP);
        // m_feederPIDController.setI(FeederConstants.kFeederI);
        // m_feederPIDController.setD(FeederConstants.kFeederD);
        // m_feederPIDController.setFF(FeederConstants.kFeederFF);
        // m_feederPIDController.setOutputRange(FeederConstants.kFeederMinOutput,
        // FeederConstants.kFeederMaxOutput);
        // m_feederPIDController.setIZone(10);
        // m_feederPIDController.setSmartMotionMaxVelocity(5600, 0);
        // m_feederPIDController.setSmartMotionMinOutputVelocity(500, 0);
        // m_feederPIDController.setSmartMotionMaxAccel(3000, 0);
        // m_feederPIDController.setSmartMotionAllowedClosedLoopError(50, 0);

        m_feederMotor.burnFlash();

        if (TUNING_MODE) {
        }

    }

    public void log() {
    }

    public void practiceDashboard() {
        // SmartDashboard.putNumber("Feeder Velocity", m_feederEncoder.getVelocity());
        SmartDashboard.putBoolean("Note detected", isNoteDetected());
        SmartDashboard.putBoolean("Sensor 1 note detected", lightSensor1.get());
        SmartDashboard.putBoolean("Sensor 2 note detected", lightSensor2.get());
        SmartDashboard.putBoolean("Need Note Correction", needNoteCorrection());

    }

    // Stop the Feeder
    public void stop() {
        m_feederMotor.set(0);
    }

    public void run() {
        m_feederMotor.set(FeederConstants.speed);
    }

    // Feeder Speed during Auto
    public void runAuto() {
        m_feederMotor.set(FeederConstants.speedAuto);
    }

    public void eject() {
        m_feederMotor.set(-0.6);
    }

    public void runBackwards() {
        m_feederMotor.set(-.25);
    }

    public Boolean isNoteDetected() {
        return lightSensor1.get() || lightSensor2.get();
    }

    public boolean isNoteNotDetected() {
        return !lightSensor1.get() && !lightSensor2.get();
    }

    /* When a note is picked up, it's in the robot (mind blown) */
    public void notePickedUp() {
        noteTrue = true;
    }

    /* Once a note is shot, it's not in robot (mind further blown) */
    public void noteShot() {
        noteTrue = false;
    }

    /* Give us a state when the note is in robot */
    public boolean noteInRobot() {
        return noteTrue;
    }

    /* Check if note is touching the shooter wheels */
    public boolean needNoteCorrection() {
        if (noteInRobot() && isNoteDetected()) {
            return true;
        } else {
            return false;
        }
    }

    /* Detect if robot missed note in auto */
    public boolean autoMissedNote() {
        if (noteInRobot()) {
            return false;
        } else {
            return true;
        }
    }

    // This method will be called once per scheduler run
    public void periodic() {

        // Put Indicator on Dashboard that a Note is in the Robot
        SmartDashboard.putBoolean("Note in Robot", noteInRobot());

        if (LoggingConstants.kLogging) {
            log();

        }

        if (PRACTICE_MODE) {
            practiceDashboard();
        }

        if (TUNING_MODE) {
        }

    }

}
