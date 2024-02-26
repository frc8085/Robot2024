
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

    DigitalInput lightSensor1 = new DigitalInput(FeederConstants.kIRPort1);
    DigitalInput lightSensor2 = new DigitalInput(FeederConstants.kIRPort2);

    private final CANSparkMax m_feederMotor = new CANSparkMax(
            CanIdConstants.kFeederCanId, MotorDefaultsConstants.Neo550MotorType);

    private double speed = FeederConstants.speed;

    // Robot starts with Note
    private boolean noteTrue = true;

    private RelativeEncoder m_feederEncoder;

    private SparkPIDController m_feederPIDController = m_feederMotor.getPIDController();

    // PID Constants for tuning
    double kFeederP = FeederConstants.kFeederP;
    double kFeederI = FeederConstants.kFeederI;
    double kFeederD = FeederConstants.kFeederD;
    double kFeederFF = FeederConstants.kFeederFF;

    // Feeder Set Points
    double kFeederSetPoint = FeederConstants.kFeederSetPoint;

    /** Creates a new ExampleSubsystem. */
    public FeederSubsystem() {
        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_feederMotor.setSmartCurrentLimit(MotorDefaultsConstants.Neo550CurrentLimit);

        // Setup encoders and PID controllers for the Feeder and shooter Feeders.
        m_feederEncoder = m_feederMotor.getEncoder();
        m_feederPIDController = m_feederMotor.getPIDController();
        m_feederPIDController.setFeedbackDevice(m_feederEncoder);

        m_feederPIDController.setP(FeederConstants.kFeederP);
        m_feederPIDController.setI(FeederConstants.kFeederI);
        m_feederPIDController.setD(FeederConstants.kFeederD);
        m_feederPIDController.setFF(FeederConstants.kFeederFF);
        m_feederPIDController.setOutputRange(FeederConstants.kFeederMinOutput,
                FeederConstants.kFeederMaxOutput);
        m_feederPIDController.setIZone(10);
        m_feederPIDController.setSmartMotionMaxVelocity(5600, 0);
        m_feederPIDController.setSmartMotionMinOutputVelocity(500, 0);
        m_feederPIDController.setSmartMotionMaxAccel(3000, 0);
        m_feederPIDController.setSmartMotionAllowedClosedLoopError(50, 0);

        m_feederMotor.burnFlash();

        if (TUNING_MODE) {
            addPIDToDashboard();
        }

    }

    // This method will be called once per scheduler run
    public void periodic() {

        SmartDashboard.putBoolean("Note in Robot", noteInRobot());

        if (LoggingConstants.kLogging) {
            log();
        }

        if (TUNING_MODE) {
            tunePIDs();
            sensorReadings();
        }

    }

    public void log() {

    }

    public void sensorReadings() {
        // SmartDashboard.putNumber("Arm Position", getArmPosition());
        // SmartDashboard.putNumber("Shooter Arm Position", getShooterArmPosition());
        SmartDashboard.putNumber("Feeder Velocity", m_feederEncoder.getVelocity());
        SmartDashboard.putBoolean("Note detected", isNoteDetected());
        SmartDashboard.putBoolean("Sensor 1 note detected", lightSensor1.get());
        SmartDashboard.putBoolean("Sensor 2 note detected", lightSensor2.get());
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kFeederP", kFeederP);
        SmartDashboard.putNumber("kFeederI", kFeederI);
        SmartDashboard.putNumber("kFeederD", kFeederD);
        SmartDashboard.putNumber("kFeederFF", kFeederFF);
        SmartDashboard.putNumber("kFeederSetPoint", kFeederSetPoint);
    }

    public void tunePIDs() {
        double feederP = SmartDashboard.getNumber("kFeederP", 0);
        double feederI = SmartDashboard.getNumber("kFeederI", 0);
        double feederD = SmartDashboard.getNumber("kFeederD", 0);
        double feederFF = SmartDashboard.getNumber("kFeederFF", 0);
        double shooter2P = SmartDashboard.getNumber("kShooter2P", 0);
        double shooter2I = SmartDashboard.getNumber("kShooter2I", 0);
        double shooter2D = SmartDashboard.getNumber("kShooter2D", 0);
        double shooter2FF = SmartDashboard.getNumber("kShooter2FF", 0);

        double feederSetPoint = SmartDashboard.getNumber("kFeederSetPoint", 0);
        double shooter2SetPoint = SmartDashboard.getNumber("kShooter2SetPoint", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((feederP != kFeederP)) {
            kFeederP = feederP;
            m_feederPIDController.setP(kFeederP);
        }
        if ((feederI != kFeederI)) {
            kFeederI = feederI;
            m_feederPIDController.setI(kFeederI);
        }
        if ((feederD != kFeederD)) {
            kFeederD = feederD;
            m_feederPIDController.setI(kFeederD);
        }
        if ((feederSetPoint != kFeederSetPoint)) {
            kFeederSetPoint = feederSetPoint;
            m_feederPIDController.setReference(kFeederSetPoint, CANSparkMax.ControlType.kVelocity);
        }
        if ((feederFF != kFeederFF)) {
            kFeederFF = feederFF;
            m_feederPIDController.setFF(kFeederFF);
        }
    }

    // Stop the Feeder
    public void stop() {
        m_feederMotor.set(0);
    }

    public void run() {
        m_feederMotor.set(1);
    }
    // public void run() {
    // setFeederSetPoint(kFeederSetPoint);
    // }

    public void eject() {
        m_feederMotor.set(-0.6);
    }

    public void runBackwards() {
        m_feederMotor.set(-.25);
    }

    public void setFeederSetPoint(double feederSetPoint) {
        if (feederSetPoint >= 5500) {
            kFeederSetPoint = 5500;
        } else {
            kFeederSetPoint = feederSetPoint;
        }
        m_feederPIDController.setReference(kFeederSetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public Boolean isNoteDetected() {
        return lightSensor1.get() || lightSensor2.get();
    }

    public boolean isNoteNotDetected() {
        return !lightSensor1.get() && !lightSensor2.get();
    }

    /* Give us a state when the note is in robot */
    public void notePickedUp() {
        noteTrue = true;
    }

    /* Unlock the climber */
    public void noteShot() {
        noteTrue = false;
    }

    public boolean noteInRobot() {
        return noteTrue;
    }

}
