
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningModeConstants;

public class ShooterSubsystem extends SubsystemBase {

    private boolean TUNING_MODE = TuningModeConstants.kShooterTuning;

    // imports motor id
    private final CANSparkMax m_shooter1Motor = new CANSparkMax(
            CanIdConstants.kShooter1CanId, MotorDefaultsConstants.NeoMotorType);
    private final CANSparkMax m_shooter2Motor = new CANSparkMax(
            CanIdConstants.kShooter2CanId, MotorDefaultsConstants.NeoMotorType);

    // Encoders
    private RelativeEncoder m_shooter1Encoder;
    private RelativeEncoder m_shooter2Encoder;

    // PID Controllers
    private SparkPIDController m_shooter1PIDController = m_shooter1Motor.getPIDController();
    private SparkPIDController m_shooter2PIDController = m_shooter2Motor.getPIDController();

    // PID Constants for tuning
    double kShooter1P = ShooterConstants.kShooter1P;
    double kShooter1I = ShooterConstants.kShooter1I;
    double kShooter1D = ShooterConstants.kShooter1D;

    double kShooter2P = ShooterConstants.kShooter2P;
    double kShooter2I = ShooterConstants.kShooter2I;
    double kShooter2D = ShooterConstants.kShooter2D;
    // Shooter Set Points
    private double kShooter1SetPoint;
    private double kShooter2SetPoint;

    /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_shooter1Motor.restoreFactoryDefaults();
        m_shooter2Motor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the Shooter1 and shooter Shooter1s.
        m_shooter1Encoder = m_shooter1Motor.getEncoder();
        m_shooter1PIDController = m_shooter1Motor.getPIDController();
        m_shooter1PIDController.setFeedbackDevice(m_shooter1Encoder);

        m_shooter2Encoder = m_shooter2Motor.getEncoder();
        m_shooter2PIDController = m_shooter2Motor.getPIDController();
        m_shooter2PIDController.setFeedbackDevice(m_shooter2Encoder);

        m_shooter1PIDController.setP(ShooterConstants.kShooter1P);
        m_shooter1PIDController.setI(ShooterConstants.kShooter1I);
        m_shooter1PIDController.setD(ShooterConstants.kShooter1D);
        m_shooter1PIDController.setFF(ShooterConstants.kShooter1FF);
        m_shooter1PIDController.setOutputRange(ShooterConstants.kShooter1MinOutput,
                ShooterConstants.kShooter1MaxOutput);
        m_shooter1PIDController.setSmartMotionMaxAccel(0.5, 0);
        m_shooter1PIDController.setSmartMotionMaxVelocity(0.5, 0);

        m_shooter2PIDController.setP(ShooterConstants.kShooter2P);
        m_shooter2PIDController.setI(ShooterConstants.kShooter2I);
        m_shooter2PIDController.setD(ShooterConstants.kShooter2D);
        m_shooter2PIDController.setFF(ShooterConstants.kShooter2FF);
        m_shooter2PIDController.setOutputRange(ShooterConstants.kShooter2MinOutput,
                ShooterConstants.kShooter2MaxOutput);
        m_shooter2PIDController.setSmartMotionMaxAccel(0.5, 0);
        m_shooter2PIDController.setSmartMotionMaxVelocity(0.5, 0);

        m_shooter1Motor.setIdleMode(ShooterConstants.kShooterMotor1IdleMode);
        m_shooter1Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter2Motor.setIdleMode(ShooterConstants.kShooterMotor2IdleMode);
        m_shooter2Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter1Motor.burnFlash();
        m_shooter2Motor.burnFlash();

    }

    // This method will be called once per scheduler run
    public void periodic() {

        if (LoggingConstants.kLogging) {
            log();
        }

        if (TUNING_MODE) {
            tunePIDs();
        }
    }

    public void log() {
        // SmartDashboard.putNumber("Description", item);
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kShooter1P", kShooter1P);
        SmartDashboard.putNumber("kShooter1I", kShooter1I);
        SmartDashboard.putNumber("kShooter2P", kShooter2P);
        SmartDashboard.putNumber("kShooter2I", kShooter2I);
    }

    public void tunePIDs() {
        kShooter1P = SmartDashboard.getNumber("kShooter1P", 0);
        kShooter1I = SmartDashboard.getNumber("kShooter1I", 0);
        SmartDashboard.putNumber("kShooter1P", kShooter1P);
        SmartDashboard.putNumber("kShooter1I", kShooter1I);

        kShooter2P = SmartDashboard.getNumber("kShooter2P", 0);
        kShooter2I = SmartDashboard.getNumber("kShooter2I", 0);
        SmartDashboard.putNumber("kShooterP", kShooter2P);
        SmartDashboard.putNumber("kShooterI", kShooter2I);
    }

    // Stop the Shooter
    public void stop() {
        m_shooter1Motor.set(0);
        m_shooter2Motor.set(0);
    }

    public void run() {
        m_shooter1Motor.set(1);
        m_shooter2Motor.set(-0.8);
    }

    // public void run() {
    // setShooter1SetPoint(kShooter1SetPoint);
    // setShooter2SetPoint(kShooter2SetPoint);
    // }

    public void setShooter1SetPoint(double shooter1SetPoint) {
        kShooter1SetPoint = Math.max(shooter1SetPoint, 4500);
        m_shooter1PIDController.setReference(kShooter1SetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setShooter2SetPoint(double shooter2SetPoint) {
        kShooter2SetPoint = Math.max(shooter2SetPoint, 4500);
        m_shooter2PIDController.setReference(kShooter2SetPoint, CANSparkMax.ControlType.kVelocity);
    }

}
