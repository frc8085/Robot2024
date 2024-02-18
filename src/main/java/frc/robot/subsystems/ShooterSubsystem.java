
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
import frc.robot.commands.Shoot;

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
    double kShooter1FF = ShooterConstants.kShooter1FF;

    double kShooter2P = ShooterConstants.kShooter2P;
    double kShooter2I = ShooterConstants.kShooter2I;
    double kShooter2D = ShooterConstants.kShooter2D;
    double kShooter2FF = ShooterConstants.kShooter2FF;

    // Shooter Set Points
    double kShooter1SetPoint = ShooterConstants.kShooter1SetPoint;
    double kShooter2SetPoint = ShooterConstants.kShooter2SetPoint;

    /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_shooter1Motor.restoreFactoryDefaults();
        m_shooter2Motor.restoreFactoryDefaults();

        m_shooter1Motor.setInverted(false);
        m_shooter2Motor.setInverted(true);

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
        m_shooter1PIDController.setIZone(10);
        m_shooter1PIDController.setSmartMotionMaxVelocity(5600, 0);
        m_shooter1PIDController.setSmartMotionMinOutputVelocity(500, 0);
        m_shooter1PIDController.setSmartMotionMaxAccel(3000, 0);
        m_shooter1PIDController.setSmartMotionAllowedClosedLoopError(50, 0);

        m_shooter2PIDController.setP(ShooterConstants.kShooter2P);
        m_shooter2PIDController.setI(ShooterConstants.kShooter2I);
        m_shooter2PIDController.setD(ShooterConstants.kShooter2D);
        m_shooter2PIDController.setFF(ShooterConstants.kShooter2FF);
        m_shooter2PIDController.setOutputRange(ShooterConstants.kShooter2MinOutput,
                ShooterConstants.kShooter2MaxOutput);
        m_shooter2PIDController.setIZone(10);
        m_shooter2PIDController.setSmartMotionMaxVelocity(5600, 0);
        m_shooter2PIDController.setSmartMotionMinOutputVelocity(500, 0);
        m_shooter2PIDController.setSmartMotionMaxAccel(3000, 0);
        m_shooter2PIDController.setSmartMotionAllowedClosedLoopError(50, 0);

        m_shooter1Motor.setIdleMode(ShooterConstants.kShooterMotor1IdleMode);
        m_shooter1Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter2Motor.setIdleMode(ShooterConstants.kShooterMotor2IdleMode);
        m_shooter2Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter1Motor.burnFlash();
        m_shooter2Motor.burnFlash();

        if (TUNING_MODE) {
            addPIDToDashboard();
        }

    }

    // This method will be called once per scheduler run
    public void periodic() {

        SmartDashboard.putBoolean("Shooter1 at SetPoint", shooter1AtSetpoint());
        SmartDashboard.putBoolean("Shooter2 at SetPoint", shooter2AtSetpoint());
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());

        if (LoggingConstants.kLogging)

        {
            log();
        }

        if (TUNING_MODE) {
            tunePIDs();
        }
    }

    public void log() {
        SmartDashboard.putNumber("Shooter1 Velocity", m_shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter2 Velocity", m_shooter2Encoder.getVelocity());
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kShooter1P", kShooter1P);
        SmartDashboard.putNumber("kShooter1I", kShooter1I);
        SmartDashboard.putNumber("kShooter1D", kShooter1D);
        SmartDashboard.putNumber("kShooter1FF", kShooter1FF);

        SmartDashboard.putNumber("kShooter2P", kShooter2P);
        SmartDashboard.putNumber("kShooter2I", kShooter2I);
        SmartDashboard.putNumber("kShooter2D", kShooter2D);
        SmartDashboard.putNumber("kShooter2FF", kShooter2FF);

        SmartDashboard.putNumber("kShooter1SetPoint", kShooter1SetPoint);
        SmartDashboard.putNumber("kShooter2SetPoint", kShooter2SetPoint);
    }

    public void tunePIDs() {
        double shooter1P = SmartDashboard.getNumber("kShooter1P", 0);
        double shooter1I = SmartDashboard.getNumber("kShooter1I", 0);
        double shooter1D = SmartDashboard.getNumber("kShooter1D", 0);
        double shooter1FF = SmartDashboard.getNumber("kShooter1FF", 0);
        double shooter2P = SmartDashboard.getNumber("kShooter2P", 0);
        double shooter2I = SmartDashboard.getNumber("kShooter2I", 0);
        double shooter2D = SmartDashboard.getNumber("kShooter2D", 0);
        double shooter2FF = SmartDashboard.getNumber("kShooter2FF", 0);

        double shooter1SetPoint = SmartDashboard.getNumber("kShooter1SetPoint", 0);
        double shooter2SetPoint = SmartDashboard.getNumber("kShooter2SetPoint", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((shooter1P != kShooter1P)) {
            kShooter1P = shooter1P;
            m_shooter1PIDController.setP(kShooter1P);
        }
        if ((shooter1I != kShooter1I)) {
            kShooter1I = shooter1I;
            m_shooter1PIDController.setI(kShooter1I);
        }
        if ((shooter1D != kShooter1D)) {
            kShooter1D = shooter1D;
            m_shooter1PIDController.setI(kShooter1D);
        }
        if ((shooter2P != kShooter2P)) {
            kShooter2P = shooter2P;
            m_shooter2PIDController.setP(kShooter2P);
        }
        if ((shooter2I != kShooter2I)) {
            kShooter2I = shooter2I;
            m_shooter2PIDController.setI(kShooter2I);
        }
        if ((shooter2D != kShooter2D)) {
            kShooter2D = shooter2D;
            m_shooter2PIDController.setI(kShooter2D);
        }
        if ((shooter1SetPoint != kShooter1SetPoint)) {
            kShooter1SetPoint = shooter1SetPoint;
            m_shooter1PIDController.setReference(kShooter1SetPoint, CANSparkMax.ControlType.kVelocity);
        }
        if ((shooter2SetPoint != kShooter2SetPoint)) {
            kShooter2SetPoint = shooter2SetPoint;
            m_shooter2PIDController.setReference(kShooter2SetPoint, CANSparkMax.ControlType.kVelocity);
        }
        if ((shooter1FF != kShooter1FF)) {
            kShooter1FF = shooter1FF;
            m_shooter1PIDController.setFF(kShooter1FF);
        }
        if ((shooter2FF != kShooter2FF)) {
            kShooter2FF = shooter2FF;
            m_shooter2PIDController.setFF(kShooter2FF);
        }

    }

    // Stop the Shooter
    public void stop() {
        m_shooter1Motor.set(0);
        m_shooter2Motor.set(0);
    }

    // public void run() {
    // m_shooter1Motor.set(1);
    // m_shooter2Motor.set(-0.8);
    // }

    public void run() {
        setShooter1SetPoint(kShooter1SetPoint);
        setShooter2SetPoint(kShooter2SetPoint);
    }

    public void setShooter1SetPoint(double shooter1SetPoint) {
        if (shooter1SetPoint >= 5500) {
            kShooter1SetPoint = 5500;
        } else {
            kShooter1SetPoint = shooter1SetPoint;
        }
        m_shooter1PIDController.setReference(kShooter1SetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setShooter2SetPoint(double shooter2SetPoint) {
        if (shooter2SetPoint >= 5500) {
            kShooter2SetPoint = 5500;
        } else {
            kShooter2SetPoint = shooter2SetPoint;
        }
        m_shooter2PIDController.setReference(kShooter2SetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public boolean shooter1AtSetpoint() {
        double encoderValue = m_shooter1Encoder.getVelocity();
        // double tolerance = Math.abs(kShooterToleranceRPMPercent * kSetPoint);
        double tolerance = 300;
        double setpoint = kShooter1SetPoint;
        double minLimit = setpoint - tolerance;
        double maxLimit = setpoint + tolerance;

        boolean withinLimits =
                // Don't consider us at setpoint for the 'motor off' case
                setpoint != 0 &&
                // Otherwise check if we're within limits
                        encoderValue >= minLimit
                        && encoderValue <= maxLimit;

        return withinLimits;
    }

    public boolean shooter2AtSetpoint() {
        double encoderValue = m_shooter2Encoder.getVelocity();
        // double tolerance = Math.abs(kShooterToleranceRPMPercent * kSetPoint);
        double tolerance = 300;
        double setpoint = kShooter2SetPoint;
        double minLimit = setpoint - tolerance;
        double maxLimit = setpoint + tolerance;

        boolean shooter2WithinLimits =
                // Don't consider us at setpoint for the 'motor off' case
                setpoint != 0 &&
                // Otherwise check if we're within limits
                        encoderValue >= minLimit
                        && encoderValue <= maxLimit;

        return shooter2WithinLimits;
    }

    public boolean readyToShoot() {
        return shooter1AtSetpoint() || shooter2AtSetpoint();

    }
}
