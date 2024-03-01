
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
    private boolean PRACTICE_MODE = TuningModeConstants.kPracticeMode;

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
        // Factory reset, so we get the SPARK to a known state before configuring
        // them. This is useful in case a SPARK is swapped out.
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
        m_shooter1PIDController.setIZone(0);
        // m_shooter1PIDController.setSmartMotionMaxVelocity(5600, 0);
        // m_shooter1PIDController.setSmartMotionMinOutputVelocity(500, 0);
        // m_shooter1PIDController.setSmartMotionMaxAccel(3000, 0);
        m_shooter1PIDController.setSmartMotionAllowedClosedLoopError(0, 0);

        m_shooter2PIDController.setP(ShooterConstants.kShooter2P);
        m_shooter2PIDController.setI(ShooterConstants.kShooter2I);
        m_shooter2PIDController.setD(ShooterConstants.kShooter2D);
        m_shooter2PIDController.setFF(ShooterConstants.kShooter2FF);
        m_shooter2PIDController.setOutputRange(ShooterConstants.kShooter2MinOutput,
                ShooterConstants.kShooter2MaxOutput);
        m_shooter2PIDController.setIZone(0);
        // m_shooter2PIDController.setSmartMotionMaxVelocity(5600, 0);
        // m_shooter2PIDController.setSmartMotionMinOutputVelocity(500, 0);
        // m_shooter2PIDController.setSmartMotionMaxAccel(3000, 0);
        m_shooter2PIDController.setSmartMotionAllowedClosedLoopError(0, 0);

        m_shooter1Motor.setIdleMode(ShooterConstants.kShooterMotor1IdleMode);
        m_shooter1Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter2Motor.setIdleMode(ShooterConstants.kShooterMotor2IdleMode);
        m_shooter2Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter1Motor.burnFlash();
        m_shooter2Motor.burnFlash();

        if (TUNING_MODE) {
        }

    }

    // Stop the Shooter
    public void stop() {
        m_shooter1Motor.set(0);
        m_shooter2Motor.set(0);
    }

    // public void run() {
    // m_shooter1Motor.set(1);
    // m_shooter2Motor.set(-.8);
    // }

    public void setShooterSpeed(double shooterSpeed) {
        setShooter1SetPoint(shooterSpeed);
        setShooter2SetPoint(.9 * shooterSpeed);
    }

    public void run() {
        setShooter1SetPoint(kShooter1SetPoint);
        setShooter2SetPoint(kShooter2SetPoint);
    }

    public void runTrap() {
        setShooter1SetPoint(ShooterConstants.kShooterSetPointTrap);
        setShooter2SetPoint(ShooterConstants.kShooterSetPointTrap);
        System.out.println("Running Trap Score");
    }

    public void runAmp() {
        setShooter1SetPoint(ShooterConstants.kShooterSetPointAmp);
        setShooter2SetPoint(ShooterConstants.kShooterSetPointAmp);
    }

    public void setShooter1SetPoint(double shooter1SetPoint) {
        if (shooter1SetPoint >= 4500) {
            kShooter1SetPoint = 4500;
        } else {
            kShooter1SetPoint = shooter1SetPoint;
        }
        m_shooter1PIDController.setReference(kShooter1SetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setShooter2SetPoint(double shooter2SetPoint) {
        if (shooter2SetPoint >= 4500) {
            kShooter2SetPoint = 4500;
        } else {
            kShooter2SetPoint = shooter2SetPoint;
        }
        m_shooter2PIDController.setReference(kShooter2SetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public boolean shooter1AtSetpoint() {
        double encoderValue = m_shooter1Encoder.getVelocity();
        double setpoint = kShooter1SetPoint;
        double tolerance = Math.abs(ShooterConstants.kShooterToleranceRPMPercent * setpoint);

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
        double setpoint = kShooter2SetPoint;
        double tolerance = Math.abs(ShooterConstants.kShooterToleranceRPMPercent * setpoint);

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

    public boolean isShooterRunning() {
        return m_shooter1Encoder.getVelocity() > 200;
    }

    public void toggleShooter() {
        if (isShooterRunning()) {
            m_shooter1Motor.set(0);
            m_shooter2Motor.set(0);
        } else {
            setShooter1SetPoint(kShooter1SetPoint);
            setShooter2SetPoint(kShooter2SetPoint);
        }
    }

    public void practiceDashboard() {
        SmartDashboard.putBoolean("Shooter1 at SetPoint", shooter1AtSetpoint());
        SmartDashboard.putBoolean("Shooter2 at SetPoint", shooter2AtSetpoint());
        SmartDashboard.putNumber("Shooter1 Velocity", m_shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter2 Velocity", m_shooter2Encoder.getVelocity());
    }

    // This method will be called once per scheduler run
    public void periodic() {

        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
        SmartDashboard.putBoolean("Shooter is Running", isShooterRunning());
        SmartDashboard.putNumber("Shooter1 Velocity", m_shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter2 Velocity", m_shooter2Encoder.getVelocity());

        if (LoggingConstants.kLogging)

        {
            log();
        }

        if (PRACTICE_MODE) {
            practiceDashboard();
        }
        if (TUNING_MODE) {
        }
    }

    public void log() {
    }

}
