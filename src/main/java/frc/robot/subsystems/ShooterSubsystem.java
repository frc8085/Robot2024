
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    Blinkin m_blinkin;

    // imports motor id
    private final CANSparkFlex m_shooter1Motor = new CANSparkFlex(
            CanIdConstants.kShooter1CanId, MotorDefaultsConstants.NeoVortexMotorType);
    private final CANSparkFlex m_shooter2Motor = new CANSparkFlex(
            CanIdConstants.kShooter2CanId, MotorDefaultsConstants.NeoVortexMotorType);

    // log titles
    private static final String SHOOTER1_LOG_ENTRY = "/Shooter1";
    private static final String SHOOTER2_LOG_ENTRY = "/Shooter2";
    private static final String DESIRED_SHOOTER1_LOG_ENTRY = "/DesiredShooter1";
    private static final String DESIRED_SHOOTER2_LOG_ENTRY = "/ShooterShooter2";

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
    public ShooterSubsystem(Blinkin blinkin) {

        m_blinkin = blinkin;

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

        m_shooter1PIDController.setP(ShooterConstants.kShooter1P, 0);
        m_shooter1PIDController.setI(ShooterConstants.kShooter1I, 0);
        m_shooter1PIDController.setD(ShooterConstants.kShooter1D, 0);
        m_shooter1PIDController.setFF(ShooterConstants.kShooter1FF, 0);
        m_shooter1PIDController.setOutputRange(ShooterConstants.kShooter1MinOutput,
                ShooterConstants.kShooter1MaxOutput);
        m_shooter1PIDController.setIZone(0);
        m_shooter1PIDController.setSmartMotionAllowedClosedLoopError(0, 0);

        m_shooter1PIDController.setP(0.0006, 1);
        m_shooter1PIDController.setFF(0.000175, 1);
        m_shooter2PIDController.setP(0.0006, 1);
        m_shooter2PIDController.setFF(0.000175, 1);

        m_shooter2PIDController.setP(ShooterConstants.kShooter2P, 0);
        m_shooter2PIDController.setI(ShooterConstants.kShooter2I, 0);
        m_shooter2PIDController.setD(ShooterConstants.kShooter2D, 0);
        m_shooter2PIDController.setFF(ShooterConstants.kShooter2FF, 0);
        m_shooter2PIDController.setOutputRange(ShooterConstants.kShooter2MinOutput,
                ShooterConstants.kShooter2MaxOutput);
        m_shooter2PIDController.setIZone(0);
        m_shooter2PIDController.setSmartMotionAllowedClosedLoopError(0, 0);

        m_shooter1Motor.setIdleMode(ShooterConstants.kShooterMotor1IdleMode);
        m_shooter1Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter2Motor.setIdleMode(ShooterConstants.kShooterMotor2IdleMode);
        m_shooter2Motor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_shooter1Motor.burnFlash();
        m_shooter2Motor.burnFlash();

    }

    // Stop the Shooter
    public void stop() {
        m_shooter1Motor.setVoltage(0);
        m_shooter2Motor.setVoltage(0);
    }

    public void runTest() {
        m_shooter1Motor.set(1);
        m_shooter2Motor.set(-1);
    }

    public void setShooterSpeed(double shooterSpeed) {
        setShooter1SetPoint(shooterSpeed);
        setShooter2SetPoint(.9 * shooterSpeed);
    }

    public void run() {
        setShooter1SetPoint(kShooter1SetPoint);
        setShooter2SetPoint(kShooter2SetPoint);
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, kShooter1SetPoint);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, kShooter2SetPoint);
        }

    }

    public void runResetTrap() {
        setShooter1SetPoint(ShooterConstants.kShooterResetTrap);
        setShooter2SetPoint(ShooterConstants.kShooterResetTrap);
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, ShooterConstants.kShooterResetTrap);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, ShooterConstants.kShooterResetTrap);
        }

    }

    public void runTrap() {
        setShooter1SetPoint(ShooterConstants.kShooterSetPointTrap);
        setShooter2SetPoint(ShooterConstants.kShooterSetPointTrap);
        System.out.println("Running Trap Score");
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, ShooterConstants.kShooterSetPointTrap);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, ShooterConstants.kShooterSetPointTrap);
        }
    }

    public void runAmp() {
        setShooter1SetPoint(ShooterConstants.kShooterSetPointAmp);
        setShooter2SetPoint(ShooterConstants.kShooterSetPointAmp);
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, ShooterConstants.kShooterSetPointAmp);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, ShooterConstants.kShooterSetPointAmp);
        }
    }

    public void runPickup() {
        setShooter1SetPoint(ShooterConstants.kShooterSetPointPickup);
        setShooter2SetPoint(ShooterConstants.kShooterSetPointPickup);
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, ShooterConstants.kShooterSetPointPickup);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, ShooterConstants.kShooterSetPointPickup);
        }
    }

    public void runHold() {
        setShooter1SetPoint(0);
        setShooter2SetPoint(0);
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, 0);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, 0);
        }
    }

    public void runFeeder() {
        setShooter1SetPoint(ShooterConstants.kShooterSetPointFeeder);
        setShooter2SetPoint(ShooterConstants.kShooterSetPointFeeder);
        if (LoggingConstants.kLogging) {
            Logger.recordOutput(getName() + DESIRED_SHOOTER1_LOG_ENTRY, ShooterConstants.kShooterSetPointFeeder);
            Logger.recordOutput(getName() + DESIRED_SHOOTER2_LOG_ENTRY, ShooterConstants.kShooterSetPointFeeder);
        }
    }

    public void setShooter1SetPoint(double shooter1SetPoint) {
        if (shooter1SetPoint >= 5400) {
            kShooter1SetPoint = 5400;
        } else {
            kShooter1SetPoint = shooter1SetPoint;
        }
        m_shooter1PIDController.setReference(kShooter1SetPoint, CANSparkFlex.ControlType.kVelocity, 0);
        m_blinkin.shooterOn();
    }

    public void setShooter2SetPoint(double shooter2SetPoint) {
        if (shooter2SetPoint >= 5400) {
            kShooter2SetPoint = 5400;
        } else {
            kShooter2SetPoint = shooter2SetPoint;
        }
        m_shooter2PIDController.setReference(kShooter2SetPoint, CANSparkFlex.ControlType.kVelocity, 0);
    }

    public boolean shooter1AtPodiumSetpoint() {
        double encoderValue = m_shooter1Encoder.getVelocity();
        double setpoint = kShooter1SetPoint;
        double tolerance = Math.abs(ShooterConstants.kShooter1PodiumToleranceRPMPercent * setpoint);

        double minLimit = setpoint - tolerance;
        // double maxLimit = setpoint + 2 * tolerance;

        boolean withinLimits =
                // Don't consider us at setpoint for the 'motor off' case
                setpoint != 0 &&
                // Otherwise check if we're within limits
                        encoderValue >= minLimit;

        return withinLimits;
    }

    public boolean shooter1AtSWSetpoint() {
        double encoderValue = m_shooter1Encoder.getVelocity();
        double setpoint = kShooter1SetPoint;
        double tolerance = Math.abs(ShooterConstants.kShooter1SWToleranceRPMPercent * setpoint);

        double minLimit = setpoint - tolerance;
        // double maxLimit = setpoint + tolerance;

        boolean withinLimits =
                // Don't consider us at setpoint for the 'motor off' case
                setpoint != 0 &&
                // Otherwise check if we're within limits
                        encoderValue >= minLimit;
        // && encoderValue <= maxLimit;

        return withinLimits;
    }

    public void shooterAtSetPoint() {
        m_blinkin.shooterAtSetPoint();
    }

    public boolean readyToShootPodium() {
        return shooter1AtPodiumSetpoint();
    }

    public boolean readyToShootSW() {
        return shooter1AtSWSetpoint();
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

    // This method will be called once per scheduler run
    public void periodic() {

        SmartDashboard.putBoolean("Ready To Shoot", readyToShootPodium());
        SmartDashboard.putBoolean("Shooter is Running", isShooterRunning());
        SmartDashboard.putBoolean("SW Ready To Shoot", readyToShootSW());

        SmartDashboard.putNumber("Shooter1 Velocity", m_shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter2 Velocity", m_shooter2Encoder.getVelocity());

        if (LoggingConstants.kLogging) {
            log();
            logOutputs();
        }

        if (readyToShootPodium()) {
            shooterAtSetPoint();
        }

    }

    public void log() {
    }

    private void logOutputs() {
        Logger.recordOutput(getName() + SHOOTER1_LOG_ENTRY, m_shooter1Encoder.getVelocity());
        Logger.recordOutput(getName() + SHOOTER2_LOG_ENTRY, m_shooter2Encoder.getVelocity());
    }

}
