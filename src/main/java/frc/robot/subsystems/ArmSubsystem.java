package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.TuningModeConstants;

public class ArmSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kTuning;

    // Motors
    private final CANSparkFlex m_armMotor = new CANSparkFlex(
            ArmConstants.kArmCanId, MotorType.kBrushless);
    private final CANSparkMax m_shooterArmMotor = new CANSparkMax(
            ArmConstants.kShooterArmCanId, MotorType.kBrushless);

    // Encoders
    private AbsoluteEncoder m_armEncoder;
    private AbsoluteEncoder m_shooterArmEncoder;

    // PID Controllers

    private SparkPIDController m_armPIDController = m_armMotor.getPIDController();
    private SparkPIDController m_shooterArmPIDController = m_shooterArmMotor.getPIDController();

    // PID Constants for tuning
    double kArmP = ArmConstants.kArmP;
    double kArmI = ArmConstants.kArmI;
    double kArmD = ArmConstants.kArmD;
    double kArmFF = ArmConstants.kArmFF;
    double kArmMaxOutput = ArmConstants.kArmMaxOutput;
    double kArmMinOutput = ArmConstants.kArmMinOutput;

    double kShooterArmP = ArmConstants.kShooterArmP;
    double kShooterArmI = ArmConstants.kShooterArmI;
    double kShooterArmD = ArmConstants.kShooterArmD;
    double kShooterArmFF = ArmConstants.kShooterArmFF;
    double kShooterArmMaxOutput = ArmConstants.kShooterArmMaxOutput;
    double kShooterArmMinOutput = ArmConstants.kShooterArmMinOutput;

    // limit switches
    private SparkLimitSwitch m_armLowerLimit;
    private SparkLimitSwitch m_armRaiseLimit;

    public boolean ArmLowerLimitHit() {
        return isArmLowerLimitHit();
    }

    public boolean ArmRaiseLimitHit() {
        return isArmRaiseLimitHit();
    }

    /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.

        m_armMotor.restoreFactoryDefaults();
        m_shooterArmMotor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the arm and shooter arms.
        m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_armPIDController = m_armMotor.getPIDController();
        m_armPIDController.setFeedbackDevice(m_armEncoder);

        m_shooterArmEncoder = m_shooterArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_shooterArmPIDController = m_shooterArmMotor.getPIDController();
        m_shooterArmPIDController.setFeedbackDevice(m_armEncoder);

        // Apply position and velocity conversion factors for the encoders. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
        m_armEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);

        m_shooterArmEncoder.setPositionConversionFactor(ArmConstants.kShooterArmEncoderPositionFactor);
        m_shooterArmEncoder.setVelocityConversionFactor(ArmConstants.kShooterArmEncoderVelocityFactor);

        // Invert the encoders, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        m_armEncoder.setInverted(ArmConstants.kArmEncoderInverted);
        m_shooterArmEncoder.setInverted(ArmConstants.kShooterArmEncoderInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_armPIDController.setPositionPIDWrappingEnabled(true);
        m_armPIDController.setPositionPIDWrappingMinInput(ArmConstants.kArmEncoderPositionPIDMinInput);
        m_armPIDController.setPositionPIDWrappingMaxInput(ArmConstants.kArmEncoderPositionPIDMaxInput);

        m_shooterArmPIDController.setPositionPIDWrappingEnabled(true);
        m_shooterArmPIDController
                .setPositionPIDWrappingMinInput(ArmConstants.kShooterArmEncoderPositionPIDMinInput);
        m_shooterArmPIDController
                .setPositionPIDWrappingMaxInput(ArmConstants.kShooterArmEncoderPositionPIDMaxInput);

        // Set the PID gains for the turning motor. 
        m_armPIDController.setP(ArmConstants.kArmP);
        m_armPIDController.setI(ArmConstants.kArmI);
        m_armPIDController.setD(ArmConstants.kArmD);
        m_armPIDController.setFF(ArmConstants.kArmFF);
        m_armPIDController.setOutputRange(ArmConstants.kArmMinOutput,
                ArmConstants.kArmMaxOutput);
        m_armPIDController.setSmartMotionMaxAccel(0.5, 0);
        m_armPIDController.setSmartMotionMaxVelocity(0.5, 0);

        m_shooterArmPIDController.setP(ArmConstants.kShooterArmP);
        m_shooterArmPIDController.setI(ArmConstants.kShooterArmI);
        m_shooterArmPIDController.setD(ArmConstants.kShooterArmD);
        m_shooterArmPIDController.setFF(ArmConstants.kShooterArmFF);
        m_shooterArmPIDController.setOutputRange(ArmConstants.kShooterArmMinOutput,
                ArmConstants.kShooterArmMaxOutput);
        m_shooterArmPIDController.setSmartMotionMaxAccel(0.5, 0);
        m_shooterArmPIDController.setSmartMotionMaxVelocity(0.5, 0);

        m_armMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_armMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

        m_shooterArmMotor.setIdleMode(ArmConstants.kShooterArmMotorIdleMode);
        m_shooterArmMotor.setSmartCurrentLimit(ArmConstants.kShooterArmMotorCurrentLimit);

        //Limit switches
        m_armLowerLimit = m_armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_armRaiseLimit = m_armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_armMotor.burnFlash();
        m_shooterArmMotor.burnFlash();

        if (TUNING_MODE) {
            addPIDToDashboard();
        }
    }

    // Returns the arm
    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }

    // Returns the Shooter arm
    public double getShooterArmPosition() {
        return m_shooterArmEncoder.getPosition();
    }

    // Maintain arm position in degrees
    public void setArmPositionDegrees(double degreesArm) {
        // set degrees for arm, convert to encoder value
        double positionArm = degreesArm * ArmConstants.kArmRevolutionsPerDegree;
        m_armPIDController.setReference(positionArm, ControlType.kPosition);
    }

    // Maintain shooter arm position in degrees
    public void setShooterArmPositionDegrees(double degreesShooterArm) {
        // set degrees for arm, convert to encoder value)
        double positionShooterArm = degreesShooterArm * ArmConstants.kShooterArmRevolutionsPerDegree;
        m_armPIDController.setReference(positionShooterArm, ControlType.kPosition);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        if (TUNING_MODE) {
            tunePIDs();
        }
    }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putNumber("Arm Position", getArmPosition());
            SmartDashboard.putNumber("Shooter Arm Position", getShooterArmPosition());
        }
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);
        SmartDashboard.putNumber("kShooterArmP", kShooterArmP);
        SmartDashboard.putNumber("kShooterArmI", kShooterArmI);
        SmartDashboard.putNumber("kShooterArmD", kShooterArmD);

    }

    public void tunePIDs() {
        kArmP = SmartDashboard.getNumber("kArmP", 0);
        kArmI = SmartDashboard.getNumber("kArmI", 0);
        kArmD = SmartDashboard.getNumber("kArmD", 0);
        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);

        kShooterArmP = SmartDashboard.getNumber("kShooterArmP", 0);
        kShooterArmI = SmartDashboard.getNumber("kShooterArmI", 0);
        kShooterArmD = SmartDashboard.getNumber("kShooterArmD", 0);
        SmartDashboard.putNumber("kShooterArmP", kShooterArmP);
        SmartDashboard.putNumber("kShooterArmI", kShooterArmI);
        SmartDashboard.putNumber("kShooterArmD", kShooterArmD);
    }

    // Limit Switches

    public boolean isArmLowerLimitHit() {
        return m_armLowerLimit.isPressed();
    }

    public boolean isArmRaiseLimitHit() {
        return m_armRaiseLimit.isPressed();
    }

    // Arm Motor Movements

    public void armRaise() {
        m_armMotor.set(ArmConstants.kArmRaiseSpeed);
    }

    public void armLower() {
        m_armMotor.set(-ArmConstants.kArmLowerSpeed);
    }

    public void armStop() {
        m_armMotor.set(0);
    }

    public double getCurrentArmPosition() {
        return m_armEncoder.getPosition();
    }

    public void keepArmPosition(double armPosition) {
        m_armPIDController.setReference(armPosition,ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Arm Position", armPosition);
            System.out.println("Keep ARM Position " + armPosition);
        }
    }

    public void shooterArmRaise() {
        m_shooterArmMotor.set(ArmConstants.kShooterArmRaiseSpeed);
    }

    public void shooterArmLower() {
        m_shooterArmMotor.set(-ArmConstants.kShooterArmLowerSpeed);
    }

    public void shooterArmStop() {
        m_shooterArmMotor.set(0);
    }

    public double getCurrentShooterArmPosition() {
        return m_shooterArmEncoder.getPosition();
    }

    public void keepShooterArmPosition(double shooterArmPosition) {
        m_armPIDController.setReference(shooterArmPosition,ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Shooter Arm Position", shooterArmPosition);
            System.out.println("Keep SHOOTER ARM Position " + shooterArmPosition);
        }
    }
}