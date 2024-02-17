package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningModeConstants;

public class ArmSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kArmTuning;

    // Motors - Arm uses a vortex, ShooterPivot uses a 550
    private final CANSparkFlex m_armMotor = new CANSparkFlex(
            CanIdConstants.kArmCanId, MotorDefaultsConstants.NeoVortexMotorType);
    private final CANSparkMax m_shooterPivotMotor = new CANSparkMax(
            CanIdConstants.kShooterPivotCanId, MotorDefaultsConstants.Neo550MotorType);

    // Encoders
    private SparkAbsoluteEncoder m_armEncoder;
    private SparkAbsoluteEncoder m_shooterPivotEncoder;

    // PID Controllers
    private SparkPIDController m_armPIDController = m_armMotor.getPIDController();
    private SparkPIDController m_shooterPivotPIDController = m_shooterPivotMotor.getPIDController();

    // PID Constants for tuning
    double kArmP = ArmConstants.kArmP;
    double kArmI = ArmConstants.kArmI;
    double kArmD = ArmConstants.kArmD;
    double kArmFF = ArmConstants.kArmFF;
    double kArmMaxOutput = ArmConstants.kArmMaxOutput;
    double kArmMinOutput = ArmConstants.kArmMinOutput;

    double kShooterPivotP = ArmConstants.kShooterPivotP;
    double kShooterPivotI = ArmConstants.kShooterPivotI;
    double kShooterPivotD = ArmConstants.kShooterPivotD;
    double kShooterPivotFF = ArmConstants.kShooterPivotFF;
    double kShooterPivotMaxOutput = ArmConstants.kShooterPivotMaxOutput;
    double kShooterPivotMinOutput = ArmConstants.kShooterPivotMinOutput;

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
        m_shooterPivotMotor.restoreFactoryDefaults();

        m_armMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_armMotor.setSmartCurrentLimit(MotorDefaultsConstants.NeoVortexCurrentLimit);

        m_shooterPivotMotor.setIdleMode(ArmConstants.kShooterPivotMotorIdleMode);
        m_shooterPivotMotor.setSmartCurrentLimit(MotorDefaultsConstants.Neo550CurrentLimit);

        // Setup encoders and PID controllers for the arm and shooter arms.

        // absolute encoder
        m_armEncoder = m_armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_armPIDController.setFeedbackDevice(m_armEncoder);

        // absolute encoder
        m_shooterPivotEncoder = m_shooterPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_shooterPivotPIDController.setFeedbackDevice(m_shooterPivotEncoder);

        // Set Position Conversion Factor which will take the encoder units and set it to degrees
        m_armEncoder.setPositionConversionFactor(ArmConstants.kArmPositionAdjustmentFactor);
        m_shooterPivotEncoder.setPositionConversionFactor(ArmConstants.kShooterPivotPositionAdjustmentFactor);

        // Encoders are not inverted
        m_armEncoder.setInverted(false);
        m_shooterPivotEncoder.setInverted(false);

        // Set Zero Offset - not sure how this works, test and figure out
        m_armEncoder.setZeroOffset(0);
        m_shooterPivotEncoder.setZeroOffset(0);

        // Set Wrapping 

        // Set the PID gains for the turning motor.
        m_armPIDController.setP(ArmConstants.kArmP);
        m_armPIDController.setI(ArmConstants.kArmI);
        m_armPIDController.setD(ArmConstants.kArmD);
        m_armPIDController.setFF(ArmConstants.kArmFF);
        m_armPIDController.setOutputRange(ArmConstants.kArmMinOutput,
                ArmConstants.kArmMaxOutput);
        m_armPIDController.setSmartMotionMaxAccel(0.5, 0);
        m_armPIDController.setSmartMotionMaxVelocity(0.5, 0);

        m_shooterPivotPIDController.setP(ArmConstants.kShooterPivotP);
        m_shooterPivotPIDController.setI(ArmConstants.kShooterPivotI);
        m_shooterPivotPIDController.setD(ArmConstants.kShooterPivotD);
        m_shooterPivotPIDController.setFF(ArmConstants.kShooterPivotFF);
        m_shooterPivotPIDController.setOutputRange(ArmConstants.kShooterPivotMinOutput,
                ArmConstants.kShooterPivotMaxOutput);
        // m_shooterPivotPIDController.setSmartMotionMaxAccel(0.5, 0);
        // m_shooterPivotPIDController.setSmartMotionMaxVelocity(0.5, 0);

        // Enable PID wrap around for the ShooterPivot motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_shooterPivotPIDController.setPositionPIDWrappingEnabled(false);
        m_shooterPivotPIDController.setPositionPIDWrappingMaxInput(.8);
        m_shooterPivotPIDController.setPositionPIDWrappingMinInput(0.0);

        // Limit switches
        m_armLowerLimit = m_armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_armRaiseLimit = m_armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_armMotor.burnFlash();
        m_shooterPivotMotor.burnFlash();

        if (TUNING_MODE) {
            addPIDToDashboard();
        }
    }

    // Returns the arm
    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }

    // Returns the Shooter arm
    public double getShooterPivotPosition() {
        return m_shooterPivotEncoder.getPosition();
    }

    // Maintain arm position
    public void setArmPosition(double armPosition) {
        m_armPIDController.setReference(armPosition, ControlType.kPosition);
    }

    // Maintain arm position in degrees
    public void setArmPositionDegrees(double armPositionDegrees) {
        // set degrees for arm, convert to encoder value
        double armPosition = armPositionDegrees * ArmConstants.kArmRevolutionsPerDegree;
        m_armPIDController.setReference(armPosition, ControlType.kPosition);
    }

    // Maintain shooter arm position
    public void setShooterPivotPosition(double shooterPivotPosition) {
        m_shooterPivotPIDController.setReference(shooterPivotPosition, ControlType.kPosition);
    }

    // Maintain shooter arm position in degrees
    public void setShooterPivotPositionDegrees(double shooterPivotPositionDegrees) {
        // set degrees for arm, convert to encoder value)
        double shooterPivotPosition = shooterPivotPositionDegrees * ArmConstants.kShooterPivotRevolutionsPerDegree;
        m_armPIDController.setReference(shooterPivotPosition, ControlType.kPosition);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        if (TUNING_MODE) {
            tunePIDs();
        }
    }

    public void moveToPosition(Position position) {
        setArmPositionDegrees(position.armPosition);
        setShooterPivotPositionDegrees(position.ShooterPivotPosition);
    }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putNumber("Arm Position", getArmPosition());
            SmartDashboard.putNumber("Shooter Arm Position", getShooterPivotPosition());
        }
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);
        SmartDashboard.putNumber("kShooterPivotP", kShooterPivotP);
        SmartDashboard.putNumber("kShooterPivotI", kShooterPivotI);
        SmartDashboard.putNumber("kShooterPivotD", kShooterPivotD);

    }

    public void tunePIDs() {
        double armP = SmartDashboard.getNumber("kArmP", 0);
        double armI = SmartDashboard.getNumber("kArmI", 0);
        double armD = SmartDashboard.getNumber("kArmD", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((armP != kArmP)) {
            kArmP = armP;
            m_armPIDController.setP(kArmP);
        }
        if ((armI != kArmI)) {
            kArmI = armI;
            m_armPIDController.setI(kArmI);
        }
        if ((armD != kArmD)) {
            kArmD = armD;
            m_armPIDController.setD(kArmD);
        }

        double shooterPivotP = SmartDashboard.getNumber("kShooterPivotP", 0);
        double shooterPivotI = SmartDashboard.getNumber("kShooterPivotI", 0);
        double shooterPivotD = SmartDashboard.getNumber("kShooterPivotD", 0);
 
        // if PID coefficients on dashboard have changed, write new values to controller
        if ((shooterPivotP != kShooterPivotP)) {
            kShooterPivotP = shooterPivotP;
            m_shooterPivotPIDController.setP(kShooterPivotP);
        }
        if ((shooterPivotI != kShooterPivotI)) {
            kShooterPivotI = shooterPivotI;
            m_shooterPivotPIDController.setI(kShooterPivotI);
        }
        if ((shooterPivotD != kShooterPivotD)) {
            kShooterPivotD = shooterPivotD;
            m_shooterPivotPIDController.setD(kShooterPivotD);
        }
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
        m_armPIDController.setReference(armPosition, ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Arm Position", armPosition);
            System.out.println("Keep ARM Position " + armPosition);
        }
    }

    public void shooterPivotRaise() {
        m_shooterPivotMotor.set(ArmConstants.kShooterPivotRaiseSpeed);
    }

    public void shooterPivotLower() {
        m_shooterPivotMotor.set(-ArmConstants.kShooterPivotLowerSpeed);
    }

    public void shooterPivotStop() {
        m_shooterPivotMotor.set(0);
    }

    public double getCurrentShooterPivotPosition() {
        return m_shooterPivotEncoder.getPosition();
    }

    public void keepShooterPivotPosition(double ShooterPivotPosition) {
        m_shooterPivotPIDController.setReference(ShooterPivotPosition, ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Shooter Arm Position", ShooterPivotPosition);
            System.out.println("Keep SHOOTER ARM Position " + ShooterPivotPosition);
        }
    }


    // if arm encoder reads greater than .4 only allow arm movement in the positive direction


}
