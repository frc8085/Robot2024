package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.TuningModeConstants;

public class ArmSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kArmTuning;

    // Motors - Arm uses a vortex, ShooterPivot uses a 550
    private final CANSparkFlex m_armMotor = new CANSparkFlex(
            CanIdConstants.kArmCanId, MotorDefaultsConstants.NeoVortexMotorType);
    private final CANSparkMax m_shooterPivotMotor = new CANSparkMax(
            CanIdConstants.kShooterPivotCanId, MotorDefaultsConstants.Neo550MotorType);

    // Encoders
    private AbsoluteEncoder m_armEncoder;
    private AbsoluteEncoder m_shooterPivotEncoder;

    // PID Controllers
    private ProfiledPIDController m_armPIDController;
    private PIDController m_shooterPivotPIDController;

    // ProfiledPIDController Constraints
    private TrapezoidProfile.Constraints armConstraints;

    // Setpoints
    private double m_armSetpoint;
    private double m_shooterPivotSetpoint;

    // PID Constants for Tuning
    double kArmP = ArmConstants.kArmP;
    double kArmI = ArmConstants.kArmI;
    double kArmD = ArmConstants.kArmD;
    double kArmMaxVelocity = ArmConstants.kArmMaxVelocity;
    double kArmMaxAcceleration = ArmConstants.kArmMaxAcceleration;

    double kShooterPivotP = ArmConstants.kShooterPivotP;
    double kShooterPivotI = ArmConstants.kShooterPivotI;
    double kShooterPivotD = ArmConstants.kShooterPivotD;

    // Limit Switches
    private SparkLimitSwitch m_armLowerLimit;
    private SparkLimitSwitch m_armRaiseLimit;

    public boolean ArmLowerLimitHit() {
        return isArmLowerLimitHit();
    }

    public boolean ArmRaiseLimitHit() {
        return isArmRaiseLimitHit();
    }

    /** Creates a new ArmSubsystem */
    public ArmSubsystem() {

        // Factory Reset motor controllers to a known state before configuring them.

        // Arm Spark Flex
        m_armMotor.restoreFactoryDefaults();

        m_armMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_armMotor.setSmartCurrentLimit(MotorDefaultsConstants.NeoVortexCurrentLimit);

        m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // m_armEncoder = m_armMotor.getEncoder();

        m_armLowerLimit = m_armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_armRaiseLimit = m_armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // Define constraints
        armConstraints = new TrapezoidProfile.Constraints(ArmConstants.kArmMaxVelocity,
                ArmConstants.kArmMaxAcceleration);

        // Shooter Pivot Spark Max
        m_shooterPivotMotor.restoreFactoryDefaults();

        m_shooterPivotMotor.setIdleMode(ArmConstants.kShooterPivotMotorIdleMode);
        m_shooterPivotMotor.setSmartCurrentLimit(MotorDefaultsConstants.Neo550CurrentLimit);

        m_shooterPivotEncoder = m_shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // m_shooterPivotEncoder = m_shooterPivotMotor.getEncoder();

        m_armPIDController = new ProfiledPIDController(kArmP, kArmI, kArmD, armConstraints);
        m_armPIDController.reset(0);

        m_shooterPivotPIDController = new PIDController(kShooterPivotP, kShooterPivotI, kShooterPivotD);

        if (TUNING_MODE) {
            // addPIDToDashboard();
        }

        // Save the configuration to the motor controllers
        m_armMotor.burnFlash();
        m_shooterPivotMotor.burnFlash();

    }

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        if (TUNING_MODE) {
            // tunePIDs();
        }
    }

    // public void tunePIDs() {
    // kArmP = SmartDashboard.getNumber("kArmP", 0);
    // kArmI = SmartDashboard.getNumber("kArmI", 0);
    // kArmD = SmartDashboard.getNumber("kArmD", 0);
    // kArmMaxVelocity = SmartDashboard.getNumber("kArmMaxVelocity", 0);
    // kArmMaxAcceleration = SmartDashboard.getNumber("kArmMaxAcceleration", 0);
    // m_armSetpoint = SmartDashboard.getNumber("Arm Setpoint", 0);
    // SmartDashboard.putNumber("kArmP", kArmP);
    // SmartDashboard.putNumber("kArmI", kArmI);
    // SmartDashboard.putNumber("kArmD", kArmD);
    // SmartDashboard.putNumber("Arm Setpoint", m_armSetpoint);

    // kShooterPivotP = SmartDashboard.getNumber("kShooterPivotP", 0);
    // kShooterPivotI = SmartDashboard.getNumber("kShooterPivotI", 0);
    // kShooterPivotD = SmartDashboard.getNumber("kShooterPivotD", 0);
    // m_shooterPivotSetpoint = SmartDashboard.getNumber("Shooter Pivot Setpoint",
    // 0);
    // SmartDashboard.putNumber("kShooterPivotP", kShooterPivotP);
    // SmartDashboard.putNumber("kShooterPivotI", kShooterPivotI);
    // SmartDashboard.putNumber("kShooterPivotD", kShooterPivotD);
    // SmartDashboard.putNumber("Shooter Pivot Setpoint", m_shooterPivotSetpoint);

    // m_armPIDController.setP(kArmP);
    // m_armPIDController.setI(kArmI);
    // m_armPIDController.setD(kArmD);
    // m_armPIDController.setConstraints(new
    // TrapezoidProfile.Constraints(kArmMaxVelocity, kArmMaxAcceleration));
    // m_armPIDController.setGoal(new TrapezoidProfile.State(m_armSetpoint, 0.0));

    // m_shooterPivotPIDController.setP(kShooterPivotP);
    // m_shooterPivotPIDController.setI(kShooterPivotI);
    // m_shooterPivotPIDController.setD(kShooterPivotD);
    // m_shooterPivotPIDController.setSetpoint(m_shooterPivotSetpoint);

    // }

    // public void addPIDToDashboard() {
    // SmartDashboard.putNumber("kArmP", kArmP);
    // SmartDashboard.putNumber("kArmI", kArmI);
    // SmartDashboard.putNumber("kArmD", kArmD);
    // SmartDashboard.putNumber("Arm Setpoint", m_armSetpoint);
    // SmartDashboard.putNumber("kShooterPivotP", kShooterPivotP);
    // SmartDashboard.putNumber("kShooterPivotI", kShooterPivotI);
    // SmartDashboard.putNumber("kShooterPivotD", kShooterPivotD);
    // SmartDashboard.putNumber("Shooter Pivot Setpoint", m_shooterPivotSetpoint);

    // }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putNumber("Arm Position", getCurrentArmPosition());
            SmartDashboard.putNumber("Shooter Pivot Position", getCurrentShooterPivotPosition());
        }

    }

    // Limit switches - determine if they have been hit
    public boolean isArmLowerLimitHit() {
        return m_armLowerLimit.isPressed();
    }

    public boolean isArmRaiseLimitHit() {
        return m_armRaiseLimit.isPressed();
    }

    // Returns the arm position
    public double getCurrentArmPosition() {
        return m_armEncoder.getPosition();
    }

    // Returns the shooter pivot position
    public double getCurrentShooterPivotPosition() {
        return m_shooterPivotEncoder.getPosition();
    }

    // // Set up Timers
    // double lastArmSpeed = 0;
    // double lastArmTime = Timer.getFPGATimestamp();

    // Arm Mechanics using Trapezoidal Profile
    public void runArmProfile() {
        m_armPIDController.setConstraints(armConstraints);
        m_armPIDController.setGoal(new TrapezoidProfile.State(m_armSetpoint,0.0));
        double pidOutput = m_armPIDController.calculate(getCurrentArmPosition(), new TrapezoidProfile.State(m_armSetpoint,0.0));
        m_armMotor.set(pidOutput);
    
    }

    public void keepArmPosition(double armPosition) {
        m_armPIDController.setConstraints(armConstraints);
        m_armPIDController.setGoal(new TrapezoidProfile.State(armPosition,0.0));
        double armPIDOutput = m_armPIDController.calculate(getCurrentArmPosition(), new TrapezoidProfile.State(armPosition,0.0));
        m_armMotor.set(armPIDOutput);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Arm Position", armPosition);
            System.out.println("Keep ARM Position " + armPosition);
        }
    }

    // Arm Motor Manual Movement
    public void armRaise() {
        m_armMotor.set(ArmConstants.kArmRaiseSpeed);
    }

    public void armLower() {
        m_armMotor.set(-ArmConstants.kArmLowerSpeed);
    }

    public void armStop() {
        m_armMotor.set(0);
    }

    // Shooter Pivot Manual Movement
    public void shooterPivotRaise() {
        m_shooterPivotMotor.set(ArmConstants.kShooterPivotRaiseSpeed);
    }

    public void shooterPivotLower() {
        m_shooterPivotMotor.set(-ArmConstants.kShooterPivotLowerSpeed);
    }

    public void shooterPivotStop() {
        m_shooterPivotMotor.set(0);
    }


    public void keepShooterPivotPosition(double shooterPivotPosition) {
        m_shooterPivotPIDController.setSetpoint(shooterPivotPosition);

        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Shooter Pivot Position", shooterPivotPosition);
            System.out.println("Keep S_PIVOT Position " + shooterPivotPosition);
        }
    }
}

// // Maintain arm position
// public void setArmPosition(double testPosition) {
// m_armPIDController.setReference(testPosition, ControlType.kPosition);
// }

// // Maintain arm position in degrees
// public void setArmPositionDegrees(double degreesArm) {
// // set degrees for arm, convert to encoder value
// double positionArm = degreesArm * ArmConstants.kArmRevolutionsPerDegree;
// m_armPIDController.setReference(positionArm, ControlType.kPosition);
// }

// // Maintain shooter arm position
// public void setShooterPivotPosition(double testShooterPivotPosition) {
// m_shooterPivotPIDController.setReference(testShooterPivotPosition,
// ControlType.kPosition);
// }

// // Maintain shooter arm position in degrees
// public void setShooterPivotPositionDegrees(double degreesShooterPivot) {
// // set degrees for arm, convert to encoder value)
// double positionShooterPivot = degreesShooterPivot *
// ArmConstants.kShooterPivotRevolutionsPerDegree;
// m_armPIDController.setReference(positionShooterPivot, ControlType.kPosition);
// }

// public void moveToPosition(Position position) {
// setArmPositionDegrees(position.armPosition);
// setShooterPivotPositionDegrees(position.ShooterPivotPosition);
// }
