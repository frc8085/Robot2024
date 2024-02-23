package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

import static frc.robot.Constants.ArmConstants.kArmPositionShift;
import static frc.robot.Constants.ArmConstants.kShooterPivotMax;
import static frc.robot.Constants.ArmConstants.kShooterPivotMin;
import static frc.robot.Constants.ArmConstants.kShooterPivotPositionShift;

import org.opencv.core.Point;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

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
    private SparkAbsoluteEncoder m_armEncoder;
    private SparkAbsoluteEncoder m_shooterPivotEncoder;

    // PID Controllers
    private SparkPIDController m_armPIDController = m_armMotor.getPIDController();
    private SparkPIDController m_shooterPivotPIDController = m_shooterPivotMotor.getPIDController();

    // PID Constants for tuning
    double kArmP = ArmConstants.kArmPSlot1;
    double kArmI = ArmConstants.kArmISlot1;
    double kArmD = ArmConstants.kArmDSlot1;
    double kArmFF = ArmConstants.kArmFFSlot1;
    double kArmMaxOutput = ArmConstants.kArmMaxOutputSlot1;
    double kArmMinOutput = ArmConstants.kArmMinOutputSlot1;
    double kArmMaxAccel = ArmConstants.kArmMaxAccelSlot1;
    double kArmMaxVelo = ArmConstants.kArmMaxVeloSlot1;

    double kShooterPivotP = ArmConstants.kShooterPivotPSlot1;
    double kShooterPivotI = ArmConstants.kShooterPivotISlot1;
    double kShooterPivotD = ArmConstants.kShooterPivotDSlot1;
    double kShooterPivotFF = ArmConstants.kShooterPivotFFSlot1;
    double kShooterPivotMaxOutput = ArmConstants.kShooterPivotMaxOutputSlot1;
    double kShooterPivotMinOutput = ArmConstants.kShooterPivotMinOutputSlot1;
    double kShooterPivotMaxAccel = ArmConstants.kShooterPivotMaxAccelSlot1;
    double kShooterPivotMaxVelo = ArmConstants.kShooterPivotMaxVeloSlot1;

    double ktuneArmSetPoint = 232;
    double ktuneSPSetPoint = 50;

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

        // Set Position Conversion Factor which will take the encoder units and set it
        // to degrees
        m_armEncoder.setPositionConversionFactor(ArmConstants.kArmPositionAdjustmentFactor);
        m_shooterPivotEncoder.setPositionConversionFactor(ArmConstants.kShooterPivotPositionAdjustmentFactor);

        // Encoders are not inverted
        m_armEncoder.setInverted(false);
        m_shooterPivotEncoder.setInverted(false);

        // Set Zero Offset - not sure how this works, test and figure out
        m_armEncoder.setZeroOffset(ArmConstants.kArmZeroOffsetFactor);
        m_shooterPivotEncoder.setZeroOffset(ArmConstants.kShooterPivotZeroOffsetFactor);

        // Set the PID gains for the turning motor.
        m_armPIDController.setP(ArmConstants.kArmP);
        m_armPIDController.setI(ArmConstants.kArmI);
        m_armPIDController.setD(ArmConstants.kArmD);
        m_armPIDController.setFF(ArmConstants.kArmFF);
        m_armPIDController.setOutputRange(ArmConstants.kArmMinOutput,
                ArmConstants.kArmMaxOutput);
        m_armPIDController.setSmartMotionMaxAccel(ArmConstants.kArmMaxAccel, 0);
        m_armPIDController.setSmartMotionMaxVelocity(ArmConstants.kArmMaxVelo, 0);

        m_shooterPivotPIDController.setP(ArmConstants.kShooterPivotP);
        m_shooterPivotPIDController.setI(ArmConstants.kShooterPivotI);
        m_shooterPivotPIDController.setD(ArmConstants.kShooterPivotD);
        m_shooterPivotPIDController.setFF(ArmConstants.kShooterPivotFF);
        m_shooterPivotPIDController.setOutputRange(ArmConstants.kShooterPivotMinOutput,
                ArmConstants.kShooterPivotMaxOutput);
        m_shooterPivotPIDController.setSmartMotionMaxAccel(ArmConstants.kShooterPivotMaxAccel, 0);
        m_shooterPivotPIDController.setSmartMotionMaxVelocity(ArmConstants.kShooterPivotMaxVelo, 0);

        // Set the PID gains for manual control
        m_armPIDController.setP(ArmConstants.kArmPSlot1, 1);
        m_armPIDController.setI(ArmConstants.kArmISlot1, 1);
        m_armPIDController.setD(ArmConstants.kArmDSlot1, 1);
        m_armPIDController.setFF(ArmConstants.kArmFFSlot1, 1);
        m_armPIDController.setOutputRange(ArmConstants.kArmMinOutputSlot1,
                ArmConstants.kArmMaxOutputSlot1, 1);
        m_armPIDController.setSmartMotionMaxAccel(ArmConstants.kArmMaxAccelSlot1, 1);
        m_armPIDController.setSmartMotionMaxVelocity(ArmConstants.kArmMaxVeloSlot1, 1);

        m_shooterPivotPIDController.setP(ArmConstants.kShooterPivotPSlot1, 1);
        m_shooterPivotPIDController.setI(ArmConstants.kShooterPivotISlot1, 1);
        m_shooterPivotPIDController.setD(ArmConstants.kShooterPivotDSlot1, 1);
        m_shooterPivotPIDController.setFF(ArmConstants.kShooterPivotFFSlot1, 1);
        m_shooterPivotPIDController.setOutputRange(ArmConstants.kShooterPivotMinOutputSlot1,
                ArmConstants.kShooterPivotMaxOutputSlot1, 1);
        m_shooterPivotPIDController.setSmartMotionMaxAccel(ArmConstants.kShooterPivotMaxAccelSlot1, 1);
        m_shooterPivotPIDController.setSmartMotionMaxVelocity(ArmConstants.kShooterPivotMaxVeloSlot1, 1);

        // Limit switches
        m_armLowerLimit = m_armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_armRaiseLimit = m_armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation it will maintain the above configurations.
        m_armMotor.burnFlash();
        m_shooterPivotMotor.burnFlash();

        if (TUNING_MODE) {
            // addPIDToDashboard();
            // addTuningSetPointToDashboard();
        }
    }

    // Returns the arm
    public double getArmPosition() {
        // return m_armEncoder.getPosition() - ArmConstants.kArmPositionShift;
        return m_armEncoder.getPosition();

    }

    // Returns the Shooter arm
    public double getShooterPivotPosition() {
        // return m_shooterPivotEncoder.getPosition() -
        // ArmConstants.kShooterPivotPositionShift;
        return m_shooterPivotEncoder.getPosition();

    }

    public void setArmPosition(double armPosition) {
        double armPositionDisplay = armPosition - kArmPositionShift;
        double armPositionHold = armPosition - ArmConstants.kArmPIDShift;

        m_armPIDController.setReference(armPositionHold, ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Arm Position", armPositionDisplay);

            SmartDashboard.putNumber("Raw Desired Arm Position", armPosition);
            System.out.println("Keep ARM Position " + armPositionDisplay);
        }
    }

    public void setShooterPivotPosition(double shooterPivotPosition) {
        double shooterPivotPositionDisplay;
        if (shooterPivotPosition > kShooterPivotMax) {
            shooterPivotPositionDisplay = kShooterPivotMax;
            m_shooterPivotPIDController.setReference(kShooterPivotMax, ControlType.kPosition);
        } else if (shooterPivotPosition < kShooterPivotMin) {
            shooterPivotPositionDisplay = kShooterPivotMin;
            m_shooterPivotPIDController.setReference(kShooterPivotMin, ControlType.kPosition);
        } else {
            shooterPivotPositionDisplay = shooterPivotPosition - kShooterPivotPositionShift;
            m_shooterPivotPIDController.setReference(shooterPivotPosition, ControlType.kPosition);
        }

        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Shooter Pivot Position", shooterPivotPositionDisplay);
            SmartDashboard.putNumber("Raw Desired Shooter Pivot Position", shooterPivotPosition);

            System.out.println("Keep SHOOTER PIVOT Position " + shooterPivotPositionDisplay);
        }
    }

    public void setManualArmPosition(double armPosition, boolean raise) {
        double armCurrentPosition = m_armEncoder.getPosition();
        if (raise) {
            armPosition = armCurrentPosition + ArmConstants.kArmManualAdjustment;
        } else {
            armPosition = armCurrentPosition - ArmConstants.kArmManualAdjustment;
        }
        // Set Arm Position using Slot 1 PID to change the Arm Position by adjustment
        // factor
        m_armPIDController.setReference(armPosition, ControlType.kPosition, 1);

    }

    public void setManualShooterPivotPosition(double shooterPivotPosition, boolean raise) {
        double shooterPivotCurrentPosition = m_shooterPivotEncoder.getPosition();
        if (raise) {
            shooterPivotPosition = shooterPivotCurrentPosition + ArmConstants.kShooterPivotManualAdjustment;
        } else {
            shooterPivotPosition = shooterPivotCurrentPosition - ArmConstants.kShooterPivotManualAdjustment;
        }

        if (shooterPivotPosition > kShooterPivotMax) {
            shooterPivotPosition = kShooterPivotMax;
            m_shooterPivotPIDController.setReference(kShooterPivotMax, ControlType.kPosition, 1);
        } else if (shooterPivotPosition < kShooterPivotMin) {
            shooterPivotPosition = kShooterPivotMin;
            m_shooterPivotPIDController.setReference(kShooterPivotMin, ControlType.kPosition, 1);
        } else {
            m_shooterPivotPIDController.setReference(shooterPivotPosition, ControlType.kPosition, 1);
        }
    }

    public void moveToPosition(Position position) {
        setArmPosition(position.armPosition);
        setShooterPivotPosition(position.shooterPivotPosition);
    }

    public boolean atArmSetpoint(double setpoint) {
        double armPosition = getArmPosition();
        return armPosition <= setpoint + 10 || armPosition >= setpoint - 10;
    }

    public boolean atShooterPivotSetpoint(double setpoint) {
        double shooterPivotPosition = getShooterPivotPosition();
        return shooterPivotPosition <= setpoint + 10 || shooterPivotPosition >= setpoint - 10;
    }

    public boolean atHomePosition() {
        double armTolerance = 5;
        double shooterPivotTolerance = 5;

        return m_armEncoder.getPosition() < Position.HOME.armPosition + armTolerance &&
                m_armEncoder.getPosition() > Position.HOME.armPosition - armTolerance &&
                m_shooterPivotEncoder.getPosition() < Position.HOME.shooterPivotPosition + shooterPivotTolerance &&
                m_shooterPivotEncoder.getPosition() > Position.HOME.shooterPivotPosition - shooterPivotTolerance;
    }

    // Limit Switches

    public boolean isArmLowerLimitHit() {
        return m_armLowerLimit.isPressed();
    }

    public boolean isArmRaiseLimitHit() {
        return m_armRaiseLimit.isPressed();
    }

    // check to see if the arm is at a point that is too high if the shooter goes
    // vertical
    public boolean armShooterAboveMaxHeight() {
        return (getArmPosition() > ArmConstants.shooterMaxHeight);
    }

    public boolean armShooterBelowMaxHeight() {
        return (getArmPosition() < ArmConstants.shooterMaxHeight);
    }

    // Manual Arm Motor Movements

    public void armRaise() {
        m_armMotor.set(ArmConstants.kArmRaiseSpeed);
    }

    public void armLower() {
        m_armMotor.set(-ArmConstants.kArmLowerSpeed);
    }

    public void armStop() {
        m_armMotor.set(0);
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

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        if (TUNING_MODE) {
            // tunePIDs();
            // tuneSetPoints();

        }
    }

    public Boolean atTravelPosition() {
        double tolerance = 10;
        double armPosition = getArmPosition();
        double shooterPivotPosition = getShooterPivotPosition();
        boolean armAtTravel = armPosition >= Position.HOME.armPosition - tolerance
                || armPosition <= Position.HOME.armPosition + tolerance;
        boolean shooterPivotAtTravel = shooterPivotPosition >= Position.HOME.shooterPivotPosition - tolerance
                || shooterPivotPosition <= Position.HOME.shooterPivotPosition + tolerance;

        return armAtTravel && shooterPivotAtTravel;
    }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putNumber("Raw Arm Position", getArmPosition());
            SmartDashboard.putNumber("Raw Shooter Pivot Position", getShooterPivotPosition());
            SmartDashboard.putNumber("Arm Position", getArmPosition() - kArmPositionShift);
            SmartDashboard.putNumber("Shooter Pivot Position",
                    getShooterPivotPosition() - kShooterPivotPositionShift);
            SmartDashboard.putBoolean("Arm above height", armShooterAboveMaxHeight());
            SmartDashboard.putBoolean("at Home Position", atHomePosition());
        }
    }

    public void addTuningSetPointToDashboard() {
        SmartDashboard.putNumber("TUNE: Arm", ktuneArmSetPoint);
        SmartDashboard.putNumber("TUNE: SP", ktuneSPSetPoint);

    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("Tuning Slot", 1);
        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);
        SmartDashboard.putNumber("kArmFF", kArmFF);

        SmartDashboard.putNumber("kArmMaxAccel", kArmMaxAccel);
        SmartDashboard.putNumber("kArmMaxVelo", kArmMaxVelo);
        SmartDashboard.putNumber("kShooterPivotP", kShooterPivotP);
        SmartDashboard.putNumber("kShooterPivotI", kShooterPivotI);
        SmartDashboard.putNumber("kShooterPivotD", kShooterPivotD);
        SmartDashboard.putNumber("kShooterPivotFF", kShooterPivotFF);
        SmartDashboard.putNumber("kShooterPivotMaxAccel", kShooterPivotMaxAccel);
        SmartDashboard.putNumber("kShooterPivotMaxVelo", kShooterPivotMaxVelo);

    }

    public void tunePIDs() {
        double armP = SmartDashboard.getNumber("kArmP", 0);
        double armI = SmartDashboard.getNumber("kArmI", 0);
        double armD = SmartDashboard.getNumber("kArmD", 0);
        double armFF = SmartDashboard.getNumber("kArmFF", 0);
        double armMaxAccel = SmartDashboard.getNumber("kArmMaxAccel", 0);
        double armMaxVelo = SmartDashboard.getNumber("kArmMaxVelo", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((armP != kArmP)) {
            kArmP = armP;
            m_armPIDController.setP(kArmP, 1);
        }
        if ((armI != kArmI)) {
            kArmI = armI;
            m_armPIDController.setI(kArmI, 1);
        }
        if ((armD != kArmD)) {
            kArmD = armD;
            m_armPIDController.setD(kArmD, 1);
        }
        if ((armFF != kArmFF)) {
            kArmFF = armFF;
            m_armPIDController.setD(kArmFF, 1);
        }
        if ((armMaxAccel != kArmMaxAccel)) {
            kArmMaxAccel = armMaxAccel;
            m_armPIDController.setSmartMotionMaxAccel(kArmMaxAccel, 1);
        }

        if ((armMaxVelo != kArmMaxVelo)) {
            kArmMaxVelo = armMaxVelo;
            m_armPIDController.setSmartMotionMaxVelocity(kArmMaxVelo, 1);
        }

        double shooterPivotP = SmartDashboard.getNumber("kShooterPivotP", 0);
        double shooterPivotI = SmartDashboard.getNumber("kShooterPivotI", 0);
        double shooterPivotD = SmartDashboard.getNumber("kShooterPivotD", 0);
        double shooterPivotFF = SmartDashboard.getNumber("kShooterPivotFF", 0);
        double shooterPivotMaxAccel = SmartDashboard.getNumber("kShooterPivotMaxAccel", 0);
        double shooterPivotMaxVelo = SmartDashboard.getNumber("kShooterPivotMaxVelo", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((shooterPivotP != kShooterPivotP)) {
            kShooterPivotP = shooterPivotP;
            m_shooterPivotPIDController.setP(kShooterPivotP, 1);
        }
        if ((shooterPivotI != kShooterPivotI)) {
            kShooterPivotI = shooterPivotI;
            m_shooterPivotPIDController.setI(kShooterPivotI, 1);
        }
        if ((shooterPivotD != kShooterPivotD)) {
            kShooterPivotD = shooterPivotD;
            m_shooterPivotPIDController.setD(kShooterPivotD, 1);
        }
        if ((shooterPivotFF != kShooterPivotFF)) {
            kShooterPivotFF = shooterPivotFF;
            m_shooterPivotPIDController.setFF(kShooterPivotFF, 1);
        }
        if ((shooterPivotMaxAccel != kShooterPivotMaxAccel)) {
            kShooterPivotMaxAccel = shooterPivotMaxAccel;
            m_armPIDController.setSmartMotionMaxAccel(kShooterPivotMaxAccel, 1);
        }

        if ((shooterPivotMaxVelo != kShooterPivotMaxVelo)) {
            kShooterPivotMaxVelo = shooterPivotMaxVelo;
            m_armPIDController.setSmartMotionMaxVelocity(kShooterPivotMaxVelo, 1);
        }

    }

    public void tuneSetPoints() {

        double tuneArmSetPoint = SmartDashboard.getNumber("TUNE: Arm", 0);
        double tuneSPSetPoint = SmartDashboard.getNumber("TUNE: SP", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((tuneArmSetPoint != ktuneArmSetPoint)) {
            ktuneArmSetPoint = tuneArmSetPoint;
            setArmPosition(ktuneArmSetPoint);
        }
        if ((tuneSPSetPoint != kArmI)) {
            ktuneSPSetPoint = tuneSPSetPoint;
            setShooterPivotPosition(ktuneSPSetPoint);
        }

    }
}
