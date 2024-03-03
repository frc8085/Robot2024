package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.ArmConstants.kArmPositionShift;
import static frc.robot.Constants.ArmConstants.kShooterPivotMax;
import static frc.robot.Constants.ArmConstants.kShooterPivotMin;
import static frc.robot.Constants.ArmConstants.kShooterPivotPositionShift;

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
    private boolean PRACTICE_MODE = TuningModeConstants.kPracticeMode;

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

    /** Creates a new Arm Subsystem. */
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
            addTuningSetPointToDashboard();
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

    public void adjustShooter(double speed) {
        double direction = speed > 0 ? 1 : -1;
        double speedWithMinimum = Math.max(0.00, Math.abs(speed)) * direction;

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

    // Manual Arm Motor Movements - Open Loop

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

    // Manual Arm Motor Movements - Attempt to use Closed Loop

    // Determine different positions
    public boolean atHomePosition() {
        double armTolerance = 10;
        double shooterPivotTolerance = 10;

        return m_armEncoder.getPosition() < (Position.HOME.armPosition + armTolerance) &&
                m_armEncoder.getPosition() > (Position.HOME.armPosition - armTolerance) &&
                m_shooterPivotEncoder.getPosition() < (Position.HOME.shooterPivotPosition + shooterPivotTolerance) &&
                m_shooterPivotEncoder.getPosition() > (Position.HOME.shooterPivotPosition - shooterPivotTolerance);
    }

    public boolean atAmpPosition() {
        double armTolerance = 25;
        double shooterPivotTolerance = 25;

        // boolean armAtAmp = m_armEncoder.getPosition() >= (Position.AMP.armPosition -
        // armTolerance)
        // && m_armEncoder.getPosition() <= (Position.AMP.armPosition + armTolerance);
        boolean shooterPivotAtAmp = m_shooterPivotEncoder
                .getPosition() >= (Position.AMP.shooterPivotPosition - shooterPivotTolerance)
                && m_shooterPivotEncoder.getPosition() <= (Position.AMP.shooterPivotPosition + shooterPivotTolerance);
        return shooterPivotAtAmp;
    }

    public boolean atPodiumPosition() {
        double armTolerance = 10;
        double shooterPivotTolerance = 10;

        return m_armEncoder.getPosition() < (Position.PODIUM.armPosition + armTolerance) &&
                m_armEncoder.getPosition() > (Position.PODIUM.armPosition - armTolerance) &&
                m_shooterPivotEncoder.getPosition() < (Position.PODIUM.shooterPivotPosition + shooterPivotTolerance) &&
                m_shooterPivotEncoder.getPosition() > (Position.PODIUM.shooterPivotPosition - shooterPivotTolerance);
    }

    // Set Arm Brake Mode
    public void setBrakeMode(boolean brake) {
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        m_armMotor.setIdleMode(mode);
    }

    public void setSPBrakeMode(boolean brake) {
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        m_shooterPivotMotor.setIdleMode(mode);
    }

    public void log() {
        if (LoggingConstants.kLogging) {

        }
    }

    public void practiceDashboard() {
        SmartDashboard.putNumber("Raw Arm Position", getArmPosition());
        SmartDashboard.putNumber("Raw Shooter Pivot Position", getShooterPivotPosition());
        SmartDashboard.putNumber("Arm Position", getArmPosition() - kArmPositionShift);
        SmartDashboard.putNumber("Shooter Pivot Position",
                getShooterPivotPosition() - kShooterPivotPositionShift);
        SmartDashboard.putBoolean("Arm above height", armShooterAboveMaxHeight());
        SmartDashboard.putBoolean("at Home Position", atHomePosition());
        SmartDashboard.putBoolean("at Amp Position", atAmpPosition());
    }

    public void addTuningSetPointToDashboard() {
        SmartDashboard.putNumber("TUNE: Arm", ktuneArmSetPoint);
        SmartDashboard.putNumber("TUNE: SP", ktuneSPSetPoint);
    }

    public void tuneSetPoints() {

        double tuneArmSetPoint = SmartDashboard.getNumber("TUNE: Arm", 0);
        double tuneSPSetPoint = SmartDashboard.getNumber("TUNE: SP", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((tuneArmSetPoint != ktuneArmSetPoint)) {
            ktuneArmSetPoint = tuneArmSetPoint;
            setArmPosition(ktuneArmSetPoint);
        }
        if ((tuneSPSetPoint != ktuneSPSetPoint)) {
            ktuneSPSetPoint = tuneSPSetPoint;
            setShooterPivotPosition(ktuneSPSetPoint);
        }
    }

    public void periodic() {
        // This method will be called once per scheduler run
        
        log();
        if (PRACTICE_MODE) {
            practiceDashboard();
        }
        if (TUNING_MODE) {
            tuneSetPoints();
        }
    }

}
