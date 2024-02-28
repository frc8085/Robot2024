package frc.robot.subsystems;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.TuningModeConstants;

public class ClimberSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kClimberTuning;

    private final CANSparkFlex m_winchMotor = new CANSparkFlex(
            CanIdConstants.kWinchCanId, MotorDefaultsConstants.NeoVortexMotorType);

    // Encoder
    // private RelativeEncoder m_winchEncoder;
    private SparkAbsoluteEncoder m_winchEncoder;

    // PID for Winch
    private SparkPIDController m_winchPIDController = m_winchMotor.getPIDController();

    // PID Constants for tuning
    double kWinchP = ClimberConstants.kWinchP;
    double kWinchI = ClimberConstants.kWinchI;
    double kWinchD = ClimberConstants.kWinchD;
    double kWinchFF = ClimberConstants.kWinchFF;
    double kWinchMaxOutput = ClimberConstants.kWinchMaxOutput;
    double kWinchMinOutput = ClimberConstants.kWinchMinOutput;
    double kWinchMaxAccel = ClimberConstants.kWinchMaxAccel;
    double kWinchMaxVelo = ClimberConstants.kWinchMaxVelo;

    // limit switches
    private SparkLimitSwitch m_winchLeftLowerLimit;
    private SparkLimitSwitch m_winchRightLowerLimit;
    private SparkLimitSwitch m_winchLeftRaiseLimit;
    private SparkLimitSwitch m_winchRightRaiseLimit;

    public boolean WinchLowerLimitHit() {
        return isWinchLowerLimitHit();
    }

    public boolean WinchRaiseLimitHit() {
        return isWinchRaiseLimitHit();
    }

    public boolean isWinchLowerLimitHit() {
        return m_winchLeftLowerLimit.isPressed() || m_winchRightLowerLimit.isPressed();
    }

    public boolean isWinchRaiseLimitHit() {
        return m_winchLeftRaiseLimit.isPressed() || m_winchRightRaiseLimit.isPressed();
    }

    // Creates new Climber Subsystem
    public ClimberSubsystem() {
        m_winchMotor.restoreFactoryDefaults();
        m_winchMotor.setIdleMode(ClimberConstants.kWinchMotorIdleMode);
        m_winchMotor.setSmartCurrentLimit(MotorDefaultsConstants.NeoVortexCurrentLimit);

        // Setup encoders and PID controllers for the arm and shooter arms.

        // alternate encoder
        // m_winchEncoder = m_winchMotor.getEncoder(RelativeEncoder.class, 8192);
        // absolute encoder
        m_winchEncoder = m_winchMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_winchPIDController.setFeedbackDevice(m_winchEncoder);

        // Set if encoder is inverted
        m_winchEncoder.setInverted(false);

        // Set the PID gains for the turning motor.
        m_winchPIDController.setFeedbackDevice(m_winchEncoder);
        m_winchPIDController.setP(ClimberConstants.kWinchP);
        m_winchPIDController.setI(ClimberConstants.kWinchI);
        m_winchPIDController.setD(ClimberConstants.kWinchD);
        m_winchPIDController.setFF(ClimberConstants.kWinchFF);
        m_winchPIDController.setOutputRange(ClimberConstants.kWinchMinOutput,
                ClimberConstants.kWinchMaxOutput);
        m_winchPIDController.setSmartMotionMaxAccel(ClimberConstants.kWinchMaxAccel, 0);
        m_winchPIDController.setSmartMotionMaxVelocity(ClimberConstants.kWinchMaxVelo, 0);

        // Limit switches
        m_winchLeftLowerLimit = m_winchMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_winchRightLowerLimit = m_winchMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_winchLeftRaiseLimit = m_winchMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_winchRightRaiseLimit = m_winchMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        m_winchMotor.burnFlash();

        if (TUNING_MODE) {
            addPIDToDashboard();

        }

    }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putNumber("Winch Position", getwinchPosition());
            SmartDashboard.putNumber("Winch Current", m_winchMotor.getOutputCurrent());
        }
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kWinchP", kWinchP);
        SmartDashboard.putNumber("kWinchI", kWinchI);
        SmartDashboard.putNumber("kWinchD", kWinchD);
        SmartDashboard.putNumber("kWinchFF", kWinchFF);

        SmartDashboard.putNumber("kWinchMaxAccel", kWinchMaxAccel);
        SmartDashboard.putNumber("kWinchMaxVelo", kWinchMaxVelo);
    }

    public void tunePIDs() {
        double winchP = SmartDashboard.getNumber("kWinchP", 0);
        double winchI = SmartDashboard.getNumber("kWinchI", 0);
        double winchD = SmartDashboard.getNumber("kWinchD", 0);
        double winchFF = SmartDashboard.getNumber("kWinchFF", 0);
        double winchMaxAccel = SmartDashboard.getNumber("kWinchMaxAccel", 0);
        double winchMaxVelo = SmartDashboard.getNumber("kWinchMaxVelo", 0);

        // if PID coefficients on dashboard have changed, write new values to controller
        if ((winchP != kWinchP)) {
            kWinchP = winchP;
            m_winchPIDController.setP(kWinchP);
        }
        if ((winchI != kWinchI)) {
            kWinchI = winchI;
            m_winchPIDController.setI(kWinchI);
        }
        if ((winchD != kWinchD)) {
            kWinchD = winchD;
            m_winchPIDController.setD(kWinchD);
        }
        if ((winchFF != kWinchFF)) {
            kWinchFF = winchFF;
            m_winchPIDController.setD(kWinchFF);
        }
        if ((winchMaxAccel != kWinchMaxAccel)) {
            kWinchMaxAccel = winchMaxAccel;
            m_winchPIDController.setSmartMotionMaxAccel(kWinchMaxAccel, 0);
        }

        if ((winchMaxVelo != kWinchMaxVelo)) {
            kWinchMaxVelo = winchMaxVelo;
            m_winchPIDController.setSmartMotionMaxVelocity(kWinchMaxVelo, 0);
        }
    }

    // Returns the winch
    public double getwinchPosition() {
        return m_winchEncoder.getPosition();
    }

    public void setWinchPosition(double winchPosition) {
        m_winchPIDController.setReference(winchPosition, ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Winch Position", winchPosition);
        }
    }

    // Open Loop Control
    public void forward() {
        m_winchMotor.set(ClimberConstants.kRaiseSpeed);
    }

    public void back() {
        m_winchMotor.set(-ClimberConstants.kLowerSpeed);
    }

    public void stop() {
        m_winchMotor.set(0);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        if (TUNING_MODE) {
            tunePIDs();

        }

    }
}