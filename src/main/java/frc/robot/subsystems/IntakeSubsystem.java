package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.TuningModeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private boolean TUNING_MODE = TuningModeConstants.kIntakeTuning;
    private boolean PRACTICE_MODE = TuningModeConstants.kPracticeMode;

    private boolean intakeOn = false;

    private static final String INTAKE_LOG_ENTRY = "/Intake";
    private static final String DESIRED_INTAKE_LOG_ENTRY = "/DesiredIntake";

    // imports motor id
    private final CANSparkMax m_intakeMotor = new CANSparkMax(CanIdConstants.kIntakeCanId,
            MotorDefaultsConstants.NeoMotorType);

    private double kSpeed = IntakeConstants.speed;

    public IntakeSubsystem() {

        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.

        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        m_intakeMotor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_intakeMotor.burnFlash();

        if (TUNING_MODE) {
            addSpeedToDashboard();
        }
    }

    public void run() {
        m_intakeMotor.set(kSpeed);
        intakeOn = true;
    }

    public void stop() {
        m_intakeMotor.set(0);
        intakeOn = false;
    }

    public void runArmMove() {
        m_intakeMotor.set(IntakeConstants.armMoveSpeed);
    }

    public void eject() {
        m_intakeMotor.set(IntakeConstants.ejectSpeed);
        intakeOn = true;
    }

    public boolean isIntakeRunning() {
        return intakeOn;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake On", isIntakeRunning());
        logOutputs();
        log();
        if (PRACTICE_MODE) {

        }
        if (TUNING_MODE) {
            tuneSpeeds();
        }
    }

    public void log() {
        if (LoggingConstants.kLogging) {
        }
    }

    public void addSpeedToDashboard() {
        SmartDashboard.putNumber("Intake speed", kSpeed);
    }

    public void tuneSpeeds() {
        double speed = SmartDashboard.getNumber("Intake speed", 0);

        if ((speed != kSpeed)) {
            kSpeed = speed;
            m_intakeMotor.set(speed);

        }

    }

    private void logOutputs() {
        Logger.recordOutput(getName() + INTAKE_LOG_ENTRY, m_intakeMotor.getEncoder().getVelocity());
        Logger.recordOutput(getName() + DESIRED_INTAKE_LOG_ENTRY, isIntakeRunning());
    }
}