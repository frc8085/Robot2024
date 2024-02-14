package frc.robot.subsystems;

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


    // imports motor id
    private final CANSparkMax m_intakeMotor = new CANSparkMax(CanIdConstants.kIntakeCanId, MotorDefaultsConstants.NeoMotorType);

    private double speed = IntakeConstants.speed;

    public IntakeSubsystem() {

        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.

        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        m_intakeMotor.setSmartCurrentLimit(MotorDefaultsConstants.NeoCurrentLimit);

        m_intakeMotor.burnFlash();

    }

    public void run() {
        m_intakeMotor.set(speed);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }

    public void runArmMove() {
        m_intakeMotor.set(IntakeConstants.armMoveSpeed);
    }
    @Override
    public void periodic() {
        log();
        tuneSpeeds();
    }

    public void log() {
        if (LoggingConstants.kLogging) {
        }
    }

    public void tuneSpeeds() {
        speed = SmartDashboard.getNumber("Intake speed", IntakeConstants.speed);

        SmartDashboard.putNumber("Intake speed", speed);
    }

}