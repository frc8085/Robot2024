package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.MotorDefaultsConstants;

public class IntakeSubsystem extends SubsystemBase {
    // imports motor id
    private final CANSparkMax m_intakeMotor = new CANSparkMax(CanIdConstants.kIntakeCanId, MotorDefaultsConstants.NeoMotorType);

    private double speed = IntakeConstants.speed;

    public IntakeSubsystem() {
        SmartDashboard.putNumber("intake speed", speed);
    }

    public void run() {
        m_intakeMotor.set(speed);
    }

    public void stop() {
        m_intakeMotor.set(0);
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