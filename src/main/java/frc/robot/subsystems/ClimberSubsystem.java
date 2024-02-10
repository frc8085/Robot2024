package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    int winchMotorcanid = ClimberConstants.kWinchCanId;
    CANSparkMax winchMotor = new CANSparkMax(winchMotorcanid, MotorType.kBrushless);

    public ClimberSubsystem() {
        winchMotor.restoreFactoryDefaults();
        winchMotor.setIdleMode(IdleMode.kBrake);
        winchMotor.burnFlash();
    }

    public void forward() {
        winchMotor.set(ClimberConstants.kSpeed);
    }

    public void back() {
        winchMotor.set(-ClimberConstants.kSpeed);
    }

    public void stop() {
        winchMotor.set(0);
    }
}