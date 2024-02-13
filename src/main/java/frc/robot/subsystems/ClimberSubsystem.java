package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorDefaultsConstants;

public class ClimberSubsystem extends SubsystemBase {
    int winchMotorcanid = CanIdConstants.kWinchCanId;
    CANSparkMax winchMotor = new CANSparkMax(winchMotorcanid, MotorDefaultsConstants.NeoVortexMotorType);

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