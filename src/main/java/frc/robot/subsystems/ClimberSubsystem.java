package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem {
    int Winchmotorcanid = ClimberConstants.kWinchCanId;
    CANSparkMax WinchMotor = new CANSparkMax(Winchmotorcanid, MotorType.kBrushless);

    public void start() {
        WinchMotor.set(.5);
    }

    public void stop() {
        WinchMotor.set(0);
    }
}