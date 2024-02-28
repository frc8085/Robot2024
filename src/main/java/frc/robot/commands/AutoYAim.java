package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoYAim extends PIDCommand {
    private ArmSubsystem m_arm;

    // TODO tune pids for AutoYAim
    static double kP = 0.01;
    static double kI = 0;
    static double kD = 0.0008;

    public AutoYAim(ArmSubsystem arm, LimelightSubsystem limelight) {
        super(new PIDController(kP, kI, kD),
                // get arm position
                arm::getShooterPivotPosition,
                // get offset
                limelight::getYfromRobotPerspective,
                output -> arm.setShooterPivotPosition(limelight.getYfromRobotPerspective()));

        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the setpoint before it is considered as having reached the
        // reference
        getController()
                .setTolerance(2, AutoConstants.kTurnRateToleranceDegPerS);

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_arm.shooterPivotStop();
    }

}