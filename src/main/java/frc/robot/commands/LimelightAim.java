package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// this code is different from other aim because this code works from anywhere in the game arena

public class LimelightAim extends SequentialCommandGroup {
    public double kSubwooferHeight = 1; // height from floor to subwoofer entrance (should be put in constants)
    public double kHeight = 1; // height from floor to the top of the robot while arm is down (should be put in
                               // constants)
    public double kLengthOfArm = 2; // self explanatory (should be put in constants)

    public double distanceFromSubwoofer; // gotten from apriltag using limelight
    public double shooterAngle; // idk at this point
    public double heightBetweenSub; // these names suck lol

    public double upperArmAngleEq(double angle) {
        double trueAngle = angle - 270;
        double height = (Math.sin(trueAngle) * kLengthOfArm) + kHeight;
        return height;
    }

    public double lowerArmAngleEq(double angle) {
        double trueAngle = angle - 180;
        double height = kHeight - (Math.cos(trueAngle) * kLengthOfArm);
        return height;
    }

    public double shooterAngleEq(double distance) {
        double angle = Math.atan(heightBetweenSub / distance);
        return angle;
    }

    public LimelightAim(ArmSubsystem m_arm, ShooterSubsystem m_shooter) {

        // checks arm position, then uses one of two equations to determine the height
        // the shooter is off of the floor

        if (m_arm.getArmPosition() > 270) {
            heightBetweenSub = kSubwooferHeight - upperArmAngleEq(m_arm.getArmPosition());
        } else if (m_arm.getArmPosition() < 270) {
            heightBetweenSub = kSubwooferHeight - lowerArmAngleEq(m_arm.getArmPosition());
        } else {
            System.out.println("Incompatible"); // I'm too lazy it's 1:18AM rn
        }

        // checks for subwoofer apriltag, then gets distance, plugging it into
        // shooterAngleEq

        if (LimelightSubsystem.getAprilTagID() == 10) {
            double[] position = LimelightSubsystem.getRobotLocation();
            double distanceFromSubwoofer = position[2];
            shooterAngle = shooterAngleEq(distanceFromSubwoofer);
        } else {
            System.out.println("Incompatible");
        }

        addCommands(
                new InstantCommand(
                        () -> m_arm.setShooterPivotPosition(125 + ((m_arm.getArmPosition() - 270) - shooterAngle)))); // finish
                                                                                                                      // it
                                                                                                                      // tomorrow,
                                                                                                                      // im
                                                                                                                      // done
                                                                                                                      // (note
                                                                                                                      // to
                                                                                                                      // self:
                                                                                                                      // this
                                                                                                                      // line
                                                                                                                      // should
                                                                                                                      // only
                                                                                                                      // work
                                                                                                                      // if
                                                                                                                      // arm
                                                                                                                      // is
                                                                                                                      // above
                                                                                                                      // 270)
    }

}