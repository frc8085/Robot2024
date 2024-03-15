package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TargetSPTwice extends SequentialCommandGroup {
        public TargetSPTwice(
                        LimelightSubsystem m_limelight,
                        ArmSubsystem m_arm) {
                addCommands(
                                new InstantCommand(() -> Logger.recordOutput("Commands/LimelightTargetSP", false)),

                                new ConditionalCommand(
                                                new AutoTargetSP(m_limelight, m_arm),
                                                new InstantCommand(),
                                                m_limelight::hasTarget),
                                new WaitCommand(0.1),
                                new ConditionalCommand(new ConditionalCommand(
                                                new AutoTargetSP(m_limelight, m_arm),
                                                new InstantCommand(),
                                                m_limelight::hasTarget),
                                                new InstantCommand(),
                                                m_arm::atPodiumPosition),
                                new InstantCommand(() -> Logger.recordOutput("Commands/LimelightTargetSP", true)));
        }
}
/*
 * center low miss
 * a little closer than stage got it
 * on black line got in
 * Joey sucks :( (sw high)
 * podium to high
 * podium low???
 * 
 */