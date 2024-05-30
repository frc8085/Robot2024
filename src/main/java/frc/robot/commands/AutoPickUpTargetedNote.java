package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutoPickUpTargetedNote extends SequentialCommandGroup {
        public AutoPickUpTargetedNote(
                        IntakeSubsystem m_intake,
                        FeederSubsystem m_feeder,
                        ArmSubsystem m_arm,
                        ShooterSubsystem m_shooter,
                        LimelightIntakeSubsystem m_limelightIntake,
                        DriveSubsystem m_drive,
                        CommandXboxController m_driverController,
                        CommandXboxController m_operatorController,
                        Blinkin m_blinkin) {
                addCommands(
                                new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                                new AutoTargetNote(m_limelightIntake, m_drive),
                                                                new ParallelCommandGroup(
                                                                                new InstantCommand(m_blinkin::intakeOn),
                                                                                new InstantCommand(m_intake::run),
                                                                                new InstantCommand(m_feeder::run)),
                                                                new WaitCommand(0),
                                                                new InstantCommand(() -> m_drive.drive(0.3, -1, 0, 0,
                                                                                false, false)),
                                                                new ParallelRaceGroup(
                                                                                new SequentialCommandGroup(
                                                                                                new WaitUntilCommand(
                                                                                                                m_feeder::isNoteDetected),
                                                                                                new InstantCommand(
                                                                                                                m_blinkin::withNote),
                                                                                                // // Rumble after
                                                                                                // pickup
                                                                                                // Commands.sequence(
                                                                                                // Commands.runOnce(()
                                                                                                // -> {
                                                                                                // m_driverController.getHID()
                                                                                                // .setRumble(
                                                                                                // RumbleType.kBothRumble,
                                                                                                // 1.0);
                                                                                                // m_operatorController.getHID()
                                                                                                // .setRumble(
                                                                                                // RumbleType.kBothRumble,
                                                                                                // 1.0);
                                                                                                // }),
                                                                                                // Commands.waitSeconds(.5),
                                                                                                // Commands.runOnce(()
                                                                                                // -> {
                                                                                                // m_driverController.getHID()
                                                                                                // .setRumble(
                                                                                                // RumbleType.kBothRumble,
                                                                                                // 0.0);
                                                                                                // m_operatorController.getHID()
                                                                                                // .setRumble(
                                                                                                // RumbleType.kBothRumble,
                                                                                                // 0.0);
                                                                                                // })),
                                                                                                new PickUpNoteCompleted(
                                                                                                                m_intake,
                                                                                                                m_feeder,
                                                                                                                m_shooter,
                                                                                                                m_blinkin)),
                                                                                new WaitCommand(1)),
                                                                new InstantCommand(m_drive::stop)),
                                                new InstantCommand(),
                                                m_limelightIntake::hasTarget));

        }
}
