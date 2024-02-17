// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.PickUpNoteCompleted;
import frc.robot.commands.MoveToPodium;
import frc.robot.commands.MoveToPositionWithNote;
import frc.robot.commands.MoveToPositionWithNote;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.MoveToTravel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final FeederSubsystem m_feeder = new FeederSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final ClimberSubsystem m_climb = new ClimberSubsystem();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // The operator's controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        addToDashboard();

        // Configure default commands
        m_drive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(() -> m_drive.drive(
                        m_driverController.getRightTriggerAxis(),
                        MathUtil.applyDeadband(m_driverController.getLeftY(),
                                OIConstants.kDriveDeadband),
                        MathUtil.applyDeadband(m_driverController.getLeftX(),
                                OIConstants.kDriveDeadband),
                       -  MathUtil.applyDeadband(m_driverController.getRightX(),
                                OIConstants.kDriveDeadband),
                        true,
                        true),
                        m_drive));
    }

    private void addToDashboard() {
        // Put a button on the dashboard for each setpoint
        for (Position pos : Position.values()) {
        SmartDashboard.putData(pos.label, new MoveToPosition(m_arm, pos));
        // SmartDashboard.putData("Travel", new MoveToTravel(m_arm));
        // SmartDashboard.putData("Podium", new MoveToPodium(m_arm));
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     */
    private void configureButtonBindings() {
        // DRIVER controlled buttons
        final Trigger shoot = m_driverController.a();
        final Trigger turnOnShooter = m_driverController.x();
        final Trigger turnOffShooter = m_driverController.b();
        final Trigger lockWheels = m_driverController.povDown();

        lockWheels.toggleOnTrue(new RunCommand(() -> m_drive.lock(),
                m_drive));

        final Trigger zeroHeadingButton = m_driverController.start();

        zeroHeadingButton.onTrue(new InstantCommand(() -> m_drive.zeroHeading(), m_drive));

        // OPERATOR controlled buttons
        final Trigger intake = m_operatorController.leftTrigger();

        // eventually the operator will control the shooter wheels
        // final Trigger turnOnShooter = m_operatorController.rightTrigger();

        final Trigger alternatePosition = m_operatorController.leftBumper();

        final Trigger moveToTravel = m_operatorController.y();
        final Trigger moveToSubwoofer = m_operatorController.b();
        final Trigger moveToAmp = m_operatorController.a();
        final Trigger moveToPodium = m_operatorController.x();

        // Climb Controls TBD
        // final Trigger moveToTrapApproach = m_operatorController.povUp();
        // final Trigger moveToTrapScore = m_operatorController.povRight();
        // final Trigger moveToTrapClimb = m_operatorController.povDown();

        // manual arm and shooter movement - arm left joystick, shooter right joystick
        final Trigger ArmRaiseButton = m_operatorController.axisLessThan(1, -.25);
        final Trigger ArmLowerButton = m_operatorController.axisGreaterThan(1, .25);
        final Trigger ShooterPivotRaiseButton = m_operatorController.axisLessThan(5, -.25);
        final Trigger ShooterPivotLowerButton = m_operatorController.axisGreaterThan(5, .25);

        final Trigger WinchForwardButton = m_operatorController.povDown();
        final Trigger WinchBackButton = m_operatorController.povUp();

        shoot.whileTrue(new InstantCommand(m_feeder::run))
                .whileFalse(new InstantCommand(m_feeder::stop));

        intake.onTrue(new PickUpNote(m_intake, m_feeder))
                .onFalse(new SequentialCommandGroup(
                        new WaitUntilCommand(
                                () -> m_feeder.isNoteDetected()),
                        new PickUpNoteCompleted(m_intake, m_feeder)));

        turnOnShooter.onTrue(new InstantCommand(m_shooter::run));
        turnOffShooter.onTrue(new InstantCommand(m_shooter::stop));

        // eventually the operator will hold the trigger to turn the shooter wheels
        // turnOnShooter.whileTrue(new InstantCommand(m_shooter::run))
        // .whileFalse(new InstantCommand(m_shooter::stop));
        /**
         * Move arms to predefined positions
         **/
        moveToTravel.whileTrue(new MoveToPosition(m_arm, Position.TRAVEL));
        moveToSubwoofer.whileTrue(
                new MoveToPositionWithNote(m_arm, m_intake, m_shooter, Position.LOW_SUBWOOFER));
        moveToAmp.whileTrue(new MoveToPositionWithNote(m_arm, m_intake, m_shooter, Position.AMP));
        moveToPodium.whileTrue(new MoveToPositionWithNote(m_arm, m_intake, m_shooter, Position.PODIUM));
        // moveToTrapApproach.whileTrue(new MoveToPosition(m_arm,
        // Position.TRAP_APPROACH));
        // moveToTrapScore.whileTrue(new MoveToPosition(m_arm, Position.TRAP_SCORE));
        // moveToTrapClimb.whileTrue(new MoveToPosition(m_arm, Position.TRAP_CLIMB));

        /**
         * Alternate positions. For these, you need to hold down the Left Bumper too.
         **/
        // HIGH Podium
        moveToPodium.and(alternatePosition).whileTrue(
                new MoveToPositionWithNote(m_arm, m_intake, m_shooter, Position.HIGH_PODIUM));
        // BACK Podium
        moveToAmp.and(alternatePosition).whileTrue(
                new MoveToPositionWithNote(m_arm, m_intake, m_shooter, Position.BACK_PODIUM));
        // HIGH Subwoofer
        moveToSubwoofer.and(alternatePosition)
                .whileTrue(new MoveToPositionWithNote(m_arm, m_intake, m_shooter,
                        Position.HIGH_SUBWOOFER));

        /**
         * Manual Arm raise and lower
         **/
        ArmRaiseButton.whileTrue(new InstantCommand(m_arm::armRaise, m_arm))
                .onFalse(new InstantCommand(
                        () -> m_arm.keepArmPosition(
                                m_arm.getArmPosition())));

        ArmLowerButton.whileTrue(new InstantCommand(m_arm::armLower, m_arm))
                .onFalse(new InstantCommand(
                        () -> m_arm.keepArmPosition(
                                m_arm.getArmPosition())));

        ShooterPivotRaiseButton.whileTrue(new InstantCommand(m_arm::shooterPivotRaise, m_arm))
                .onFalse(new InstantCommand(
                        () -> m_arm.keepShooterPivotPosition(
                                m_arm.getShooterPivotPosition())));

        ShooterPivotLowerButton.whileTrue(new InstantCommand(m_arm::shooterPivotLower, m_arm))
                .onFalse(new InstantCommand(
                        () -> m_arm.keepShooterPivotPosition(
                                m_arm.getShooterPivotPosition())));

        // Climber motor on and off
        WinchForwardButton.whileTrue(
                new InstantCommand(m_climb::forward))
                .onFalse(new InstantCommand(m_climb::stop));

        WinchBackButton.whileTrue(
                new InstantCommand(m_climb::back))
                .onFalse(new InstantCommand(m_climb::stop));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drive::setModuleStates,
                m_drive);

        // Reset odometry to the starting pose of the trajectory.
        m_drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, 0, false, false));
    }
}
