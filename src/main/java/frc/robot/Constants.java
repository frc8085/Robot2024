// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final class CanIdConstants {
                public static final int kGyroCanId = 15;
                public static final int kIntakeCanId = 21;
                public static final int kWinchCanId = 22;
                public static final int kArmCanId = 23;
                public static final int kShooterPivotCanId = 24;
                public static final int kFeederCanId = 25;
                public static final int kShooter1CanId = 26;
                public static final int kShooter2CanId = 27;
        }

        public static final class LoggingConstants {
                public static final boolean kLogging = true;
        }

        public static final class MotorDefaultsConstants {
                public static final int NeoCurrentLimit = 40;
                public static final int NeoVortexCurrentLimit = 40;
                public static final int Neo550CurrentLimit = 20;
                public static final MotorType NeoMotorType = MotorType.kBrushless;
                public static final MotorType Neo550MotorType = MotorType.kBrushless;
                public static final MotorType NeoVortexMotorType = MotorType.kBrushless;
        }

        public static final class TuningModeConstants {
                public static final boolean kTuning = true;
                public static final boolean kDriveTuning = true;
                public static final boolean kArmTuning = true;
                public static final boolean kClimberTuning = true;
                public static final boolean kFeederTuning = false;
                public static final boolean kIntakeTuning = true;
                public static final boolean kLimelightTuning = true;
                public static final boolean kShooterTuning = false;
        }

        public static final class IntakeConstants {
                public static final double speed = .9;
                public static final double armMoveSpeed = .25;
                public static final double ejectSpeed = -.75;
        }

        public static final class FeederConstants {
                public static final int kIRPort1 = 0;
                public static final int kIRPort2 = 1;

                public static final double speed = 1;
                public static final double kLoadWaitTime = .5;

                public static double kFeederMaxOutput = 1;
                public static double kFeederMinOutput = -1;
                public static double kFeederFF = 0.0001;
                public static double kFeederP = 0.0001;
                public static double kFeederI = 0.0001;
                public static double kFeederD = 0;

                public static double kFeederSetPoint = 3000;

        }

        public static final class ShooterConstants {

                public static final IdleMode kShooterMotor2IdleMode = IdleMode.kBrake;
                public static IdleMode kShooterMotor1IdleMode = IdleMode.kBrake;
                public static double kShooter2MaxOutput = 1;
                public static double kShooter2MinOutput = -1;
                public static double kShooter2FF = 0.0002;
                public static double kShooter2P = 0.0004;
                public static double kShooter2I = 0.001;
                public static double kShooter2D = 0;

                public static double kShooter1MaxOutput = 1;
                public static double kShooter1MinOutput = -1;
                public static double kShooter1FF = 0.0002;
                public static double kShooter1P = 0.0004;
                public static double kShooter1I = 0.001;
                public static double kShooter1D = 0;

                public static double kShooterEncoder2VelocityFactor = (2 * Math.PI) / 60.0;
                public static double kShooterEncoder2PositionFactor = (2 * Math.PI);
                public static double kShooterEncoder1VelocityFactor = (2 * Math.PI) / 60.0;
                public static double kShooterEncoder1PositionFactor = (2 * Math.PI);

                public static double kShooter1SetPoint = 4800;
                public static double kShooter2SetPoint = 3850;

                public static double kShooterSetPointTrap = 1000;
        }

        public static final class ArmConstants {

                public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kShooterPivotMotorIdleMode = IdleMode.kBrake;

                public static final double kArmMaxSpeed = .6;
                public static final double kShooterPivotMaxSpeed = 1;

                // Manual Arm movement speeds
                public static final double kArmRaiseSpeed = .25;
                public static final double kArmLowerSpeed = .15;

                public static final double kShooterPivotRaiseSpeed = .25;
                public static final double kShooterPivotLowerSpeed = .25;

                // arm height where shooter is above max height
                public static final double shooterMaxHeight = 320;

                // shooter pivot minimum and maximum
                public static final double kShooterPivotMin = 20;
                public static final double kShooterPivotMax = 235;

                // PIDS
                // Arm PID coefficients
                public static final int kArmPIDSlot = 0;
                public static final double kArmP = 0.01;
                public static final double kArmI = 0;
                public static final double kArmD = 0;
                public static final double kArmFF = 0.001;
                public static final double kArmMaxOutput = kArmMaxSpeed;
                public static final double kArmMinOutput = -.5;
                public static final double kArmMaxAccel = .1;
                public static final double kArmMaxVelo = .5;

                // Shooter Pivot PID coefficients
                public static final int kShooterPivotPIDSlot = 0;
                public static final double kShooterPivotP = 0.02;
                public static final double kShooterPivotI = 0;
                public static final double kShooterPivotD = 0;
                public static final double kShooterPivotFF = 0.0005;
                public static final double kShooterPivotMaxOutput = kShooterPivotMaxSpeed;
                public static final double kShooterPivotMinOutput = -kShooterPivotMaxSpeed;
                public static final double kShooterPivotMaxAccel = .25;
                public static final double kShooterPivotMaxVelo = .5;

                // SETPOINTS

                // Offset
                public static final double kArmZeroOffsetFactor = 147;
                public static final double kShooterPivotZeroOffsetFactor = 306;

                // We are zeroing the arm at the top limit, so all our positions should be
                // adjusted
                public static final double kArmPositionShift = 232;

                public static final double kArmAdjustmentFactor = 90 + kArmPositionShift;

                // the arm position is about 25 higher when it holds than what you want
                public static final double kArmPIDShift = 25;

                // Zeroing the shooter pivot at the far limit, so all our positions should be
                // adjusted
                public static final double kShooterPivotPositionShift = 47;

                public static final double kShooterPivotAdjustmentFactor = 0 + kShooterPivotPositionShift;

                // Arm Positions
                public enum Position {
                        HOME("Home",
                                        232,
                                        50,
                                        false,
                                        false,
                                        true,
                                        false),
                        AMP("Amp",
                                        325,
                                        205,
                                        true,
                                        false,
                                        true,
                                        true),
                        PODIUM("Podium",
                                        256,
                                        60,
                                        true,
                                        true,
                                        true,
                                        true),
                        SUBWOOFER("Subwoofer",
                                        322,
                                        125,
                                        true,
                                        true,
                                        true,
                                        true),
                        TRAP_APPROACH("Trap Approach",
                                        320,
                                        160,
                                        true,
                                        true,
                                        false,
                                        false),
                        TRAP_CLIMB("Trap Climb",
                                        320,
                                        160,
                                        true,
                                        true,
                                        false,
                                        false),
                        TRAP_SCORE("Trap Score",
                                        310,
                                        90,
                                        true,
                                        true,
                                        false,
                                        false),
                        HIGH_PODIUM("High Podium",
                                        338,
                                        289,
                                        true,
                                        true,
                                        true,
                                        true),

                        BACK_PODIUM("Back Podium",
                                        338,
                                        166,
                                        true,
                                        true,
                                        true,
                                        true),

                        BACK_SUBWOOFER("Back Subwoofer",
                                        320,
                                        42,
                                        true,
                                        true,
                                        true,
                                        true);

                        public final String label;
                        public final double armPosition;
                        public final double shooterPivotPosition;
                        public final boolean moveArmFirst;
                        public final boolean parallelMovement;
                        public final boolean HeightCheck;
                        public final boolean shooterOn;

                        private Position(String label,
                                        double armPosition,
                                        double shooterPivotPosition,
                                        boolean moveArmFirst,
                                        boolean parallelMovement,
                                        boolean HeightCheck,
                                        boolean shooterOn) {
                                this.label = label;
                                this.armPosition = armPosition;
                                this.shooterPivotPosition = shooterPivotPosition;
                                this.moveArmFirst = moveArmFirst;
                                this.parallelMovement = parallelMovement;
                                this.HeightCheck = HeightCheck;
                                this.shooterOn = shooterOn;
                        }

                }

                // Estimates, fix this once we get exact measurements
                public static final double kArmTotalDegrees = 360;
                public static final double kArmTotalRevolutions = 1;

                // Convert angle of travel to encoder rotations, where encoder reading of .1 is
                // 0 degrees and reading of 5.5 is 90 degrees
                public static final double kArmRevolutionsPerDegree = (kArmTotalRevolutions)
                                / kArmTotalDegrees;

                public static final double kArmPositionAdjustmentFactor = 1 / kArmRevolutionsPerDegree;

                // Estimates, fix this once we get exact measurements
                public static final double kShooterPivotTotalDegrees = 360;
                public static final double kShooterPivotTotalRevolutions = 1;

                // Convert angle of travel to encoder rotations
                public static final double kShooterPivotRevolutionsPerDegree = (kShooterPivotTotalRevolutions)
                                / kShooterPivotTotalDegrees;

                public static final double kShooterPivotPositionAdjustmentFactor = 1
                                / kShooterPivotRevolutionsPerDegree;

                // Temporary Arm PID configuration
                public static final double homeArmPosition = 50 + kArmPositionShift;
                public static final double podiumArmPosition = 50 + kArmPositionShift;

                public static final double homeShooterPivotPosition = 40;
                public static final double podiumShooterPivotPosition = 124;
        }

        public static final class ClimberConstants {
                public static final double kSpeed = 0.25;
        }

        public static final class BlinkinConstants {
                public static int pwmPort = 0;

                public enum ledColor {

                        WITHNOTE("With Note", 0.65),
                        TARGETFOUND("Target Found", .69),
                        READYTOSHOOT("Ready to Shoot", .77);

                        public final String label;
                        public final double color;

                        private ledColor(String label, double color) {
                                this.label = label;
                                this.color = color;
                        }

                }

        }

        public static final class DriveConstants {
                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double kMaxSpeedMetersPerSecond = 4.8;
                public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

                public static final double kDirectionSlewRate = 1.2; // radians per second
                public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(24.0);
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(22.0);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;

                // SPARK MAX CAN IDs
                public static final int kFrontLeftDrivingCanId = 1;
                public static final int kRearLeftDrivingCanId = 2;
                public static final int kFrontRightDrivingCanId = 3;
                public static final int kRearRightDrivingCanId = 4;

                public static final int kFrontLeftTurningCanId = 11;
                public static final int kRearLeftTurningCanId = 12;
                public static final int kFrontRightTurningCanId = 13;
                public static final int kRearRightTurningCanId = 14;

                // Gyro Constants
                public static final boolean kGyroReversed = false;
        }

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 14;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22)
                                / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction) / 60.0; // meters per second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per
                                                                                                 // second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;

                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 50; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
        }

        public static final class OIConstants {
                public static final int kDriverControllerPort = 0;
                public static final double kDriveDeadband = 0.05;
                public static int kOperatorControllerPort = 1;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                // Constraint for the motion profiled robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        public static final class NeoMotorConstants {
                public static final double kFreeSpeedRpm = 5676;
        }
}
