// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
  public static final double kMaxSpeedMetersPerSecond = 4;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2;
  public static final double kMaxAngularSpeedRadiansPerSecond = MathMethods.Tau;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = MathMethods.Tau;
  public static final CustomHolonomicDrive holonomicDrive = new CustomHolonomicDrive(
                                                              new PIDController(0.5, 0.0, 0.0),
                                                              new PIDController(0.04, 0, 0));

  public static final class LEDConstants {
    public static final int kAddressableLightsID = 0;
    public static final int kAddressableLightsLength = 30;
    public static final int wordDisplayLength = 15;

  }

  public static final class DriveConstants {
    //CAN IDs for Sparkmaxes
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontLeftTurningMotorPort = 7;

    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontLeftDriveMotorPort = 8;


    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kFrontLeftTurningMotorReversed = false;

    public static final InvertedValue kFrontRightDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearRightDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearLeftDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kFrontLeftDriveMotorReversed = InvertedValue.Clockwise_Positive;

    //CAN IDs for Encoders on Swerve Modules
    public static final int kFrontRightAbsEncoderPort = 18;
    public static final int kRearRightAbsEncoderPort = 17;
    public static final int kRearLeftAbsEncoderPort = 16;
    public static final int kFrontLeftAbsEncoderPort = 19;
    
    //Before = all fals e
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kRearRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kRearLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;

    //Fix Front and rear left offsets (radians)
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(-3.5156);
    public static final double kRearRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(-104.4);
    public static final double kRearLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(113.203);
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-81.7);

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.6604;

    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.6604;

    //Distance between center of robot and furthest module
    public static final double driveBaseRadius = 0.5 * (kTrackWidth / Math.sin(Math.atan(kTrackWidth / kWheelBase)));

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //+- = Front Right 
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //++ = Front Left 
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //-- = Rear Right 
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //-+ = Rear Left

    public static final DifferentialDriveKinematics VISIONK_KINEMATICS = new DifferentialDriveKinematics(kTrackWidth);

    // IF YOU UPDATE kDriveKinematics, UPDATE kDriveTranslation2Ds TOO PLEASE OMG
    public static final Translation2d[] kDriveTranslation2Ds =
        new Translation2d[] {
          new Translation2d(kWheelBase / 2, kTrackWidth / 2), //++ = Front Left 
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //+- = Front Right 
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //-+ = Rear Left 
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)}; //-- = Rear Right


    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // These values "should" be right dw
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kSlowDriveCoefficient = 0.21;

    public static final double botMass = 55.792; // in kg
    public static final double botMOI = 4.27224; // kg * m^2

  }

  //ID Numbers for Xbox controller buttons
  public static final class Buttons {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;

    //That one button next to the menu button
    public static final int Maria = 7;
    public static final int Menu = 8;

    public static final int L3 = 9;
    public static final int R3 = 10;

    public static final int UP_ARR = 0;
    public static final int RIGHT_ARR = 90;
    public static final int DOWN_ARR = 180;
    public static final int LEFT_ARR = 270;

    
  }
  
    public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = Math.PI;
    //kSteerReduction = (7.0 / 150.0) for Mk4i, (1.0 / 12.8) for Mk4
    public static final double kSteerReduction = (7.0 / 150.0);
    public static final double kDriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final int kEncoderCPR = 4096;
    public static final double kPTurning = 0.5;
    public static final double kDriveEncoderRot2Meter = kDriveReduction * Math.PI * kWheelDiameterMeters * (89.0/100.0);
    public static final double kTurningEncoderRot2Rad = kSteerReduction * MathMethods.Tau;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI * kDriveReduction) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI * kSteerReduction) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;
    public static final double kPModuleDriveController = 1;

    public static final double maxDriveVelocityMPS = 12.9; //TEST BC I DON'T KNOW THE REAL NUMBER
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    public static final double kDeadband = 0.1;
  }

  public static final class OuttakeConstants {
    public static final int kOuttakeMotorPort = 9;
    public static final double kOuttakeMotorSpeedFast = 0.8;
    public static final double kOuttakeMotorSpeedSlow = 0.25;
  }

  public static final class PivotConstants {
    //Motor ID
    public static final int kPivotMotorPort = 10;
    
    //Speed percentage (scale of 0-1)
    public static final double kPivotMotorSpeed = 0.5;
    public static final double kPivotDefaultMotorSpeed = -0.15;

    //Pivot Positions
    public static final double kMaxPivotPosition = 6.5;
    public static final double kReefPivotPosition = 4.88;
    public static final double kMinPivotPosition = -0.2;

    public static final double kPivotOffset = 181;
    public static final double disPerRot = 360;
    public static final double pivotCompensation = 0.08;
    public static final double kPivotEncoderBreakpoint = 0.5;

    public static final double kS_Pivot = 0.0;
    public static final double kG_Pivot = 0;
    public static final double kV_Pivot = 0;
    public static final double kA_Pivot = 0;

    public static final double kP_Pivot = 1;
    public static final double kI_Pivot = 0.0;
    public static final double kD_Pivot = 0.0;

    public static final double kAprilCamSpeedFactor = 0.05;
    public static final double kAprilCamMaxSpeedMetersPerSecond = 0.69;
    public static final double kAprilCamDeadbandDegrees = 3;
    public static final double kAprilCamMinSpeed = 0.13;

    //TEST Simple goToSetpoint() method constants
    public static final double tinyPivotSpeed = 0.7;
    public static final double tinyPivotAccel = 0.05;
    public static final double deadbandAngle = 0.05;

    public static final double pivotSetpointFactor = 1;
  }

  public static final class ElevatorConstants {
    public static final int kElevatorMotorRightPort = 11;
    public static final int kElevatorMotorLeftPort = 12;
    public static final double kElevatorMotorSpeed = 0.8; //value from 0-1
    public static final double kElevatorMotorAccel = 0.5;
    public static final double deadbandAngle = 0.1;

    public static final double kMaxElevatorPosition = 86.4;
    public static final double kMinElevatorPosition = 0;
    public static final double upRightElevatorPosition = 0;
    public static final double scoringPosition = 0;
    public static final double l1 = 0.0; //change when robot is built
    public static final double l2 = 12.20;
    public static final double lowAlgae = 13.70;
    public static final double l3 = 39.22;
    public static final double highAlgae = 40.72;
    public static final double l4 = 82.9;

    public static final double kElevatorOffset = 0;
    public static final double disPerRot = 360; 
    public static final double elevatorCompensation = 0.035;
    public static final double kElevatorEncoderBreakpoint = 0.5;

    public static final double kS_Elevator = 0;
    public static final double kG_Elevator = 0;
    public static final double kV_Elevator = 0;
    public static final double kA_Elevator = 0;

    public static final double kP_Elevator = 0.8;
    public static final double kI_Elevator = 0.0;
    public static final double kD_Elevator = 0.0;
  } 
  
  public static final class IntakeConstants {
    public static final int kIntakeMotor = 13;
    public static final double kIntakeMotorSpeed = -0.5; //value from 0-1
  }

  public static final class ClimbConstants {
    public static final int kClimbMotorRight = 14;
    public static final int kClimbMotorLeft = 15;
    public static final double kClimbMotorSpeed = 0.6; //value from 0-1

    //through bore encoder values
    public static final double rightUpperLimit = 1000;
    public static final double rightLowerLimit = -1000;

    public static final double leftUpperLimit = 1000;
    public static final double leftLowerLimit = -1000;

    //Preferences constants
    public static final String rightPosKey = "Right Key";
    public static final String leftPosKey = "Left Key";

    public static final double disPerRot = 1;
  }
  
  public static final class CameraConstants {
    //Meters
    public static final double h1 = 1.43; //AprilTag to camera height
    public static final double h2 = 0.47; //goal to AprilTag height
    public static final double L = 0.66; //shooter length
    public static final double tempTheta = Math.PI / 4; //TEST just to maybe figure out setpoint
    public static final double deltaX = 0.08; //camera to pivot distance

    public static final double camHeight = 0.08;
    public static final double targetHeight = h1 + camHeight;

    //Radians
    public static final double camPitch = 0;

    //Apriltag Megatag Botpose array positions
    public static final int kMegaBotPoseTransX = 0;
    public static final int kMegaBotPoseTransY = 1;
    public static final int kMegaBotPoseTransZ = 2;
    public static final int kMegaBotPoseRoll = 3;
    public static final int kMegaBotPosePitch = 4;
    public static final int kMegaBotPoseYaw = 5;
  }

  public static final class LimelightConstants {
    public static final double klimelightOneHeight = 0.0;
    public static final double klimelightOneAngleDeg = 0.0;

    public static final int kledModePipeline = 0;
    public static final int kledModeOff = 1;
    public static final int kledModeBlink = 2;
    public static final int kledModeOn = 3;

    public static final int kcamModeVisionProcessor = 0;
    public static final int kcamModeDriverCamera = 1;

    public static final int kpipelineZero = 0;
    public static final int kpipelineOne = 1;

    public static final double klimelightTwoHeight = 0.0;
    public static final double klimelightTwoAngleDeg = 0.0;

    //Apriltag Megatag Botpose array positions
    public static final int kMegaBotPoseTransX = 0;
    public static final int kMegaBotPoseTransY = 1;
    public static final int kMegaBotPoseTransZ = 2;
    public static final int kMegaBotPoseRoll = 3;
    public static final int kMegaBotPosePitch = 4;
    public static final int kMegaBotPoseYaw = 5;
  }

  public static final class AutoConstants {

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    public static final double kAutoMaxSpeedMetersPerSecond = 5;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kAprilDriveSpeedFactor = 0.5;
    public static final double kAprilDriveMaxSpeedMetersPerSecond = 0.69;
    public static final double kAprilDriveDeadbandDegrees = 3;
    public static final double kAprilDriveMinSpeed = 0.13;
    public static final double kFieldEndXCoordinate = 16.5;

    public static final double kLameSpeedCap = 1.0;
    public static final double kLameAccelCap = 1.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    public static final double camXBottom = 0.0;
    public static final double camYBottom = 0.0;
    public static final double camZBottom = 0.0;

    public static final double camXTop = -0.34; //-0.173
    public static final double camYTop = 0.165; //0.358
    public static final double camZTop = 0.6096; //0.617-0.0444444

    public static final double kDeadband = 0.001;

    public static final double yawCoefficient = 1.0;
    public static final double pitchCoefficient = 1.0;

    public static final double rightLimeReefYawGoal = -23;
    public static final double leftLimeReefYawGoal = 17;

    public static final double camToReefHeight = 0.6096;
    public static final double kPositionTolerance = 0.1;// TWEAK
    public static final double kAngleTolerance = 3;//TWEAK
    public static final double kMaxSpeed = 3;// TWEAK
    public static final double kMaxAcceleration = 2;// TWEAK
    public static final double kMaxAngularSpeed = 8;// TWEAK
    public static final double kMaxAngularAcceleration = 8;// TWEAK
    public static final double kXYP = 3;// TWEAK
    public static final double kXYI = 0;// TWEAK
    public static final double kXYD = 0;// TWEAK

    public static final double kRP = 3;// TWEAK
    public static final double kRI = 0;// TWEAK
    public static final double kRD = 0;// TWEAK

    public static final double k2pi = Math.PI * 2;

    public static final Pose2d kAprilTag1 = new Pose2d(657.37 * 0.0254, 25.80 * 0.0254,
                    new Rotation2d(126 * k2pi / 360));
    public static final Pose2d kAprilTag2 = new Pose2d(657.37 * 0.0254, 291.20 * 0.0254,
                    new Rotation2d(234 * k2pi / 360));
    public static final Pose2d kAprilTag3 = new Pose2d(455.15 * 0.0254, 317.15 * 0.0254,
                    new Rotation2d(270 * k2pi / 360));
    public static final Pose2d kAprilTag4 = new Pose2d(365.20 * 0.0254, 241.64 * 0.0254,
                    new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag5 = new Pose2d(365.20 * 0.0254, 75.39 * 0.0254,
                    new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag6 = new Pose2d(530.49 * 0.0254, 130.17 * 0.0254,
                    new Rotation2d(300 * k2pi / 360));
    public static final Pose2d kAprilTag7 = new Pose2d(546.87 * 0.0254, 158.50 * 0.0254,
                    new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag8 = new Pose2d(530.49 * 0.0254, 186.83 * 0.0254,
                    new Rotation2d(60 * k2pi / 360));
    public static final Pose2d kAprilTag9 = new Pose2d(497.77 * 0.0254, 186.83 * 0.0254,
                    new Rotation2d(120 * k2pi / 360));
    public static final Pose2d kAprilTag10 = new Pose2d(481.39 * 0.0254, 158.50 * 0.0254,
                    new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag11 = new Pose2d(497.77 * 0.0254, 130.17 * 0.0254,
                    new Rotation2d(240 * k2pi / 360));

    // THE ONLY ACTUALLY IMPLEMENTED APRILTAG OF CURRENT DATE (2/8/2025, 10:32AM)
    public static final Pose2d kAprilTag12 = new Pose2d(33.51 * 0.0254, 25.80 * 0.0254,
                    new Rotation2d(54 * k2pi / 360));
    public static final Pose2d kAprilTag13 = new Pose2d(33.51 * 0.0254, 291.20 * 0.0254,
                    new Rotation2d(306 * k2pi / 360));
    public static final Pose2d kAprilTag14 = new Pose2d(325.68 * 0.0254, 241.64 * 0.0254,
                    new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag15 = new Pose2d(325.68 * 0.0254, 75.39 * 0.0254,
                    new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag16 = new Pose2d(235.73 * 0.0254, -0.15 * 0.0254,
                    new Rotation2d(90 * k2pi / 360));
    public static final Pose2d kAprilTag17 = new Pose2d(160.39 * 0.0254, 130.17 * 0.0254,
                    new Rotation2d(240 * k2pi / 360));
    public static final Pose2d kAprilTag18 = new Pose2d(144.00 * 0.0254, 158.50 * 0.0254,
                    new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag19 = new Pose2d(160.39 * 0.0254, 186.83 * 0.0254,
                    new Rotation2d(120 * k2pi / 360));
    public static final Pose2d kAprilTag20 = new Pose2d(193.10 * 0.0254, 186.83 * 0.0254,
                    new Rotation2d(60 * k2pi / 360));
    public static final Pose2d kAprilTag21 = new Pose2d(209.49 * 0.0254, 158.50 * 0.0254,
                    new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag22 = new Pose2d(193.10 * 0.0254, 130.17 * 0.0254,
                    new Rotation2d(300 * k2pi / 360));

    public static final Pose2d[] kAprilTags = { kAprilTag1, kAprilTag2, kAprilTag3, kAprilTag4,
                    kAprilTag5, kAprilTag6, kAprilTag7, kAprilTag8, kAprilTag9, kAprilTag10, kAprilTag11,
                    kAprilTag12, kAprilTag13, kAprilTag14, kAprilTag15, kAprilTag16, kAprilTag17,
                    kAprilTag18,
                    kAprilTag19, kAprilTag20, kAprilTag21, kAprilTag22 };
  }
    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision less. This matrix is in the form
     * [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
}
  

