package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToPointCmd extends Command {
    DriveSubsystem swerveSubsystem;
    VisionSubsystem visionSubsystem;
    Supplier<Pose2d> targetPoseSupplier;
    Supplier<Pose2d> robotPoseSupplier;
    double m_maxSpeedWanted;
    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> zRotation;
    private Translation2d lastSetpointTranslation = new Translation2d();
    Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;


    private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.2);
    private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.2);



    public DriveToPointCmd(DriveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, 
    Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(swerveSubsystem, visionSubsystem);
      }


    @Override
    public void initialize() {
      Pose2d currentPose = robotPoseSupplier.get();
      ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(swerveSubsystem.getRobotRelativeSpeeds(), swerveSubsystem.getRotation());
      Translation2d linearFieldVelocity =
          new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
      driveController.reset(
          currentPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation()),
          Math.min(
              0.0,
              -linearFieldVelocity
                  .rotateBy(
                    targetPoseSupplier
                          .get()
                          .getTranslation()
                          .minus(currentPose.getTranslation())
                          .getAngle()
                          .unaryMinus())
                  .getX()));
      thetaController.reset(
          currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
      lastSetpointTranslation = currentPose.getTranslation();

    }
  
    @Override
    public void execute() {
      Pose2d robotPose = robotPoseSupplier.get();
      Pose2d targetPose = targetPoseSupplier.get();

      //Calculate drive
      double currentDistance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
      // double ffScaler = MathUtil.clamp(
      //       (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
      //       0.0,
      //       1.0);
      double driveErrorAbs = currentDistance;
      driveController.reset(
          lastSetpointTranslation.getDistance(targetPose.getTranslation()),
          driveController.getSetpoint().velocity);
      double driveVelocityScalar =
          driveController.getSetpoint().velocity //* ffScaler
              + driveController.calculate(driveErrorAbs, 0.0);
      if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
      lastSetpointTranslation =
          new Pose2d(
                  targetPose.getTranslation(),
                  robotPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0, new Rotation2d()))
              .getTranslation();

      // Calculate theta speed
      double thetaVelocity =
          thetaController.getSetpoint().velocity //* ffScaler
              + thetaController.calculate(
                robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
      double thetaErrorAbs =
          Math.abs(robotPose.getRotation().minus(targetPose.getRotation()).getRadians());
      if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

      Translation2d driveVelocity =
          new Pose2d(
                  new Translation2d(),
                  robotPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
              .getTranslation();

      // Scale feedback velocities by input ff
      final double linearS = linearFF.get().getNorm() * 3.0;
      final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
      driveVelocity =
          driveVelocity.interpolate(linearFF.get().times(ModuleConstants.maxDriveVelocityMPS), linearS);
      thetaVelocity =
          MathUtil.interpolate(
              thetaVelocity, omegaFF.getAsDouble() * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, thetaS);

      // Command speeds
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, robotPose.getRotation());
      
      // 5. Convert chassis speeds to individual module states
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

      // 6. Output each module states to wheels
      swerveSubsystem.setModuleStates(moduleStates);
    
    }
  
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
