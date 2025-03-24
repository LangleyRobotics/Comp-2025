// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;


public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem drivebase;

  public AlignToReefTagRelative(boolean isRightScore, DriveSubsystem drivebase) {
    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(isRightScore ? VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_RIGHT : VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_LEFT);
    rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(isRightScore ? VisionConstants.X_SETPOINT_REEF_ALIGNMENT_RIGHT : VisionConstants.X_SETPOINT_REEF_ALIGNMENT_LEFT);
    xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT : VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
    yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-left") || LimelightHelpers.getTV("limelight-right")) {
      this.dontSeeTagTimer.reset();
      double[] postions = null;

      if(isRightScore) {
        postions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
      } else {
        postions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
      }

      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);
  }
}