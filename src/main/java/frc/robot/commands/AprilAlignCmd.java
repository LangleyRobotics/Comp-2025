// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilAlignCmd extends Command {
  private final DriveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;

  public AprilAlignCmd(VisionSubsystem visionSubsystem, DriveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(visionSubsystem, swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d pose = visionSubsystem.getCurrentPose();
    swerveSubsystem.setPose(pose);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
