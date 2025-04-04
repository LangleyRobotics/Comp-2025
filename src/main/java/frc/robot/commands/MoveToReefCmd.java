// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.OIConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class MoveToReefCmd extends Command {
//   private DriveSubsystem swerveSubsystem;
//   private VisionSubsystem visionSubsystem;
//   private final SlewRateLimiter yLimiter;

//   public MoveToReefCmd(DriveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
//     this.swerveSubsystem = swerveSubsystem;
//     this.visionSubsystem = visionSubsystem;
//     this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);

//     addRequirements(swerveSubsystem, visionSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     double ySpeed;
//     ySpeed = Math.sin(Math.toRadians(
//       (VisionConstants.leftLimeReefYawGoal - visionSubsystem.getLeftAngles()[0]))) * 0.65;
//     // SmartDashboard.putNumber("Yaw during MoveToReefCmd Left", visionSubsystem.getLeftAngles()[0]);
//     // SmartDashboard.putNumber("Difference in yaw during MoveToReefCmd Left", VisionConstants.leftLimeReefYawGoal - visionSubsystem.getLeftAngles()[0]);

//     // SmartDashboard.putNumber("ySpeed during MoveToReefCmd", ySpeed);
    
//     // 2. Apply deadband
//     ySpeed = Math.abs(ySpeed) > VisionConstants.kDeadband ? ySpeed : 0.0;

//     // 3. Make the driving smoother
//     ySpeed = yLimiter.calculate(ySpeed) * Constants.kMaxSpeedMetersPerSecond;
//     // SmartDashboard.putNumber("ySpeed", ySpeed);

//     // 4. Construct desired chassis speeds
//     ChassisSpeeds chassisSpeeds;
//             // Relative to robot
//             chassisSpeeds = new ChassisSpeeds(0, ySpeed, 0);
  
  
//             // 5. Convert chassis speeds to individual module states
//     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

//     // 6. Output each module states to wheels
//     swerveSubsystem.setModuleStates(moduleStates);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.stopModules();
        
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
