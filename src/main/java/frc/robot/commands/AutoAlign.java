package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlign extends Command {

    private final DriveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public AutoAlign(DriveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, Supplier <Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(Constants.kMaxAngularAccelerationRadiansPerSecondSquared);
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        addRequirements(swerveSubsystem, visionSubsystem);
    }


    @Override
    public void initialize() {
    }


    @Override
    public void execute() {

        // double horizontalDist = Math.abs(VisionConstants.camToReefHeight / Math.tan(visionSubsystem.getAngles()[1]));
        double turningSpeed = -visionSubsystem.getAngles()[0] / 150;
        double ySpeed = -ySpdFunction.get();
        double xSpeed = -visionSubsystem.getAngles()[1] / 50;
        
        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > VisionConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.kMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
            // Relative to robot
            if (fieldOrientedFunction.get()) {
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d().times(1));
            } else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

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