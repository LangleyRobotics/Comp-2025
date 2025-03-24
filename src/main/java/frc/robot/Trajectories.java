package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;



public class Trajectories {
    public static final TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kAutoMaxSpeedMetersPerSecond,
                AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    
    public static final TrajectoryConfig reverseConfig = config.setReversed(true);

    public static final TrajectoryConfig lameConfig =
        new TrajectoryConfig(
                1,
                1)
            .setKinematics(DriveConstants.kDriveKinematics);

    public static final TrajectoryConfig reverseLameConfig = lameConfig.setReversed(true);

    public static final TrajectoryConfig normalConfig =
        new TrajectoryConfig(
                AutoConstants.kAutoMaxSpeedMetersPerSecond,
                AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);


    public static final Trajectory defaultTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);


    //Go Straight
    // public static final Trajectory goStraight =
    // TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(5.792, 4.045, new Rotation2d(0)),
    //     List.of(new Translation2d(6.5, 4.045)),
    //     new Pose2d(7.166, 4.045, new Rotation2d(0)),
    //     config);

    //Should go forward
    public static final Trajectory goStraight =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(-2, 0, new Rotation2d(0)),
        config);

    //Go Straight and Turn
    public static final Trajectory goStraightTurn =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0)),
        new Pose2d(10, 0, new Rotation2d(Math.PI)),
        config);
}
