
package frc.robot.subsystems;
import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveSubsystem extends SubsystemBase {
  
  SwerveDrive swerveDrive;

  VisionSubsystem vision;
  
  public final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);

  double xAxisJoystickInverted = (DriveConstants.xAxisDriveReversedJoystick ? -1 :  1);
  double yAxisJoystickInverted = (DriveConstants.yAxisDriveReversedJoystick ? -1 :  1);

  public DriveSubsystem(File directory, VisionSubsystem vision) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    this.vision = vision;

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Limits.maximumSpeed,
                                                                  new Pose2d(new Translation2d(Meter.of(0),
                                                                                               Meter.of(0)),
                                                                             Rotation2d.fromDegrees(0)));
      // DataLogManager.log("Found and read the file");

      
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
                                               swerveDrive.setModuleEncoderAutoSynchronize(false,
                                               1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    swerveDrive.setMotorIdleMode(true);
    for (SwerveModule module : swerveDrive.getModules()) {
      module.getAngleMotor().setPosition(module.getAbsolutePosition());
      module.getAngleMotor().burnFlash();
    }

    setupPathPlanner();
  }




  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                              translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                              translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      });
    }

    public Command driveCommand(Double translationX, Double translationY, Double angularRotationX)
    {
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                              translationX * swerveDrive.getMaximumChassisVelocity(),
                              translationY * swerveDrive.getMaximumChassisVelocity()), 0.8),
                          Math.pow(angularRotationX, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      });
    }

    /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.Limits.maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.Limits.maximumSpeed);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle, double maxSpeed)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maxSpeed);
  }




  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // driv?
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }




public SwerveDrive getSwerveDrive(){
  return swerveDrive;
}

public void invertXAxisJoystick() {
  xAxisJoystickInverted *= -1;
}

public void invertYAxisJoystick() {
  yAxisJoystickInverted *= -1;
}

public double getXAxisInverted() {
  return xAxisJoystickInverted;
}

public double getYAxisInverted() {
  return yAxisJoystickInverted;
}

public void driveFieldOriented(ChassisSpeeds velocity){
  swerveDrive.driveFieldOriented(velocity);
}

public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
{
  return run(()->{
    swerveDrive.driveFieldOriented(velocity.get());
  });
}
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

    /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return swerveDrive.getYaw();
  }

  public Optional<SwerveDriveSimulation>getMapleSimDrive(){
    return swerveDrive.getMapleSimDrive();
  }

  public Pose2d getMapleSimPose(){
    return getMapleSimDrive().get().getSimulatedDriveTrainPose();
  }
 
  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
    m_gyro.reset();
  }


   /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public Command zeroHeadingCommand() {
    return runOnce(
      () -> {zeroHeading();} 
      );
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }
  

  @Override
  public void periodic() {

    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Rotation").setNumber(swerveDrive.getPose().getRotation().getDegrees());
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Position x").setNumber(swerveDrive.getPose().getX());
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Position y").setNumber(swerveDrive.getPose().getY());
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Yaw").setNumber(swerveDrive.getYaw().getDegrees());
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Robot Velocity Rotation").setNumber(swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Robot Velocity x").setNumber(swerveDrive.getRobotVelocity().vxMetersPerSecond);
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("Robot Velocity y").setNumber(swerveDrive.getRobotVelocity().vyMetersPerSecond);
    NetworkTableInstance.getDefault().getTable("Odometry").getEntry("MT2 Rotation").setNumber(getHeading().getDegrees());

    vision.updatePosesEstimator(swerveDrive);
    swerveDrive.updateOdometry(); 

    SmartDashboard.putNumber("Absolute Encoder Angle Front Left", swerveDrive.getModules()[0].getAbsolutePosition() % 360);
    SmartDashboard.putNumber("Current Angle Front Left", swerveDrive.getModules()[0].getRelativePosition() % 360);
    SmartDashboard.putNumber("Front Left Offset", (swerveDrive.getModules()[0].getAbsolutePosition() - swerveDrive.getModules()[0].getRelativePosition()) % 360);
   
    SmartDashboard.putNumber("Absolute Encoder Angle Front Right", swerveDrive.getModules()[1].getAbsolutePosition() % 360);
    SmartDashboard.putNumber("Current Angle Front Right", swerveDrive.getModules()[1].getRelativePosition() % 360);
    SmartDashboard.putNumber("Front Right Offset", (swerveDrive.getModules()[1].getAbsolutePosition() - swerveDrive.getModules()[1].getRelativePosition()) % 360);

    SmartDashboard.putNumber("Absolute Encoder Angle Rear Left", swerveDrive.getModules()[2].getAbsolutePosition() % 360);
    SmartDashboard.putNumber("Current Angle Rear Left", swerveDrive.getModules()[2].getRelativePosition() % 360);
    SmartDashboard.putNumber("Rear Left Offset", (swerveDrive.getModules()[2].getAbsolutePosition() - swerveDrive.getModules()[2].getRelativePosition()) % 360);

    SmartDashboard.putNumber("Absolute Encoder Angle Rear Right", swerveDrive.getModules()[3].getAbsolutePosition() % 360);
    SmartDashboard.putNumber("Current Angle Rear Right", swerveDrive.getModules()[3].getRelativePosition() % 360);
    SmartDashboard.putNumber("Rear Right Offset", (swerveDrive.getModules()[3].getAbsolutePosition() - swerveDrive.getModules()[3].getRelativePosition()) % 360);

    SmartDashboard.putNumber("Current X Pose", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Current Y Pose", swerveDrive.getPose().getY());
    SmartDashboard.putNumber("Current Rot2D Pose", swerveDrive.getPose().getRotation().getDegrees());
    
    SmartDashboard.putNumber("Front Left Drive Motor Voltage", swerveDrive.getModules()[0].getDriveMotor().getVoltage());
  }


  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
    PathfindingCommand.warmupCommand().schedule();
  }

  public boolean isRedSide() {
    var alliance = DriverStation.getAlliance();
      if (alliance.isPresent())
      {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @param constraints Maximum linear and rotational speeds and accelerations.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose, PathConstraints constraints)
  {
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @param constraints Maximum linear and rotational speeds and accelerations.
   * @param endSpeed Final velocity at goal.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose, PathConstraints constraints, double endSpeed)
  {
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        endSpeed);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to in relation to robot pose.
   * @param constraints Maximum linear and rotational speeds and accelerations.
   * @param endSpeed Final velocity at goal.
   * @return PathFinding command
   */
  public Command driveToPoseRobotRelative(Pose2d pose, PathConstraints constraints, double endSpeed)
  {
    Pose2d robot2d = getPose();

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        new Pose2d(pose.getX()+robot2d.getX(), pose.getY()+robot2d.getY(), 
            new Rotation2d(pose.getRotation().getRadians()+robot2d.getRotation().getRadians())),
        constraints,
        endSpeed // Goal end velocity in meters/sec
                                     );
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public LinearVelocity getVelocityMagnitude() {
    ChassisSpeeds cs = getFieldVelocity();
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}