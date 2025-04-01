// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import javax.sound.midi.Sequencer;

import org.ejml.dense.row.decompose.TriangularSolver_CDRM;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.AllForNaught;
// import frc.robot.commands.AprilAlignCmd;
// import frc.robot.commands.AutoAlign;
// import frc.robot.commands.DriveToPointCmd;
import frc.robot.commands.ElevatorAutoCmd;
//import frc.robot.Trajectories;
import frc.robot.commands.ElevatorControllerCmd;
import frc.robot.commands.PivotControllerCmd;
//Auto Commands
import frc.robot.commands.IntakeControllerCmd;
//import frc.robot.commands.MoveToReefCmd;
import frc.robot.commands.OuttakeAutoCmd;
import frc.robot.commands.OuttakeControllerCmd;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.SetElevatorCmd;
import frc.robot.commands.SetPivotCmd;
//import frc.robot.commands.SwerveControllerCmd;
//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PivotSubsystem;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ReefCentering;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import swervelib.SwerveInputStream;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private VisionSubsystem vision = new VisionSubsystem();
  private final DriveSubsystem driveBase = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), vision); // where to configure the robot or "choose" it
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();
//  private ReefCentering reefCentering = new ReefCentering(driveBase, elevatorSubsystem);

  private final SendableChooser<Command> autoChooser;

  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

    private final CommandXboxController m_gunnerController =
      new CommandXboxController(OIConstants.kSecondaryControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings
    configureBindings();
    configureNamedCommands();

    Command goStraight = driveBase.driveToPoseRobotRelative(
      new Pose2d(1, 0, new Rotation2d(0)), 
      new PathConstraints(3, 3, 540, 720),
      0.0);

    SequentialCommandGroup manualStraightAuto = new SequentialCommandGroup(
      driveBase.driveToDistanceCommand(2, 3),
      new SetElevatorCmd(elevatorSubsystem, 4).withTimeout(1.5),
      new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedFast, () -> false).withTimeout(0.5)
    );
    

    autoChooser = AutoBuilder.buildAutoChooser("Backflip");
    autoChooser.addOption("Manual Straight Auto", goStraight);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    

    
    }

    
    SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            () -> m_driverController.getLeftY() * driveBase.getYAxisInverted(), 
                                            () -> m_driverController.getLeftX() * driveBase.getXAxisInverted())
                                            .withControllerRotationAxis(() -> m_driverController.getRightX())
                                            .deadband(0.1)
                                            .scaleTranslation(0.8)
                                            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy().scaleTranslation(0.43);

  
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                            .withControllerHeadingAxis(m_driverController::getRightX, 
                                                            m_driverController::getRightY).headingWhile(false);
                                                          //withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightX  <- change this to Y for special mode

    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
    SwerveInputStream driveRobotOrientedReefSpeed = driveRobotOriented.copy().scaleTranslation(0.2);

    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAngularVelocitySlow = driveBase.driveFieldOriented(driveAngularVelocitySlow);
    
    Command driveRobotOrientedAngularVelocitySuperSlow = driveBase.driveFieldOriented(driveRobotOrientedReefSpeed);

    

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                () -> m_driverController.getLeftY() * driveBase.getYAxisInverted(),
                                                () -> m_driverController.getLeftX() * driveBase.getXAxisInverted())
                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                .deadband(0.1)
                                                .scaleTranslation(0.8)
                                                .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                      .withControllerHeadingAxis(() -> Math.sin(
                                                                                                      m_driverController.getRawAxis(
                                                                                                          4) * Math.PI) * (Math.PI * 2),
                                                                                                  () -> Math.cos(
                                                                                                      m_driverController.getRawAxis(
                                                                                                          4) * Math.PI) *
                                                                                                        (Math.PI * 2))
                                                                      .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = driveBase.driveFieldOriented(driveDirectAngleSim);


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

      driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);



      m_driverController.start().onTrue(Commands.runOnce(driveBase::zeroGyro));
      m_driverController.back().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(ElevatorConstants.kMinElevatorPosition)));


      m_driverController.leftStick().whileTrue(driveRobotOrientedAngularVelocitySuperSlow);
      m_driverController.rightStick().whileTrue(driveFieldOrientedAngularVelocitySlow);
      m_driverController.x().onTrue(Commands.runOnce(driveBase::zeroGyro));
      m_driverController.leftBumper().toggleOnTrue(new AlignToReefTagRelative(false, driveBase));
      m_driverController.rightBumper().toggleOnTrue(new AlignToReefTagRelative(true, driveBase));

      m_driverController.leftTrigger().onTrue(new InstantCommand(() -> driveBase.invertXAxisJoystick()));
      m_driverController.rightTrigger().onTrue(new InstantCommand(() -> driveBase.invertYAxisJoystick()));
      
     // m_driverController.leftBumper().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Left).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
     // m_driverController.rightBumper().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Right).until(() -> reefCentering.haveConditionsChanged()).repeatedly());

      // m_driverController.povDown().whileTrue();
      // m_driverController.povUp().whileTrue();
      // m_driverController.povLeft().whileTrue(new CenterLimelightOnReef(driveBase, Reef.left));
      // m_driverController.povRight().whileTrue(new CenterLimelightOnReef(driveBase, Reef.right));
      m_gunnerController.axisGreaterThan(3, 0.4).whileTrue(new ElevatorControllerCmd(elevatorSubsystem,() -> true,() -> false));
      m_gunnerController.axisGreaterThan(2, 0.4).whileTrue(new ElevatorControllerCmd(elevatorSubsystem,() -> false,() -> true));
      


      //Intaking coral
      m_gunnerController.b().toggleOnTrue(new ParallelRaceGroup(
        new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedSlow, ()-> true),
        new IntakeControllerCmd(intakeSubsystem, () -> IntakeConstants.kIntakeMotorSpeed, 1)));
     
      //Outtaking coral
      m_gunnerController.a().whileTrue(new ParallelCommandGroup(new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedFast, () -> false), new IntakeControllerCmd(intakeSubsystem, () -> 0.6, 1) ));
      
      //Intaking algae
      m_gunnerController.x().whileTrue(new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedSuperFast, ()-> false));
      
      //Outtaking algae
      m_gunnerController.y().whileTrue(new OuttakeControllerCmd(outtakeSubsystem, () -> OuttakeConstants.kOuttakeMotorSpeedSuperSlow, () -> 0.0, ()-> false));
      
      //Manual pivot
      m_gunnerController.leftStick().whileTrue(new PivotControllerCmd(pivotSubsystem, () -> 0.0, () -> 1.0));
      m_gunnerController.rightStick().whileTrue(new PivotControllerCmd(pivotSubsystem, () -> 1.0, () -> 0.0));
      
      //Algae setpoints
      //m_gunnerController.leftBumper().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 5), new SetPivotCmd(pivotSubsystem, 1)));
      //m_gunnerController.rightBumper().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 6), new SetPivotCmd(pivotSubsystem, 1)));
      m_gunnerController.start().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 0), new SetPivotCmd(pivotSubsystem, 2)));
      m_gunnerController.rightBumper().onTrue(new SequentialCommandGroup(new SetElevatorCmd(elevatorSubsystem, 1),new ParallelCommandGroup( new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedSuperFast, ()-> false).withTimeout(2.5),new SequentialCommandGroup(new WaitCommand(0.4),  new SetPivotCmd(pivotSubsystem, 1),new WaitCommand(0.4),new SetElevatorCmd(elevatorSubsystem,2),new SetPivotCmd(pivotSubsystem, 0)))));
      m_gunnerController.leftBumper().onTrue(new SequentialCommandGroup(new SetElevatorCmd(elevatorSubsystem, 2),new ParallelCommandGroup( new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedSuperFast, ()-> false).withTimeout(2.5),new SequentialCommandGroup(new WaitCommand(0.4),  new SetPivotCmd(pivotSubsystem, 1),new WaitCommand(0.4),new SetElevatorCmd(elevatorSubsystem,3),new SetPivotCmd(pivotSubsystem, 0)))));
      
      //Elevator setpoints
      m_gunnerController.povUp().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 4), new SetPivotCmd(pivotSubsystem, 0)));
      m_gunnerController.povLeft().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 3), new SetPivotCmd(pivotSubsystem, 0)));
      m_gunnerController.povRight().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem,2), new SetPivotCmd(pivotSubsystem, 0)));
      m_gunnerController.povDown().whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 1), new SetPivotCmd(pivotSubsystem, 0)));
    
     
  }

  private void configureNamedCommands() {
    var pivotToUp = new SetPivotCmd(pivotSubsystem, 0).withTimeout(1.5);
    var pivotToDealgae = new SetPivotCmd(pivotSubsystem, 1).withTimeout(1.5);

    var elevatatorToL1 = new SetElevatorCmd(elevatorSubsystem, 1).withTimeout(1.5);
    var elevatatorToL2 = new SetElevatorCmd(elevatorSubsystem, 2).withTimeout(1.5);
    var elevatatorToL3 = new SetElevatorCmd(elevatorSubsystem, 3).withTimeout(1.5);
    var elevatatorToL4 = new SetElevatorCmd(elevatorSubsystem, 4).withTimeout(1.5);

    var outtake = new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedFast, () -> false).withTimeout(0.75);
    OuttakeAutoCmd intake = new OuttakeAutoCmd(outtakeSubsystem, intakeSubsystem, () -> true);
    OuttakeAutoCmd intake2 = new OuttakeAutoCmd(outtakeSubsystem, intakeSubsystem, () -> true);

    var alignReefLeft = new AlignToReefTagRelative(false, driveBase).withTimeout(1.7);
    var alignReefRight = new AlignToReefTagRelative(true, driveBase).withTimeout(1.7);

    var alignReefLeftLong = new AlignToReefTagRelative(false, driveBase).withTimeout(2.3);
    var alignReefRightLong = new AlignToReefTagRelative(true, driveBase).withTimeout(2.3);

    var invertDriverXJoysticks = new InstantCommand(() -> driveBase.invertXAxisJoystick());

    var invertDriverYJoysticks = new InstantCommand(() -> driveBase.invertYAxisJoystick());

    var invertDriverXYJoysticks = new ParallelCommandGroup(
      new InstantCommand(() -> driveBase.invertXAxisJoystick()),
      new InstantCommand(() -> driveBase.invertYAxisJoystick())
    );

    //Named Commands for PathPlanner
    NamedCommands.registerCommand("Pivot To Up", pivotToUp);
    NamedCommands.registerCommand("Pivot To Dealgae", pivotToDealgae);

    NamedCommands.registerCommand("Elevator To L1", elevatatorToL1);
    NamedCommands.registerCommand("Elevator To L2", elevatatorToL2);
    NamedCommands.registerCommand("Elevator To L3", elevatatorToL3);
    NamedCommands.registerCommand("Elevator To L4", elevatatorToL4);

    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("Intake 2", intake2);
    NamedCommands.registerCommand("Outtake", outtake);

    NamedCommands.registerCommand("Align Reef Left", alignReefLeft);
    NamedCommands.registerCommand("Align Reef Right", alignReefRight);

    NamedCommands.registerCommand("Align Reef Left Long", alignReefLeftLong);
    NamedCommands.registerCommand("Align Reef Right Long", alignReefRightLong);

    NamedCommands.registerCommand("Invert X Joystick", invertDriverXJoysticks);
    NamedCommands.registerCommand("Invert Y Joystick", invertDriverYJoysticks);
    NamedCommands.registerCommand("Invert Driver Joysticks", invertDriverXYJoysticks);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();

  }
}