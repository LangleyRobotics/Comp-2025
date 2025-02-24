// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.sound.midi.Sequencer;

import org.ejml.dense.row.decompose.TriangularSolver_CDRM;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.commands.AllForNaught;
import frc.robot.commands.AutoAlign;
//import frc.robot.Trajectories;
import frc.robot.commands.ElevatorControllerCmd;
import frc.robot.commands.PivotControllerCmd;
//Auto Commands
import frc.robot.commands.IntakeAutoCmd;
import frc.robot.commands.IntakeControllerCmd;
import frc.robot.commands.OuttakeControllerCmd;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.SetElevatorCmd;
import frc.robot.commands.SetPivotCmd;
import frc.robot.commands.SwerveControllerCmd;
//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PivotSubsystem;

 



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(robotDrive::getRotation2d, robotDrive::getSwerveModulePositions);
   private final OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();


  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);
  
 SendableChooser<Command> autoChooser = new SendableChooser<>();






  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    // Configure default commands/

     
    robotDrive.setDefaultCommand(
        new SwerveControllerCmd(
            robotDrive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> true));

    intakeSubsystem.setDefaultCommand(
      new IntakeControllerCmd(
        intakeSubsystem, 
        () -> 0.0, 
        0));
        
    elevatorSubsystem.setDefaultCommand(
      new ElevatorControllerCmd(
        elevatorSubsystem, 
        () -> operatorController.getRightTriggerAxis(),
        () -> operatorController.getLeftTriggerAxis()));

    outtakeSubsystem.setDefaultCommand(
      new OuttakeControllerCmd(
        outtakeSubsystem, 
        () -> 0.0, 
        () -> 0.0));

    pivotSubsystem.setDefaultCommand(
      new PivotControllerCmd(
        pivotSubsystem, 
        () -> 0.0, 
        () -> 0.0 
        ));
    

    // robotDrive.zeroHeading();

    //lightingSubsystem.setDefaultCommand(lightingSubsystem.splitColor(Color.kAquamarine, Color.kDarkCyan));



    // Register Named Commands
    // Named commands = commands other than driving around that still need to be executed in auto

    var pivotToUp = new SetPivotCmd(pivotSubsystem, 0).withTimeout(2);
    var pivotToDealgae = new SetPivotCmd(pivotSubsystem, 1).withTimeout(2);
    var pivotToProccessor = new SetPivotCmd(pivotSubsystem, 2).withTimeout(2);

    var elevatatorToL1 = new SetElevatorCmd(elevatorSubsystem, 1).withTimeout(1.5);
    var elevatatorToL2 = new SetElevatorCmd(elevatorSubsystem, 2).withTimeout(1.5);
    var elevatatorToL3 = new SetElevatorCmd(elevatorSubsystem, 3).withTimeout(1.5);
    var elevatatorToL4 = new SetElevatorCmd(elevatorSubsystem, 4).withTimeout(1.5);

    var outtakeCoral = new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedFast).withTimeout(2.5);
    var intake = new IntakeAutoCmd(intakeSubsystem, 1).withTimeout(2.5);
    


    //Named Commands for PathPlanner
    NamedCommands.registerCommand("Pivot To Up", pivotToUp);
    NamedCommands.registerCommand("Pivot To Dealgae", pivotToDealgae);
    NamedCommands.registerCommand("Pivot To Processor", pivotToProccessor);

    NamedCommands.registerCommand("Elevator To L1", elevatatorToL1);
    NamedCommands.registerCommand("Elevator To L2", elevatatorToL2);
    NamedCommands.registerCommand("Elevator To L3", elevatatorToL3);
    NamedCommands.registerCommand("Elevator To L4", elevatatorToL4);

    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("Outtake Coral", outtakeCoral);


    // Configure the button bindings
    configureButtonBindings();


    // var pivotAutoOuttake = new SequentialCommandGroup(
    //   new SetPivotCmd(pivotSubsystem, 0).withTimeout(0.2),
    //   new PivotControllerCmd(pivotSubsystem).withTimeout(1.3)).withTimeout(1.5);

    // var elevatorAutoOuttake = new SequentialCommandGroup(
    //   new SetElevatorCmd(elevatorSubsystem, 0).withTimeout(0.2),
    //   new ElevatorControllerCmd(elevatorSubsystem, () -> false, () -> false).withTimeout(1.3)).withTimeout(1.5);


    //Parallel elevator and pivot for auto
    // var pivotElevatorAutoOuttake = new ParallelCommandGroup(pivotAutoOuttake, elevatorAutoOuttake);


    var outtake2 = new IntakeAutoCmd(intakeSubsystem, 1).withTimeout(1.5);

   

    SequentialCommandGroup goStraight = robotDrive.AutoCommandFactory(Trajectories.goStraight);
    SequentialCommandGroup goStraightTurn = robotDrive.AutoCommandFactory(Trajectories.goStraightTurn);
    // SequentialCommandGroup OneBucketAutoNoDrive = new SequentialCommandGroup(
    //   pivotElevatorAutoOuttake,
    //   outtake2);
    SequentialCommandGroup OneBucketAuto = new SequentialCommandGroup(
      new WaitCommand(2),
      outtake2,
      goStraight);

  //  autoChooser.addOption("Nothing", null);
  //  autoChooser.addOption("Straight Auto", goStraight);
  //  autoChooser.addOption("Straight and Turn Auto", goStraightTurn);
  //  autoChooser.addOption("One Bucket Auto NO DRIVE", OneBucketAutoNoDrive);
  //  autoChooser.addOption("One Bucket Auto", OneBucketAuto);


   autoChooser = AutoBuilder.buildAutoChooser();
   
  SmartDashboard.putData("Auto Chooser", autoChooser);

  }
 
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

   // SmartDashboard.putData("Straight Auto", new PathPlannerAuto("Straight Auto"));
    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));



    // //Auto-align
    // new JoystickButton(driverController, Buttons.A).whileTrue(new AutoAlign(robotDrive, visionSubsystem));

    
    //Intake coral into arm
    new JoystickButton(operatorController, Buttons.B).whileTrue(new ParallelCommandGroup(
      new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedSlow),
      new IntakeControllerCmd(intakeSubsystem, () -> IntakeConstants.kIntakeMotorSpeed, 1)));

    //Outtake coral from arm
    new JoystickButton(operatorController, Buttons.A).whileTrue(new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedFast));

    //Intake algae
    new JoystickButton(operatorController, Buttons.X).whileTrue(new OuttakeControllerCmd(outtakeSubsystem, () -> 0.0, () -> OuttakeConstants.kOuttakeMotorSpeedFast));

    //Outtake algae
    new JoystickButton(operatorController, Buttons.Y).whileTrue(new OuttakeControllerCmd(outtakeSubsystem, () -> OuttakeConstants.kOuttakeMotorSpeedFast, () -> 0.0));

    //Pivot up
    new JoystickButton(operatorController, Buttons.R3).whileTrue(new PivotControllerCmd(pivotSubsystem, () -> 1.0, () -> 0.0));

    //Pivot down
    new JoystickButton(operatorController, Buttons.L3).whileTrue(new PivotControllerCmd(pivotSubsystem, () -> 0.0, () -> 1.0));

    //Pivot to up (coral)
    new JoystickButton(operatorController, Buttons.Maria).whileTrue(new SetPivotCmd(pivotSubsystem, 0));



    //Pivot to up position (coral)
    new JoystickButton(operatorController, Buttons.RB).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 3), new SetPivotCmd(pivotSubsystem, 1)));

    //Pivot to middle position (reef)
    new JoystickButton(operatorController, Buttons.Menu).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 0), new SetPivotCmd(pivotSubsystem, 2)));

    //Pivot to down position (processor)
    new JoystickButton(operatorController, Buttons.LB).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 2), new SetPivotCmd(pivotSubsystem, 1)));

    
    

   

    //Elevator to L4 position
    new POVButton(operatorController, Buttons.UP_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 4), new SetPivotCmd(pivotSubsystem, 0)));
    
    //Elevator to L3 position
    new POVButton(operatorController, Buttons.LEFT_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 3), new SetPivotCmd(pivotSubsystem, 0)));

    //Elevator to L2 position
    new POVButton(operatorController, Buttons.RIGHT_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 2), new SetPivotCmd(pivotSubsystem, 0)));

    //Elevator to L1 position
    new POVButton(operatorController, Buttons.DOWN_ARR).whileTrue(new ParallelCommandGroup(new SetElevatorCmd(elevatorSubsystem, 1), new SetPivotCmd(pivotSubsystem, 0)));








    // //Rumble controllers
    // new JoystickButton(driverController, Buttons.Maria).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    // new JoystickButton(operatorController, Buttons.L3).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    // new JoystickButton(operatorController, Buttons.R3).whileTrue(new RumbleCmd(driverController, 2, 1.00));



    //Zero Heading
    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));

    //Reset pivot position (pivot must be all the way down)
    new JoystickButton(driverController, Buttons.Y).onTrue(new InstantCommand(() -> pivotSubsystem.setPivotPosition(PivotConstants.kMaxPivotPosition)));

    //Reset elevator position (elevator must be all the way down)
    new JoystickButton(driverController, Buttons.B).onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(ElevatorConstants.kMinElevatorPosition)));

    //Slow drive with d-pad
    new POVButton(driverController, Buttons.UP_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> false));
    new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> false));
    new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> false));
    new POVButton(driverController, Buttons.RIGHT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> false));



   // -----------------------------------------------new JoystickButton(driverController, Buttons.B).onTrue(new InstantCommand(() -> robotDrive.resetEncoders()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return the autonomous command given by the drop-down selector in ShuffleBoard
   return autoChooser.getSelected();
  }

}
