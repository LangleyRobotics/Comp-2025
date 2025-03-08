// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorCmd extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final int position;
  private final double targetPosition;
 
  public SetElevatorCmd(ElevatorSubsystem elevatorSubsystem, int position) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;

    if(position == 1) {
      //**L1 position**
      this.targetPosition = ElevatorConstants.l1;
    } else if (position == 2) {
      //**L2 position**
      this.targetPosition = ElevatorConstants.l2;
    } else if (position == 3) {
      //**L3 position**
      this.targetPosition = ElevatorConstants.l3;
    } else if (position == 4) {
      //**L4 position**
      this.targetPosition = ElevatorConstants.l4;

    } else if(position == 5){
      this.targetPosition = ElevatorConstants.lowAlgae;
    }  else if (position == 6){
        this.targetPosition = ElevatorConstants.highAlgae;
    }
    else if (position == 9){
    this.targetPosition = 82.8;
    } else {
      this.targetPosition = ElevatorConstants.kMinElevatorPosition;
    }

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setGoal(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopElevatorMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtSetpoint();
  }
}
