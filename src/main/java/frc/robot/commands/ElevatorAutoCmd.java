// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class ElevatorAutoCmd extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final int position;
    private final double targetPosition;

    public ElevatorAutoCmd(ElevatorSubsystem elevatorSubsystem, int position) {
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
    @Override
    public void initialize() {
        elevatorSubsystem.setGoal(targetPosition);
    }
  
  
    @Override
    public void execute() {
        elevatorSubsystem.pidElevNoBounds();  
      
      //Limits on the top and bottom
      if(elevatorSubsystem.getGoal() < 0){
        elevatorSubsystem.setGoal(0.1);
      }
      if(elevatorSubsystem.getGoal() > ElevatorConstants.kMaxElevatorPosition){ 
        elevatorSubsystem.setGoal(ElevatorConstants.kMaxElevatorPosition);
  
      }
      
    }
  
  
    @Override
    public void end(boolean interrupted) {
      elevatorSubsystem.stopElevatorMotor();
    }
  
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }
