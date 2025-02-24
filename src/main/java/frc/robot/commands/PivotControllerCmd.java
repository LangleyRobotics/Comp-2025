// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.MathMethods;

/*
 * NOTE:
 * 0 degrees = arm back parallel to ground
 * 180 degrees = intake position
 * Get degrees from encoder output
 * Must calibrate encoder to that range = set offset and distance per rotation (144)
 */

public class PivotControllerCmd extends Command{

  private final PivotSubsystem pivotSubsystem;
  private final Supplier<Double> pivotPositiveDirFunction;
  private final Supplier<Double> pivotNegativeDirFunction;



  public PivotControllerCmd(PivotSubsystem pivotSubsystem, 
  Supplier<Double> pivotPositiveDirFunction, Supplier<Double> pivotNegativeDirFunction) {

    this.pivotSubsystem = pivotSubsystem;
    this.pivotPositiveDirFunction = pivotPositiveDirFunction;
    this.pivotNegativeDirFunction = pivotNegativeDirFunction;

    addRequirements(pivotSubsystem);
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    pivotSubsystem.pidPivot();

    //**Pivot up**
    if(pivotPositiveDirFunction.get() > 0) {
      pivotSubsystem.setGoal(pivotSubsystem.getGoal() + 0.1);
    } 
    
    //**Pivot down**
    else if(pivotNegativeDirFunction.get() > 0) {
      pivotSubsystem.setGoal(pivotSubsystem.getGoal() - 0.1);
    }

    //Put max and min limits on pivot
    if(pivotSubsystem.getGoal() < PivotConstants.kMinPivotPosition){
      pivotSubsystem.setGoal(PivotConstants.kMinPivotPosition);
    }

    if(pivotSubsystem.getGoal() > PivotConstants.kMaxPivotPosition){ 
      pivotSubsystem.setGoal(PivotConstants.kMaxPivotPosition);

    }
    
  }


  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopPivotMotor();

  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
