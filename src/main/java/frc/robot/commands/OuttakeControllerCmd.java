// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.MathMethods;

/*
 * NOTE:
 * 0 degrees = arm back parallel to ground
 * 180 degrees = intake position
 * Get degrees from encoder output
 * Must calibrate encoder to that range = set offset and distance per rotation (144)
 */

public class OuttakeControllerCmd extends Command{

  private final OuttakeSubsystem outtakeSubsystem;

  private final Supplier<Double> outtakePositiveDirFunction;
  private final Supplier<Double> outtakeNegativeDirFunction;
  private Supplier<Boolean> intaking;


  public OuttakeControllerCmd(OuttakeSubsystem outtakeSubsystem,
  Supplier<Double> outtakePositiveDirFunction, Supplier<Double> outtakeNegativeDirFunction,Supplier<Boolean> intaking) {
    this.outtakeSubsystem = outtakeSubsystem;
    this.outtakePositiveDirFunction = outtakePositiveDirFunction;
    this.outtakeNegativeDirFunction = outtakeNegativeDirFunction;
    this.intaking = intaking;

    addRequirements(outtakeSubsystem);
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
      
      if(!intaking.get()){
        if(outtakeNegativeDirFunction.get() > 0){
          outtakeSubsystem.setOuttakeMotor(-outtakeNegativeDirFunction.get());
        }
        else if(outtakePositiveDirFunction.get() > 0){
          outtakeSubsystem.setOuttakeMotor(outtakePositiveDirFunction.get());
        }
      }
      else{
        
          if(outtakeSubsystem.getAllGood()&&!outtakeSubsystem.getMoveForward()){
            outtakeSubsystem.stopOuttakeMotor();
          }

          if(outtakeNegativeDirFunction.get() > 0 && !outtakeSubsystem.getAllGood()){
            outtakeSubsystem.setOuttakeMotor(-outtakeNegativeDirFunction.get());
          }

          else if(outtakePositiveDirFunction.get() > 0){
            outtakeSubsystem.setOuttakeMotor(outtakePositiveDirFunction.get());
          }
        }
      }
    
  


  @Override
  public void end(boolean interrupted) {
    outtakeSubsystem.stopOuttakeMotor();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
