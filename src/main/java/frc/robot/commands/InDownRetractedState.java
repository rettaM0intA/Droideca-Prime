// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.enums.ExtenderPosition;

public class InDownRetractedState extends CommandBase {

  private boolean finish = false;
  private int progress = 0;

  /** Creates a new InDownRetractedState. */
  public InDownRetractedState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //work in progress

    // if(progress == 0){
    //   if(RobotContainer.extendPos == ExtenderPosition.Retracted){
    //     progress = 2;
    //   }else{
    //     RobotContainer.extendPos = ExtenderPosition.Retracted;
    //     progress = 1;
    //   }
    //   return;
    // }

    // if(progress == 1 && RobotContainer.extender.AtDestination()){
    //   progress = 2;
    // }

    // if(progress == 2){

    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
