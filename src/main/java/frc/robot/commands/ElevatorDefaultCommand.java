// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.enums.ElevatorPosition;

public class ElevatorDefaultCommand extends CommandBase {
  
  // PIDController pid = new PIDController(0.0, 0.0, 0);

  ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, null);
  ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.05, 1.13, 0);

  ElevatorPosition previousPostion = ElevatorPosition.Floor; 
  boolean hold = false;

  /** Creates a new ElevatorCommand. */
  public ElevatorDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ElevatorPosition position = RobotContainer.elevatPos;

    if(previousPostion != position){
      hold = false;
    }

    //Check if the elevator needs to be held in position
    if(hold){
      if(!RobotContainer.elevator.TopLimitReached() && !RobotContainer.elevator.BottomLimitReached()){
        
          if(position == ElevatorPosition.Mid){
            // RobotContainer.voltageMove();
            // RobotContainer.elevator.Move(0.08);
          }
        return;
      }else{
        RobotContainer.elevator.Move(0);
        return;
      }
    }
    
    if(position == ElevatorPosition.Mid && !RobotContainer.elevator.TopLimitReached()){
      RobotContainer.elevator.voltageMove(1.13); //1.13 Volts .05 static
      if(RobotContainer.elevator.MidReached()){
        hold = true;
      }
    }else if(position == ElevatorPosition.Floor && !RobotContainer.elevator.BottomLimitReached()){
      RobotContainer.elevator.Move(-.001);
    }else{
      RobotContainer.elevator.Move(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}