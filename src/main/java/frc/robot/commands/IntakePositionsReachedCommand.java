// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakePositionsReachedCommand extends CommandBase {

  int _maxTime;
  int currentTime = 0;

  /** Creates a new IntakePositionsReachedCommand. */
  public IntakePositionsReachedCommand(int maxTime) {
    // Use addRequirements() here to declare subsystem dependencies.

    _maxTime = maxTime;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentTime++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentTime > _maxTime || 
    (RobotContainer.elevator.goalReached() && 
    RobotContainer.extender.goalReached() && 
    RobotContainer.hinge.goalReached())){
      return true;
    }
    return false;
  }
}
