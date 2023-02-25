// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.HingePosition;

public class HingeSubsystem extends SubsystemBase {

  public WPI_TalonFX hingeMotor = new WPI_TalonFX(16);

  private int currentPid = 1;
  
  private final double hingeOutLimit = 55000;
  private final double hingeOutChangePoint = 40000;

  private final double hingeFloorLimit = 60000; //62000

  // private final double hingeInStartGoal = 6000;
  private final double hingeInChangePoint = 10000;
  private final double hingeInLimit = 1000;

  /** Creates a new Hinge. */
  public HingeSubsystem() {

    hingeMotor.setInverted(true);
    hingeMotor.setNeutralMode(NeutralMode.Brake);
    hingeMotor.setSelectedSensorPosition(0);

    Pid(0);
    Pivot(0);

  }

  /**
   * 
   * @param goalPosition Set to the HingePosition you want it to go to
   */
  public boolean Pivot(HingePosition goalPosition, boolean hold){
    if(hold){
      if (goalPosition == HingePosition.Straight){
        Pid(1);
        hingeMotor.set(TalonFXControlMode.Position, hingeOutLimit);
        return true;
      }
      if(goalPosition == HingePosition.Floor){
        Pid(1);
        hingeMotor.set(TalonFXControlMode.Position, hingeFloorLimit);
        return true;
      }
    }
    
    if(goalPosition == HingePosition.Straight || goalPosition == HingePosition.Floor){
      if(hingeMotor.getSelectedSensorPosition() <= hingeOutChangePoint){
        Pid(0);
        hingeMotor.set(TalonFXControlMode.Position, hingeOutLimit);

      }else if(goalPosition == HingePosition.Straight){
          
        if(hingeMotor.getSelectedSensorPosition() <= hingeOutLimit - 1200){
            Pid(2);
            hingeMotor.set(TalonFXControlMode.Position, hingeOutLimit);

        }else{
          hold = true;
        }
      }else if(goalPosition == HingePosition.Floor){

        if(hingeMotor.getSelectedSensorPosition() <= hingeFloorLimit){

          Pid(2);
          hingeMotor.set(TalonFXControlMode.Position, hingeFloorLimit);
          
        }else
          hold = true;
      }

    }

    return hold;
  }
  
  public void Pivot(double speed){
    double rtn = speed;
    
    if(rtn > .4){
      rtn = .4;
    }else if(rtn < -.7)
    rtn = -.7;

    if(rtn < 0 && hingeMotor.getSelectedSensorPosition() >= hingeInLimit){
      hingeMotor.set(TalonFXControlMode.PercentOutput, rtn);
    }else if(rtn > 0 && hingeMotor.getSelectedSensorPosition() <= hingeFloorLimit){
      hingeMotor.set(TalonFXControlMode.PercentOutput, rtn);
    }else{
      hingeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    // SmartDashboard.putNumber("RTN", rtn);

  }

  public void PivotCustom(int pidType, double GoalPosition){

    if(GoalPosition < 0 && hingeMotor.getSelectedSensorPosition() >= hingeInChangePoint){
      Pid(pidType);
      hingeMotor.set(TalonFXControlMode.Position, GoalPosition);
    }else if(GoalPosition > 0 && hingeMotor.getSelectedSensorPosition() <= hingeOutLimit){
      Pid(pidType);
      hingeMotor.set(TalonFXControlMode.Position, GoalPosition);
    }else{
      hingeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

  }

  public void Pid(int pidType){
    
    if(pidType == 0 && currentPid != 0){
      //Pull out start
      hingeMotor.config_kP(0, 0.0145);
      hingeMotor.config_kI(0, 0);
      hingeMotor.config_kD(0, 0);
      hingeMotor.config_kF(0, 0);

      hingeMotor.configPeakOutputForward(.9);

      currentPid = 0;
      return;
    }
    if(pidType == 1 && currentPid != 1){
      //Hold at position. Don't start until right by goal angle
      hingeMotor.config_kP(0, 0.06);
      hingeMotor.config_kI(0, 0);
      hingeMotor.config_kD(0, 0);
      hingeMotor.config_kF(0, -0.00006);

      currentPid = 1;
      return;
    }
    if(pidType == 2 && currentPid != 2){
      //Pull out second phase
      hingeMotor.config_kP(0, 0.017);
      hingeMotor.config_kI(0, 0);
      hingeMotor.config_kD(0, 0);
      hingeMotor.config_kF(0, -0.00002);
      

      currentPid = 2;
      return;

    }
    if(pidType == 3 && currentPid != 3){
      //Pull in second phase
      hingeMotor.config_kP(0, 0.009);
      hingeMotor.config_kI(0, -0.000018);
      hingeMotor.config_kD(0, 0);
      hingeMotor.config_kF(0, -0.00010);

      currentPid = 3;
      return;
    }
    if(pidType == 4 && currentPid != 4){
      //Pull in third phase
      hingeMotor.config_kP(0, 0.0029);
      hingeMotor.config_kI(0, 0.0000);
      hingeMotor.config_kD(0, 0);
      hingeMotor.config_kF(0, -0.00007);

      currentPid = 4;
      return;
    }
    if(pidType == 5 && currentPid != 5){
      //Pull in Start
      hingeMotor.config_kP(0, 0.0055);
      hingeMotor.config_kI(0, -0.000012);
      hingeMotor.config_kD(0, 0);
      hingeMotor.config_kF(0, -0.00015);

      currentPid = 5;
      return;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hinge position", hingeMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hinge Output", hingeMotor.getMotorOutputPercent());
  }
}
