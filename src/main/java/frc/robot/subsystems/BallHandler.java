/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Ball handler controls the delivery of the balls from the retriever to the shooter
 * 
 * @author Luc Suzuki, Noah Tomkins, Kyla Rowan
 */
public class BallHandler extends SubsystemBase {

  TalonSRX[] TALON_BALLS = {
    new TalonSRX(RobotMap.TALON_BALLHANDLE0),
    new TalonSRX(RobotMap.TALON_BALLHANDLE1),
    new TalonSRX(RobotMap.TALON_BALLHANDLE2),
    new TalonSRX(RobotMap.TALON_BALLHANDLE3)
  };

  DigitalInput BALLSENSE0 = new DigitalInput(RobotMap.DIO_BALLSENSE0);
  DigitalInput BALLSENSE1 = new DigitalInput(RobotMap.DIO_BALLSENSE1);
  DigitalInput BALLSENSE2 = new DigitalInput(RobotMap.DIO_BALLSENSE2);
  DigitalInput BALLSENSE3 = new DigitalInput(RobotMap.DIO_BALLSENSE3);
  DigitalInput BALLSENSE4 = new DigitalInput(RobotMap.DIO_BALLSENSE4);

  private boolean[] balls = {false, false, false, false, false};

  /**
   * Creates a new BallHandler
   */
  public BallHandler() {

    TALON_BALLS[0].setInverted(false);
    TALON_BALLS[1].setInverted(false);
    TALON_BALLS[2].setInverted(false);
    TALON_BALLS[3].setInverted(false);

  }

  /**
   * intakes the ball into the ball handler
   * 
   * @param speed speed of the loading [0-1]
   */
  public void load(double speed){

    for(int i = 0; i < 4; i++){
      if(!balls[i] && balls[i+1]){
        TALON_BALLS[i].set(ControlMode.PercentOutput, speed);
      } else {
        TALON_BALLS[i].set(ControlMode.PercentOutput, 0);
      }
    }

  }

  /**
   * Lets out the balls from the ball handler to the shooter
   * 
   * @param speed speed of unloading [0-1]
   */
  public void unload(double speed){

    for(int i = 0; i < 4; i++){
      TALON_BALLS[i].set(ControlMode.PercentOutput, speed);
    }

  }

  /**
   * Lets out the balls from the ball handler
   * 
   * @param speed speed of reverse unloading [0-1]
   */
  public void reverseUnload(double speed){

    for(int i = 0; i < 4; i++){
      TALON_BALLS[i].set(ControlMode.PercentOutput, -speed);
    }

  }

 

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Ball0", this.balls[0]);
    SmartDashboard.putBoolean("Ball1", this.balls[1]);
    SmartDashboard.putBoolean("Ball2", this.balls[2]);
    SmartDashboard.putBoolean("Ball3", this.balls[3]);
    SmartDashboard.putBoolean("Ball4", this.balls[4]);

    balls[0] = !BALLSENSE0.get();
    balls[1] = !BALLSENSE1.get();
    balls[2] = !BALLSENSE2.get();
    balls[3] = !BALLSENSE3.get();
    balls[4] = !BALLSENSE4.get();
  }
}
