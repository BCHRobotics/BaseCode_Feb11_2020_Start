/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


/**
 * Raises the robot at the end of the game.
 * 
 * @author Luc Suzuki, Noah Tomkins, Kyla Rowan
 */
public class Climber extends SubsystemBase {
  
  private Relay relay = new Relay(RobotMap.LIFT_SOLENOID);

  private CANSparkMax SPARK_LIFT = new CANSparkMax(RobotMap.SPARK_LIFT, MotorType.kBrushless);
  private CANEncoder encoderLift = new CANEncoder(SPARK_LIFT);

  private double encoderCal = 1;

  private double min = 0, max = 0;
  
  /**
   * Creates a new Climber.
   */
  public Climber() {

    SPARK_LIFT.setInverted(false);

  }

  /**
   * Sets the speed of the lift, limited by the min and max encoder value.
   * 
   * @param speed sets the speed of the lift mech.
   */
  public void lift(double speed){
   
    if((getEncoder() <= min && speed < 0) || (getEncoder() >= max && speed > 0)){
      SPARK_LIFT.set(0);
    } else {
      SPARK_LIFT.set(speed);
    }

  }

  /**
   * Gets the encoder value of the lift motor
   * 
   * @return the calibrated encoder position
   */
  public double getEncoder(){
    return encoderLift.getPosition() * encoderCal;
  }

  /**
   * Applies power to retract the solenoid
   */
  public void solenoidIn(){
    relay.set(Value.kOn);
  }

  /**
   * Turns off power to extend the solenoid
   */
  public void solenoidOut(){
    relay.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
