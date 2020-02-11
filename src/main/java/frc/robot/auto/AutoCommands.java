/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

/**
 * Collection of all PID commands to run autonomous
 */
public class AutoCommands {

    private AHRS ahrs;
    private Drivetrain mDrivetrain;

    private double speedENCO = -0.004, speedGYRO = 0.005;

    private double kP = 0, kI = 0, kD = 0;
    private double error = 0, derivative = 0, integral = 0, rcw = 0;
    private double endTime = 0, inRange = 0, inRangeSet = 0;

    private double kP2 = 0, kI2 = 0, kD2 = 0;
    private double error2 = 0, derivative2 = 0, integral2 = 0, rcw2 = 0;

    private double previous_error = 0;
    private double previous_error2 = 0;

    public AutoCommands(AHRS ahrs, Drivetrain mDrivetrain, boolean newBot){

        this.ahrs = ahrs;
        this.mDrivetrain = mDrivetrain;

        // Set local variables to the RobotMap varibales
        if(newBot){
            this.kP = RobotMap.P_DRIVETRAIN;
            this.kI = RobotMap.I_DRIVETRAIN;
            this.kD = RobotMap.D_DRIVETRAIN;

            this.kP2 = RobotMap.P_NAVX;
            this.kI2 = RobotMap.I_NAVX;
            this.kD2 = RobotMap.D_NAVX;
        } else{
            this.kP = RobotMap.P_DRIVETRAIN_OLD;
            this.kI = RobotMap.I_DRIVETRAIN_OLD;
            this.kD = RobotMap.D_DRIVETRAIN_OLD;

            this.kP2 = RobotMap.P_NAVX_OLD;
            this.kI2 = RobotMap.I_NAVX_OLD;
            this.kD2 = RobotMap.D_NAVX_OLD;
        }

    }

    /**
     * Drive the robot only using the encoders
     * 
     * @param setpoint the point at which we want the encoders to be at
     * @param speed speed at which the motors are running at in auto. [-1.0,1.0]
     * @param range The range we want the value to be between 
     * @param rangeTimeMs How long you want to be in the range
     * @param timeoutMs Max time you want this command to run
     */
    public void encoderDrive(double setpoint, double range, double rangeTimeMs, double timeoutMs){

        this.endTime = System.currentTimeMillis() + timeoutMs;
        this.inRange = 0;
    
        boolean runLoop = true;

        //Run until you reach the timeout or are within the range for a set amount of time.
        while(runLoop){

            if(System.currentTimeMillis() >= this.endTime) runLoop = false;
            if(this.inRange >= this.inRangeSet) runLoop = false;

            /**
            * when the error is within the range of the setpoint then start counting 
            * the inRange until it catches up to the inRangeSet.
            */
            if((mDrivetrain.getEncoderBL() < (setpoint + range)) && (mDrivetrain.getEncoderBL() > (setpoint - range))){
                this.inRange = System.currentTimeMillis();
            } else {
                this.inRangeSet = System.currentTimeMillis() + rangeTimeMs;
            }

            error = setpoint - mDrivetrain.getEncoderBL();
            integral = integral + (error * 0.02);
            derivative = (error - previous_error) / 0.02;
            rcw = ( kP * error) + ( kI * integral) + ( kD * derivative);
            previous_error = error;

            mDrivetrain.arcade(rcw * speedENCO, 0);

            SmartDashboard.putNumber("endTime", this.endTime);
            SmartDashboard.putNumber("currentTime", System.currentTimeMillis());

            SmartDashboard.putNumber("encoderBL", mDrivetrain.getEncoderBL());
            SmartDashboard.putNumber("gyro", ahrs.getAngle());

        }

        mDrivetrain.arcade(0, 0);
    }

    /**
     * Drive the robot only using the robot gyro
     * 
     * @param setpoint the angle we want the gyro to be at
     * @param speed the speed at whcih the motors are turning 
     * @param range the range we want the angle to be between
     * @param rangeTimeMs How long you want to be in the range
     * @param timeoutMs Max time you want to run this command
     */
    public void gyroTurn(double setpoint, double range, double rangeTimeMs, double timeoutMs){

        this.endTime = System.currentTimeMillis() + timeoutMs;
        this.inRange = 0;
    
        boolean runLoop = true;

        //Run until you reach the timeout or are within the range for a set amount of time.
        while(runLoop){

            if(System.currentTimeMillis() >= this.endTime) runLoop = false;
            if(this.inRange >= this.inRangeSet) runLoop = false;

            /**
            * when the error is within the range of the setpoint then start counting 
            * the inRange until it catches up to the inRangeSet.
            */
            if((ahrs.getAngle() < (setpoint + range)) && (ahrs.getAngle() > (setpoint - range))){
                this.inRange = System.currentTimeMillis();
            } else {
                this.inRangeSet = System.currentTimeMillis() + rangeTimeMs;
            }

            error2 = setpoint - ahrs.getAngle();
            integral2 = integral2 + (error2 * 0.02);
            derivative2 = (error2 - previous_error2) / 0.02;
            rcw2 = ( kP2 * error2) + ( kI2 * integral2) + ( kD * derivative2);
            previous_error2 = error2;

            mDrivetrain.arcade(0, rcw2 * speedGYRO);

            SmartDashboard.putNumber("endTime", this.endTime);
            SmartDashboard.putNumber("currentTime", System.currentTimeMillis());

            SmartDashboard.putNumber("encoderBL", mDrivetrain.getEncoderBL());
            SmartDashboard.putNumber("gyro", ahrs.getAngle());

        }

        mDrivetrain.arcade(0, 0);
    }

    /**
     * Drives the robot straight
     * 
     * @param setpoint How far we want the robot to move
     * @param speedENCO how fast we want the robot to move in the y-axis
     * @param speedGYRO how fast we want the robot to adjust the side to side speed
     * @param range The range we want the value to be between for the encoder
     * @param rangeTimeMs How long you want to be in the range
     * @param timeoutMs Max time you want to run this command
     */
    public void straightDrive(double setpoint, double range, double rangeTimeMs, double timeoutMs){

        this.endTime = System.currentTimeMillis() + timeoutMs;
        this.inRange = 0;
    
        boolean runLoop = true;

        //Run until you reach the timeout or are within the range for a set amount of time.
        while(runLoop){

            if(System.currentTimeMillis() >= this.endTime) runLoop = false;
            if(this.inRange >= this.inRangeSet) runLoop = false;

            /**
            * when the error is within the range of the setpoint then start counting 
            * the inRange until it catches up to the inRangeSet.
            */
            if((mDrivetrain.getEncoderBL() < (setpoint + range)) && (mDrivetrain.getEncoderBL() > (setpoint - range))){
                this.inRange = System.currentTimeMillis();
            } else {
                this.inRangeSet = System.currentTimeMillis() + rangeTimeMs;
            }

            error = setpoint - mDrivetrain.getEncoderBL();
            integral = integral + (error * 0.02);
            derivative = (error - previous_error) / 0.02;
            rcw = ( kP * error) + ( kI * integral) + ( kD * derivative);
            previous_error = error;

            error2 = 0 - ahrs.getAngle();
            integral2 = integral2 + (error2 * 0.02);
            derivative2 = (error2 - previous_error2) / 0.02;
            rcw2 = ( kP2 * error2) + ( kI2 * integral2) + ( kD2 * derivative2);
            previous_error2 = error2;

            mDrivetrain.arcade(rcw * speedENCO, rcw2 * speedGYRO);

            SmartDashboard.putNumber("endTime", this.endTime);
            SmartDashboard.putNumber("currentTime", System.currentTimeMillis());

            SmartDashboard.putNumber("encoderBL", mDrivetrain.getEncoderBL());
            SmartDashboard.putNumber("gyro", ahrs.getAngle());

        }

        mDrivetrain.arcade(0, 0);
    }

    public void printPID(){

        SmartDashboard.putNumber("DriveP", this.kP);
        SmartDashboard.putNumber("DriveI", this.kI);
        SmartDashboard.putNumber("DriveD", this.kD);

        SmartDashboard.putNumber("GyroP", this.kP2);
        SmartDashboard.putNumber("GyroI", this.kI2);
        SmartDashboard.putNumber("GyroD", this.kD2);
    }

    public void setPID(){

        this.kP = SmartDashboard.getNumber("DrivePSet", 0);
        this.kI = SmartDashboard.getNumber("DriveISet", 0);
        this.kD = SmartDashboard.getNumber("DriveDSet", 0);

        this.kP2 = SmartDashboard.getNumber("GyroPSet", 0);
        this.kI2 = SmartDashboard.getNumber("GyroISet", 0);
        this.kD2 = SmartDashboard.getNumber("GyroDSet", 0);
    }

}
