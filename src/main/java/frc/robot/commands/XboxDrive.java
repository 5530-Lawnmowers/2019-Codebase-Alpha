/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSS;
import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.ctre.phoenix.motorcontrol.ControlMode;


public class XboxDrive extends Command {

  public XboxDrive() {
    super("XboxDriveCMD");
		requires(Robot.drivetrainSS);
  }

    // A method to limit an input double to the range -1.0 to 1.0
	public double limit(double prelimNumber){
		if(prelimNumber >= 1.0) {
			return 1.0;	
		}else if(prelimNumber <= -1.0) {
			return -1.0;
		}else if(prelimNumber < 1.0 && prelimNumber >-1.0) {
			return prelimNumber;
		}else {
			return 0;
		}
  }
  
  // A method for making sure gears aren't shearing
  double GetPositionFiltered(double currentInput, double currentSpeed){
		if (Math.abs(currentSpeed - currentInput) > .025 && Math.abs(currentSpeed) < 0.3) {
			if (currentSpeed < currentInput) return currentSpeed + .025;
			else return currentSpeed - .025;
		}else {
			// if (currentSpeed < currentInput) return currentSpeed + .15;
			// else return currentSpeed - .15;
			return currentInput;
		}
	}

  // Get xAxis value of Xbox joystick; argument is stick side
	public double getStickHorizontal(char side){
		if(side == 'r'){
			return limit(OI.XBController.getX(Hand.kRight));
		
		}else if(side == 'l'){
			return limit(OI.XBController.getX(Hand.kLeft));
			
		}else{
			return 0;
		}
	}
	// Get Trigger values; arguement is trigger side
	public double getTriggerValue(char side){
		if(side == 'r'){
			return OI.XBController.getTriggerAxis(Hand.kRight);
		
		}else if(side == 'l'){
			return OI.XBController.getTriggerAxis(Hand.kLeft);
			
		}else{
			return 0;
			
		}
	}

  public double getModifiedStick(double rTrigger, double lTrigger, double stick) {
		if (rTrigger - lTrigger >= 0) return stick;
		return -stick;
	}

  //Calculates right speed based on controller output
  public double XBControllerR(double lStick, double rTrigger, double lTrigger) {
    //speed of left side = amount Accelerator is pushed down minus
    //amount Deccelerator is pushed down - lateral input from left Joystick
    
    return -(rTrigger - lTrigger - getModifiedStick(rTrigger, lTrigger, lStick));
  }
  
  //Calculates left speed based on Controller output
  public double XBControllerL(double lStick, double rTrigger, double lTrigger){
    //speed of left side = amount Accelerator is pushed down minus
    //amount Deccelerator is pushed down + lateral input from left Joystick
    
    return rTrigger - lTrigger + getModifiedStick(rTrigger, lTrigger, lStick);
  
  }

  public void setSpeeds(double lStick, double rTrigger, double lTrigger){
			
    if (XBControllerR(lStick, rTrigger, lTrigger) > 0) DrivetrainSS.frontRight.set(ControlMode.PercentOutput, Math.pow(XBControllerR(lStick, rTrigger, lTrigger), 2) * .75);
    else DrivetrainSS.frontRight.set(ControlMode.PercentOutput, -Math.pow(XBControllerR(lStick, rTrigger, lTrigger), 2) * .75);
    if (XBControllerL(lStick, rTrigger, lTrigger) > 0) DrivetrainSS.frontLeft.set(ControlMode.PercentOutput, Math.pow(XBControllerL(lStick, rTrigger, lTrigger), 2) * .75);
    else DrivetrainSS.frontLeft.set(ControlMode.PercentOutput, -Math.pow(XBControllerL(lStick, rTrigger, lTrigger), 2) * .75);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    DrivetrainSS.setFollowing();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    setSpeeds(getStickHorizontal('l'), getTriggerValue('r'), getTriggerValue('l'));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
