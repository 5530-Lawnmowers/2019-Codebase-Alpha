package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.*;

import frc.robot.data.Profile;

import edu.wpi.first.wpilibj.Notifier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 *
 */
public class MotionProfile extends Command {
	
	private MotionProfileStatus _status = new MotionProfileStatus();
	private double [][][] _profile = Profile.rightSwitchProfile5;
	private int _state = 0;
	private int _loopTimeout = -1;
	private boolean _bStart = false;
	private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
	private static final int kMinPointsInTalon = 5;
	private static final int kNumLoopsTimeout = 10;
  
  // Called continuously to funnel points in top level program buffer to 
  // Talon buffer
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  
	    		DrivetrainSS.frontRight.processMotionProfileBuffer();
	    		DrivetrainSS.frontLeft.processMotionProfileBuffer();
	    }
	}
	
	Notifier _notifier = new Notifier(new PeriodicRunnable());
	
  public MotionProfile(double [][][] profile) {
      requires(Robot.drivetrainSS);
      DrivetrainSS.frontRight.changeMotionControlFramePeriod(5);
      DrivetrainSS.frontLeft.changeMotionControlFramePeriod(5);
      _notifier.startPeriodic(0.005);
      _profile = profile;
      // Use requires() here to declare subsystem dependencies
      // eg. requires(chassis);
  }
  
  public void reset() {
    DrivetrainSS.frontRight.clearMotionProfileTrajectories();
    DrivetrainSS.frontLeft.clearMotionProfileTrajectories();
    _setValue = SetValueMotionProfile.Disable;
    _loopTimeout = -1;
    _state = 0;
    _bStart = false;
    
  }
  
  // State Machine
  public void control() {
      DrivetrainSS.frontRight.getMotionProfileStatus(_status);
      DrivetrainSS.frontLeft.getMotionProfileStatus(_status);
      
      if (_loopTimeout < 0) {
        
      } else {
        
        if (_loopTimeout == 0) {
          System.out.println("Something's run");
        } else {
          --_loopTimeout;
        }
      }
      
      if (DrivetrainSS.frontRight.getControlMode() != ControlMode.MotionProfile || DrivetrainSS.frontRight.getControlMode() != ControlMode.MotionProfile) {
        System.out.println("NO PASS");
        _state = 0;
        _loopTimeout = -1;
      } else {
        switch(_state ) {
        case 0:
          //Waiting for start to Motion Profile and if we are starting then start filling the talon
          if (_bStart ) {
            _bStart = false;
            _setValue = SetValueMotionProfile.Disable;
            startFilling();
            _state = 1;
            _loopTimeout = kNumLoopsTimeout;
            System.out.println("STATE 0");
          }
          break;
        case 1:
          //Checking if we have enough points in the talon to start 
          if (_status.btmBufferCnt > kMinPointsInTalon) {
            _setValue = SetValueMotionProfile.Enable;
            _state = 2;
            _loopTimeout = kNumLoopsTimeout;
            System.out.println("STATE 1");
          }
          break;
        case 2:
          //Checking to make sure there are still enough points in the talon buffer
          if (_status.isUnderrun == false) {
            _loopTimeout = kNumLoopsTimeout;
          }
          //Checking if the motion profile is done
          if (_status.activePointValid && _status.isLast) {
            _setValue = SetValueMotionProfile.Hold;
            _state = 0;
            _loopTimeout = -1;
            
          }
          break;
        }
      }
  }
  
  // Start filling high level buffer
  private void startFilling() {
      startFilling(_profile, _profile[0][0].length);
  }
  
  private double convertToTicks(double inches ) {
      return 1024 * (inches / (6 * Math.PI));
  }
  
  // Conver inches per second to encoder velocity units
  private double convertVelocity(double inchesPerSecond){
    return 1024 * (inchesPerSecond / (60 * Math.PI));
  }
  
  private void startFilling (double[][][] profile, int totalCnt) {
    TrajectoryPoint rightPoint = new TrajectoryPoint();
    TrajectoryPoint leftPoint = new TrajectoryPoint();
    if (_status.hasUnderrun) {
      System.out.println("Has Underrun");
      DrivetrainSS.frontRight.clearMotionProfileHasUnderrun(0);
      DrivetrainSS.frontLeft.clearMotionProfileHasUnderrun(0);
      
    }
    DrivetrainSS.frontLeft.clearMotionProfileTrajectories();
    DrivetrainSS.frontRight.clearMotionProfileTrajectories();
    
    for (int i = 0; i < totalCnt; ++i) {
      rightPoint.position = -convertToTicks(profile[0][0][i]);
      rightPoint.velocity = -convertVelocity(profile[0][1][i]);
      leftPoint.position = convertToTicks(profile[1][0][i]);
      leftPoint.velocity = convertVelocity(profile[1][1][i]);
      DrivetrainSS.frontRight.configMotionProfileTrajectoryPeriod(0, 0);
      DrivetrainSS.frontLeft.configMotionProfileTrajectoryPeriod(0, 0);
      //TODO: Check this profile Slot Select
      rightPoint.profileSlotSelect0 = 3;
      rightPoint.headingDeg = 0;
      rightPoint.zeroPos = false;
      rightPoint.profileSlotSelect1 = 0;
      rightPoint.timeDur = TrajectoryDuration.Trajectory_Duration_10ms;
      leftPoint.profileSlotSelect0 = 3;
      leftPoint.headingDeg = 0;
      leftPoint.zeroPos = false;
      leftPoint.profileSlotSelect1 = 0;
      leftPoint.timeDur = TrajectoryDuration.Trajectory_Duration_10ms;
      if (i == 0) {
        rightPoint.zeroPos = true;
        leftPoint.zeroPos = true;
      }
        
      
      
      rightPoint.isLastPoint = false;
      leftPoint.isLastPoint = false;
      if ((i + 1) == totalCnt) {
        rightPoint.isLastPoint = true;
        leftPoint.isLastPoint = true;
      }
      System.out.println("Pusing point: " + rightPoint.velocity + ", " + leftPoint.position);
      
      DrivetrainSS.frontRight.pushMotionProfileTrajectory(rightPoint);
      DrivetrainSS.frontLeft.pushMotionProfileTrajectory(leftPoint);
    }
  }
  
  public void startMotionProfile() {
      _bStart = true;
  }
  
  SetValueMotionProfile getSetValue() {
      return _setValue;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
      reset();
      startMotionProfile();
      DrivetrainSS.frontRight.set(ControlMode.MotionProfile, getSetValue().value);
      DrivetrainSS.frontLeft.set(ControlMode.MotionProfile, getSetValue().value);
      control();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    DrivetrainSS.frontRight.set(ControlMode.MotionProfile, getSetValue().value);
    DrivetrainSS.frontLeft.set(ControlMode.MotionProfile, getSetValue().value);
    control();
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
      return _status.activePointValid && _status.isLast;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
