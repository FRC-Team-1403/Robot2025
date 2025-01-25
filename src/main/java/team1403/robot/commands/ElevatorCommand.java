package team1403.robot.commands;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Elevator;


public class ElevatorCommand extends Command {
  private Elevator m_elevator;
  private boolean isRampDone;
  private boolean isGoingUp;
  private boolean isGoingDown;
  private double setPoint;
  private double rampUpTimeUp;
  private double rampUpTimeDown;
  private double rampDownTimeUp;
  private double rampDownTimeDown;
  private double currMotorOutput;
  private double desiredMotorOutput; 
  private double maxVel;
  private double minVel;
  private double currentPos;
  private double upGain;
  private double downGain;
  private double setPointMargin; 
  private double simVar; 

  public ElevatorCommand(Elevator elevator, double m_setPoint, double m_currentPos, double m_currentMotorOutput) {
    m_elevator = elevator;
    setPoint = m_setPoint;
    currentPos = m_currentPos;
    currMotorOutput = m_currentMotorOutput;
    isRampDone = false;
    rampUpTimeUp = 1;
    rampUpTimeDown = 0.01;
    rampDownTimeDown = 0.01;
    rampDownTimeUp = 1;
    upGain = 3.5;
    downGain = 3.0;
    maxVel = 90;
    minVel = 5;
    setPointMargin = 0.5;
    desiredMotorOutput = 0;
    simVar = 1;
    

    // check whether elevator needs to go up or down 
    if(m_setPoint > currentPos - setPointMargin) {
      isGoingUp = true;
      isGoingDown = false;
    } 
    else if(m_setPoint < currentPos + setPointMargin) {
      isGoingUp = false;
      isGoingDown = true;
    } 
    else {
      isGoingUp = false;
      isGoingDown = false;
    } 
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    // set desired motor output equal to the difference between current position and setpoint * a gain constant
    double posError = setPoint - currentPos;
    if(isGoingUp) {
      posError *= upGain;
    } 
    else if(isGoingDown){
      posError *= downGain;
    }
    
    desiredMotorOutput = posError;

    // checks conditions that don't require any output by the motor 
    if(!isGoingUp && !isGoingDown || (isGoingUp && desiredMotorOutput < 0) || (isGoingDown && desiredMotorOutput > 0)) {
      desiredMotorOutput = 0;
    }

    // clamp desired motor output to a maximum value
    desiredMotorOutput = Math.abs(desiredMotorOutput);
    if(desiredMotorOutput > maxVel) {
      desiredMotorOutput = maxVel;
    }

    // run ramp function with parameters depending on whether elevator needs to go up or down
    if(isGoingUp) {
      currMotorOutput = ramp(rampUpTimeUp, rampUpTimeDown, currMotorOutput, desiredMotorOutput);
    }
    else if(isGoingDown) {
      currMotorOutput = ramp(rampDownTimeUp, rampDownTimeDown, Math.abs(currMotorOutput), desiredMotorOutput);
    }

    // once ramp function is done and the elevator is moving up or down, set velocity to a minimum value
    if((isGoingUp || isGoingDown) && isRampDone && currMotorOutput < minVel) {
      currMotorOutput = minVel;
    }

    // invert output if elevator is moving down
    if(isGoingDown) {
      currMotorOutput *= -1;
    }

    // if elevator is within the window of the setpoint, stop the motor from running and set booleans to false
    if(currentPos > setPoint - setPointMargin && currentPos < setPoint + setPointMargin) {
      currMotorOutput = 0;
      isGoingUp = false;
      isGoingDown = false;

    }
    //else set isRampDone to false and continue the following steps above 
    else {
      isRampDone = false;
    }

    currentPos += (currMotorOutput / 100) * simVar;
    if(currentPos > 150) {
      currentPos = 150;
    }
    else if (currentPos < 0) {
      currentPos = 0;
    }
    // logs all values of elevator motion
    DogLog.log("desired motor output velocity", desiredMotorOutput);
    DogLog.log("current motor output", currMotorOutput);
    DogLog.log("is ramp done", isRampDone);
    DogLog.log("set point", setPoint);
    DogLog.log("current position", currentPos);
    DogLog.log("position error", posError);
    DogLog.log("motor output error", desiredMotorOutput - currMotorOutput);
    DogLog.log("is going up", isGoingUp);
    DogLog.log("is going down", isGoingDown);
 
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // ramp function gradually brings up the output of the motor to the desired motor output
  public double ramp(double rampUpTime, double rampDownTime, double currentOutput, double desiredOutput) {
    System.out.println(currentOutput);
    // if desired output is greater than current output, run upwards ramp function 
    if(desiredOutput > currentOutput) {
      // increment current output by 100/rampUpTime/cycle rate and if that output is less than desired 
      if((currentOutput + (100/(rampUpTime/0.02)) < desiredOutput)) {
        currentOutput += (100/(rampUpTime/0.02));
      }
      // set current output to desired output
      else {
        currentOutput = desiredOutput;
      }
    }
    // if desired output is less than current output, run downwards ramp function 
    else if(desiredOutput < currentOutput) {
      if((currentOutput - (100/(rampDownTime/0.02)) > desiredOutput)) 
      {
        // decrement current output by 100/rampDownTime/cycle rate and if that output is less than desired 
        currentOutput -= (100/(rampDownTime/0.02));

      }
      else {
        currentOutput = desiredOutput;
      }
    }
    // if current motor output reaches the desired output set isRampDone to true
    if(desiredOutput == currentOutput) {
      isRampDone = true;
    }
    // returns our current output
    return currentOutput;
  }
}
