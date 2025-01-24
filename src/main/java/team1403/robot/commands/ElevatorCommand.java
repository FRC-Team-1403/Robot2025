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
  private double setPoint;
  private double timeUp;
  private double timeDown;
  private double currentVel;
  private double targetVel;
  private double maxVel;
  private double minVel;
  private double currentPos;
  private double gain; 
  private int counter;

  public ElevatorCommand(Elevator elevator, double m_setPoint, double m_currentPos, double m_timeUp, double m_timeDown,double m_currentVel, double m_targetVel) {
    m_elevator = elevator;
    setPoint = m_setPoint;
    currentPos = m_currentPos;
    timeUp = m_timeUp;
    timeDown = m_timeDown;
    currentVel = m_currentVel;
    targetVel = m_targetVel;
    isRampDone = false;
    gain = 1;
    maxVel = 100;
    minVel = 1;
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    DogLog.log("target velocity", targetVel);
    DogLog.log("Velocity Error", targetVel - currentVel);
    DogLog.log("current velocity", currentVel);
    DogLog.log("is ramp done", isRampDone);
    DogLog.log("set point", setPoint);
    DogLog.log("current position", currentPos);
    DogLog.log("position error", setPoint - currentPos);
    
    double posError = setPoint - currentPos;
    posError *= gain;

    if((currentPos > setPoint - 0.5 && currentPos < setPoint - 0.5)  || (setPoint > currentPos && currentVel < 0) || (setPoint < currentPos && currentVel > 0)) {
      currentVel = 0;
    }

    if(Math.abs(currentVel) > maxVel) {
      currentVel = maxVel;
    }

    ramp();

    if(isRampDone) {
      currentVel = minVel;
      if(setPoint < currentPos) {
        currentVel *= -1;
      }
    }

    if(currentPos > setPoint - 0.5 && currentPos < setPoint + 0.5) {
      currentVel = 0;
    } else {
      isRampDone = false;
    }

    //m_elevator.setMotorSpeed(currentVel);
    
    // RampOutput = RampOutput + (UnitPerRampTime[100] / (RampTime[Input] / ProgRate[Rate Program Runs]))
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void ramp() {
    if(targetVel > currentVel) {
      if((currentVel + (100/(timeUp/0.02)) < targetVel)) {
        currentVel = currentVel + (100/(timeUp/0.02));
      }
      else {
        currentVel = targetVel;
      }
    } 
    else if(targetVel < currentVel) {
      if((currentVel - (100/(timeDown/0.02)) > targetVel)) {
      }
      else {
        currentVel = targetVel;
      }
    }

    if(targetVel == currentVel) {
      isRampDone = true;
    }
  }
}
