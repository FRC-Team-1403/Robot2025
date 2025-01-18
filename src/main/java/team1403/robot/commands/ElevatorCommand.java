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
  
  private double acceleration;
  private double time;
  private double currentVel;
  private double targetVel;
  private int counter;

  public ElevatorCommand(Elevator elevator, double m_time, double m_currentVel, double m_targetVel) {
    m_elevator = elevator;
    time = m_time;
    currentVel = m_currentVel;
    targetVel = m_targetVel;
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // counter ++;
    DogLog.log("time", time);
    DogLog.log("target velocity", targetVel);
    DogLog.log("Error", targetVel - currentVel);

    acceleration = (targetVel - currentVel)/time;

    if((currentVel + (100/(time/0.02)) < targetVel)) {
      //currentVel = targetVel - acceleration/(time / (50 * counter));
      currentVel = currentVel + (100/(time/0.02));
      System.out.println(currentVel);
      DogLog.log("current velocity", currentVel);
    }
    else {
      currentVel = targetVel;
    }
    //m_elevator.setMotorSpeed(currentVel);
    
    // RampOutput = RampOutput + (UnitPerRampTime[100] / (RampTime[Input] / ProgRate[Rate Program Runs]))
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
