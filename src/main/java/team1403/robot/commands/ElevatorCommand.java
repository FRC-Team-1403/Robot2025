package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
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
  private BooleanSupplier m_slow;
  private BooleanSupplier m_fast;

  public ElevatorCommand(Elevator elevator, double m_time, BooleanSupplier slow, BooleanSupplier fast) {
    m_elevator = elevator;
    time = m_time;
    m_slow = slow;
    m_fast = fast;
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_slow.getAsBoolean() && targetVel != 0.1) {
      targetVel = 0.1;
    }
    if (m_slow.getAsBoolean() && targetVel != 0.5) {
      targetVel = 0.5;
    }

    // counter ++;
    DogLog.log("time", time);
    DogLog.log("target velocity", targetVel);
    DogLog.log("Error", targetVel - currentVel);
    DogLog.log("current velocity", currentVel);
    currentVel = m_elevator.getSpeed();

    if((currentVel + (100/(time/0.02)) < targetVel)) {
      currentVel = currentVel + (100/(time/0.02));
      System.out.println(currentVel);
    }
    else {
      currentVel = targetVel;
    }
    if (m_elevator.limitSwitch()) {
      m_elevator.stopMotors();
    }
    else if (m_elevator.getSpeed() != currentVel) {
      m_elevator.setMotorSpeed(currentVel);
    }    
    // RampOutput = RampOutput + (UnitPerRampTime[100] / (RampTime[Input] / ProgRate[Rate Program Runs]))
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
