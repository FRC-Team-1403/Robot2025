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
  private final double time = 10;
  private double currentVel;
  private double targetVel;
  private int counter;
  private BooleanSupplier m_close;
  private BooleanSupplier m_far;

  public ElevatorCommand(Elevator elevator, BooleanSupplier close, BooleanSupplier far) {
    m_elevator = elevator;
    m_close = close;
    m_far = far;
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_elevator.limitSwitch()) {
      m_elevator.moveToSetPoint(0);
    }
    else if (m_close.getAsBoolean() && targetVel != 5) {
      targetVel = 5;
      m_elevator.moveToSetPoint(5);
    }
    else if (m_close.getAsBoolean() && targetVel != 10) {
      targetVel = 10;
      m_elevator.moveToSetPoint(10);
    }

    // DogLog.log("time", time);
    currentVel = m_elevator.getSpeed();
    DogLog.log("target velocity", targetVel);
    DogLog.log("Error", targetVel - currentVel);
    DogLog.log("current velocity", currentVel);

    // if((currentVel + (100/(time/0.02)) < targetVel)) {
    //   currentVel = currentVel + (100/(time/0.02));
    //   System.out.println(currentVel);
    // }
    // else {
    //   currentVel = targetVel;
    // }
    // if (m_elevator.limitSwitch()) {
    //   m_elevator.stopMotors();
    // }
    // else if (m_elevator.getSpeed() != currentVel) {
    //   m_elevator.setMotorSpeed(currentVel);
    // }    
    // RampOutput = RampOutput + (UnitPerRampTime[100] / (RampTime[Input] / ProgRate[Rate Program Runs]))
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
