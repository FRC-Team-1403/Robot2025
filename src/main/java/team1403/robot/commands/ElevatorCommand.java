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
  
  private final double time = 10;
  private double currentVel;
  private double targetVel;
  private BooleanSupplier m_close;
  private BooleanSupplier m_far;
  private BooleanSupplier m_backward;
  private BooleanSupplier m_stop;

  public ElevatorCommand(Elevator elevator, BooleanSupplier close, BooleanSupplier far, BooleanSupplier backward, BooleanSupplier stop) {
    m_elevator = elevator;
    m_close = close;
    m_far = far;
    m_backward = backward;
    m_stop = stop;
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_backward.getAsBoolean()) {
      m_elevator.setMotorSpeed(-0.1);
    }
    else if (m_elevator.limitSwitch() || m_stop.getAsBoolean()) {
      m_elevator.moveToSetPoint(0);
    }
    else if (m_close.getAsBoolean() && targetVel != .5) {
      targetVel = .5;
      m_elevator.moveToSetPoint(.5);
    }
    else if (m_far.getAsBoolean() && targetVel != 1) {
      targetVel = 1;
      m_elevator.moveToSetPoint(1);
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
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
