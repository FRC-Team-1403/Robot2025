package team1403.robot.commands;
import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.AlgaeIntake;
import team1403.robot.Constants;

public class AlgaeIntakeCommand extends Command {
    private AlgaeIntake m_algaeIntake;
    private BooleanSupplier m_downPos;
    private BooleanSupplier m_upPos;
    private BooleanSupplier m_expel;
    private double setpoint;

  public AlgaeIntakeCommand(AlgaeIntake algaeIntake, BooleanSupplier downPos, BooleanSupplier upPos, BooleanSupplier expel) {
    m_algaeIntake = algaeIntake;
    m_downPos = downPos;
    m_upPos = upPos;
    m_expel = expel;
    setpoint = 0;
    
    addRequirements(m_algaeIntake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (m_downPos.getAsBoolean() && setpoint != Constants.AlgaeIntake.downPos) {
      setpoint = Constants.AlgaeIntake.downPos; // 0
    }
    else if (m_upPos.getAsBoolean() && setpoint != Constants.AlgaeIntake.upPos) {
      setpoint = Constants.AlgaeIntake.upPos; // 1
    }

    if (m_algaeIntake.getAngle() > setpoint) {
        m_algaeIntake.setElbowSpeed(-0.1);
    }
    else if (m_algaeIntake.getAngle() > setpoint) {
        m_algaeIntake.setElbowSpeed(0.1);
    }
    else {
        m_algaeIntake.stopElbow();
    }

    if (m_algaeIntake.isAlgaeIntaked()) {
        m_algaeIntake.intakeStop();
    }
    else if (m_algaeIntake.ready()) {
        m_algaeIntake.setIntakeSpeed(0.3);
    }

    if (m_algaeIntake.isAlgaeIntaked() && m_expel.getAsBoolean()) {
        m_algaeIntake.setIntakeSpeed(-0.3);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
