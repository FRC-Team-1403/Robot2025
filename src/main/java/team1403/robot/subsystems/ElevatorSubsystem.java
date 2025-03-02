package team1403.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;
import team1403.robot.Constants.Elevator.Setpoints;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private ElevatorFeedforward m_ElevatorFeedforward;
  private double currentPos;
  private double currMotorOutput;
  private double desiredMotorOutput;
  private boolean isRampDone;
  private boolean isGoingUp;
  private boolean isGoingDown;
  private boolean directionFlag;
  private double posError;
  private double setpoint;
  private double minSpeed;
  private double maxSpeed;

  private SysIdRoutine m_sysIdRoutine;

  public ElevatorSubsystem() {
    m_leftMotor = new SparkMax(Constants.CanBus.leftElevatorMotorID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(Constants.CanBus.rightElevatorMotorID, MotorType.kBrushless);
    configMotors();

    m_ElevatorFeedforward = new ElevatorFeedforward(0, Constants.Elevator.kFeedforwardG, Constants.Elevator.kFeedforwardV, 0, Constants.kLoopTime);
 
    m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, 
      (state) -> Logger.recordOutput("SysIDElevatorFeedforward", state.toString())),
    new SysIdRoutine.Mechanism((voltage) -> {
      m_rightMotor.setVoltage(voltage.in(Volts));
    }, null, this));  

}

   private void configMotors() {
    SparkMaxConfig leftconfig = new SparkMaxConfig();
    leftconfig
        .idleMode(IdleMode.kBrake)
        .follow(m_rightMotor, true);
    SparkMaxConfig rightconfig = new SparkMaxConfig();
    rightconfig
        .idleMode(IdleMode.kBrake)
        .inverted(true);

    leftconfig.smartCurrentLimit(45);
    rightconfig.smartCurrentLimit(45);

    m_leftMotor.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }
  
  public void setMotorSpeed(double speed) {
    m_rightMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stopMotors() {
    setMotorSpeed(0);
  }
  
  public double getSpeed() {
    return m_rightMotor.getEncoder().getVelocity();
  }

  public double getPosition() {
    return m_rightMotor.getEncoder().getPosition();
  }

  public double FFcalculation() {
    return m_ElevatorFeedforward.calculate(0);
  }

  public Command getSysIDQ(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.quasistatic(dir);
  }

  public Command getSysIDD(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.dynamic(dir);
  }

    public void moveToSetpoint(double setPoint) {
        setpoint = setPoint;
    }

    //check whether component is moving up or down
    private void checkDirection(double setPoint) {
        if (currentPos < 1.0) {
            isGoingUp = false;
            isGoingDown = false;
        }
        else if(setPoint > currentPos + Constants.Elevator.Command.setPointMargin) {
            isGoingUp = true;
            isGoingDown = false;
        } 
        else if(setPoint < currentPos - Constants.Elevator.Command.setPointMargin) {
            isGoingUp = false;
            isGoingDown = true;
        } 
        else {
            isGoingUp = false;
            isGoingDown = false;
        } 
    }
    
    private double getDesiredOutput(double setPoint) {
        // set desired motor output equal to the difference between current position and setpoint * a gain constant
        posError = setPoint - currentPos;
        if(isGoingUp) {
            minSpeed = Constants.Elevator.Command.upMinSpeed;
            maxSpeed = Constants.Elevator.Command.upMaxSpeed;
            posError *= Constants.Elevator.Command.movementUpGain;
        } 
        else if(isGoingDown){
            minSpeed = Constants.Elevator.Command.downMinSpeed;
            maxSpeed = Constants.Elevator.Command.downMaxSpeed;
            posError *= Constants.Elevator.Command.movementDownGain;
        }
        double desiredOutput = posError;
        // checks conditions that don't require any output by the motor 
        if(!isGoingUp && !isGoingDown || (isGoingUp && desiredOutput < 0) || (isGoingDown && desiredOutput > 0)) {
            desiredOutput = 0;
        }
        // clamp desired motor output to a maximum value
        desiredOutput = Math.abs(desiredOutput);
        if(desiredOutput > maxSpeed) {
            desiredOutput = maxSpeed;
        }      
        return desiredOutput;
    }

    // ramp function gradually brings up the output of the motor to the desired motor output
    private double ramp(double rampUpTime, double rampDownTime, double currentOutput, double desiredOutput) {
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
            if((currentOutput - (100/(rampDownTime/0.02)) > desiredOutput)) {
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

    private void adjustCurrentOutput() {
        // once ramp function is done and the elevator is moving up or down, set velocity to a minimum value
        if((isGoingUp || isGoingDown) && isRampDone && currMotorOutput < minSpeed) {
            currMotorOutput = minSpeed;
        }
        // invert output if elevator is moving down
        if(isGoingDown) {
            currMotorOutput *= -1;
        }
    }

    private void checkIfReachedSetPoint(double setPoint) {
        // if elevator is within the window of the setpoint, stop the motor from running and set booleans to false
        if(currentPos > setPoint - Constants.Elevator.Command.setPointMargin && currentPos < setPoint + Constants.Elevator.Command.setPointMargin) {
            currMotorOutput = 0;
            isGoingUp = false;
            isGoingDown = false;
            directionFlag = true;
        }
        // else set isRampDone to false and continue the following steps above 
        else {
            isRampDone = false;
        }
    }

    public boolean isAtSetpoint() {
        return Math.abs(setpoint - currentPos) <= 3;
    }

    public void periodic() {

        // update current position with encoder
        currentPos = (Constants.Elevator.kMultiplier * (getPosition() / Constants.Elevator.kGearRatio) * Constants.Elevator.kConversionFactorRotationstoInches);
        //setpoint = setPoint;
        if(directionFlag && Math.abs(setpoint - currentPos) > Constants.Elevator.Command.setPointMargin) {
            checkDirection(setpoint);
        }
        desiredMotorOutput = getDesiredOutput(setpoint);

        // run ramp function with parameters depending on whether elevator needs to go up or down
        if(isGoingUp) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorUpRampUpTime, Constants.Elevator.Command.elevatorUpRampDownTime, currMotorOutput, desiredMotorOutput);
        }
        else if(isGoingDown) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorDownRampUpTime, Constants.Elevator.Command.elevatorDownRampDownTime, Math.abs(currMotorOutput), desiredMotorOutput);
        }
        checkDirection(setpoint);
        adjustCurrentOutput();
        checkIfReachedSetPoint(setpoint);
        currMotorOutput += 100.0 * FFcalculation();
        // set the speed to the motors
        setMotorSpeed(currMotorOutput / 100.0); 
        //simulatePos();

        Logger.recordOutput("Right Motor RPM", getSpeed());
        Logger.recordOutput("Left Motor Encoder", m_leftMotor.getEncoder().getPosition());
        Logger.recordOutput("Right Motor Encoder", m_rightMotor.getEncoder().getPosition());
        Logger.recordOutput("right motor degrees mod", (m_rightMotor.getEncoder().getPosition() * 360) % 360);
        Logger.recordOutput("right motor degrees", (m_rightMotor.getEncoder().getPosition() * 360) );
        Logger.recordOutput("Left Motor Speed", m_leftMotor.get());
        Logger.recordOutput("Right Motor Speed", m_rightMotor.get());
        Logger.recordOutput("desired motor output velocity", desiredMotorOutput);
        Logger.recordOutput("current motor output", currMotorOutput);
        Logger.recordOutput("is ramp done", isRampDone);
        Logger.recordOutput("current position", currentPos);
        Logger.recordOutput("position error", posError);
        Logger.recordOutput("motor output error", desiredMotorOutput - currMotorOutput);
        Logger.recordOutput("is going up", isGoingUp);
        Logger.recordOutput("is going down", isGoingDown);
        Logger.recordOutput("checking direction", directionFlag);
        Logger.recordOutput("Feedforward", FFcalculation());
        Logger.recordOutput("Elevator Setpoint", setpoint);
    }
}