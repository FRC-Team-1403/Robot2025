package team1403.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import dev.doglog.DogLog;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private final ElevatorFeedforward m_ElevatorFeedforward;
  private double currentPos;
  private double currMotorOutput;
  private double desiredMotorOutput;
  private boolean isRampDone;
  private boolean isGoingUp;
  private boolean isGoingDown;
  private boolean directionFlag;
  private double posError;
  private double setpoint;

  public Elevator() {
    //m_leftMotor = new SparkMax(Constants.Canbus.leftElevatorMotorID, MotorType.kBrushless);
    // m_rightMotor = new SparkMax(Constants.CanBus.rightElevatorMotorID, MotorType.kBrushless);
    // configMotors();

    m_ElevatorFeedforward = new ElevatorFeedforward(0, Constants.Elevator.kFeedforwardG, Constants.Elevator.kFeedforwardV, 0, Constants.kLoopTime);
  }

//   private void configMotors() {
//     SparkMaxConfig leftconfig = new SparkMaxConfig();
//     leftconfig
//         .idleMode(IdleMode.kBrake)
//         .follow(m_rightMotor, true);
//     SparkMaxConfig rightconfig = new SparkMaxConfig();
//     rightconfig
//         .idleMode(IdleMode.kBrake)
//         .inverted(true);

//     m_leftMotor.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     m_rightMotor.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//    }
  
  public void setMotorSpeed(double speed) {
    // m_rightMotor.set(MathUtil.clamp(speed, -0.1, 0.1));
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

  public double calculation(double pos, double setpoint) {
    return m_ElevatorFeedforward.calculate(pos, setpoint);
  }

  public void periodic() {
    // DogLog.log("Right Motor RPM", getSpeed());
    // DogLog.log("Left Motor Encoder", m_leftMotor.getEncoder().getPosition());
    // DogLog.log("Right Motor Encoder", m_rightMotor.getEncoder().getPosition());
    // DogLog.log("Left Motor Speed", m_leftMotor.get());
    // DogLog.log("Right Motor Speed", m_rightMotor.get());
  }

    public void MotionProfiler() {
        currentPos = 0;
        currMotorOutput = 0;
        isRampDone = false;
        directionFlag = true;
    }

    public void moveToSetPoint(double setPoint) {
        setpoint = setPoint;
        if(directionFlag && Math.abs(setPoint - currentPos) > Constants.Elevator.Command.setPointMargin) {
            checkDirection(setPoint);
        }
        desiredMotorOutput = getDesiredOutput(setPoint);

        // run ramp function with parameters depending on whether elevator needs to go up or down
        if(isGoingUp) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorUpRampUpTime, Constants.Elevator.Command.elevatorUpRampDownTime, currMotorOutput, desiredMotorOutput);
        }
        else if(isGoingDown) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorDownRampUpTime, Constants.Elevator.Command.elevatorDownRampDownTime, Math.abs(currMotorOutput), desiredMotorOutput);
        }

        adjustCurrentOutput();
        checkIfReachedSetPoint(setPoint);
        currMotorOutput += calculation(currentPos, setpoint); 
        simulatePos();
        logValues();
    }

    //check whether component is moving up or down
    private void checkDirection(double setPoint) {
        if(setPoint > currentPos - Constants.Elevator.Command.setPointMargin) {
            isGoingUp = true;
            isGoingDown = false;
        } 
        else if(setPoint < currentPos + Constants.Elevator.Command.setPointMargin) {
            isGoingUp = false;
            isGoingDown = true;
        } 
        else {
            isGoingUp = false;
            isGoingDown = false;
        } 

        directionFlag = false;
    }
    
    private double getDesiredOutput(double setPoint) {
        // set desired motor output equal to the difference between current position and setpoint * a gain constant
        posError = setPoint - currentPos;
        if(isGoingUp) {
            posError *= Constants.Elevator.Command.movementUpGain;
        } 
        else if(isGoingDown){
            posError *= Constants.Elevator.Command.movementDownGain;
        }

        double desiredOutput = posError;

        // checks conditions that don't require any output by the motor 
        if(!isGoingUp && !isGoingDown || (isGoingUp && desiredOutput < 0) || (isGoingDown && desiredOutput > 0)) {
            desiredOutput = 0;
        }

        // clamp desired motor output to a maximum value
        desiredOutput = Math.abs(desiredOutput);
        if(desiredOutput > Constants.Elevator.Command.maxSpeed) {
            desiredOutput = Constants.Elevator.Command.maxSpeed;
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

    private void adjustCurrentOutput() {
        // once ramp function is done and the elevator is moving up or down, set velocity to a minimum value
        if((isGoingUp || isGoingDown) && isRampDone && currMotorOutput < Constants.Elevator.Command.minSpeed) {
            currMotorOutput = Constants.Elevator.Command.minSpeed + calculation(currentPos, setpoint);
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

    private void simulatePos() {
        // simulate position of elevator 
        currentPos += ((currMotorOutput / 100) * Constants.Elevator.Command.simPositionFactor);
        currentPos -= 0.001;
        if(currentPos > 150) {
            currentPos = 150;
        }
        else if (currentPos < 0) {
            currentPos = 0;
        }
    }



    private void logValues() {
        DogLog.log("desired motor output velocity", desiredMotorOutput);
        DogLog.log("current motor output", currMotorOutput);
        DogLog.log("is ramp done", isRampDone);
        DogLog.log("current position", currentPos);
        DogLog.log("position error", posError);
        DogLog.log("motor output error", desiredMotorOutput - currMotorOutput);
        DogLog.log("is going up", isGoingUp);
        DogLog.log("is going down", isGoingDown);
        DogLog.log("checking direction", directionFlag);
        DogLog.log("Feedforward", calculation(currentPos, setpoint));
    }
 }