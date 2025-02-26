// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;

import org.ejml.dense.row.MatrixFeatures_CDRM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.commands.*;
import team1403.robot.subsystems.*;
import team1403.lib.RepeatNTimes;
import team1403.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private ElevatorSubsystem m_elevator;
  private WristSubsystem m_wrist;
  private CoralIntakeSubsystem m_coralIntake;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Command m_pathFinder = Commands.none();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);

    m_elevator = new ElevatorSubsystem();
    m_wrist = new WristSubsystem();
    m_coralIntake = new CoralIntakeSubsystem();

    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    //m_operatorController.b().whileTrue(() -> m_intakeSubsystem.setIntakeMotorSpeed(0));
   // m_operatorController.a().whileTrue().new InstantCommand(() -> m_elevator.)
    


    SmartDashboard.putData(autoChooser);

    autoChooser.addOption("Elevator SysID QF", m_elevator.getSysIDQ(Direction.kForward));
    autoChooser.addOption("Elevator SysID QR", m_elevator.getSysIDQ(Direction.kReverse));
    autoChooser.addOption("Elevator SysID DF", m_elevator.getSysIDD(Direction.kForward));
    autoChooser.addOption("Elevator SysID DR", m_elevator.getSysIDD(Direction.kReverse));

   configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_operatorController.b().onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L1), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L1)
    )); 
    m_operatorController.a().onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L2), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L2)
    )); 
    m_operatorController.x().onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L3), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L3)
    )); 
    m_operatorController.y().onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L4), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L4)
    )); 
    m_operatorController.rightBumper().onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.Source), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.Source)
    )); 

    m_coralIntake.setDefaultCommand(new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.intake));

    // coral intake
    // stop intake
    m_operatorController.leftBumper().whileTrue(
      new CoralIntakeSpeed(m_coralIntake, 0)
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );
    // release coral
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5).onTrue(
      new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.release)
      .withTimeout(.3)
    );
    // start intake
    new Trigger(() -> !m_coralIntake.hasPiece())
      .onFalse(
        new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.neutral).repeatedly()
    );
    m_operatorController.leftStick().onTrue(
    new RepeatNTimes(Commands.sequence(
        new CoralIntakeSpeed(m_coralIntake, -Constants.CoralIntake.wiggle).withTimeout(0.3),
        new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.wiggle).withTimeout(0.3)
      ), 4)
      .andThen(new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.neutral))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
