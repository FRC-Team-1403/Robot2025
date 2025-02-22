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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.commands.*;
import team1403.robot.subsystems.*;
import team1403.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private Elevator m_elevator;
  private WristSubsystem m_wrist;
  private CoralIntakeSubsystem m_coralIntakeSubsystem;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Command m_pathFinder = Commands.none();
  private Command m_teleopCommand = Commands.none();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);

    m_elevator = new Elevator();
    m_wrist = new WristSubsystem();
    m_coralIntakeSubsystem = new CoralIntakeSubsystem();

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
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subPsystem
    // red
    
    // m_swerve.setDefaultCommand(new DefaultSwerveCommand(
    //     m_swerve,
    //     () -> -m_driverController.getLeftX(),
    //     () -> -m_driverController.getLeftY(),
    //     () -> -m_driverController.getRightX(),
    //     () -> m_driverController.getHID().getYButtonPressed(),
    //     () -> m_driverController.getHID().getBButtonPressed(),
    //     () -> m_driverController.getHID().getXButton(),
    //     () -> m_driverController.getRightTriggerAxis(),
    //     () -> m_driverController.getLeftTriggerAxis()));

    // Command driverVibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);

    // //m_driverController.povRight().onTrue(Blackbox.reefSelect(ReefSelect.RIGHT));
    // //m_driverController.povLeft().onTrue(Blackbox.reefSelect(ReefSelect.LEFT));

    // m_driverController.rightBumper().onTrue(Blackbox.setAligningCmd(true, ReefSelect.RIGHT));

    // m_driverController.leftBumper().whileTrue(Blackbox.setAligningCmd(true,ReefSelect.LEFT));

    // //m_driverController.a().onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1));
    // //SmartDashboard.putNumber("vibration", 0);

    // m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.zeroHeading()));

    m_wrist.setDefaultCommand(new WristCommand(m_wrist, 
    () -> m_operatorController.getHID().getXButton(), () -> m_operatorController.getHID().getAButton(),  
    () -> m_operatorController.getHID().getBButton(), () -> m_operatorController.getHID().getYButton(),
    () -> m_operatorController.getHID().getRightBumperButton()));

    m_elevator.setDefaultCommand(new ElevatorCommand(m_elevator, 
    () -> m_driverController.getHID().getXButton(), () -> m_driverController.getHID().getAButton(),  
    () -> m_driverController.getHID().getBButton(), () -> m_driverController.getHID().getYButton()));

    m_coralIntakeSubsystem.setDefaultCommand(new CoralIntakeCommand(m_coralIntakeSubsystem, () -> m_operatorController.getRightTriggerAxis() > 0.5));
    
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

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }
}
