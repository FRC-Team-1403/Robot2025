// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.lib.util.RepeatNTimes;
// import team1403.robot.commands.AlgaeIntakeCommand;
import team1403.robot.commands.AlignCommand;
import team1403.robot.commands.ClimberCommand;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.CoralIntakeSpeed;
import team1403.robot.commands.CoralMechanism;
import team1403.robot.commands.DefaultIntakeCommand;
import team1403.robot.commands.DefaultSwerveCommand;
import team1403.robot.commands.DriveWheelCharacterization;
import team1403.robot.commands.ElevatorCommand;
import team1403.robot.commands.StateMachine;
import team1403.robot.commands.WristCommand;
import team1403.robot.commands.auto.AutoHelper;
// import team1403.robot.subsystems.AlgaeIntakeSubsystem;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.Blackbox.ReefScoreLevel;
import team1403.robot.subsystems.Blackbox.ReefSelect;
import team1403.robot.subsystems.Blackbox.State;
import team1403.robot.subsystems.ClimberSubsystem;
import team1403.robot.subsystems.CoralIntakeSubsystem;
import team1403.robot.subsystems.ElevatorSubsystem;
import team1403.robot.subsystems.WristSubsystem;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerve;
  private final ElevatorSubsystem m_elevator;
  private final WristSubsystem m_wrist;
  private final CoralIntakeSubsystem m_coralIntake;
  private final StateMachine m_stateMachine;
  private final ClimberSubsystem m_climber;
  // private final AlgaeIntakeSubsystem m_algaeIntake;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private LoggedDashboardChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);

    // Configure the trigger bindings
    Blackbox.init();
    m_swerve = TunerConstants.createDrivetrain();
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);

    m_elevator = new ElevatorSubsystem();
    m_wrist = new WristSubsystem();
    m_coralIntake = new CoralIntakeSubsystem();
    m_stateMachine = new StateMachine(m_wrist, m_elevator, m_swerve, m_operatorController.getHID());
    m_climber = new ClimberSubsystem();
    // m_algaeIntake = new AlgaeIntakeSubsystem();

    if (AutoBuilder.isConfigured()) m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    else
    {
      m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
      DriverStation.reportError("Auto builder wasn't configured!", true);
    }
    
    //avoid cluttering up auto chooser at competitions
    if (Constants.ENABLE_SYSID) {
      m_autoChooser.addOption("Swerve SysID QF", m_swerve.sysIdQuasistatic(Direction.kForward));
      m_autoChooser.addOption("Swerve SysID QR", m_swerve.sysIdQuasistatic(Direction.kReverse));
      m_autoChooser.addOption("Swerve SysID DF", m_swerve.sysIdDynamic(Direction.kForward));
      m_autoChooser.addOption("Swerve SysID DR", m_swerve.sysIdDynamic(Direction.kReverse));
    }

    m_autoChooser.addOption("Drive Wheel Characterization", new DriveWheelCharacterization(m_swerve));

    // autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));
    // autoChooser.addOption("FivePieceCenter", AutoHelper.getFivePieceAuto(m_swerve));

    SmartDashboard.putData("Auto Chooser", m_autoChooser.getSendableChooser());
    if(Constants.DEBUG_MODE) {
      SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
      SmartDashboard.putData("Power Distribution", m_powerDistribution);
    }

    if(TunerConstants.SWERVE_DEBUG_MODE) {
      SmartDashboard.putData("Swerve Drive", m_swerve);
    }

    configureBindings();
  }

  private Command getAlignCommand(ReefSelect select) {
    Command vibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);
    return Commands.sequence(
      Blackbox.setAligningCmd(true),
      new DeferredCommand(() -> {
        Blackbox.reefSelect(select);
        Pose2d currentPose = m_swerve.getPose();
        Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
        if (target == null) return Commands.none();
          
      
        if(select == ReefSelect.LEFT) {
          target = CougarUtil.addDistanceToPoseLeft(target,((m_coralIntake.getDistance() - 0.201)) + 0.05);
          switch(Blackbox.reefLevel) {
            case L1: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(0)); break;
            case L2: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(-1.5)); break;
            case L3: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(-1.5)); break;
            case L4: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(-1.5)); break;
            case drive: default: /* do nothing */ break;
          }
        }
        else {
          target = CougarUtil.addDistanceToPoseLeft(target,((m_coralIntake.getDistance() - 0.201)) + 0.03);
          switch(Blackbox.reefLevel) {
            case L1: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(0)); break;
            case L2: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(-1.5)); break;
            case L3: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(-1.5)); break;
            case L4: target = CougarUtil.addDistanceToPose(target, Units.inchesToMeters(-1.5)); break;
            case drive: default: /* do nothing */ break;
          }
        } 

        return Commands.sequence(
          AutoUtil.pathFindToPose(target),
          new AlignCommand(m_swerve, target)
        );
      }, Set.of(m_swerve)),
      Blackbox.setAligningCmd(false)
    ).finallyDo((interrupted) -> {
      if(!interrupted) vibrationCmd.schedule();
      //just in case
      Blackbox.setAligning(false);
    });
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj+.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subPsystem
    // red

    //new Trigger(() -> true).whileTrue(m_stateMachine);
    RobotModeTriggers.disabled().negate()
      .and(() -> Blackbox.robotState != Blackbox.State.ManualElevator).whileTrue(m_stateMachine);
    RobotModeTriggers.disabled().negate()
      .and(() -> Blackbox.robotState == Blackbox.State.ManualElevator).whileTrue(
        Commands.run(() -> {
          m_elevator.moveToSetpoint(m_elevator.getSetpoint() - 
            MathUtil.applyDeadband(m_operatorController.getRightY(), 0.05) * Constants.kLoopTime * 32);
            m_wrist.moveToSetpoint(m_wrist.getSetpoint() + 
            MathUtil.applyDeadband(m_operatorController.getLeftY(), 0.05) * Constants.kLoopTime / 7.0);
        }, m_elevator, m_wrist));
    //Logs elevator + wrist mechanism in advantage kit
    new CoralMechanism(m_wrist, m_elevator).ignoringDisable(true).schedule();
    //RobotModeTriggers.disabled().whileFalse(m_stateMachine);

    
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getRightX(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getHID().getYButton(),
        () -> m_driverController.getHID().getAButton(),
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis()));


    // m_driverController.leftBumper().whileTrue(Commands.sequence(
    //   Blackbox.setAligningCmd(true),
    //   new DeferredCommand(() -> {
    //   Blackbox.reefSelect(ReefSelect.LEFT);
    //   Pose2d currentPose = m_swerve.getPose();
    //   Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
    //   if (target == null) return Commands.none();
    //   target = CougarUtil.addDistanceToPoseLeft(target, ((m_coralIntake.getDistance() - 0.201) - Units.inchesToMeters(2)));
    //   //target = CougarUtil.addDistanceToPoseLeft(target, (Units.inchesToMeters(2.75)));
    //   return Commands.sequence(
    //     AutoUtil.pathFindToPose(target),
    //     new AlignCommand(m_swerve, target).finallyDo((interrupted) -> {
    //       if(!interrupted) vibrationCmd.schedule();
    //     })
    //   );
    //   }, Set.of(m_swerve)), 
    //   Blackbox.setAligningCmd(false)));

    // m_driverController.rightBumper().whileTrue(Commands.sequence(
    //   Blackbox.setAligningCmd(true),
    //   new DeferredCommand(() -> {
    //   Blackbox.reefSelect(ReefSelect.RIGHT);
    //   Pose2d currentPose = m_swerve.getPose();
    //   Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
    //   if (target == null) return Commands.none();
    //   target = CougarUtil.addDistanceToPoseLeft(target, ((m_coralIntake.getDistance() - 0.201) - Units.inchesToMeters(2))); 
    //   //target = CougarUtil.addDistanceToPoseLeft(target, (Units.inchesToMeters(2.75)));
    //   return Commands.sequence(
    //     AutoUtil.pathFindToPose(target),
    //     new AlignCommand(m_swerve, target).finallyDo((interrupted) -> {
    //       if(!interrupted) vibrationCmd.schedule();
    //     })
    //   );
    //   }, Set.of(m_swerve)), 
    //   Blackbox.setAligningCmd(false)));

    m_driverController.rightBumper()
      .and(() -> Blackbox.reefLevel != ReefScoreLevel.drive 
            || Blackbox.robotState == State.ManualElevator).whileTrue(getAlignCommand(ReefSelect.RIGHT));
    m_driverController.leftBumper()
      .and(() -> Blackbox.reefLevel != ReefScoreLevel.drive
            || Blackbox.robotState == State.ManualElevator).whileTrue(getAlignCommand(ReefSelect.LEFT));

    Command vibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);
    Command opVibrationCmd = new ControllerVibrationCommand(m_operatorController.getHID(), 0.28, 1);

    //m_driverController.a().onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1));
    //SmartDashboard.putNumber("vibration", 0);
    m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.resetShallowHeading()));

    m_driverController.b().debounce(2.0).onTrue(Commands.sequence(
      m_swerve.runOnce(() -> m_swerve.resetShallowHeading(Rotation2d.kZero)),
      m_swerve.runOnce(() -> m_swerve.resetRotation(Rotation2d.kZero)),
      new DeferredCommand(() -> vibrationCmd, Set.of()) //empty set, no requirements
    ));

    //there's really no other good buttons unfortunately
    m_driverController.leftStick().whileTrue(new ClimberCommand(m_climber, Constants.Climber.upSpeed));
    m_driverController.rightStick().whileTrue(new ClimberCommand(m_climber, Constants.Climber.downSpeed));

    m_operatorController.b()
      .and(() -> Blackbox.robotState != State.ManualElevator)
      .onTrue(Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L1));
    m_operatorController.a()
      .and(() -> Blackbox.robotState != State.ManualElevator)
      .onTrue(Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L2));
    m_operatorController.x()
      .and(() -> Blackbox.robotState != State.ManualElevator)
      .onTrue(Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L3));
    m_operatorController.y()
      .and(() -> Blackbox.robotState != State.ManualElevator)
      .onTrue(Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L4));

    m_operatorController.rightBumper().onTrue(
      Commands.sequence(
        Blackbox.robotStateCmd(State.loading),
        Blackbox.setAligningCmd(false)));

    m_operatorController.povUp().debounce(0.5).onTrue(
      Commands.sequence(
        Blackbox.robotStateCmd(State.ManualElevator),
        opVibrationCmd)
    );

    
    m_operatorController.b()
      .and(() -> Blackbox.robotState == State.ManualElevator)
      .onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L1), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L1)
    )); 
    m_operatorController.a()
      .and(() -> Blackbox.robotState == State.ManualElevator)
      .onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L2), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L2)
    )); 
    m_operatorController.x()
      .and(() -> Blackbox.robotState == State.ManualElevator)
      .onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L3), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L3)
    )); 
    m_operatorController.y()
      .and(() -> Blackbox.robotState == State.ManualElevator)
      .onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L4), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.L4)
    ));
    m_operatorController.povRight()
      .and(() -> Blackbox.robotState == State.ManualElevator)
      .onTrue(
      Commands.sequence(
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.L3Algae), 
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.Source)
    ));
    /*
    m_operatorController.rightBumper()
      .and(() -> Blackbox.robotState == State.ManualElevator).onTrue(
      Commands.sequence(
        new WristCommand(m_wrist, Constants.Wrist.Setpoints.Source),
        new ElevatorCommand(m_elevator, Constants.Elevator.Setpoints.Source) 
    ).withTimeout(2)); */

    m_coralIntake.setDefaultCommand(new DefaultIntakeCommand(m_coralIntake));

    // coral intake
    // stop intake
    m_operatorController.leftBumper().whileTrue(
      new CoralIntakeSpeed(m_coralIntake, 0)
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );
    // release coral
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5)
      .debounce(0.3, DebounceType.kFalling).whileTrue(
      new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.release)
    );
    m_operatorController.leftStick().onTrue(
      Commands.sequence(
        //initially run inward
        new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.wiggle).withTimeout(0.3),
        //then wiggle
        new RepeatNTimes(Commands.sequence(
          new CoralIntakeSpeed(m_coralIntake, -Constants.CoralIntake.wiggle).withTimeout(0.3),
          new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.wiggle).withTimeout(0.4) //runs inward for longer to avoid piece falling out
      ), 4)));

    NamedCommands.registerCommand("CoralScore", new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.release).withTimeout(0.5));
    NamedCommands.registerCommand("CoralL1", Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L1));
    NamedCommands.registerCommand("CoralL2", Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L2));
    NamedCommands.registerCommand("CoralL3", Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L3));
    NamedCommands.registerCommand("CoralL4", Blackbox.reefScoreLevelCmd(Blackbox.ReefScoreLevel.L4));
    NamedCommands.registerCommand("WaitForCoral", 
      Commands.waitUntil(() -> Blackbox.isCoralLoaded()).withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand("ReefAlignL", getAlignCommand(Blackbox.ReefSelect.LEFT));
    NamedCommands.registerCommand("ReefAlignR", getAlignCommand(Blackbox.ReefSelect.RIGHT));
    
    /* Move forward 1 m from any position on the starting line 
      (make sure robot is facing a tag to seed the position) */
    m_autoChooser.addOption("Move Auto", AutoHelper.getMoveAuto(m_swerve));
    m_autoChooser.addOption("OneP Center", AutoHelper.getOnePCenter(m_swerve));
    m_autoChooser.addOption("OneP Processor", AutoHelper.getOnePProc(m_swerve));
    m_autoChooser.addOption("OneP Non Processor", AutoHelper.getOnePNonProc(m_swerve));
  }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.get();
  }
}