// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.WinchIOFalcon;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.dispenser.*;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOFalcon;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.slam.Slam;
import frc.robot.subsystems.superstructure.slam.SlamIO;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.DynamicAuto;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  RobotState robotState = RobotState.getInstance();
  DynamicAuto dynamicAuto;
  // Subsystems
  private Drive drive;
  private Vision vision;
  Elevator elevator;
  Dispenser dispenser;
  Slam slam;
  Climb climb;
  private final LED leds = LED.getInstance();
  private final Superstructure superstructure;
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight", drive::getRotation),
                new VisionIOPhotonVision(
                    camera0Name,
                    robotToCamera0,
                    drive::getRotation,
                    robotState.isShouldTrigSolve()),
                new VisionIOPhotonVision(
                    camera1Name,
                    robotToCamera1,
                    drive::getRotation,
                    robotState.isShouldTrigSolve()));
        elevator = new Elevator(new ElevatorIOFalcon());
        dispenser = new Dispenser(new PivotIOFalconIntegrated(), new TunnelIOFalcon());
        slam = new Slam(new SlamIO() {}, new TunnelIO() {});

        climb = new Climb(new WinchIOFalcon());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name,
                    robotToCamera0,
                    drive::getRotation,
                    drive::getPose,
                    RobotState.getInstance().isShouldTrigSolve()),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    robotToCamera1,
                    drive::getRotation,
                    drive::getPose,
                    RobotState.getInstance().isShouldTrigSolve()));
        elevator = new Elevator(new ElevatorIOSim());
        dispenser =
            new Dispenser(new PivotIOSim(), new TunnelIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        dispenser = new Dispenser(new PivotIO() {}, new TunnelIO() {});
        slam = new Slam(new SlamIO() {}, new TunnelIO() {});
        break;
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (dispenser == null) {
      dispenser = new Dispenser(new PivotIO() {}, new TunnelIO() {});
    }
    if (slam == null) {
      slam = new Slam(new SlamIO() {}, new TunnelIO() {});
    }
    superstructure = new Superstructure(elevator, dispenser, slam);

    // Set up auto routines
    dynamicAuto = new DynamicAuto(drive, superstructure);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    /*
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

     */
    autoChooser.addOption("Elevator static", elevator.staticCharacterization(2.0));
    autoChooser.addOption("Pivot static", dispenser.staticCharacterization(2.0));

    // Configure the button bindings
    if (Robot.isReal()) {
      configureButtonBindingsREAL();
    } else {
      configureButtonBindingsSIM();
    }
  }

  private void configureButtonBindingsREAL() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * 0.4,
                () -> -driveController.getLeftX() * 0.4,
                () -> -driveController.getRightX() * 0.3));

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () ->
                    switch (RobotState.getInstance().getCurrentZone()) {
                      case Z5 -> AllianceFlipUtil.getCorrected(
                              new Pose2d(0, 0, Rotation2d.fromDegrees(55)))
                          .getRotation();
                      case Z1 -> AllianceFlipUtil.getCorrected(
                              new Pose2d(0, 0, Rotation2d.fromDegrees(-55)))
                          .getRotation();
                      default -> RobotState.getInstance().getPose().getRotation();
                    }));

    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                      drive.setYaw(new Rotation2d());
                      RobotState.getInstance().resetPose(drive.getPose());
                    },
                    drive)
                .ignoringDisable(true));

    driveController
        .y()
        .whileTrue(superstructure.runGoal(() -> superstructure.getState().getEject()));

    driveController
        .povLeft()
        .whileTrue(Commands.run(() -> climb.runVolts(8)))
        .onFalse(Commands.runOnce(climb::stop));

    driveController
        .povRight()
        .whileTrue(Commands.run(() -> climb.runVolts(-8)))
        .onFalse(Commands.runOnce(climb::stop));

    driveController
        .leftTrigger(0.8)
        .whileTrue(
            AutoScore.getAutoScoreCommand(
                () -> RobotState.getInstance().getDesiredState(), false, drive, superstructure))
        .onFalse(AutoScore.getClearReefCommand(drive));

    driveController
        .rightTrigger(0.8)
        .whileTrue(
            AutoScore.getAutoScoreCommand(
                () -> RobotState.getInstance().getDesiredState(), true, drive, superstructure))
        .onFalse(AutoScore.getClearReefCommand(drive));

    driveController
        .rightBumper()
        .whileTrue(superstructure.runGoal(RobotState.getInstance()::getDesiredState));

    driveController.start().whileTrue(Commands.runOnce(() -> climb.setServoPosition(0.0)));

    driveController.back().whileTrue(Commands.runOnce(() -> climb.setServoPosition(1)));

    operatorController
        .x()
        .whileTrue(superstructure.runGoal(SuperstructureState.INTAKE).withName("Running Intake"));

    operatorController
        .b()
        .whileTrue(
            Commands.run(
                () -> {
                  superstructure.runVoltsTunnel(-2);
                },
                superstructure));

    operatorController
        .rightBumper() // right bumper
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setDesiredState(SuperstructureState.L2_CORAL)));

    operatorController
        .leftTrigger(0.8) // left trigger
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setDesiredState(SuperstructureState.L3_CORAL)));

    operatorController
        .rightTrigger(0.8) // right trigger
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setDesiredState(SuperstructureState.L4_CORAL)));

    operatorController
        .y()
        .whileTrue(
            superstructure.runGoal(SuperstructureState.ALGAE_L3_INTAKE).withName("L3 Algae"));

    operatorController
        .a()
        .whileTrue(
            superstructure.runGoal(SuperstructureState.ALGAE_L2_INTAKE).withName("L2 Algae"));

    operatorController.povUp().onTrue(superstructure.runGoal(SuperstructureState.BARGE));

    operatorController.povDown().onTrue(superstructure.runGoal(SuperstructureState.PROCESSING));

    operatorController
        .start()
        .whileTrue(Commands.run(() -> superstructure.runVoltsPivot(-0.5)))
        .onFalse(Commands.run(() -> superstructure.runVoltsPivot(0.0)));
    operatorController.back().onTrue(Commands.runOnce(() -> superstructure.setPositionPivot(18)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false)); // Rumble three times
    new Trigger(
            () ->
                DriverStation.isDisabled()
                    && dynamicAuto.getStartPose().equals(RobotState.getInstance().getPose()))
        .onTrue(
            Commands.run(() -> leds.autoAligned = true).andThen(() -> leds.autoUnaligned = false))
        .onFalse(
            Commands.run(() -> leds.autoUnaligned = true).andThen(() -> leds.autoAligned = false));
  }

  private void configureButtonBindingsSIM() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * 0.4,
                () -> -driveController.getLeftX() * 0.4,
                () -> -driveController.getRightX() * 0.3));

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () ->
                    switch (RobotState.getInstance().getCurrentZone()) {
                      case Z5 -> AllianceFlipUtil.getCorrected(
                              new Pose2d(0, 0, Rotation2d.fromDegrees(55)))
                          .getRotation();
                      case Z1 -> AllianceFlipUtil.getCorrected(
                              new Pose2d(0, 0, Rotation2d.fromDegrees(-55)))
                          .getRotation();
                      default -> RobotState.getInstance().getPose().getRotation();
                    }));
    /*
       driveController
           .b()
           .onTrue(
               Commands.runOnce(
                       () -> {
                         drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                         drive.setYaw(new Rotation2d());
                         RobotState.getInstance().resetPose(drive.getPose());
                       },
                       drive)
                   .ignoringDisable(true));

    */

    driveController
        .x()
        .whileTrue(
            AutoScore.getAutoScoreCommand(
                    () -> RobotState.getInstance().getDesiredState(), true, drive, superstructure)
                .alongWith(Commands.runOnce(() -> leds.autoScoring = true)))
        .onFalse(Commands.runOnce(() -> leds.autoScoring = false));

    driveController
        .y()
        .whileTrue(
            AutoScore.getAutoScoreCommand(
                    () -> RobotState.getInstance().getDesiredState(), false, drive, superstructure)
                .alongWith(Commands.runOnce(() -> leds.autoScoring = true)))
        .onFalse(Commands.runOnce(() -> leds.autoScoring = false));

    driveController
        .b()
        .whileTrue(superstructure.runGoal(() -> superstructure.getState().getEject()));

    operatorController.x().whileTrue(superstructure.runGoal(SuperstructureState.ALGAE_L2_INTAKE));

    operatorController.b().onTrue(superstructure.runGoal(SuperstructureState.PROCESSING));

    operatorController.a().onTrue(superstructure.runGoal(SuperstructureState.BARGE));

    /*
       var random = new Random();
       Container<Integer> randomInt = new Container<>();
       randomInt.value = 1;
       operatorController
           .x()
           .onTrue(
               Commands.runOnce(
                   () -> randomInt.value = random.nextInt(SuperstructureState.values().length)));
       operatorController
           .y()
           .whileTrue(
               Commands.defer(
                   () -> {
                     Logger.recordOutput("RandomState", SuperstructureState.values()[randomInt.value]);
                     return superstructure.runGoal(SuperstructureState.values()[randomInt.value]);
                   },
                   Set.of(superstructure)));

    */

  }

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driveController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driveController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driveController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return dynamicAuto.getAutoCommand();
  }
}
