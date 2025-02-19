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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.dispenser.*;
import frc.robot.subsystems.superstructure.dispenser.TunnelIO;
import frc.robot.subsystems.superstructure.dispenser.TunnelIOFalcon;
import frc.robot.subsystems.superstructure.dispenser.TunnelIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOFalcon;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.slam.Slam;
import frc.robot.subsystems.superstructure.slam.SlamIO;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  RobotState robotState = RobotState.getInstance();
  // Subsystems
  private Drive drive;
  private Vision vision;
  Elevator elevator;
  Dispenser dispenser;
  Slam slam;
  private final Superstructure superstructure;
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
                new VisionIOLimelight("limelight", drive::getRotation));

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIOFalcon());

        dispenser = new Dispenser(new PivotIOFalconIntegrated(), new TunnelIOFalcon());
        slam = new Slam(new SlamIO() {}, new TunnelIO() {});

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
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
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
    /*
       NamedCommands.registerCommand(
           "READY_L4",
           superstructure.runGoal(SuperstructureState.L4_CORAL).until(superstructure::atGoal));

       NamedCommands.registerCommand(
           "SCORE_L4",
           superstructure.runGoal(SuperstructureState.L4_CORAL_EJECT).until(superstructure::atGoal));

       NamedCommands.registerCommand(
           "INTAKE", superstructure.runGoal(SuperstructureState.INTAKE).withTimeout(1));

       NamedCommands.registerCommand(
           "STOW", superstructure.runGoal(SuperstructureState.STOW).until(superstructure::atGoal));

       NamedCommands.registerCommand(
           "ALIGN_RIGHT", new DriveToPose(drive, () -> FieldConstants.getNearestBranch(true, -0.5)));

       NamedCommands.registerCommand(
           "ALIGN_LEFT", new DriveToPose(drive, () -> FieldConstants.getNearestBranch(false, -0.5)));

    */

    NamedCommands.registerCommand(
        "INTAKE", superstructure.runGoal(SuperstructureState.INTAKE).withTimeout(1));

    NamedCommands.registerCommand(
        "SCORE_L4_RIGHT",
        AutoScore.getAutoScore(SuperstructureState.L4_CORAL, true, drive, superstructure));
    NamedCommands.registerCommand(
        "SCORE_L4_LEFT",
        AutoScore.getAutoScore(SuperstructureState.L4_CORAL, false, drive, superstructure));

    // Set up auto routines
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
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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

    // Lock to 0° when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.fromDegrees(60)));

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
    /*
       driveController
           .x()
           .whileTrue(
               Commands.sequence(
                   new DriveToPose(drive, () -> FieldConstants.getNearestBranch(false, -0.4)),
                   Commands.runOnce(
                       () -> {
                         SuperstructureState currentState = superstructure.getState();
                         SuperstructureState ejectState = SuperstructureState.getEject(currentState);
                         if (!currentState.equals(ejectState)) {
                           superstructure.runGoal(SuperstructureState.L4_CORAL_EJECT).schedule();
                         }
                       },
                       superstructure)))
           .onFalse(
               Commands.sequence(
                   Commands.run(
                           () -> {
                             drive.runVelocity(new ChassisSpeeds(-1, 0, 0));
                           })
                       .withTimeout(0.5),
                   Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))),
                   superstructure.runGoal(SuperstructureState.STOW)));

    */

    driveController
        .x()
        .whileTrue(
            AutoScore.getAutoScore(SuperstructureState.L4_CORAL, true, drive, superstructure))
        .onFalse(
            AutoScore.getClearReef(drive)
                .andThen(superstructure.runGoal(SuperstructureState.STOW)));

    driveController
        .rightBumper()
        .whileTrue(
            Commands.run(
                () -> {
                  SuperstructureState currentState = superstructure.getState();
                  SuperstructureState ejectState = SuperstructureState.getEject(currentState);
                  if (!currentState.equals(ejectState)) {
                    superstructure.runGoal(ejectState).schedule();
                  }
                }))
        .onFalse(superstructure.runGoal(SuperstructureState.STOW));

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
        .rightBumper()
        .whileTrue(
            superstructure.runGoal(SuperstructureState.L2_CORAL).withName("Scoring L2 Coral"));

    operatorController
        .leftTrigger(0.8)
        .whileTrue(
            superstructure.runGoal(SuperstructureState.L3_CORAL).withName("Scoring L3 Coral"));

    operatorController
        .rightTrigger(0.8)
        .whileTrue(
            superstructure.runGoal(SuperstructureState.L4_CORAL).withName("Scoring L4 Coral"));

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
