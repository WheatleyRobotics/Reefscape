package frc.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import lombok.Data;

@Data
public class Fault {
  private double timestamp;
  private Elastic.ElasticNotification.NotificationLevel level =
      Elastic.ElasticNotification.NotificationLevel.ERROR;
  private String message;

  public Fault(
      double timestamp, Elastic.ElasticNotification.NotificationLevel level, String message) {
    this.timestamp = timestamp;
    this.level = level;
    this.message = message;
  }

  public Elastic.ElasticNotification toNotification(String subsystemLabel) {
    return new Elastic.ElasticNotification(level, subsystemLabel, message);
  }

  public static List<Fault> checkForFaults(String label, CANcoder cancoder) {
    List<Fault> faults = new ArrayList<>();

    if (cancoder.getFault_Hardware().getValue()) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Hardware fault", label)));
    }
    if (cancoder.getStickyFault_BootDuringEnable().getValue()) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              String.format("[%s]: Boot during enable", label)));
    }
    if (cancoder.getFault_BadMagnet().getValue()) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Bad magnet", label)));
    }
    if (cancoder.getFault_Undervoltage().getValue()) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              String.format("[%s]: Undervoltage", label)));
    }

    StatusSignal.refreshAll(
        cancoder.getFault_Hardware(),
        cancoder.getStickyFault_BootDuringEnable(),
        cancoder.getFault_BadMagnet(),
        cancoder.getFault_Undervoltage());

    return faults;
  }

  public static List<Fault> checkForFaults(String label, CANSparkMax sparkMax) {
    List<Fault> faults = new ArrayList<>();

    if (sparkMax.getFault(CANSparkBase.FaultID.kDRVFault)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Driver Fault", label)));
    }
    if (sparkMax.getStickyFault(CANSparkBase.FaultID.kSensorFault)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Sensor Fault", label)));
    }
    if (sparkMax.getFault(CANSparkBase.FaultID.kMotorFault)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Motor Fault", label)));
    }
    if (sparkMax.getFault(CANSparkBase.FaultID.kStall)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              String.format("[%s]: Stall", label)));
    }

    return faults;
  }

  public static List<Fault> checkForFaults(String label, CANSparkFlex sparkFlex) {
    List<Fault> faults = new ArrayList<>();

    if (sparkFlex.getFault(CANSparkBase.FaultID.kDRVFault)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Driver Fault", label)));
    }
    if (sparkFlex.getStickyFault(CANSparkBase.FaultID.kSensorFault)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Sensor Fault", label)));
    }
    if (sparkFlex.getFault(CANSparkBase.FaultID.kMotorFault)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: Motor Fault", label)));
    }
    if (sparkFlex.getFault(CANSparkBase.FaultID.kStall)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              String.format("[%s]: Stall", label)));
    }
    if (sparkFlex.getFault(CANSparkBase.FaultID.kEEPROMCRC)) {
      faults.add(
          new Fault(
              Timer.getFPGATimestamp(),
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              String.format("[%s]: EEPROM CRC", label)));
    }

    return faults;
  }
}
