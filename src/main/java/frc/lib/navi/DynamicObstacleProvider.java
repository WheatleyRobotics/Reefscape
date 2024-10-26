package frc.lib.navi;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public interface DynamicObstacleProvider {
  List<Pair<Translation2d, Translation2d>> getDynamicObstacles();
}
