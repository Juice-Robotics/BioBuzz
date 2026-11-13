package org.firstinspires.ftc.teamcode.util.luts;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class ShotLUT {
    private static final TreeMap<Double, List<Double>> SHOT_LUT;

    static {
        TreeMap<Double, List<Double>> map = new TreeMap<>();
        // 0 -> speed
        // 1 -> hood
        map.put(41.194, List.of(2850.0, 0.07));
        map.put(45.107, List.of(2925.0, 0.13));
        map.put(49.142, List.of(3050.0, 0.2));
        map.put(53.371, List.of(3100.0, 0.23));
        map.put(57.529, List.of(3150.0, 0.25));
        map.put(61.577, List.of(3210.0, 0.28));
        map.put(65.608, List.of(3220.0, 0.32));
        map.put(70.421, List.of(3300.0, 0.35));
        map.put(74.564, List.of(3350.0, 0.40));
        map.put(78.314, List.of(3420.0, 0.43));
        map.put(81.852, List.of(3530.0, 0.46));
        map.put(86.034, List.of(3630.0, 0.49));
        map.put(90.066, List.of(3720.0, 0.51));
        map.put(94.841, List.of(3730.0, 0.51));
        map.put(99.298, List.of(3800.0, 0.51));
        map.put(103.403, List.of(3870.0, 0.51));
        map.put(107.515, List.of(3905.0, 0.51));
        map.put(112.060, List.of(3920.0, 0.51));
        map.put(117.398, List.of(3980.0, 0.51));
        map.put(121.511, List.of(4045.0, 0.51));
        map.put(125.422, List.of(4105.0, 0.51));
        map.put(130.328, List.of(4220.0, 0.51));
        map.put(134.091, List.of(4290.0, 0.50));
        map.put(138.270, List.of(4350.0, 0.49));
        map.put(143.131, List.of(4440.0, 0.48));
        map.put(147.2, List.of(4500.0, 0.48));
        map.put(151.981, List.of(4560.0, 0.48));
        map.put(155.221, List.of(4680.0, 0.465));
        SHOT_LUT = map;
    }

    public static List<Double> getShot(double key) {
        if (SHOT_LUT.isEmpty()) return null;

        Map.Entry<Double, List<Double>> floor = SHOT_LUT.floorEntry(key);
        Map.Entry<Double, List<Double>> ceiling = SHOT_LUT.ceilingEntry(key);

        // Handle out-of-bounds keys (below minimum or above maximum)
        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();

        // Handle exact matches to prevent division by zero during interpolation
        if (floor.getKey().equals(ceiling.getKey())) {
            return floor.getValue();
        }

        // Variables for linear interpolation
        double x1 = floor.getKey();
        double x2 = ceiling.getKey();

        double y1Rpm = floor.getValue().get(0);
        double y2Rpm = ceiling.getValue().get(0);

        double y1Hood = floor.getValue().get(1);
        double y2Hood = ceiling.getValue().get(1);

        // Linear interpolation formula: y = y1 + ((y2 - y1) / (x2 - x1)) * (x - x1)
        double rpm = y1Rpm + ((y2Rpm - y1Rpm) / (x2 - x1)) * (key - x1);
        double hood = y1Hood + ((y2Hood - y1Hood) / (x2 - x1)) * (key - x1);

        return List.of(rpm, hood);
    }
}