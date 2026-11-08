package org.firstinspires.ftc.teamcode.util.luts;

import java.util.NavigableMap;
import java.util.TreeMap;


public class TimeOfFlightLUT {
    // We use TreeMap directly to stay compatible with API 25
    // while keeping floorKey/ceilingKey functionality.
    private static final NavigableMap<Double, Double> TOF_LUT;

    static {
        TreeMap<Double, Double> map = new TreeMap<>();

        map.put(41.194, 0.3700);
        map.put(45.107, 0.3850);
        map.put(49.142, 0.3931);
        map.put(53.371, 0.4135);
        map.put(57.529, 0.4331);
        map.put(61.577, 0.4501);
        map.put(65.608, 0.4748);
        map.put(70.421, 0.4927);
        map.put(74.564, 0.5109);
        map.put(78.314, 0.5227);
        map.put(81.852, 0.5261);
        map.put(86.034, 0.5348);
        map.put(90.066, 0.5439);
        map.put(94.841, 0.5505);
        map.put(99.298, 0.5551);
        map.put(103.403, 0.5591);
        map.put(107.515, 0.5658);
        map.put(112.060, 0.5674);
        map.put(117.398, 0.5797);
        map.put(121.511, 0.5760);
        map.put(125.422, 0.5823);
        map.put(130.328, 0.6037);
        map.put(134.091, 0.6100);
        map.put(138.270, 0.6021);
        map.put(143.131, 0.5977);
        map.put(151.981, 0.6039);

        TOF_LUT = map;
    }

    public static Double getTimeOfFlight(double distance) {
        if (TOF_LUT.isEmpty()) return 0.0;

        Double floor = TOF_LUT.floorKey(distance);
        Double ceiling = TOF_LUT.ceilingKey(distance);

        // Handle out-of-bounds (Clamping)
        if (floor == null) return TOF_LUT.get(ceiling);
        if (ceiling == null) return TOF_LUT.get(floor);

        // Handle exact matches to avoid division by zero
        if (floor.equals(ceiling)) return TOF_LUT.get(floor);

        double lowTime = TOF_LUT.get(floor);
        double highTime = TOF_LUT.get(ceiling);

        // Linear Interpolation Formula:
        // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        return lowTime + (distance - floor) * (highTime - lowTime) / (ceiling - floor);
    }
}