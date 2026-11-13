package org.firstinspires.ftc.teamcode.util.helpers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.luts.ShotLUT;
import org.firstinspires.ftc.teamcode.util.luts.TimeOfFlightLUT;
import org.firstinspires.ftc.teamcode.util.models.ShotSolution;

import java.util.List;

public class AutoShoot {
    public static double shooterOffsetRadius = 0.5; // inches from center of bot to flywheel
    public static Pose2D redTarget  = new Pose2D(DistanceUnit.INCH, -69,  69, AngleUnit.RADIANS, 0);
    public static Pose2D blueTarget = new Pose2D(DistanceUnit.INCH, -69, -69, AngleUnit.RADIANS, 0);

    static final int    MAX_ITERATIONS              = 10;
    static final double CONVERGENCE_THRESHOLD_INCHES = 0.1;
    static final double SHOOTER_LATENCY_SECONDS      = 0.0;
    static final double HEADING_LEAD_GAIN            = 0;      // 0=no lead, 1=full; tune empirically
    static final double shooterOffsetAngle           = 0.0;    // radians from robot forward axis
    static final double TANGENTAL_BIAS               = 0.005;  // cross-track bias magnitude; tune empirically

    // FIX 5: Track whether derivatives have ever been populated so calculateShot
    // can detect a stale/standalone call rather than silently using zeros.
    private static boolean derivativesInitialized = false;

    public static boolean isRobotInShootingZone(Pose2D botPose) {
        double robotLength = 5.78740157;
        double robotWidth  = 4.72440945;

        double h = botPose.getHeading(AngleUnit.RADIANS);
        double c = Math.cos(h);
        double s = Math.sin(h);

        double[][] corners = {
                {  (robotLength)*c - ( robotWidth)*s,  (robotLength)*s + ( robotWidth)*c },
                {  (robotLength)*c - (-robotWidth)*s,  (robotLength)*s + (-robotWidth)*c },
                { (-robotLength)*c - ( robotWidth)*s, (-robotLength)*s + ( robotWidth)*c },
                { (-robotLength)*c - (-robotWidth)*s, (-robotLength)*s + (-robotWidth)*c }
        };

        for (double[] corner : corners) {
            double posX = botPose.getX(DistanceUnit.INCH) + corner[0];
            double posY = botPose.getY(DistanceUnit.INCH) + corner[1];
            if (posX > -72 && posX < -4) {
                if (Math.abs(posY) < (-posX + 4)) {
                    return true;
                }
            }
        }
        return false;
    }

    private static double prevHVel    = 0;
    private static double prevXVel    = 0;
    private static double prevYVel    = 0;
    private static double prevDist    = 0;
    private static long   prevTimeNs  = -1;

    private static double hAccel      = 0;
    private static double xAccel      = 0;
    private static double yAccel      = 0;
    private static double distanceDer = 0;

    /**
     * Call once per loop tick before calculateNormalizedVirtualTarget.
     * Estimates acceleration by finite-differencing the velocity inputs.
     */
    public static void updateDerivatives(double xVel, double yVel, double hVel,
                                         Pose2D rPose, AllianceColor color) {
        long now = System.nanoTime();
        Pose2D tPose = color == AllianceColor.RED ? redTarget : blueTarget;
        double dx = tPose.getX(DistanceUnit.INCH) - rPose.getX(DistanceUnit.INCH);
        double dy = tPose.getY(DistanceUnit.INCH) - rPose.getY(DistanceUnit.INCH);

        if (prevTimeNs > 0) {
            double dt = (now - prevTimeNs) * 1e-9;
            if (dt > 0 && dt < 0.5) {
                hAccel      = (hVel - prevHVel) / dt;
                xAccel      = (xVel - prevXVel) / dt;
                yAccel      = (yVel - prevYVel) / dt;
                distanceDer = (Math.hypot(dx, dy) - prevDist) / dt;
            }
        }

        prevHVel   = hVel;
        prevXVel   = xVel;
        prevYVel   = yVel;
        prevDist   = Math.hypot(dx, dy);
        prevTimeNs = now;

        // FIX 5: Mark derivatives as valid so calculateShot knows they're populated.
        derivativesInitialized = true;
    }

    public static Pose2D calculateNormalizedVirtualTarget(double xVel, double yVel, double hVel,
                                                          Pose2D rPose, AllianceColor color) {
        updateDerivatives(xVel, yVel, hVel, rPose, color);

        double targetX = color == AllianceColor.RED
                ? redTarget.getX(DistanceUnit.INCH)
                : blueTarget.getX(DistanceUnit.INCH);
        double targetY = color == AllianceColor.RED
                ? redTarget.getY(DistanceUnit.INCH)
                : blueTarget.getY(DistanceUnit.INCH);

        double heading = rPose.getHeading(AngleUnit.RADIANS);

        // Shooter launch origin offset from robot center.
        double launchX = rPose.getX(DistanceUnit.INCH)
                + shooterOffsetRadius * Math.cos(heading + shooterOffsetAngle);
        double launchY = rPose.getY(DistanceUnit.INCH)
                + shooterOffsetRadius * Math.sin(heading + shooterOffsetAngle);

        // Predict robot velocity at the moment the ball actually leaves the shooter.
        double launchXVel = xVel + xAccel * SHOOTER_LATENCY_SECONDS;
        double launchYVel = yVel + yAccel * SHOOTER_LATENCY_SECONDS;
        double launchHVel = hVel + hAccel * SHOOTER_LATENCY_SECONDS;

        // Tangential velocity contributed by robot rotation at the shooter offset point.
        // FIX 3: Use (heading + shooterOffsetAngle) so this stays correct if the
        // shooter is mounted off-axis. Was just `heading` before.
        double tanSpeed = launchHVel * shooterOffsetRadius;
        double tanVx = -tanSpeed * Math.sin(heading + shooterOffsetAngle);
        double tanVy =  tanSpeed * Math.cos(heading + shooterOffsetAngle);

        double totalVx = launchXVel + tanVx;
        double totalVy = launchYVel + tanVy;

        double virtualTx = targetX;
        double virtualTy = targetY;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double dx   = virtualTx - launchX;
            double dy   = virtualTy - launchY;
            double dist = Math.hypot(dx, dy);

            // The 1.05 factor adds a 5% time-of-flight margin to slightly over-lead
            // and compensate for aerodynamic drag not captured in the LUT.
            // Validated empirically for the 72mm / 6k RPM flywheel setup -- adjust if LUT changes.
            double flightTime = TimeOfFlightLUT.getTimeOfFlight(dist) * 1.05;
            if (flightTime == -1) {
                System.err.println("[SOTM] TOF out of range at dist=" + dist);
                return new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.RADIANS, 0);
            }

            double newVTx = targetX - (totalVx * flightTime);
            double newVTy = targetY - (totalVy * flightTime);

            double residual = Math.hypot(newVTx - virtualTx, newVTy - virtualTy);
            virtualTx = newVTx;
            virtualTy = newVTy;

            if (residual < CONVERGENCE_THRESHOLD_INCHES) {
                break;
            }
        }

        // FIX 1 + 2: Apply tangential (cross-track) bias AFTER convergence, as a
        // displacement perpendicular to the line-of-sight rather than scaling the
        // absolute field coordinate (which was dimensionally incorrect).
        // This runs unconditionally on the converged result.
        if (TANGENTAL_BIAS != 0 && distanceDer != 0) {
            double dx  = virtualTx - launchX;
            double dy  = virtualTy - launchY;
            double los = Math.atan2(dy, dx);
            // Perpendicular unit vector (90 degrees CCW from LOS)
            double perpX = -Math.sin(los);
            double perpY =  Math.cos(los);
            double biasMag = distanceDer * TANGENTAL_BIAS;
            virtualTx += perpX * biasMag;
            virtualTy += perpY * biasMag;
        }

        return new Pose2D(DistanceUnit.INCH, virtualTx, virtualTy, AngleUnit.RADIANS, 0);
    }

    public static Pose2D calculateNormalizedVirtualTarget(AllianceColor color) {
        double targetX = color == AllianceColor.RED
                ? redTarget.getX(DistanceUnit.INCH)
                : blueTarget.getX(DistanceUnit.INCH);
        double targetY = color == AllianceColor.RED
                ? redTarget.getY(DistanceUnit.INCH)
                : blueTarget.getY(DistanceUnit.INCH);
        return new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.RADIANS, 0);
    }

    /**
     * Calculate the shot solution (heading angle, hood, speed) for a given robot
     * pose and virtual target.
     *
     * FIX 5: hVel is now passed explicitly so this method is self-contained and
     * does not silently depend on updateDerivatives having been called first.
     * hAccel still comes from the static field (populated by updateDerivatives);
     * a warning is logged if derivatives have never been initialized.
     *
     * @param rPose  Current robot pose from Pinpoint.
     * @param tPose  Virtual target pose (output of calculateNormalizedVirtualTarget).
     * @param hVel   Current robot angular velocity in radians/sec.
     */
    public static ShotSolution calculateShot(Pose2D rPose, Pose2D tPose, double hVel) {
        // FIX 5: Warn if derivatives are stale so the caller knows heading lead is unreliable.
        if (!derivativesInitialized) {
            System.err.println("[Shot] WARNING: calculateShot called before updateDerivatives -- heading lead disabled.");
        }

        double heading = rPose.getHeading(AngleUnit.RADIANS);

        // FIX 4: Compute distance from the actual shooter launch point, not robot center,
        // to stay consistent with the origin used in calculateNormalizedVirtualTarget.
        double launchX = rPose.getX(DistanceUnit.INCH)
                + shooterOffsetRadius * Math.cos(heading + shooterOffsetAngle);
        double launchY = rPose.getY(DistanceUnit.INCH)
                + shooterOffsetRadius * Math.sin(heading + shooterOffsetAngle);

        double dx       = tPose.getX(DistanceUnit.INCH) - launchX;
        double dy       = tPose.getY(DistanceUnit.INCH) - launchY;
        double distance = Math.hypot(dx, dy);

        List<Double> shotConfig = ShotLUT.getShot(distance);
        double optimalHood  = shotConfig != null ? shotConfig.get(1) : 0.3;
        double optimalSpeed = shotConfig != null ? shotConfig.get(0) : 1600;

        double flightTime = TimeOfFlightLUT.getTimeOfFlight(distance);

        // Lead heading by the predicted rotation during ball flight.
        double effectiveHeading = heading;
        if (flightTime > 0 && derivativesInitialized) {
            effectiveHeading += hVel     * flightTime * HEADING_LEAD_GAIN
                    + 0.5 * hAccel * flightTime * flightTime * HEADING_LEAD_GAIN;
        }

        double absTargetAngle = Math.atan2(dy, dx);
        double relativeAngle  = AngleUnit.normalizeRadians(absTargetAngle - effectiveHeading);

        System.out.printf("[Shot] dist=%.1f spd=%.0f hood=%.2f relAng=%.1f° hVel=%.3f hAccel=%.3f%n",
                distance, optimalSpeed, optimalHood,
                Math.toDegrees(relativeAngle), hVel, hAccel);

        return new ShotSolution(relativeAngle, optimalHood, optimalSpeed);
    }
}