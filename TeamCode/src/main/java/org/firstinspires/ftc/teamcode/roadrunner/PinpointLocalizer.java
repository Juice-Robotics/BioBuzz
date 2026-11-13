package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.messages.TwoDeadWheelInputsMessage;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    @Config
    public static final class TwoDeadWheelLocalizer implements Localizer {
        public static class Params {
            public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
            public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        }

        public static Params PARAMS = new Params();

        public final Encoder par, perp;
        public final IMU imu;

        private int lastParPos, lastPerpPos;
        private Rotation2d lastHeading;

        private final double inPerTick;

        private double lastRawHeadingVel, headingVelOffset;
        private boolean initialized;
        private Pose2d pose;

        public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick, Pose2d initialPose) {
            // TODO: make sure your config has **motors** with these names (or change them)
            //   the encoders should be plugged into the slot matching the named motor
            //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
            par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

            // TODO: reverse encoder directions if needed
            //   par.setDirection(DcMotorSimple.Direction.REVERSE);

            this.imu = imu;

            this.inPerTick = inPerTick;

            FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

            pose = initialPose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair parPosVel = par.getPositionAndVelocity();
            PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
            AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            AngularVelocity angularVelocity = new AngularVelocity(
                    UnnormalizedAngleUnit.RADIANS,
                    (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                    (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                    (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                    angularVelocityDegrees.acquisitionTime
            );

            FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
            double rawHeadingVel = angularVelocity.zRotationRate;
            if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
                headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
            }
            lastRawHeadingVel = rawHeadingVel;
            double headingVel = headingVelOffset + rawHeadingVel;

            if (!initialized) {
                initialized = true;

                lastParPos = parPosVel.position;
                lastPerpPos = perpPosVel.position;
                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            int parPosDelta = parPosVel.position - lastParPos;
            int perpPosDelta = perpPosVel.position - lastPerpPos;
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = new Twist2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<Time>(new double[] {
                                    parPosDelta - PARAMS.parYTicks * headingDelta,
                                    parPosVel.velocity - PARAMS.parYTicks * headingVel,
                            }).times(inPerTick),
                            new DualNum<Time>(new double[] {
                                    perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                    perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                            }).times(inPerTick)
                    ),
                    new DualNum<>(new double[] {
                            headingDelta,
                            headingVel,
                    })
            );

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            pose = pose.plus(twist.value());
            return twist.velocity().value();
        }
    }
}
