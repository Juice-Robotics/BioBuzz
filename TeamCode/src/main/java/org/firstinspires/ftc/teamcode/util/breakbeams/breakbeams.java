
package org.firstinspires.ftc.teamcode.util.breakbeams;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class breakbeams {

    private final DigitalChannel front;
    private final DigitalChannel back;

    private int ballCount = 0;
    private static final boolean BLOCKED_STATE = false;

    private static final long DEBOUNCE_NS = 30_000_000L;

    private boolean frontStable;
    private boolean backStable;

    private boolean frontRaw;
    private boolean backRaw;

    private long frontChangedAt;
    private long backChangedAt;

    private boolean frontSawBall = false;
    private boolean backSawBall = false;

    public breakbeams(HardwareMap hardwareMap) {

        front = hardwareMap.get(
                DigitalChannel.class, "frontBeam");

        back = hardwareMap.get(
                DigitalChannel.class, "backBeam");

        front.setMode(DigitalChannel.Mode.INPUT);
        back.setMode(DigitalChannel.Mode.INPUT);

        frontRaw = isFrontBlocked();
        backRaw = isBackBlocked();

        frontStable = frontRaw;
        backStable = backRaw;

        long now = System.nanoTime();
        frontChangedAt = now;
        backChangedAt = now;
    }

    private boolean isFrontBlocked() {
        return front.getState() == BLOCKED_STATE;
    }

    private boolean isBackBlocked() {
        return back.getState() == BLOCKED_STATE;
    }

    public void update() {

        long now = System.nanoTime();

        boolean currentFront = isFrontBlocked();

        if (currentFront != frontRaw) {
            frontRaw = currentFront;
            frontChangedAt = now;
        }

        if (frontStable != frontRaw &&
                now - frontChangedAt >= DEBOUNCE_NS) {

            frontStable = frontRaw;

            if (frontStable) {
                frontSawBall = true;
            } else if (frontSawBall) {
                ballCount = Math.min(4, ballCount + 1);
                frontSawBall = false;
            }
        }

        boolean currentBack = isBackBlocked();

        if (currentBack != backRaw) {
            backRaw = currentBack;
            backChangedAt = now;
        }

        if (backStable != backRaw &&
                now - backChangedAt >= DEBOUNCE_NS) {

            backStable = backRaw;

            if (backStable) {
                backSawBall = true;
            } else if (backSawBall) {
                ballCount = Math.max(0, ballCount - 1);
                backSawBall = false;
            }
        }
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean isFull() {
        return ballCount >= 4;
    }

    public boolean isEmpty() {
        return ballCount == 0;
    }

    public boolean isFrontBlockedNow() {
        return isFrontBlocked();
    }

    public boolean isBackBlockedNow() {
        return isBackBlocked();
    }

    public void reset() {
        ballCount = 0;
        frontSawBall = false;
        backSawBall = false;

        frontRaw = isFrontBlocked();
        backRaw = isBackBlocked();
        frontStable = frontRaw;
        backStable = backRaw;

        long now = System.nanoTime();
        frontChangedAt = now;
        backChangedAt = now;
    }
}
