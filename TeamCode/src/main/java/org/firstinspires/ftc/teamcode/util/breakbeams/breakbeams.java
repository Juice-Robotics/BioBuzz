
package org.firstinspires.ftc.teamcode.util.breakbeams;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class breakbeams {

    private static final int CAPACITY = 4;

    public DigitalChannel beam1, beam2, beam3, beam4;

    public breakbeams(HardwareMap map) {
        beam1 = map.digitalChannel.get("beam1");
        beam2 = map.digitalChannel.get("beam2");
        beam3 = map.digitalChannel.get("beam3");
        beam4 = map.digitalChannel.get("beam4");

        beam1.setMode(DigitalChannel.Mode.INPUT);
        beam2.setMode(DigitalChannel.Mode.INPUT);
        beam3.setMode(DigitalChannel.Mode.INPUT);
        beam4.setMode(DigitalChannel.Mode.INPUT);
    }

    public int getBalls() {
        int count = 0;

        if (!beam1.getState()) count++;
        if (!beam2.getState()) count++;
        if (!beam3.getState()) count++;
        if (!beam4.getState()) count++;

        return count;
    }

    public boolean isFull() {
        return getBalls() == CAPACITY;
    }

    public boolean isEmpty() {
        return getBalls() == 0;
    }

    public void update() {
    }
}
