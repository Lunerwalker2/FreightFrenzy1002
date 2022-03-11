package org.firstinspires.ftc.teamcode.testing.lib;

public class basic {
    public boolean toggleRun (boolean condition, boolean locked) {
        if (condition && !locked) {
            locked = true;
            return true;
        }
        if (!condition && locked) {
            locked = false;
        }
        return false;
    }
}

