package org.firstinspires.ftc.teamcode.teleOp.testing;

public class BVState {
    boolean toggled, condition, sentinel;

    public BVState (boolean toggledIn, boolean conditionIn, boolean sentinelIn){
        assignState(toggledIn, conditionIn, sentinelIn);
    }

    public void assignState(boolean toggledIn, boolean conditionIn, boolean sentinelIn){
        toggled = toggledIn;
        condition = conditionIn;
        sentinel = sentinelIn;
    }
    public void runState(){
        if (condition && !sentinel) {
            if (!toggled) {
                toggled = true;
            }
            else {
                toggled = false;
            }
            sentinel = true;
        }
        if (!condition && sentinel){
            toggled = false;
        }
    }
    public boolean getState() {
        return toggled;
    }

}
