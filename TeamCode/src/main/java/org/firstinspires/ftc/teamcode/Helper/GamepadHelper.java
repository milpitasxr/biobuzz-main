package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;

// Used to manage toggle and button presses
public class GamepadHelper {

    private OpMode op;
    private Gamepad xbox;

    // previous states for detection
    public static HashMap<String, Boolean> prev = new HashMap<>();

    // output: 1 = rising, -1 = falling, 0 = none
    public static HashMap<String, Integer> edge = new HashMap<>();

    public GamepadHelper(OpMode opmode){
        op = opmode;
        xbox = op.gamepad1;

        // face buttons
        init("a");
        init("b");
        init("x");
        init("y");

        // dpads
        init("dpad_up");
        init("dpad_down");
        init("dpad_left");
        init("dpad_right");

        // bumpers
        init("left_bumper");
        init("right_bumper");
    }

    private void init(String key) {
        prev.put(key, false);
        edge.put(key, 0);
    }

    public void update() {

        // face buttons
        updateButton("a", xbox.a);
        updateButton("b", xbox.b);
        updateButton("x", xbox.x);
        updateButton("y", xbox.y);

        // dpad
        updateButton("dpad_up", xbox.dpad_up);
        updateButton("dpad_down", xbox.dpad_down);
        updateButton("dpad_left", xbox.dpad_left);
        updateButton("dpad_right", xbox.dpad_right);

        // bumpers
        updateButton("left_bumper", xbox.left_bumper);
        updateButton("right_bumper", xbox.right_bumper);
    }

    private void updateButton(String name, boolean current) {
        boolean previous = prev.get(name);

        if (!previous && current) {
            edge.put(name, 1);   // rising
        } else if (previous && !current) {
            edge.put(name, -1);  // falling
        } else {
            edge.put(name, 0);   // none
        }

        prev.put(name, current);
    }
}