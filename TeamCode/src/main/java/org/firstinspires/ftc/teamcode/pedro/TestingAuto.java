package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

@Autonomous
public class TestingAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(0,0,0);
    private final Pose park = p.of(10,10,90);

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    @Override
    public void init(){
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        follower.update();
        // Save the field pose every loop so TeleOp starts where Auto ended
        PoseStorage.currentPose = follower.pose();
    }
}
