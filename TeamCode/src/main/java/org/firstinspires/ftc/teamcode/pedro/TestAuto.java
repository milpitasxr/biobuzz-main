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
public class TestAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(0,0,0);
    private final Pose park = p.of(10,10,90);
    private final Pose forwardPose = p.of(10,0,0);

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    // +x is forward when heading is 0, so this drives 10 inches straight ahead
    private Path forward() {
        return line(startPose, forwardPose).constant(startPose);
    }

    @Override
        public void init(){
            follower = Constants.create(hardwareMap);
            follower.setPose(startPose);
        }
    @Override
    public void start(){
        follower.follow(forward());

    }
    @Override
    public void loop(){
        follower.update();

        telemetry.addData("Busy", follower.isBusy());
        Pose pose = follower.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
