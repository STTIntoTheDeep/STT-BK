package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.pedro.FollowerNotInitializedException;

import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

//TODO: documentation
public class TurnTo extends Command {
    private final PathChain path;
    private final boolean holdEnd;
    private final Double maxPower;

    private final boolean interruptible = true;
    private final Follower follower;

    public TurnTo(PathChain path, boolean holdEnd, Double maxPower, Follower follower) {
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.follower = follower;
    }

    public TurnTo(Path path, boolean holdEnd, Double maxPower, Follower follower) {this(new PathChain(path), holdEnd, maxPower, follower);}

    /**
     * Defaults to holdEnd false, maxPower 1.0
     * @param path
     * @param follower
     */
    public TurnTo(PathChain path, Follower follower) {this(path, false, 1.0, follower);}

    /**
     * Defaults to holdEnd false, maxPower 1.0
     * @param path
     * @param follower
     */
    public TurnTo(Path path, Follower follower) {this(path, false, 1.0, follower);}

    @Override
    public boolean isDone() {
        return !follower.isBusy();
    }

    @Override
    public void start() {
        if (follower == null) try {
            throw new FollowerNotInitializedException();
        } catch (FollowerNotInitializedException e) {
            throw new RuntimeException(e);
        }
        if (maxPower != null && (maxPower < 0.0 || maxPower > 1.0)) {
            throw new IllegalArgumentException("maxPower must be null or between 0 and 1");
        }

        if (maxPower != null) {
            follower.setMaxPower(maxPower);
        }
        follower.followPath(path, holdEnd);
    }

    @Override
    public void update() {follower.update();}
}
