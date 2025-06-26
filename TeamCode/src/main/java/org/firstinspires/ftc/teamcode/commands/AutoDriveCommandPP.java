package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import java.util.Optional;

public class AutoDriveCommandPP extends CommandBase {
    private final Follower follower;
    private final Path path;
    private final PathChain pathChain;
    private final boolean holdPoint;

    public AutoDriveCommandPP(
            Follower follower, Optional<Path> path, Optional<PathChain> pathChain, boolean holdPoint) {
        this.follower = follower;
        if (path.isPresent() && pathChain.isPresent()) {
            throw new IllegalArgumentException("Cannot provide both Path and PathChain");
        }
        this.path = path.orElse(null);
        this.pathChain = pathChain.orElse(null);
        this.holdPoint = holdPoint;
    }

    public AutoDriveCommandPP(Follower follower, PathChain pathChain, boolean holdPoint) {
        this(follower, Optional.empty(), Optional.ofNullable(pathChain), holdPoint);
    }

    public AutoDriveCommandPP(Follower follower, Path path) {
        this(follower, Optional.of(path), Optional.empty(), false);
    }

    @Override
    public void initialize() {
        if (path != null) {
            follower.followPath(path);
        } else if (pathChain != null) {
            follower.followPath(pathChain, holdPoint);
        } else {
            throw new IllegalArgumentException("No Path or PathChain provided");
        }
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}