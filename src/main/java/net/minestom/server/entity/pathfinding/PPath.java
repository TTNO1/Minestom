package net.minestom.server.entity.pathfinding;

import java.util.ArrayList;
import java.util.List;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

public final class PPath {

	private final List<PNode> nodes = new ArrayList<>();

	private final double pathVariance;
	private final double maxDistance;
	private int index = 0;
	private volatile State state = State.CALCULATING;

	public Point getNext() {
		if (index + 1 >= nodes.size()) return null;
		var current = nodes.get(index + 1);
		return new Vec(current.x(), current.y(), current.z());
	}

	public void setState(@NotNull PPath.State newState) {
		state = newState;
	}

	public enum State {
		CALCULATING,
		FOLLOWING,
		TERMINATING, TERMINATED, COMPUTED, BEST_EFFORT, INVALID
	}

	@NotNull State getState() {
		return state;
	}

	public @NotNull List<PNode> getNodes() {
		return nodes;
	}

	public PPath(double maxDistance, double pathVariance) {
		this.maxDistance = maxDistance;
		this.pathVariance = pathVariance;
	}

	@Override
	public @NotNull String toString() {
		return nodes.toString();
	}

	@Nullable PNode.Type getCurrentType() {
		if (index >= nodes.size()) return null;
		var current = nodes.get(index);
		return current.getType();
	}

	@Nullable Point getCurrent() {
		if (index >= nodes.size()) return null;
		var current = nodes.get(index);
		return new Vec(current.x(), current.y(), current.z());
	}

	void next() {
		if (index >= nodes.size()) return;
		index++;
	}

	double maxDistance() {
		return maxDistance;
	}

	double pathVariance() {
		return pathVariance;
	}
}
