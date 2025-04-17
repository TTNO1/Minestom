package net.minestom.server.entity.pathfinding;

import java.util.Objects;

import org.jetbrains.annotations.ApiStatus;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import net.minestom.server.coordinate.BlockVec;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

/**
 * Represents a node in a path.
 * <p>
 * Using this class in a hash Collection may have bad performance (see hashCode).
 */
public class PNode {

	public enum Type {
		WALK,
		JUMP,
		FALL,
		CLIMB,
		CLIMB_WALL,
		SWIM,
		FLY,
		REPATH
	}

	private final Point point;
	private double cost;
	private double heuristic;
	private PNode parent;
	private Type type;
	
	/*
	 * For each possible neighbor position (-1, 0, or 1 increment along each axis)
	 * contains a PNode representing the move from this node to the neighboring
	 * position if it is a valid move or null if it is an invalid move or has not
	 * yet been evaluated. Indices 0, 1, and 2, correspond to -1, 0, and 1
	 * increments respectively.
	 * Used to avoid evaluating the same move multiple times.
	 * Lazily initialized.
	 */
	private PNode[][][] neighborCache = null;
	/**
	 * true if the neighbor has been evaluated, false otherwise
	 * Lazily initialized
	 */
	private boolean[][][] neighborCacheStatus = null;

	@Deprecated
	public PNode(double px, double py, double pz, double g, double h, @Nullable PNode parent) {
		this(px, py, pz, g, h, Type.WALK, parent);
	}

	@Deprecated
	public PNode(double px, double py, double pz, double cost, double heuristic, @NotNull PNode.Type type, @Nullable PNode parent) {
		this(new Vec(px, py, pz), cost, heuristic, type, parent);
	}

	public PNode(Point point, double cost, double heuristic, @Nullable PNode parent) {
		this(point, cost, heuristic, Type.WALK, parent);
	}

	/**
	 * Creates a new path node.
	 * 
	 * @param point     the location of the node
	 * @param cost      the cost of reaching this node from the start
	 * @param heuristic the heuristic of this node in relation to the goal
	 * @param type      the type of this node
	 * @param parent    the node that precedes this node
	 */
	public PNode(Point point, double cost, double heuristic, @NotNull PNode.Type type, @Nullable PNode parent) {
		this.cost = cost;
		this.heuristic = heuristic;
		this.parent = parent;
		this.type = type;
		this.point = point;
	}

	/**
	 * @deprecated use {@linkplain #getPoint()} instead
	 */
	@Deprecated
	@ApiStatus.Internal
	public double x() {
		return point.x();
	}

	/**
	 * @deprecated use {@linkplain #getPoint()} instead
	 */
	@Deprecated
	@ApiStatus.Internal
	public double y() {
		return point.y();
	}

	/**
	 * @deprecated use {@linkplain #getPoint()} instead
	 */
	@Deprecated
	@ApiStatus.Internal
	public double z() {
		return point.z();
	}

	public Point getPoint() {
		return point;
	}

	public Point getBlockPoint() {
		return new BlockVec(point);
	}

	/**
	 * @deprecated use {@linkplain #getBlockPoint()} instead
	 */
	@Deprecated
	public int blockX() {
		return (int) Math.floor(point.x());
	}

	/**
	 * @deprecated use {@linkplain #getBlockPoint()} instead
	 */
	@Deprecated
	public int blockY() {
		return (int) Math.floor(point.y());
	}

	/**
	 * @deprecated use {@linkplain #getBlockPoint()} instead
	 */
	@Deprecated
	public int blockZ() {
		return (int) Math.floor(point.z());
	}

	@ApiStatus.Internal
	public @NotNull Type getType() {
		return type;
	}

	@ApiStatus.Internal
	public double getCost() {
		return cost;
	}

	@ApiStatus.Internal
	public double getHeuristic() {
		return heuristic;
	}

	@ApiStatus.Internal
	public void setCost(double v) {
		this.cost = v;
	}

	@ApiStatus.Internal
	public void setHeuristic(double heuristic) {
		this.heuristic = heuristic;
	}

	@ApiStatus.Internal
	public void setType(@NotNull PNode.Type newType) {
		this.type = newType;
	}

	@ApiStatus.Internal
	public @Nullable PNode getParent() {
		return parent;
	}

	@ApiStatus.Internal
	public void setParent(@Nullable PNode current) {
		this.parent = current;
	}
	
	/**
	 * Returns whether or not the neighbor at the specified offset (in increments)
	 * has been evaluated yet (a neighbor is considered evaluated if
	 * setNeighborCache has been called for it).
	 * 
	 * @param x the x offset in increments (-1, 0, or 1)
	 * @param y the y offset in increments (-1, 0, or 1)
	 * @param z the z offset in increments (-1, 0, or 1)
	 * @return true if the neighbor has been evaluated, false otherwise
	 * @throws IllegalArgumentException if any of the arguments are less than -1 or
	 *                                  greater than 1 or if all arguments are 0
	 */
	@ApiStatus.Internal
	public boolean getNeighborCacheStatus(int x, int y, int z) {
		if(x < -1 || x > 1 || y < -1 || y > 1 || z < -1 || z > 1) {
			throw new IllegalArgumentException("Offsets must be between -1 and 1");
		}
		if(x == 0 && y == 0 && z == 0) {
			throw new IllegalArgumentException("Offsets cannot all be zero");
		}
		//if not yet initialized
		if(neighborCacheStatus == null) {
			return false;
		}
		return neighborCacheStatus[x + 1][y + 1][z + 1];
	}
	
	/**
	 * Equivalent to calling {@linkplain #getNeighborCacheStatus(x, y, z)} where x,
	 * y, and z are the signum values of the x, y, and, z components of
	 * {@code direction} respectively.
	 */
	@ApiStatus.Internal
	public boolean getNeighborCacheStatus(Vec direction) {
		int x = (int) Math.signum(direction.x());
		int y = (int) Math.signum(direction.y());
		int z = (int) Math.signum(direction.z());
		return getNeighborCacheStatus(x, y, z);
	}
	
	/**
	 * Returns a PNode representing the move from this node to the neighboring node
	 * at the specified offset (in increments) if it has already been evaluated and
	 * is a valid move (a neighbor is considered evaluated if setNeighborCache has
	 * been called for it).
	 * 
	 * @param x the x offset in increments (-1, 0, or 1)
	 * @param y the y offset in increments (-1, 0, or 1)
	 * @param z the z offset in increments (-1, 0, or 1)
	 * @return a PNode representing the move to the neighbor if it has already been
	 *         evaluated and is valid else null
	 * @throws IllegalArgumentException if any of the arguments are less than -1 or
	 *                                  greater than 1 or if all arguments are 0
	 */
	@ApiStatus.Internal
	public PNode getNeighborCache(int x, int y, int z) {
		if(x < -1 || x > 1 || y < -1 || y > 1 || z < -1 || z > 1) {
			throw new IllegalArgumentException("Offsets must be between -1 and 1");
		}
		if(x == 0 && y == 0 && z == 0) {
			throw new IllegalArgumentException("Offsets cannot all be zero");
		}
		//if not yet initialized
		if(neighborCache == null) {
			return null;
		}
		return neighborCache[x + 1][y + 1][z + 1];
	}
	
	/**
	 * Equivalent to calling {@linkplain #getNeighborCache(x, y, z)} where x,
	 * y, and z are the signum values of the x, y, and, z components of
	 * {@code direction} respectively.
	 */
	@ApiStatus.Internal
	public PNode getNeighborCache(Vec direction) {
		int x = (int) Math.signum(direction.x());
		int y = (int) Math.signum(direction.y());
		int z = (int) Math.signum(direction.z());
		return getNeighborCache(x, y, z);
	}
	
	/**
	 * Sets the cached neighbor node for the given offset (in increments) to the
	 * specified value. If the move is invalid, {@code node} should be null.<br>
	 * After calling this method, the neighbor will be considered evaluated.
	 * 
	 * @param x    the x offset in increments (-1, 0, or 1)
	 * @param y    the y offset in increments (-1, 0, or 1)
	 * @param z    the z offset in increments (-1, 0, or 1)
	 * @param node the node representing the move or null if the move is invalid
	 * @throws IllegalArgumentException if any of the arguments are less than -1 or
	 *                                  greater than 1 or if all arguments are 0
	 */
	@ApiStatus.Internal
	public void setNeighborCache(int x, int y, int z, PNode node) {
		if(x < -1 || x > 1 || y < -1 || y > 1 || z < -1 || z > 1) {
			throw new IllegalArgumentException("Offsets must be between -1 and 1");
		}
		if(x == 0 && y == 0 && z == 0) {
			throw new IllegalArgumentException("Offsets cannot all be zero");
		}
		lazyInitNeighborCache();
		neighborCacheStatus[x + 1][y + 1][z + 1] = true;
		neighborCache[x + 1][y + 1][z + 1] = node;
	}
	
	/**
	 * Equivalent to calling {@linkplain #setNeighborCache(x, y, z, node)} where x,
	 * y, and z are the signum values of the x, y, and, z components of
	 * {@code direction} respectively.
	 */
	@ApiStatus.Internal
	public void setNeighborCache(Vec direction, PNode node) {
		int x = (int) Math.signum(direction.x());
		int y = (int) Math.signum(direction.y());
		int z = (int) Math.signum(direction.z());
		setNeighborCache(x, y, z, node);
	}
	
	/**
	 * Lazily initialize the arrays so that we don't waste memory if this node
	 * doesn't use cache.
	 */
	private void lazyInitNeighborCache() {
		if(neighborCache == null) {
			//                        x  y  z
			neighborCache = new PNode[3][3][3];
			//                                 x  y  z
			neighborCacheStatus = new boolean [3][3][3];
		}
	}
	
	/**
	 * This method will call hashCode on all parent nodes of this node
	 */
	@Override
	public int hashCode() {
		return Objects.hash(cost, heuristic, parent, point, type);
	}

	/**
	 * This method will call equals on all parent nodes of this node
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (!(obj instanceof PNode)) {
			return false;
		}
		PNode other = (PNode) obj;
		return Double.compare(cost, other.cost) == 0 && Double.compare(heuristic, other.heuristic) == 0
				&& Objects.equals(parent, other.parent) && Objects.equals(point, other.point) && type == other.type;
	}
	
	/**
	 * Returns a string representation of this node
	 */
	@Override
	public String toString() {
		return "PNode{" +
				"point=(" + point.x() + ", " + point.y() + ", " + point.z() +
				"), cost=" + cost +
				", heuristic=" + heuristic +
				", type=" + type +
				'}';
	}
	
}
