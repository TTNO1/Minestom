package net.minestom.server.entity.pathfinding.generators;

import java.util.ArrayList;
import java.util.Collection;
import java.util.OptionalDouble;
import java.util.Set;

import org.jetbrains.annotations.NotNull;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.collision.PhysicsResult;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.pathfinding.PNode;
import net.minestom.server.entity.pathfinding.PNode.Type;
import net.minestom.server.instance.block.Block;
import net.minestom.server.instance.block.Block.Getter;

/**
 * A node generator that generates nodes for entities that walk on the ground.<br>
 * This generator uses the standard A* algorithm.
 */
//TODO use y coord as well to make jumps more expensive
public class GroundNodeGenerator implements NodeGenerator {

	private final static double SQRT_2 = Math.sqrt(2);
	/**
	 * Used when floating-point error can be larger than {@linkplain Vec#EPSILON}.
	 */
	private final static double BIG_EPSILON = 0.001;
	
	private final Entity entity;
	private final double maxFallHeight;
	private final double maxLedgeHeight;
	private final double maxJumpHeight;
	private final boolean canJump;
	private final NodeCostSupplier costSupplier;
	
	/**
	 * @param entity         the entity that is navigating
	 * @param maxFallHeight  the maximum height the entity can fall
	 * @param maxLedgeHeight the maximum ledge height the entity can walk up
	 * @param maxJumpHeight  the maximum height the entity can jump
	 * @param costSupplier   the {@link NodeCostSupplier} that will be used to offset costs
	 */
	public GroundNodeGenerator(Entity entity, double maxFallHeight, double maxLedgeHeight, double maxJumpHeight, NodeCostSupplier costSupplier) {
		
		this.entity = entity;
		this.maxFallHeight = maxFallHeight;
		this.maxLedgeHeight = maxLedgeHeight;
		this.maxJumpHeight = maxJumpHeight;
		this.canJump = maxJumpHeight > 0 && maxJumpHeight > maxLedgeHeight;
		this.costSupplier = costSupplier;
		
	}
	
	@Override
	public @NotNull Collection<PNode> getTraversableNodes(@NotNull PNode currentNode, @NotNull PNode start, @NotNull Point goal, @NotNull Set<Point> visited) {
		
		final Getter getter = entity.getInstance();
		final BoundingBox boundingBox = entity.getBoundingBox();
		// List of traversable neighboring nodes to return
		Collection<PNode> neighbors = new ArrayList<PNode>();
		
		// If rounded-up bounding box size is even, increment by 0.5 instead of 1 so that entities can
		// walk through openings of their own size and not fall through holes of their own size
		final double increment = Math.ceil(Math.max(boundingBox.width(), boundingBox.depth())) % 2 == 0 ? 0.5 : 1;
		
		Point currentPoint = currentNode.getPoint();
		
		//Loop through all potential neighboring node x-z coordinates
		for (double x = -increment; x <= increment; x += increment) {
			for (double z = -increment; z <= increment; z += increment) {
				
				if (x == 0 && z == 0) continue;
				
				//Assume no change in y to potentially avoid doing a square root
				//If change in y, cost will be recomputed later
				double stepCost = ((x == 0 || z == 0 ? 1 : SQRT_2) * increment);
				
				//TODO make sure the first point is on a smooth coord
				Point point = currentPoint.add(x, 0, z);
				
				//We can ignore the point if it has already been expanded since the heuristic is consistent
				// This is an optimization that assumes it is not possible to jump or climb in
				// this direction if it has already been expanded
				if(visited.contains(point)) {
					continue;
				}
				
				//Use CollisionUtils to check if the entity can move to the new point without hitting anything
				Vec horizontalVelocity = Vec.fromPoint(point.sub(currentPoint));
				PhysicsResult horizontalResult = CollisionUtils.handlePhysics(getter, boundingBox, Pos.fromPoint(currentPoint), horizontalVelocity, null, true, true, maxLedgeHeight);
				boolean canMoveHorizontally = !horizontalResult.collisionX() && !horizontalResult.collisionZ();
				//If a ledge was walked up, the y-coordinate will change
				double newY = horizontalResult.newPosition().y();
				
				if(canMoveHorizontally) {
					//if equal, horizontal move was normal, else there was a ledge that was walked up
					if(newY == point.y()) {
						//Make sure that we can stand on the ground (i.e. we didn't walk off a cliff)
						OptionalDouble optionalY = gravitySnap(getter, point, boundingBox, maxFallHeight);
						if(optionalY.isEmpty()) {//Will be empty if there was no ground to stand on
							continue;
						}
						//If the gravity snap y-coordinate is the same as the point y, keep the original y-coordinate
						//We use BIG_EPSILON because collision calculations can have big floating-point error
						newY = Math.abs(optionalY.getAsDouble() - point.y()) <= BIG_EPSILON ? point.y() : optionalY.getAsDouble();
					} else {
						//Recompute cost with added y change
						double yChange = newY - point.y();
						stepCost = Math.sqrt((stepCost * stepCost) + (yChange * yChange));
					}
					Point nodePoint = point.withY(newY);
					Type type = nodePoint.y() < point.y() ? Type.FALL : Type.WALK;
					PNode node = new PNode(nodePoint, currentNode.getCost() + stepCost, xzDistance(nodePoint, goal), type, currentNode);
					//TODO LEFT OFF: contains will not work because PNode equals compares cost
					if(visited.contains(node)) {//We can ignore the node if it has already been expanded since the heuristic is consistent
						continue;
					}
					// Get cost offset after we validate physics so that we are not sending invalid
					// locations into cost supplier
					double costOffset = costSupplier.getCostOffset(nodePoint, boundingBox, getter);
					if(costOffset < 0) {//Indicates an invalid block according to the cost supplier
						continue;
					}
					node.setCost(node.getCost() + costOffset);
					neighbors.add(node);
				} else if(canJump) {// Try to jump
					//Check for ground to stand on at a higher point that we might be able to jump to
					OptionalDouble jumpOptionalY = gravitySnap(getter, point.withY(point.y() + maxJumpHeight), boundingBox, maxJumpHeight);
					// Second part of OR is to make sure the jump point is not at same y-coordinate as the
					// initial point; we use BIG_EPSILON in case of floating point error in collision calculations
					if(jumpOptionalY.isEmpty() || Math.abs(jumpOptionalY.getAsDouble() - point.y()) <= BIG_EPSILON) {
						continue;
					}
					Point jumpPoint = point.withY(jumpOptionalY.getAsDouble());
					//Recompute cost with added y change
					double yChange = jumpPoint.y() - point.y();
					stepCost = Math.sqrt((stepCost * stepCost) + (yChange * yChange));
					PNode jumpNode = new PNode(jumpPoint, currentNode.getCost() + stepCost, xzDistance(jumpPoint, goal), Type.JUMP, currentNode);
					if(visited.contains(jumpNode)) {//We can ignore the node if it has already been expanded since the heuristic is consistent
						continue;
					}
					//Check if the entity can move up without hitting anything
					//Add arbitrary small amount 0.0001 to get over block otherwise horizontal movement will collide
					Vec jumpUpVelocity = new Vec(0, jumpPoint.y() - point.y() + 0.0001, 0);
					PhysicsResult jumpUpResult = CollisionUtils.handlePhysics(getter, boundingBox, Pos.fromPoint(currentPoint), jumpUpVelocity, null, true, false, -1);
					if(jumpUpResult.collisionY()) {
						continue;
					}
					//Check if the entity can move over without hitting anything
					PhysicsResult jumpOverResult = CollisionUtils.handlePhysics(getter, boundingBox, jumpUpResult.newPosition(), horizontalVelocity, null, true, false, -1);
					if(jumpOverResult.collisionX() || jumpOverResult.collisionZ()) {
						continue;
					}
					// Get cost offset after we validate physics so that we are not sending invalid
					// locations into cost supplier
					double costOffset = costSupplier.getCostOffset(jumpPoint, boundingBox, getter);
					if(costOffset < 0) {//Indicates an invalid block according to the cost supplier
						continue;
					}
					jumpNode.setCost(jumpNode.getCost() + costOffset);
					neighbors.add(jumpNode);
				}
				
			}
		}

		return neighbors;
	}

	//TODO why does this exist?
	@Override
	public boolean hasGravitySnap() {
		return true;
	}

	/**
	 * Used to find the highest point that can be stood on by the given bounding box
	 * at the given position. This method checks for a block that can be stood on
	 * from {@code point.y()} to {@code point.y() - maxFallHeight} using
	 * {@linkplain CollisionUtils#handlePhysics} with a downward velocity.<br>
	 * Note: this method will not check for collisions with blocks above
	 * {@code point.y()}.
	 * 
	 * @param getter        the instance
	 * @param point         the location to check
	 * @param boundingBox   the bounding box of the entity
	 * @param maxFallHeight the maximum height that can be fallen
	 * @return the highest point that can be stood on or empty if there is none
	 */
	@Override
	public @NotNull OptionalDouble gravitySnap(Block.@NotNull Getter getter, Point point, @NotNull BoundingBox boundingBox, double maxFallHeight) {
		
		// If velocity is zero, there will be no collision, so we add a very small
		// downward velocity to check if we are standing on a block
		if(maxFallHeight == 0) {
			maxFallHeight = 0.0001;
		}
		// Note: velocity just means how much to move (not speed) since
		// CollisionUtils#handlePhysics is meant to be called every tick for actual movement simulation
		Vec velocity = new Vec(0, -maxFallHeight, 0);
		
		PhysicsResult result = CollisionUtils.handlePhysics(getter, boundingBox, Pos.fromPoint(point), velocity, null, true, false, -1);
		Point collisionPoint = result.collisionPoints()[1];// 1 = y-collision point
		
		if(result.collisionY()) {
			return OptionalDouble.of(collisionPoint.y());
		} else {
			return OptionalDouble.empty();
		}
		
	}

}
