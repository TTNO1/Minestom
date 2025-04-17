package net.minestom.server.entity.pathfinding.generators;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;

import org.jetbrains.annotations.NotNull;

import net.kyori.adventure.text.logger.slf4j.ComponentLogger;
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
 * A ground {@linkplain NodeGenerator} that uses the Jump Point Search
 * optimization. This optimization can only be used in a uniform-cost grid.
 * <p>
 * See <a href=
 * "https://web.archive.org/web/20250101164536/https://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-aaai11.pdf">
 * D. Harabor and A. Grastien, “Online Graph Pruning for Pathfinding On Grid
 * Maps”, AAAI, vol. 25, no. 1, pp. 1114-1119, Aug. 2011. </a> for details and
 * implementation logic.
 */
public class JPSGroundNodeGenerator implements NodeGenerator {
	
	private static final ComponentLogger log = ComponentLogger.logger(JPSGroundNodeGenerator.class);
	
	private final static double SQRT_2 = Math.sqrt(2);
	private final static double BIG_EPSILON = 0.001;

	private final Entity entity;
	private final double maxFallHeight;
	private final double maxLedgeHeight;
	private final double maxJumpHeight;
	private final double maxSearchDistanceSquared;
	private final double maxPathVariance;
	private final boolean canJump;
	private final NodeCostSupplier costSupplier;
	
	/**
	 * @param entity            the entity that is navigating
	 * @param maxFallHeight     the maximum height the entity can fall
	 * @param maxLedgeHeight    the maximum ledge height the entity can walk up
	 * @param maxJumpHeight     the maximum height the entity can jump
	 * @param maxSearchDistance the maximum allowed distance from the start for any
	 *                          node
	 * @param maxPathVariance   the maximum allowed variance for any node in the
	 *                          path where the variance of node {@code n} is given
	 *                          as
	 *                          {@code variance = n.cost() + n.heuristic() - distance(start, goal)}.
	 * @param costSupplier      the {@link NodeCostSupplier} that is used to
	 *                          invalidate locations when
	 *                          {@link NodeCostSupplier#getCostOffset(Point, BoundingBox, Getter)}
	 *                          returns negative (<b>the cost is not offset in this
	 *                          class since JPS requires uniform cost</b>)
	 */
	public JPSGroundNodeGenerator(Entity entity, double maxFallHeight, double maxLedgeHeight, double maxJumpHeight, double maxSearchDistance, double maxPathVariance, NodeCostSupplier costSupplier) {
		
		this.entity = entity;
		this.maxFallHeight = maxFallHeight;
		this.maxLedgeHeight = maxLedgeHeight;
		this.maxJumpHeight = maxJumpHeight;
		this.maxSearchDistanceSquared = maxSearchDistance * maxSearchDistance;
		this.maxPathVariance = maxPathVariance;
		this.canJump = maxJumpHeight > 0 && maxJumpHeight > maxLedgeHeight;
		this.costSupplier = costSupplier;
		
	}
	
	/**
	 * Gets the traversable nodes next to {@code currentNode} that are not in
	 * {@code visited}.
	 * <p>
	 * This method mirrors "Algorithm 1 Identify Successors" in the paper referenced
	 * above.
	 * <p>
	 * The description below is copied from the paper.<br>
	 * <blockquote>We start with the pruned set of neighbors immediately adjacent to
	 * the current node {@code currentNode}. Then, instead of adding each neighbor
	 * {@code neghborNode} to the set of successors for {@code currentNode}, we try
	 * to “jump” to a node that is farther away but which lies in the same relative
	 * direction to {@code currentNode} as {@code neighborNode}. If we find a jump
	 * point, we add it to the set of successors instead of {@code neighborNode}. If
	 * we fail to find a jump point, we add nothing. The process continues until the
	 * set of neighbors is exhausted and we return the set of successors for
	 * {@code currentNode}.</blockquote><cite><a href=
	 * "https://web.archive.org/web/20250101164536/https://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-aaai11.pdf">D.
	 * Harabor and A. Grastien, “Online Graph Pruning for Pathfinding On Grid Maps”,
	 * AAAI, vol. 25, no. 1, pp. 1114-1119, Aug. 2011.</a></cite><br>
	 * <br>
	 *
	 * @param currentNode the current node
	 * @param start       the first node in the path
	 * @param goal        the path goal/destination
	 * @param visited     the nodes that have already been expanded
	 * @return the traversable nodes
	 */
	@Override
	public @NotNull Collection<PNode> getTraversableNodes(@NotNull PNode currentNode, @NotNull PNode start, @NotNull Point goal, @NotNull Set<PNode> visited) {
		
		final Getter getter = entity.getInstance();
		final BoundingBox boundingBox = entity.getBoundingBox();
		final double startGoalDistance = start.getPoint().distance(goal);
		
		//Maximum of 8 successors for each direction
		Collection<PNode> successors = new ArrayList<PNode>(8);
		
		Collection<PNode> neighbors;
		if(currentNode.getParent() == null) {
			neighbors = getAllNeighbors(currentNode, goal, startGoalDistance, getter, boundingBox);
		} else {
			Vec direction = Vec.fromPoint(currentNode.getPoint().sub(currentNode.getParent().getPoint()).withY(0));
			neighbors = getForcedNeighbors(currentNode, direction, start, goal, startGoalDistance, getter, boundingBox);
			neighbors.addAll(getNaturalNeighbors(currentNode, direction, start, goal, startGoalDistance, getter, boundingBox));
		}
		
		for(PNode neighbor : neighbors) {
			Vec direction = Vec.fromPoint(neighbor.getPoint().sub(currentNode.getPoint()).withY(0));
			PNode jumpPoint = jump(currentNode, direction, start, goal, startGoalDistance, getter, boundingBox);
			if(jumpPoint != null) {
				successors.add(jumpPoint);
			}
		}
		
		return successors;
		
	}
	
	/**
	 * Finds any jump point successors to {@code initialNode} in the given
	 * direction.
	 * <p>
	 * This method mirrors "Algorithm 2 Function jump" in the paper referenced
	 * above.
	 * <p>
	 * The description below is copied from the paper.<br>
	 * <blockquote>In rough overview, the algorithm attempts to establish whether
	 * {@code initialNode} has any jump point successors by stepping in the
	 * direction {@code direction} and testing if the node {@code stepNode} at that
	 * location satisfies Definition 2. When this is the case, {@code stepNode} is
	 * designated a jump point and returned. When {@code stepNode} is not a jump
	 * point the algorithm recurses and steps again in direction {@code direction}
	 * but this time {@code stepNode} is the new initial node. The recursion
	 * terminates when an obstacle is encountered and no further steps can be taken.
	 * Note that before each diagonal step the algorithm must first fail to detect
	 * any straight jump points. This check corresponds to the third condition of
	 * Definition 2 and is essential for preserving optimality.</blockquote>
	 * <cite><a href=
	 * "https://web.archive.org/web/20250101164536/https://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-aaai11.pdf">D.
	 * Harabor and A. Grastien, “Online Graph Pruning for Pathfinding On Grid Maps”,
	 * AAAI, vol. 25, no. 1, pp. 1114-1119, Aug. 2011.</a></cite><br>
	 * <br>
	 * 
	 * @param initialNode       the current node to jump from
	 * @param direction         the direction to jump in
	 * @param start             the first node of the path
	 * @param goal              the goal point of the path
	 * @param startGoalDistance the distance from the start to the goal
	 * @param getter            the instance
	 * @param boundingBox       the bounding box of the entity
	 * @return a node representing a jump point if one was found otherwise null
	 */
	private PNode jump(PNode initialNode, Vec direction, PNode start, Point goal, double startGoalDistance, Getter getter, BoundingBox boundingBox) {
		
		PNode stepNode = cachedStep(initialNode, direction, start, goal, startGoalDistance, getter, boundingBox);
		if(stepNode == null) {
			return null;
		}
		
		Point stepPoint = stepNode.getPoint();
		//TODO make sure goal is along increment numbers and snapped to ground
		if(stepPoint.samePoint(goal)) {
			return stepNode;
		}
		
		if(!getForcedNeighbors(stepNode, direction, start, goal, startGoalDistance, getter, boundingBox).isEmpty()) {
			return stepNode;
		}
		
		boolean isDiagonalMove = direction.x() != 0 && direction.z() != 0;
		if(isDiagonalMove) {
			Vec[] directionComponents = new Vec[] {direction.withZ(0), direction.withX(0)};
			for(Vec component : directionComponents) {
				if(jump(stepNode, component, start, goal, startGoalDistance, getter, boundingBox) != null) {
					return stepNode;
				}
			}
		}
		
		return jump(stepNode, direction, start, goal, startGoalDistance, getter, boundingBox);
		
	}
	
	/**
	 * Attempts to "step" (i.e. move) from {@code initialNode} along the direction vector
	 * returning a new node at {@code initialNode + direction} if successful or null if the
	 * movement was blocked or exceeds the maximum distance or variance.
	 * <p>
	 * This method combines lines 1 and 2 from "Algorithm 2 Function jump" in the
	 * paper referenced above.
	 * 
	 * @param initialNode       the node to step from
	 * @param direction         the direction to step in
	 * @param start             the first node
	 * @param goal              the goal point of the path
	 * @param startGoalDistance the distance from the start to the goal
	 * @param getter            the instance
	 * @param boundingBox       the bounding box of the entity
	 * @return a node if the movement is possible otherwise null
	 */
	//Note: for more detailed implementation comments, see GroundNodeGenerator#getTraversableNodes()
	private PNode step(PNode initialNode, Vec direction, PNode start, Point goal, double startGoalDistance, Getter getter, BoundingBox boundingBox) {
		
		Point initialPoint = initialNode.getPoint();
		Point stepPoint = initialPoint.add(direction);
		//TODO make sure mob starts in middle/smooth of block for cost to work
		double cost = ((direction.x() == 0 || direction.z() == 0 ? 1 : SQRT_2) * Math.max(direction.x(), direction.z())) + initialNode.getCost();
		
		PhysicsResult horizontalResult = CollisionUtils.handlePhysics(getter, boundingBox, Pos.fromPoint(initialPoint), direction, null, true, true, maxLedgeHeight);
		boolean canMoveHorizontally = !horizontalResult.collisionX() && !horizontalResult.collisionZ();
		double newY = horizontalResult.newPosition().y();
		
		if (canMoveHorizontally) {
			OptionalDouble optionalY;
			//if equal, horizontal move was normal, else there was a ledge that was walked up
			if(newY == stepPoint.y()) {
				optionalY = gravitySnap(getter, stepPoint, boundingBox, maxFallHeight);
			} else {
				optionalY = OptionalDouble.of(newY);
			}
			if(optionalY.isEmpty()) {
				return null;
			}
			Point nodePoint = Math.abs(optionalY.getAsDouble() - stepPoint.y()) <= BIG_EPSILON ? stepPoint : stepPoint.withY(optionalY.getAsDouble());
			Type type = nodePoint.y() < stepPoint.y() ? Type.FALL : Type.WALK;
			PNode node = new PNode(nodePoint, cost, xzDistance(nodePoint, goal), type, initialNode);
			if(nodePoint.distanceSquared(start.getPoint()) > maxSearchDistanceSquared || node.getCost() + node.getHeuristic() - startGoalDistance > maxPathVariance) {
				return null;
			}
			// Get cost offset after we validate physics so that we are not sending invalid locations into cost supplier
			if(costSupplier.getCostOffset(nodePoint, boundingBox, getter) < 0) {
				return null;
			}
			return node;
		} else if(canJump) {// Try to jump
			OptionalDouble jumpOptionalY = gravitySnap(getter, stepPoint.withY(stepPoint.y() + maxJumpHeight), boundingBox, maxJumpHeight);
			if(jumpOptionalY.isEmpty() || jumpOptionalY.getAsDouble() - stepPoint.y() <= BIG_EPSILON) {
				return null;
			}
			Point jumpPoint = stepPoint.withY(jumpOptionalY.getAsDouble());
			PNode jumpNode = new PNode(jumpPoint, cost, xzDistance(jumpPoint, goal), Type.JUMP, initialNode);
			if(jumpPoint.distanceSquared(start.getPoint()) > maxSearchDistanceSquared || jumpNode.getCost() + jumpNode.getHeuristic() - startGoalDistance > maxPathVariance) {
				return null;
			}
			//Add arbitrary small amount 0.0001 to get over block otherwise horizontal movement will collide
			Vec jumpUpVelocity = new Vec(0, jumpPoint.y() - stepPoint.y() + 0.0001, 0);
			PhysicsResult jumpUpResult = CollisionUtils.handlePhysics(getter, boundingBox, Pos.fromPoint(initialPoint), jumpUpVelocity, null, true, false, -1);
			if(jumpUpResult.collisionY()) {
				return null;
			}
			PhysicsResult jumpOverResult = CollisionUtils.handlePhysics(getter, boundingBox, jumpUpResult.newPosition(), direction, null, true, false, -1);
			if(jumpOverResult.collisionX() || jumpOverResult.collisionZ()) {
				return null;
			}
			// Get cost offset after we validate physics so that we are not sending invalid locations into cost supplier
			if(costSupplier.getCostOffset(jumpPoint, boundingBox, getter) < 0) {
				return null;
			}
			return jumpNode;
		}
		
		return null;
		
	}
	
	//DEBUG variables
	//TODO remove
	private List<PNode> finalNeighborsCalledNodes = new ArrayList<PNode>();
	double duplicateCalls = 0;
	
	/**
	 * Gets the "forced" neighbors of the given node as defined in the paper above.
	 * <p>
	 * Note that moves such as the one depicted in Figure 2(d) in the paper are not
	 * valid here, so this implementation will be slightly different than the
	 * examples given in the paper although still adhering to the pruning rules
	 * given in the paper.
	 * <p>
	 * The logic below is used to identify any forced neighbors. Here, d represents
	 * the vector of the direction of movement from the parent, p, to the node, n,
	 * d⊥ represents the direction perpendicular to d, d_1 represents an x/z
	 * component of d, and d_2 represents the other x/z component of d.
	 * <p>
	 * For straight moves:
	 * <dl>
	 * If n + d⊥ cannot be moved to from p, then
	 * <dd>n + d⊥ is a forced neighbor if it can be moved to from n and</dd>
	 * <dd>n + d⊥ + d is a forced neighbor if it can be moved to from n.</dd>
	 * </dl>
	 * <p>
	 * For diagonal moves:
	 * <dl>
	 * If n - d_1 cannot be moved to from p, then
	 * <dd>n - d_1 + d_2 is a forced neighbor if it can be moved to from n.</dd>
	 * </dl>
	 * <br>
	 * 
	 * @param node              the node to get the forced neighbors of
	 * @param direction         the direction moved from the parent to get to
	 *                          {@code node}
	 * @param start             the first node
	 * @param goal              the goal point of the path
	 * @param startGoalDistance the distance from the start to the goal
	 * @param getter            the instance
	 * @param boundingBox       the bounding box of the entity
	 * @return the forced neighbors
	 */
	private Collection<PNode> getForcedNeighbors(PNode node, Vec direction, PNode start, Point goal, double startGoalDistance, Getter getter, BoundingBox boundingBox) {
		//TODO remove
		if(finalNeighborsCalledNodes.contains(node)) {
			duplicateCalls++;
		} else {
			finalNeighborsCalledNodes.add(node);
		}
		//log.debug(duplicateCalls + " duplicate calls = " + duplicateCalls/finalNeighborsCalledNodes.size() + " of all calls");
		
		// At most 5 pruned neighbors (leave room in case natural neighbors are added to
		// list after returned)
		Collection<PNode> forcedNeighbors = new ArrayList<PNode>(5);
		
		boolean isStraightMove = direction.x() == 0 ^ direction.z() == 0;
		
		if(isStraightMove) {
			Vec[] perpendicularDirections = new Vec[] {new Vec(direction.z(), 0, direction.x()), new Vec(-direction.z(), 0, -direction.x())};
			for(Vec perpendicular : perpendicularDirections) {
				Vec diagonal = direction.add(perpendicular);
				
				//n + d⊥ moved to from p
				PNode perpendicularFromParent = cachedStep(node.getParent(), diagonal, start, goal, startGoalDistance, getter, boundingBox);
				if(perpendicularFromParent != null) {
					continue;
				}
				
				//n + d⊥ moved to from n
				PNode perpendicularFromNode = cachedStep(node, perpendicular, start, goal, startGoalDistance, getter, boundingBox);
				if(perpendicularFromNode != null) {
					forcedNeighbors.add(perpendicularFromNode);
				}
				//n + d⊥ + d moved to from n
				PNode diagonalFromNode = cachedStep(node, diagonal, start, goal, startGoalDistance, getter, boundingBox);
				if(diagonalFromNode != null) {
					forcedNeighbors.add(diagonalFromNode);
				}
			}
		} else {//diagonal move
			Vec[] directionComponents = new Vec[] {direction.withZ(0), direction.withX(0)};
			for(int i = 0; i < directionComponents.length; i++) {
				Vec component = directionComponents[i];
				Vec otherComponent = directionComponents[directionComponents.length - 1 - i];
				
				//n - d_1 moved to from p                             (n - d_1 = p + d_2)
				PNode componentFromParent = cachedStep(node.getParent(), otherComponent, start, goal, startGoalDistance, getter, boundingBox);
				if(componentFromParent != null) {
					continue;
				}
				
				//n - d_1 + d_2 moved to from n
				PNode componentFromNode = cachedStep(node, otherComponent.sub(component), start, goal, startGoalDistance, getter, boundingBox);
				if(componentFromNode != null) {
					forcedNeighbors.add(componentFromNode);
				}
			}
		}
		
		return forcedNeighbors;
		
	}
	
	/**
	 * Gets the "natural" neighbors of the given node as defined in the paper above.
	 * 
	 * @param node              the node to get the natural neighbors of
	 * @param direction         the direction moved from the parent to get to
	 *                          {@code node}
	 * @param start             the first node
	 * @param goal              the goal point of the path
	 * @param startGoalDistance the distance from the start to the goal
	 * @param getter            the instance
	 * @param boundingBox       the bounding box of the entity
	 * @return the natural neighbors
	 */
	private Collection<PNode> getNaturalNeighbors(PNode node, Vec direction, PNode start, Point goal, double startGoalDistance, Getter getter, BoundingBox boundingBox) {
		
		Collection<PNode> naturalNeighbors;
		
		boolean isStraightMove = direction.x() == 0 ^ direction.z() == 0;
		
		if(isStraightMove) {
			// max 1 natural neighbor for straight moves
			naturalNeighbors = new ArrayList<PNode>(1);
			PNode naturalNeighbor = cachedStep(node, direction, start, goal, startGoalDistance, getter, boundingBox);
			if(naturalNeighbor != null) {
				naturalNeighbors.add(naturalNeighbor);
			}
		} else {
			// max 3 natural neighbors for diagonal moves
			naturalNeighbors = new ArrayList<PNode>(3);
			Vec[] directions = new Vec[] {direction.withZ(0), direction, direction.withX(0)};
			for(Vec dir : directions) {
				PNode naturalNeighbor = cachedStep(node, dir, start, goal, startGoalDistance, getter, boundingBox);
				if(naturalNeighbor != null) {
					naturalNeighbors.add(naturalNeighbor);
				}
			}
		}
		
		return naturalNeighbors;
		
	}
	
	/**
	 * Gets all the neighbors of the given node whether they are forced or not. This
	 * method does not return nodes that would constitute invalid moves. This method
	 * is used to get the neighbors of the start node and assumes that the given
	 * node is the start of the path.
	 * 
	 * @param startNode         the node to get all the neighbors of
	 * @param goal              the goal point of the path
	 * @param startGoalDistance the distance from the start to the goal
	 * @param getter            the instance
	 * @param boundingBox       the bounding box of the entity
	 * @return all the neighbors of the given node
	 */
	private Collection<PNode> getAllNeighbors(PNode startNode, Point goal, double startGoalDistance, Getter getter, BoundingBox boundingBox) {
		
		Collection<PNode> neighbors = new ArrayList<PNode>();
		
		//See GroundNodeGenerator for explanation of increment
		final double increment = Math.ceil(Math.max(boundingBox.width(), boundingBox.depth())) % 2 == 0 ? 0.5 : 1;
		
		for (double x = -increment; x <= increment; x += increment) {
			for (double z = -increment; z <= increment; z += increment) {
				if (x == 0 && z == 0) continue;
				Vec direction = new Vec(x, 0, z);
				PNode neighbor = cachedStep(startNode, direction, startNode, goal, startGoalDistance, getter, boundingBox);
				if(neighbor != null) {
					neighbors.add(neighbor);
				}
			}
		}
		
		return neighbors;
		
	}
	
	//DEBUG variables
	//TODO remove
	private double numCallsWithoutCache = 0;
	private double numCallsWithCache = 0;
	private List<PNode> cachedCalls = new ArrayList<PNode>();
	
	/**
	 * Returns the cached node representing the move from {@code initialNode} along
	 * {@code direction} if it exists; otherwise, generates the node using
	 * {@linkplain #step()} and stores it in the cache.<br>
	 * If the move is invalid, this method returns null.
	 * 
	 * @param initialNode       the node to step from
	 * @param direction         the direction to step in
	 * @param start             the first node
	 * @param goal              the goal point of the path
	 * @param startGoalDistance the distance from the start to the goal
	 * @param getter            the instance
	 * @param boundingBox       the bounding box of the entity
	 * @return a node if the movement is possible otherwise null
	 */
	private PNode cachedStep(PNode initialNode, Vec direction, PNode start, Point goal, double startGoalDistance, Getter getter, BoundingBox boundingBox) {
		if(initialNode.getNeighborCacheStatus(direction)) {
			//TODO remove
			if(!cachedCalls.contains(initialNode)) {
				numCallsWithCache++;
				cachedCalls.add(initialNode);
			}
			return initialNode.getNeighborCache(direction);
		}//TODO benchmark memory usage and figure out which nodes are actually benefiting from cache and cache for those only (should be diagonal moves and jump points)
		PNode result = step(initialNode, direction, start, goal, startGoalDistance, getter, boundingBox);
		initialNode.setNeighborCache(direction, result);
		//TODO remove
		numCallsWithoutCache++;
		//log.debug("cached calls/uncached calls (should be 1): " + numCallsWithCache/numCallsWithoutCache);
		return result;
	}
	
	/**
	 * Calculates the 2-dimensional x-z distance between two points.<br>
	 * Used instead of 3-D distance so that the heuristic is admissible.
	 * (Since the cost only considers 2-D movement)
	 */
	private double xzDistance(Point p1, Point p2) {
		return p1.withY(0).distance(p2.withY(0));
	}
	
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
