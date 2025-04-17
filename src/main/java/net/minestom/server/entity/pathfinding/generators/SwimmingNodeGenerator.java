package net.minestom.server.entity.pathfinding.generators;

import java.util.ArrayList;
import java.util.Collection;
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
import net.minestom.server.instance.block.Block.Getter.Condition;

public class SwimmingNodeGenerator implements NodeGenerator {
	
	private static final ComponentLogger log = ComponentLogger.logger(SwimmingNodeGenerator.class);
	private static final double SQRT_2 = Math.sqrt(2);
	private static final double SQRT_3 = Math.sqrt(3);
	
	private final Entity entity;
	private final Block fluid;
	
	/**
	 * @param fluid the block type to swim in like {@code Block.WATER} or
	 *              {@code Block.LAVA}.
	 */
	public SwimmingNodeGenerator(Entity entity, Block fluid) {
		this.entity = entity;
		this.fluid = fluid;
	}
	
	/**
	 * Equivalent to {@code SwimmingNodeGenerator(entity, Block.WATER)}.
	 */
	public SwimmingNodeGenerator(Entity entity) {
		this(entity, Block.WATER);
	}

    @Override
    public @NotNull Collection<PNode> getTraversableNodes(@NotNull PNode currentNode, @NotNull PNode start, @NotNull Point goal, @NotNull Set<PNode> visited) {
    	
    	final Getter getter = entity.getInstance();
		final BoundingBox boundingBox = entity.getBoundingBox();
    	//List of traversable neighboring nodes to return
    	Collection<PNode> neighbors = new ArrayList<PNode>();

    	for (int x = -1; x <= 1; x++) {
    		for (int z = -1; z <= 1; z++) {
    			for(int y = -1; y <= 1; y++) {
    				
    				if (x == 0 && z == 0 && y == 0) continue;

    				Point currentPoint = currentNode.getPoint();
    				Point point = currentNode.getBlockPoint().add(0.5 + x, 0.5 + y, 0.5 + z);
    				
    				if(!getter.getBlock(point, Condition.TYPE).compare(fluid)) {
    					continue;
    				}
    				
    				Vec velocity = Vec.fromPoint(point.sub(currentPoint));
    				PhysicsResult result = CollisionUtils.handlePhysics(getter, boundingBox, Pos.fromPoint(currentPoint), velocity, null, true, false, -1);
    				boolean canMove = !result.collisionX() && !result.collisionY() && !result.collisionZ();

    				if (canMove) {
    					double cost = getCost(x, y, z, currentNode.getCost());
    					PNode node = new PNode(point, cost, point.distance(goal), Type.FLY, currentNode);
    					if(!visited.contains(node)) {
    						neighbors.add(node);
    					}
    				}

    			}
    		}
    	}

    	return neighbors;
    	
    }
    
    /**
	 * Gets the cost of the node
	 * 
	 * @param x          the x offset
	 * @param y          the y offset
	 * @param z          the z offset
	 * @param parentCost the cost of the parent node
	 * @return the cost of the node
	 */
    private double getCost(int x, int y, int z, double parentCost) {
    	int absSum = Math.abs(x) + Math.abs(y) + Math.abs(z);
		return switch (absSum) {
		case 1: yield 1;
		case 2: yield SQRT_2;
		case 3: yield SQRT_3;
		default: yield SQRT_3;
		} + parentCost;
    }

    @Override
    public boolean hasGravitySnap() {
        return false;
    }

    @Override
    public @NotNull OptionalDouble gravitySnap(Block.@NotNull Getter getter, Point point, @NotNull BoundingBox boundingBox, double maxFallHeight) {
        //Remove this method when gravitySnap is removed from NodeGenerator interface
    	log.warn("gravitySnap was called on SwimmingNodeGenerator that does not support it");
        return OptionalDouble.of(point.y());
    }
}
