package net.minestom.server.entity.pathfinding.generators;

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
import net.minestom.server.entity.pathfinding.PNode;
import net.minestom.server.entity.pathfinding.PathGenerator;
import net.minestom.server.instance.block.Block;

/**
 * Generates a list of nodes next to the current node that can be traveled
 * to.<br>
 * Used by {@linkplain PathGenerator} to get the next potential movements for
 * consideration.
 */
public interface NodeGenerator {
	
    /**
	 * Gets the traversable nodes next to {@code currentNode} that are not in
	 * {@code visited}.
	 *
	 * @param currentNode the current node
	 * @param start       the first node in the path
	 * @param goal        the path goal/destination
	 * @param visited     the nodes that have already been expanded
	 * @return the traversable nodes
	 */
    @NotNull Collection<PNode> getTraversableNodes(@NotNull PNode currentNode, @NotNull PNode start,
    		@NotNull Point goal, @NotNull Set<Point> visited);
    //TODO when navigableEntity instance is set, repath
    /**
     * @return snap start and end points to the ground
     * @deprecated this method is pointless
     */
    @Deprecated
    boolean hasGravitySnap();

    /**
	 * Snap point to the ground
	 *
	 * @param getter      the block getter
	 * @param point       the point
	 * @param boundingBox the bounding box
	 * @param maxFallHeight     the maximum fall distance
	 * @return the snapped y coordinate. Empty if the snap point is not found
	 * @deprecated this method should not be on the interface
	 */
    @Deprecated
    @NotNull OptionalDouble gravitySnap(Block.@NotNull Getter getter, Point point,
            @NotNull BoundingBox boundingBox, double maxFallHeight);

    /**
     * Check if we can move directly from one point to another.<br>
     * Default implementation uses {@linkplain CollisionUtils#handlePhysics} to check for collisions.
     * 
     * @param getter
     * @param start
     * @param end
     * @param boundingBox
     * @return true if we can move directly from start to end
     * @deprecated This method should not be part of the interface
     */
    @Deprecated
    default boolean canMoveTowards(Block.@NotNull Getter getter, @NotNull Point start, @NotNull Point end, @NotNull BoundingBox boundingBox) {
        final Point diff = end.sub(start);
        
        PhysicsResult res = CollisionUtils.handlePhysics(getter, boundingBox,
                Pos.fromPoint(start), Vec.fromPoint(diff), null, false, true, 0.6);//TODO change just testing
        
        return !res.collisionZ() && !res.collisionY() && !res.collisionX();
    }

    /**
     * Check if the point is invalid
     *
     * @param getter
     * @param point
     * @param boundingBox
     * @return true if the point is invalid
     * @deprecated this method should not be on the interface and does not work well
     */
    @Deprecated
    default boolean pointInvalid(Block.@NotNull Getter getter, @NotNull Point point, @NotNull BoundingBox boundingBox) {
        var iterator = boundingBox.getBlocks(point);
        while (iterator.hasNext()) {
            var block = iterator.next();
            if (getter.getBlock(block.blockX(), block.blockY(), block.blockZ(), Block.Getter.Condition.TYPE).isSolid()) {
                return true;
            }
        }

        return false;
    }

    /**
     * Heuristic use for the distance from the node to the target
     *
     * @param node
     * @param target
     * @return the heuristic
     * @deprecated this method should not be on the interface
     */
    @Deprecated
    default double heuristic(@NotNull Point node, @NotNull Point target) {
        return node.distance(target);
    }
}
