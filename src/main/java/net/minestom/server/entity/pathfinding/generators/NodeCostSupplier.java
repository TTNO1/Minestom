package net.minestom.server.entity.pathfinding.generators;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block.Getter;

/**
 * This interface is used to provide custom cost offsets for certain block
 * types. (i.e. villagers prefer walking on path blocks)
 */
public interface NodeCostSupplier {
	
	/**
	 * Gets the cost offset for the given location.<br>
	 * The returned value is added to the default cost to get the final cost.<br>
	 * A negative return value will cause the location to be treated as
	 * unpassable.<br>
	 * To make a particular block more preferable, return a greater
	 * value for all other blocks.<br>
	 * To not change the cost, return zero.
	 * 
	 * @param point       the location of the entity
	 * @param boundingBox the bounding box of the entity
	 * @param getter      the instance
	 * @return the cost offset
	 */
	double getCostOffset(Point point, BoundingBox boundingBox, Getter getter);
	
}
