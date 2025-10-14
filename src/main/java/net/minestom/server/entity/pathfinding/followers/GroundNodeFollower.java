package net.minestom.server.entity.pathfinding.followers;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import net.minestom.server.ServerFlag;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.entity.attribute.Attribute;
import net.minestom.server.utils.position.PositionUtils;

/**
 * A simple NodeFollower for simple ground walking entities.
 * <p>
 * Supports jumping and walking via updating the entity's velocity.
 */
public class GroundNodeFollower implements NodeFollower {
	
    private final Entity entity;
    
    public GroundNodeFollower(@NotNull Entity entity) {
        this.entity = entity;
    }

    /**
	 * Used to move the entity toward {@code target} in the X and Z axis. Gravity
	 * is still applied but the entity will not attempt to jump. Also update the yaw
	 * and pitch of the entity to look at {@code lookAt}.
	 * <p>
	 * Works by updating the entity's velocity if necessary.
	 *
	 * @param target the point to move towards
	 * @param speed  the length of the velocity vector (blocks per tick)
	 * @param lookAt the point to look at
	 */
    //TODO consider changing speed to blocks per second depending on what is passed in
    public void moveTowards(@NotNull Point target, double speed, @NotNull Point lookAt) {
        Pos position = entity.getPosition();
        
        // the purpose of these few lines is to slow down entities when they reach their destination
        /*final double distSquared = target.distanceSquared(position);
        if (speed > distSquared) {
            speed = distSquared;
        }*/
        
        Vec currentVelocity = entity.getVelocity();
        Vec velocity = Vec.fromPoint(target).sub(position).withY(0).normalize().mul(speed * ServerFlag.SERVER_TICKS_PER_SECOND);
        if(velocity.sub(currentVelocity).lengthSquared() >= Vec.EPSILON) {
        	entity.setVelocity(velocity.withY(currentVelocity.y()));
        }
        
        Point lookVec = lookAt.sub(position);
        final float yaw = PositionUtils.getLookYaw(lookVec.x(), lookVec.z());
        //final float pitch = PositionUtils.getLookPitch(lookVec.x(), lookVec.y(), lookVec.z());
        //TODO entity should look at target with its head but not its body
        double lastYaw = entity.getPosition().yaw();
        if(Math.abs(lastYaw - yaw) >= Vec.EPSILON) {
        	entity.setView(yaw, 0);
        }
    }

    /**
	 * Gives the entity a vertical velocity to jump from {@code point} to
	 * {@code target}.<br>
	 * Does nothing if the entity is not on the ground.
	 * 
	 * @param point  the starting point
	 * @param target the target point
	 */
    //TODO why does this method exist?
    @Override
    public void jump(@Nullable Point point, @Nullable Point target) {
        jump(target.y() - point.y());
    }
    
    /**
	 * Gives the entity a vertical velocity to jump the specified height.<br>
	 * Does nothing if the entity is not on the ground.
	 * 
	 * @param height the height to jump
	 */
    public void jump(double height) {
    	if(!entity.isOnGround()) {
    		return;
    	}
    	//Slight boost
    	height = height + 0.25;
    	double gravity = entity.getAerodynamics().gravity();
    	//Kinematics (ignoring drag)
    	double yVelocity = Math.sqrt(2 * gravity * height);
    	
        this.entity.setVelocity(entity.getVelocity().withY(yVelocity));
    }

    /**
	 * @return true if the entity is in the same block as {@code point}
	 */
    //TODO Why does this exist?
    @Override
    public boolean isAtPoint(@NotNull Point point) {
        return entity.getPosition().sameBlock(point);
    }

    /**
	 * Gets the movement speed attribute of this entity if there is one or returns a
	 * default value (0.1).
	 * 
	 * @return the movement speed of this entity or 0.1 if none
	 */
    //TODO why does this exist?
    @Override
    public double movementSpeed() {
        if (entity instanceof LivingEntity living) {
            return living.getAttribute(Attribute.MOVEMENT_SPEED).getValue();
        }
        return 0.1;
    }
}
