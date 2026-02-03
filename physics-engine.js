/**
 * PHYSICS ENGINE - NEWTON'S LAWS SIMULATOR
 * Complete implementation of classical mechanics
 * All SI units: meters, kilograms, seconds, Newtons
 */

// ============================================
// VECTOR2 CLASS - 2D Vector Mathematics
// ============================================
class Vector2 {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    // Vector addition: v1 + v2
    add(v) {
        return new Vector2(this.x + v.x, this.y + v.y);
    }

    // Vector subtraction: v1 - v2
    subtract(v) {
        return new Vector2(this.x - v.x, this.y - v.y);
    }

    // Scalar multiplication: k * v
    multiply(scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }

    // Dot product: v1 · v2 = |v1||v2|cos(θ)
    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    // Cross product (z-component for 2D): v1 × v2
    cross(v) {
        return this.x * v.y - this.y * v.x;
    }

    // Magnitude: |v| = √(x² + y²)
    magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    // Squared magnitude (faster, avoids sqrt)
    magnitudeSquared() {
        return this.x * this.x + this.y * this.y;
    }

    // Normalize: v̂ = v / |v|
    normalize() {
        const mag = this.magnitude();
        if (mag === 0) return new Vector2(0, 0);
        return new Vector2(this.x / mag, this.y / mag);
    }

    // Distance to another vector
    distanceTo(v) {
        return Math.sqrt((this.x - v.x) ** 2 + (this.y - v.y) ** 2);
    }

    // Angle in radians
    angle() {
        return Math.atan2(this.y, this.x);
    }

    // Rotate by angle (radians)
    rotate(angle) {
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);
        return new Vector2(
            this.x * cos - this.y * sin,
            this.x * sin + this.y * cos
        );
    }

    // Project this onto v: proj_v(this) = (this·v / |v|²) * v
    projectOnto(v) {
        const scalar = this.dot(v) / v.magnitudeSquared();
        return v.multiply(scalar);
    }

    // Clone
    clone() {
        return new Vector2(this.x, this.y);
    }

    // Set values
    set(x, y) {
        this.x = x;
        this.y = y;
    }

    // Static methods
    static zero() {
        return new Vector2(0, 0);
    }

    static fromAngle(angle, magnitude = 1) {
        return new Vector2(Math.cos(angle) * magnitude, Math.sin(angle) * magnitude);
    }
}

// ============================================
// RIGID BODY CLASS - Physical Object
// ============================================
class RigidBody {
    constructor(mass = 1, position = new Vector2()) {
        // Physical properties
        this.mass = mass;
        this.inverseMass = mass > 0 ? 1 / mass : 0;
        
        // Kinematic state (Newton's First Law - state of motion)
        this.position = position.clone();
        this.velocity = new Vector2();
        this.acceleration = new Vector2();
        
        // Rotational state
        this.angle = 0;
        this.angularVelocity = 0;
        this.angularAcceleration = 0;
        this.momentOfInertia = mass * 0.4; // Approximate for sphere
        
        // Force accumulator (Newton's Second Law)
        this.netForce = new Vector2();
        this.forces = []; // Array of {force: Vector2, name: string, point: Vector2}
        this.netTorque = 0;
        
        // Material properties
        this.restitution = 0.8; // Coefficient of restitution (elasticity)
        this.friction = {
            static: 0.5,
            kinetic: 0.3
        };
        
        // Geometry
        this.radius = 0.3; // meters
        this.shape = 'circle'; // 'circle', 'box'
        this.width = 0.6;
        this.height = 0.6;
        
        // State flags
        this.isStatic = false; // Immovable objects
        this.isGrounded = false;
        this.isSliding = false;
        
        // Visual properties
        this.color = '#4fc3f7';
        this.trail = [];
        this.maxTrailLength = 200;
        
        // Constraints
        this.constraints = [];
    }

    // Newton's Second Law: F = m·a → a = F/m
    applyForce(force, name = 'External Force', applicationPoint = null) {
        this.forces.push({
            force: force.clone(),
            name: name,
            point: applicationPoint ? applicationPoint.clone() : this.position.clone()
        });
        this.netForce = this.netForce.add(force);
    }

    // Apply impulse: Δp = F·Δt → change in momentum
    applyImpulse(impulse) {
        if (this.isStatic) return;
        this.velocity = this.velocity.add(impulse.multiply(this.inverseMass));
    }

    // Apply torque for rotation
    applyTorque(torque) {
        this.netTorque += torque;
    }

    // Clear force accumulator (called each frame)
    clearForces() {
        this.netForce = new Vector2();
        this.forces = [];
        this.netTorque = 0;
    }

    // Update position using numerical integration
    update(dt, integrator = 'semiImplicit') {
        if (this.isStatic) return;

        switch (integrator) {
            case 'euler':
                this.updateEuler(dt);
                break;
            case 'semiImplicit':
                this.updateSemiImplicitEuler(dt);
                break;
            case 'rk4':
                this.updateRK4(dt);
                break;
        }

        // Store trail
        this.trail.push(this.position.clone());
        if (this.trail.length > this.maxTrailLength) {
            this.trail.shift();
        }
    }

    // Euler integration (simple but less stable)
    updateEuler(dt) {
        // Linear motion: a = F/m
        this.acceleration = this.netForce.multiply(this.inverseMass);
        
        // Update position: x(t+dt) = x(t) + v(t)·dt
        this.position = this.position.add(this.velocity.multiply(dt));
        
        // Update velocity: v(t+dt) = v(t) + a(t)·dt
        this.velocity = this.velocity.add(this.acceleration.multiply(dt));

        // Rotational motion
        this.angularAcceleration = this.netTorque / this.momentOfInertia;
        this.angle += this.angularVelocity * dt;
        this.angularVelocity += this.angularAcceleration * dt;
    }

    // Semi-Implicit Euler (more stable for physics)
    updateSemiImplicitEuler(dt) {
        // Calculate acceleration from net force
        this.acceleration = this.netForce.multiply(this.inverseMass);
        
        // Update velocity first: v(t+dt) = v(t) + a(t)·dt
        this.velocity = this.velocity.add(this.acceleration.multiply(dt));
        
        // Update position using new velocity: x(t+dt) = x(t) + v(t+dt)·dt
        this.position = this.position.add(this.velocity.multiply(dt));

        // Rotational motion
        this.angularAcceleration = this.netTorque / this.momentOfInertia;
        this.angularVelocity += this.angularAcceleration * dt;
        this.angle += this.angularVelocity * dt;
    }

    // Runge-Kutta 4th order (most accurate)
    updateRK4(dt) {
        const state = {
            pos: this.position.clone(),
            vel: this.velocity.clone()
        };

        // k1 = f(t, y)
        const k1_vel = this.acceleration.clone();
        const k1_pos = state.vel.clone();

        // k2 = f(t + dt/2, y + k1*dt/2)
        const k2_vel = this.acceleration.clone();
        const k2_pos = state.vel.add(k1_vel.multiply(dt / 2));

        // k3 = f(t + dt/2, y + k2*dt/2)
        const k3_vel = this.acceleration.clone();
        const k3_pos = state.vel.add(k2_vel.multiply(dt / 2));

        // k4 = f(t + dt, y + k3*dt)
        const k4_vel = this.acceleration.clone();
        const k4_pos = state.vel.add(k3_vel.multiply(dt));

        // y(t+dt) = y(t) + dt/6 * (k1 + 2k2 + 2k3 + k4)
        const velIncrement = k1_vel.add(k2_vel.multiply(2)).add(k3_vel.multiply(2)).add(k4_vel).multiply(dt / 6);
        const posIncrement = k1_pos.add(k2_pos.multiply(2)).add(k3_pos.multiply(2)).add(k4_pos).multiply(dt / 6);

        this.velocity = state.vel.add(velIncrement);
        this.position = state.pos.add(posIncrement);

        // Rotational (simplified)
        this.angularAcceleration = this.netTorque / this.momentOfInertia;
        this.angularVelocity += this.angularAcceleration * dt;
        this.angle += this.angularVelocity * dt;
    }

    // Kinetic Energy: KE = ½mv²
    kineticEnergy() {
        return 0.5 * this.mass * this.velocity.magnitudeSquared();
    }

    // Rotational Kinetic Energy: KE_rot = ½Iω²
    rotationalKineticEnergy() {
        return 0.5 * this.momentOfInertia * this.angularVelocity ** 2;
    }

    // Momentum: p = mv
    momentum() {
        return this.velocity.multiply(this.mass);
    }
}

// ============================================
// PHYSICS WORLD - Manages all physics
// ============================================
class PhysicsWorld {
    constructor() {
        this.bodies = [];
        this.constraints = [];
        this.gravity = new Vector2(0, 9.8); // Standard Earth gravity
        this.groundLevel = 6.0; // meters from top
        this.airDensity = 1.225; // kg/m³
        this.dragCoefficients = {
            linear: 0.0,
            quadratic: 0.0
        };
        
        // Non-inertial frame
        this.frameAcceleration = new Vector2();
        
        // Time
        this.time = 0;
        this.fixedTimeStep = 0.016; // 60 FPS
        
        // Energy tracking
        this.energyHistory = [];
        this.maxHistoryLength = 300;
    }

    addBody(body) {
        this.bodies.push(body);
        return body;
    }

    removeBody(body) {
        const index = this.bodies.indexOf(body);
        if (index > -1) {
            this.bodies.splice(index, 1);
        }
    }

    addConstraint(constraint) {
        this.constraints.push(constraint);
        return constraint;
    }

    // Main physics step
    step(dt, integrator = 'semiImplicit') {
        // Clear forces from previous frame
        this.bodies.forEach(body => body.clearForces());

        // Apply global forces
        this.applyGravity();
        this.applyDrag();
        this.applyPseudoForces();

        // Solve constraints
        this.solveConstraints();

        // Apply ground collision and friction
        this.handleGroundCollision();
        this.applyFriction();

        // Integrate motion (Newton's Laws in action)
        this.bodies.forEach(body => body.update(dt, integrator));

        // Handle inter-body collisions
        this.handleCollisions();

        // Track energy
        this.trackEnergy();

        this.time += dt;
    }

    // Newton's Law of Universal Gravitation (simplified to constant g)
    // F = mg (weight)
    applyGravity() {
        this.bodies.forEach(body => {
            if (!body.isStatic) {
                const weight = this.gravity.multiply(body.mass);
                body.applyForce(weight, 'Gravity (Weight)');
            }
        });
    }

    // Drag force: F_drag = -b·v - c·v²·v̂
    // Linear drag (Stokes): F = -b·v
    // Quadratic drag: F = -½·ρ·C_d·A·v²·v̂
    applyDrag() {
        this.bodies.forEach(body => {
            if (body.isStatic || body.velocity.magnitude() < 0.001) return;

            const vel = body.velocity;
            const speed = vel.magnitude();
            const velDir = vel.normalize();

            // Linear drag
            if (this.dragCoefficients.linear > 0) {
                const linearDrag = vel.multiply(-this.dragCoefficients.linear);
                body.applyForce(linearDrag, 'Linear Drag');
            }

            // Quadratic drag (F = -½ρC_dAv²v̂)
            if (this.dragCoefficients.quadratic > 0) {
                const area = Math.PI * body.radius ** 2; // Cross-sectional area
                const dragMagnitude = 0.5 * this.airDensity * this.dragCoefficients.quadratic * area * speed * speed;
                const quadDrag = velDir.multiply(-dragMagnitude);
                body.applyForce(quadDrag, 'Quadratic Drag');
            }
        });
    }

    // Pseudo forces in non-inertial reference frames
    // F_pseudo = -m·a_frame
    applyPseudoForces() {
        if (this.frameAcceleration.magnitude() < 0.001) return;

        this.bodies.forEach(body => {
            if (!body.isStatic) {
                const pseudoForce = this.frameAcceleration.multiply(-body.mass);
                body.applyForce(pseudoForce, 'Pseudo Force (Non-inertial)');
            }
        });
    }

    // Ground collision and normal force
    // Newton's Third Law: Ground pushes up with normal force
    handleGroundCollision() {
        this.bodies.forEach(body => {
            if (body.isStatic) return;

            const groundY = this.groundLevel;
            
            if (body.position.y + body.radius >= groundY) {
                // Object touching ground
                body.position.y = groundY - body.radius;
                body.isGrounded = true;

                // Normal force opposes gravity + any downward forces
                // N = mg·cos(θ) for inclined planes
                const normalMagnitude = body.mass * this.gravity.y;
                const normal = new Vector2(0, -normalMagnitude);
                body.applyForce(normal, 'Normal Force');

                // Prevent sinking (velocity constraint)
                if (body.velocity.y > 0) {
                    body.velocity.y *= -body.restitution; // Bounce
                    if (Math.abs(body.velocity.y) < 0.1) {
                        body.velocity.y = 0; // Stop bouncing
                    }
                }
            } else {
                body.isGrounded = false;
            }
        });
    }

    // Friction force: F_friction ≤ μ·N
    // Static friction: |F| ≤ μ_s·N (prevents motion)
    // Kinetic friction: F = μ_k·N (opposes motion)
    applyFriction() {
        this.bodies.forEach(body => {
            if (!body.isGrounded || body.isStatic) return;

            const normalForce = body.mass * this.gravity.y;
            const horizontalVel = new Vector2(body.velocity.x, 0);
            const speed = horizontalVel.magnitude();

            // Find net horizontal force (excluding friction)
            let netHorizontalForce = 0;
            body.forces.forEach(f => {
                if (f.name !== 'Static Friction' && f.name !== 'Kinetic Friction') {
                    netHorizontalForce += f.force.x;
                }
            });

            if (speed < 0.01) {
                // Object at rest - apply static friction
                const maxStaticFriction = body.friction.static * normalForce;
                
                if (Math.abs(netHorizontalForce) <= maxStaticFriction) {
                    // Static friction prevents motion
                    const staticFriction = new Vector2(-netHorizontalForce, 0);
                    body.applyForce(staticFriction, 'Static Friction');
                    body.velocity.x = 0;
                    body.isSliding = false;
                } else {
                    // Force exceeds static friction - start sliding
                    const kineticFriction = body.friction.kinetic * normalForce;
                    const frictionDir = netHorizontalForce > 0 ? -1 : 1;
                    body.applyForce(new Vector2(frictionDir * kineticFriction, 0), 'Kinetic Friction');
                    body.isSliding = true;
                }
            } else {
                // Object moving - apply kinetic friction (opposes velocity)
                const kineticFriction = body.friction.kinetic * normalForce;
                const frictionForce = horizontalVel.normalize().multiply(-kineticFriction);
                body.applyForce(frictionForce, 'Kinetic Friction');
                body.isSliding = true;
            }
        });
    }

    // Constraint solver (ropes, rods, springs)
    solveConstraints() {
        this.constraints.forEach(constraint => {
            constraint.solve();
        });
    }

    // Collision detection and resolution
    handleCollisions() {
        for (let i = 0; i < this.bodies.length; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const bodyA = this.bodies[i];
                const bodyB = this.bodies[j];

                if (bodyA.isStatic && bodyB.isStatic) continue;

                const collision = this.detectCollision(bodyA, bodyB);
                if (collision) {
                    this.resolveCollision(bodyA, bodyB, collision);
                }
            }
        }
    }

    // Circle-circle collision detection
    detectCollision(bodyA, bodyB) {
        const delta = bodyB.position.subtract(bodyA.position);
        const distance = delta.magnitude();
        const minDist = bodyA.radius + bodyB.radius;

        if (distance < minDist) {
            return {
                normal: delta.normalize(),
                penetration: minDist - distance,
                point: bodyA.position.add(delta.multiply(0.5))
            };
        }
        return null;
    }

    // Collision resolution using impulses
    // Conservation of momentum: m1·v1 + m2·v2 = m1·v1' + m2·v2'
    // Coefficient of restitution: e = -(v1' - v2')/(v1 - v2)
    resolveCollision(bodyA, bodyB, collision) {
        const { normal, penetration } = collision;

        // Separate bodies
        const correction = normal.multiply(penetration * 0.5);
        if (!bodyA.isStatic) bodyA.position = bodyA.position.subtract(correction);
        if (!bodyB.isStatic) bodyB.position = bodyB.position.add(correction);

        // Relative velocity
        const relativeVel = bodyB.velocity.subtract(bodyA.velocity);
        const velAlongNormal = relativeVel.dot(normal);

        // Don't resolve if velocities separating
        if (velAlongNormal > 0) return;

        // Calculate restitution
        const e = Math.min(bodyA.restitution, bodyB.restitution);

        // Impulse magnitude: J = -(1+e)·v_rel·n / (1/m1 + 1/m2)
        const impulseMagnitude = -(1 + e) * velAlongNormal;
        const impulseScalar = impulseMagnitude / (bodyA.inverseMass + bodyB.inverseMass);

        const impulse = normal.multiply(impulseScalar);

        // Apply impulses (Newton's Third Law: equal and opposite)
        if (!bodyA.isStatic) bodyA.applyImpulse(impulse.multiply(-1));
        if (!bodyB.isStatic) bodyB.applyImpulse(impulse);
    }

    // Energy tracking for visualization
    trackEnergy() {
        let totalKE = 0;
        let totalPE = 0;

        this.bodies.forEach(body => {
            totalKE += body.kineticEnergy();
            // Gravitational PE: PE = m·g·h
            totalPE += body.mass * this.gravity.y * (this.groundLevel - body.position.y);
        });

        this.energyHistory.push({
            time: this.time,
            kinetic: totalKE,
            potential: totalPE,
            total: totalKE + totalPE
        });

        if (this.energyHistory.length > this.maxHistoryLength) {
            this.energyHistory.shift();
        }
    }

    reset() {
        this.bodies = [];
        this.constraints = [];
        this.time = 0;
        this.energyHistory = [];
        this.frameAcceleration = new Vector2();
    }
}

// ============================================
// CONSTRAINT CLASSES
// ============================================

// Spring constraint: Hooke's Law F = -k·Δx
class SpringConstraint {
    constructor(bodyA, bodyB, restLength, stiffness, damping = 0.1) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.restLength = restLength;
        this.stiffness = stiffness; // k (N/m)
        this.damping = damping; // Damping coefficient
    }

    solve() {
        const delta = this.bodyB.position.subtract(this.bodyA.position);
        const currentLength = delta.magnitude();
        const extension = currentLength - this.restLength;

        if (currentLength < 0.001) return;

        const direction = delta.normalize();

        // Spring force: F = -k·Δx
        const springForceMagnitude = this.stiffness * extension;

        // Damping force (proportional to velocity difference)
        const relativeVel = this.bodyB.velocity.subtract(this.bodyA.velocity);
        const dampingForceMagnitude = this.damping * relativeVel.dot(direction);

        const totalForceMagnitude = springForceMagnitude + dampingForceMagnitude;
        const force = direction.multiply(totalForceMagnitude);

        // Newton's Third Law: Apply equal and opposite forces
        if (!this.bodyA.isStatic) {
            this.bodyA.applyForce(force, 'Spring Force');
        }
        if (!this.bodyB.isStatic) {
            this.bodyB.applyForce(force.multiply(-1), 'Spring Force');
        }
    }
}

// Rope/String constraint (no compression, only tension)
class RopeConstraint {
    constructor(bodyA, bodyB, maxLength) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.maxLength = maxLength;
        this.tension = 0;
    }

    solve() {
        const delta = this.bodyB.position.subtract(this.bodyA.position);
        const currentLength = delta.magnitude();

        // Rope only applies force when stretched
        if (currentLength > this.maxLength) {
            const direction = delta.normalize();
            const extension = currentLength - this.maxLength;

            // Large spring constant to simulate inextensible rope
            const tensionMagnitude = extension * 10000;
            this.tension = tensionMagnitude;

            const force = direction.multiply(tensionMagnitude);

            if (!this.bodyA.isStatic) {
                this.bodyA.applyForce(force, 'Rope Tension');
            }
            if (!this.bodyB.isStatic) {
                this.bodyB.applyForce(force.multiply(-1), 'Rope Tension');
            }
        } else {
            this.tension = 0;
        }
    }
}

// Rod constraint (fixed length, can push and pull)
class RodConstraint {
    constructor(bodyA, bodyB, length) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.length = length;
    }

    solve() {
        const delta = this.bodyB.position.subtract(this.bodyA.position);
        const currentLength = delta.magnitude();
        const difference = currentLength - this.length;

        if (Math.abs(difference) < 0.001) return;

        const direction = delta.normalize();
        const correction = direction.multiply(difference * 0.5);

        // Position correction (constraint stabilization)
        if (!this.bodyA.isStatic) {
            this.bodyA.position = this.bodyA.position.add(correction);
        }
        if (!this.bodyB.isStatic) {
            this.bodyB.position = this.bodyB.position.subtract(correction);
        }
    }
}

// Export for use in other files
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { Vector2, RigidBody, PhysicsWorld, SpringConstraint, RopeConstraint, RodConstraint };
}
