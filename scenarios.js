/**
 * SCENARIOS - Complete Newton's Laws Topic Coverage
 * Each scenario demonstrates specific physics concepts
 */

const SCENARIOS = {
    // ============================================
    // (A) BASIC MOTION
    // ============================================
    '1d_motion': {
        name: '1D Uniform Motion',
        description: 'Newton\'s First Law: Object moves at constant velocity in absence of net force',
        setup: (world) => {
            const body = new RigidBody(2.0, new Vector2(2, 3));
            body.velocity = new Vector2(2, 0);
            body.radius = 0.3;
            body.color = '#4fc3f7';
            world.addBody(body);
            world.gravity = new Vector2(0, 0); // No gravity
        },
        parameters: ['mass', 'velocity'],
        equations: ['v = constant', 'a = 0', 'F_net = 0']
    },

    '1d_acceleration': {
        name: '1D Uniform Acceleration',
        description: 'Newton\'s Second Law: F = ma produces constant acceleration',
        setup: (world) => {
            const body = new RigidBody(1.0, new Vector2(2, 3));
            body.radius = 0.3;
            body.color = '#81c784';
            world.addBody(body);
            world.gravity = new Vector2(0, 0);
        },
        parameters: ['mass', 'force'],
        equations: ['F = ma', 'a = F/m', 'v = u + at', 'x = ut + ½at²']
    },

    '2d_motion': {
        name: '2D Motion',
        description: 'Two-dimensional motion with independent x and y components',
        setup: (world) => {
            const body = new RigidBody(1.5, new Vector2(2, 2));
            body.velocity = new Vector2(3, 1);
            body.radius = 0.3;
            body.color = '#ff9800';
            world.addBody(body);
        },
        parameters: ['mass', 'velocity', 'gravity'],
        equations: ['v_x = constant', 'v_y = u_y + gt', 'x = v_x·t', 'y = u_y·t + ½gt²']
    },

    'relative_motion': {
        name: 'Relative Motion',
        description: 'Motion observed from different reference frames',
        setup: (world) => {
            // Moving observer (frame)
            const observer = new RigidBody(1.0, new Vector2(1, 3));
            observer.velocity = new Vector2(1.5, 0);
            observer.color = '#f44336';
            observer.radius = 0.25;
            world.addBody(observer);

            // Object being observed
            const object = new RigidBody(1.0, new Vector2(4, 3));
            object.velocity = new Vector2(-1, 0);
            object.color = '#4fc3f7';
            object.radius = 0.25;
            world.addBody(object);
            
            world.gravity = new Vector2(0, 0);
        },
        parameters: [],
        equations: ['v_rel = v_object - v_observer']
    },

    // ============================================
    // (B) FORCES
    // ============================================
    'applied_force': {
        name: 'Applied Force',
        description: 'External force applied to object on frictionless surface',
        setup: (world) => {
            const body = new RigidBody(2.0, new Vector2(3, 5.7));
            body.radius = 0.3;
            body.color = '#4fc3f7';
            body.friction.static = 0;
            body.friction.kinetic = 0;
            world.addBody(body);
        },
        parameters: ['mass', 'force', 'forceAngle'],
        equations: ['F_net = F_applied', 'a = F/m']
    },

    'weight_and_normal': {
        name: 'Weight & Normal Force',
        description: 'Newton\'s Third Law: Weight and normal force are action-reaction pair',
        setup: (world) => {
            const body = new RigidBody(5.0, new Vector2(4, 5.7));
            body.radius = 0.3;
            body.color = '#81c784';
            body.velocity = new Vector2(0, 0);
            world.addBody(body);
        },
        parameters: ['mass', 'gravity'],
        equations: ['W = mg', 'N = W (equilibrium)', 'F_net = 0']
    },

    'spring_force': {
        name: 'Spring Force (Hooke\'s Law)',
        description: 'Restoring force proportional to displacement: F = -kx',
        setup: (world) => {
            // Fixed anchor
            const anchor = new RigidBody(1, new Vector2(3, 2));
            anchor.isStatic = true;
            anchor.radius = 0.2;
            anchor.color = '#666';
            world.addBody(anchor);

            // Mass on spring
            const mass = new RigidBody(1.0, new Vector2(3, 4));
            mass.radius = 0.3;
            mass.color = '#4fc3f7';
            world.addBody(mass);

            // Spring constraint
            const spring = new SpringConstraint(anchor, mass, 2.0, 20, 0.5);
            world.addConstraint(spring);
        },
        parameters: ['mass', 'springK', 'restLength'],
        equations: ['F = -kx', 'F = -k(l - l₀)', 'T = 2π√(m/k)']
    },

    'tension': {
        name: 'Tension in String',
        description: 'Two masses connected by rope demonstrating tension',
        setup: (world) => {
            const mass1 = new RigidBody(2.0, new Vector2(2, 3));
            mass1.radius = 0.3;
            mass1.color = '#4fc3f7';
            world.addBody(mass1);

            const mass2 = new RigidBody(1.5, new Vector2(4, 3));
            mass2.radius = 0.3;
            mass2.color = '#81c784';
            world.addBody(mass2);

            const rope = new RopeConstraint(mass1, mass2, 2.0);
            world.addConstraint(rope);
            
            world.gravity = new Vector2(0, 0);
        },
        parameters: ['mass', 'force'],
        equations: ['T = F_applied (massless rope)', 'T₁ = T₂ (same rope)']
    },

    // ============================================
    // (C) FRICTION
    // ============================================
    'static_friction': {
        name: 'Static Friction',
        description: 'Force opposing the start of motion: f_s ≤ μ_s·N',
        setup: (world) => {
            const body = new RigidBody(3.0, new Vector2(4, 5.7));
            body.radius = 0.3;
            body.color = '#ff9800';
            body.friction.static = 0.6;
            body.friction.kinetic = 0.4;
            world.addBody(body);
        },
        parameters: ['mass', 'force', 'mus', 'muk'],
        equations: ['f_s ≤ μ_s·N', 'f_s = F (when static)', 'N = mg']
    },

    'kinetic_friction': {
        name: 'Kinetic Friction',
        description: 'Force opposing motion: f_k = μ_k·N',
        setup: (world) => {
            const body = new RigidBody(2.5, new Vector2(2, 5.7));
            body.velocity = new Vector2(3, 0);
            body.radius = 0.3;
            body.color = '#f44336';
            body.friction.static = 0.5;
            body.friction.kinetic = 0.3;
            world.addBody(body);
        },
        parameters: ['mass', 'muk', 'velocity'],
        equations: ['f_k = μ_k·N', 'a = -f_k/m', 'v² = u² - 2as']
    },

    'friction_comparison': {
        name: 'Static vs Kinetic Friction',
        description: 'Comparing μ_s > μ_k: harder to start than to keep moving',
        setup: (world) => {
            // Static block
            const static = new RigidBody(2.0, new Vector2(2, 5.7));
            static.radius = 0.3;
            static.color = '#4fc3f7';
            static.friction.static = 0.7;
            static.friction.kinetic = 0.4;
            world.addBody(static);

            // Moving block
            const moving = new RigidBody(2.0, new Vector2(5, 5.7));
            moving.velocity = new Vector2(2, 0);
            moving.radius = 0.3;
            moving.color = '#f44336';
            moving.friction.static = 0.7;
            moving.friction.kinetic = 0.4;
            world.addBody(moving);
        },
        parameters: ['mus', 'muk'],
        equations: ['μ_s > μ_k', 'f_s,max > f_k']
    },

    // ============================================
    // (D) INCLINED PLANES
    // ============================================
    'inclined_plane': {
        name: 'Block on Incline',
        description: 'Force decomposition on inclined plane',
        setup: (world) => {
            const body = new RigidBody(2.0, new Vector2(3, 4));
            body.radius = 0.3;
            body.color = '#4fc3f7';
            world.addBody(body);
        },
        parameters: ['mass', 'incline', 'mus', 'muk'],
        equations: [
            'F_parallel = mg·sin(θ)',
            'F_perpendicular = mg·cos(θ)',
            'N = mg·cos(θ)',
            'f = μ·N',
            'a = g(sin(θ) - μ·cos(θ))'
        ]
    },

    'incline_with_force': {
        name: 'Incline with Applied Force',
        description: 'Block on incline with external force',
        setup: (world) => {
            const body = new RigidBody(3.0, new Vector2(3, 4.5));
            body.radius = 0.35;
            body.color = '#81c784';
            body.friction.static = 0.4;
            body.friction.kinetic = 0.3;
            world.addBody(body);
        },
        parameters: ['mass', 'force', 'forceAngle', 'incline', 'muk'],
        equations: ['F_net = F + mg·sin(θ) - f']
    },

    // ============================================
    // (E) CONNECTED SYSTEMS
    // ============================================
    'two_blocks_connected': {
        name: 'Two Blocks Connected',
        description: 'Two masses connected by string with force applied',
        setup: (world) => {
            const m1 = new RigidBody(2.0, new Vector2(2, 5.7));
            m1.radius = 0.3;
            m1.color = '#4fc3f7';
            m1.friction.kinetic = 0.2;
            world.addBody(m1);

            const m2 = new RigidBody(1.5, new Vector2(4.5, 5.7));
            m2.radius = 0.3;
            m2.color = '#81c784';
            m2.friction.kinetic = 0.2;
            world.addBody(m2);

            const rope = new RopeConstraint(m1, m2, 2.5);
            world.addConstraint(rope);
        },
        parameters: ['mass', 'force'],
        equations: [
            'F - T = m₁·a',
            'T = m₂·a',
            'a = F/(m₁ + m₂)',
            'T = m₂F/(m₁ + m₂)'
        ]
    },

    'atwood_machine': {
        name: 'Atwood Machine',
        description: 'Classic pulley system: (m₁ - m₂)g = (m₁ + m₂)a',
        setup: (world) => {
            // Pulley (fixed position)
            const pulley = new RigidBody(0.5, new Vector2(4, 1.5));
            pulley.isStatic = true;
            pulley.radius = 0.4;
            pulley.color = '#666';
            world.addBody(pulley);

            // Mass 1 (heavier)
            const m1 = new RigidBody(3.0, new Vector2(2.5, 4));
            m1.radius = 0.35;
            m1.color = '#4fc3f7';
            world.addBody(m1);

            // Mass 2 (lighter)
            const m2 = new RigidBody(2.0, new Vector2(5.5, 4));
            m2.radius = 0.3;
            m2.color = '#81c784';
            world.addBody(m2);

            // Rope over pulley
            const rope1 = new RopeConstraint(pulley, m1, 2.5);
            const rope2 = new RopeConstraint(pulley, m2, 2.5);
            world.addConstraint(rope1);
            world.addConstraint(rope2);
        },
        parameters: ['mass'],
        equations: [
            'a = (m₁ - m₂)g/(m₁ + m₂)',
            'T = 2m₁m₂g/(m₁ + m₂)'
        ]
    },

    // ============================================
    // (F) CIRCULAR MOTION
    // ============================================
    'uniform_circular': {
        name: 'Uniform Circular Motion',
        description: 'Centripetal force: F_c = mv²/r pointing to center',
        setup: (world) => {
            const center = new Vector2(4, 3.5);
            const radius = 2.0;
            
            const body = new RigidBody(1.0, new Vector2(center.x + radius, center.y));
            body.radius = 0.25;
            body.color = '#4fc3f7';
            
            // Set velocity tangent to circle
            const omega = 2.0; // angular velocity
            body.velocity = new Vector2(0, omega * radius);
            world.addBody(body);

            // String to center
            const anchor = new RigidBody(0.1, center);
            anchor.isStatic = true;
            anchor.radius = 0.15;
            anchor.color = '#666';
            world.addBody(anchor);

            const rope = new RopeConstraint(anchor, body, radius);
            world.addConstraint(rope);
            
            world.gravity = new Vector2(0, 0);
        },
        parameters: ['mass'],
        equations: [
            'F_c = mv²/r = mω²r',
            'a_c = v²/r = ω²r',
            'T = F_c (tension provides centripetal force)'
        ]
    },

    'vertical_circle': {
        name: 'Vertical Circle',
        description: 'Tension varies: T_bottom > T_top, minimum speed at top',
        setup: (world) => {
            const center = new Vector2(4, 3.5);
            const radius = 1.5;
            
            const body = new RigidBody(0.5, new Vector2(center.x, center.y - radius));
            body.radius = 0.2;
            body.color = '#ff9800';
            
            // Initial velocity for circular motion
            const v = Math.sqrt(5 * world.gravity.y * radius); // > √(gr) for complete loop
            body.velocity = new Vector2(v, 0);
            world.addBody(body);

            const anchor = new RigidBody(0.1, center);
            anchor.isStatic = true;
            anchor.radius = 0.15;
            anchor.color = '#666';
            world.addBody(anchor);

            const rope = new RopeConstraint(anchor, body, radius);
            world.addConstraint(rope);
        },
        parameters: ['mass', 'gravity'],
        equations: [
            'At bottom: T - mg = mv²/r',
            'At top: T + mg = mv²/r',
            'v_min,top = √(gr)',
            'T_bottom = T_top + 6mg (if v_min at top)'
        ]
    },

    // ============================================
    // (G) PROJECTILE MOTION
    // ============================================
    'projectile': {
        name: 'Projectile Motion',
        description: 'Parabolic trajectory under gravity',
        setup: (world) => {
            const body = new RigidBody(0.5, new Vector2(1, 5));
            body.velocity = new Vector2(4, -3);
            body.radius = 0.2;
            body.color = '#f44336';
            world.addBody(body);
        },
        parameters: ['velocity', 'gravity'],
        equations: [
            'x = v₀·cos(θ)·t',
            'y = v₀·sin(θ)·t - ½gt²',
            'Range = v₀²·sin(2θ)/g',
            'H_max = v₀²·sin²(θ)/(2g)',
            'T_flight = 2v₀·sin(θ)/g'
        ]
    },

    'projectile_with_drag': {
        name: 'Projectile with Air Resistance',
        description: 'Realistic projectile motion with quadratic drag',
        setup: (world) => {
            const body = new RigidBody(0.5, new Vector2(1, 5));
            body.velocity = new Vector2(5, -2);
            body.radius = 0.2;
            body.color = '#ff9800';
            world.addBody(body);
            
            world.dragCoefficients.quadratic = 0.05;
        },
        parameters: ['velocity', 'dragQuad'],
        equations: [
            'F_drag = -½ρC_dAv²v̂',
            'Terminal velocity: v_t = √(2mg/ρC_dA)'
        ]
    },

    // ============================================
    // (H) ROCKET & VARIABLE MASS
    // ============================================
    'rocket': {
        name: 'Rocket Propulsion',
        description: 'Variable mass system: F_thrust = v_e·dm/dt',
        setup: (world) => {
            const rocket = new RigidBody(10.0, new Vector2(3, 5));
            rocket.radius = 0.4;
            rocket.color = '#f44336';
            rocket.shape = 'rocket';
            world.addBody(rocket);
            
            // Store rocket-specific properties
            rocket.fuelMass = 8.0;
            rocket.dryMass = 2.0;
            rocket.exhaustVelocity = 15.0; // m/s
            rocket.massFlowRate = 0.5; // kg/s
            rocket.isThrusting = false;
        },
        parameters: ['mass'],
        equations: [
            'F_thrust = v_e·(dm/dt)',
            'Tsiolkovsky: Δv = v_e·ln(m₀/m_f)',
            'm(t) = m₀ - (dm/dt)·t'
        ]
    },

    // ============================================
    // (I) NON-INERTIAL FRAMES
    // ============================================
    'accelerating_frame': {
        name: 'Accelerating Reference Frame',
        description: 'Pseudo force in accelerating elevator/frame',
        setup: (world) => {
            const body = new RigidBody(2.0, new Vector2(4, 4));
            body.radius = 0.3;
            body.color = '#9c27b0';
            world.addBody(body);
            
            // Frame accelerating upward
            world.frameAcceleration = new Vector2(0, -2.0);
        },
        parameters: ['mass'],
        equations: [
            'F_pseudo = -m·a_frame',
            'N = m(g + a) (elevator accelerating up)',
            'N = m(g - a) (elevator accelerating down)'
        ]
    },

    'elevator': {
        name: 'Elevator Problem',
        description: 'Weight changes in accelerating elevator',
        setup: (world) => {
            const person = new RigidBody(70.0, new Vector2(4, 5.7));
            person.radius = 0.4;
            person.color = '#4fc3f7';
            world.addBody(person);
            
            // Simulated elevator acceleration
            world.frameAcceleration = new Vector2(0, 0);
        },
        parameters: ['mass'],
        equations: [
            'N = m(g ± a)',
            'Weightlessness: a = -g',
            'Apparent weight = N'
        ]
    },

    // ============================================
    // (J) COLLISIONS
    // ============================================
    'elastic_collision': {
        name: 'Elastic Collision',
        description: 'Both momentum and KE conserved: e = 1',
        setup: (world) => {
            const body1 = new RigidBody(2.0, new Vector2(2, 3));
            body1.velocity = new Vector2(3, 0);
            body1.radius = 0.3;
            body1.color = '#4fc3f7';
            body1.restitution = 1.0;
            world.addBody(body1);

            const body2 = new RigidBody(1.5, new Vector2(5, 3));
            body2.velocity = new Vector2(-1, 0);
            body2.radius = 0.3;
            body2.color = '#81c784';
            body2.restitution = 1.0;
            world.addBody(body2);
            
            world.gravity = new Vector2(0, 0);
        },
        parameters: ['mass', 'velocity'],
        equations: [
            'm₁v₁ + m₂v₂ = m₁v₁\' + m₂v₂\' (momentum)',
            '½m₁v₁² + ½m₂v₂² = ½m₁v₁\'² + ½m₂v₂\'² (energy)',
            'e = 1 (elastic)'
        ]
    },

    'inelastic_collision': {
        name: 'Inelastic Collision',
        description: 'Momentum conserved, KE lost: 0 < e < 1',
        setup: (world) => {
            const body1 = new RigidBody(3.0, new Vector2(2, 3));
            body1.velocity = new Vector2(4, 0);
            body1.radius = 0.35;
            body1.color = '#f44336';
            body1.restitution = 0.4;
            world.addBody(body1);

            const body2 = new RigidBody(2.0, new Vector2(6, 3));
            body2.radius = 0.3;
            body2.color = '#ff9800';
            body2.restitution = 0.4;
            world.addBody(body2);
            
            world.gravity = new Vector2(0, 0);
        },
        parameters: ['mass', 'velocity'],
        equations: [
            'm₁v₁ + m₂v₂ = m₁v₁\' + m₂v₂\'',
            'KE_lost = KE_initial - KE_final',
            'e = (v₂\' - v₁\')/(v₁ - v₂) < 1'
        ]
    },

    'perfectly_inelastic': {
        name: 'Perfectly Inelastic Collision',
        description: 'Objects stick together: e = 0, maximum KE loss',
        setup: (world) => {
            const body1 = new RigidBody(2.5, new Vector2(2, 3));
            body1.velocity = new Vector2(5, 0);
            body1.radius = 0.3;
            body1.color = '#9c27b0';
            body1.restitution = 0.0;
            world.addBody(body1);

            const body2 = new RigidBody(1.5, new Vector2(6, 3));
            body2.radius = 0.25;
            body2.color = '#673ab7';
            body2.restitution = 0.0;
            world.addBody(body2);
            
            world.gravity = new Vector2(0, 0);
        },
        parameters: ['mass', 'velocity'],
        equations: [
            'm₁v₁ + m₂v₂ = (m₁ + m₂)v_final',
            'v_final = (m₁v₁ + m₂v₂)/(m₁ + m₂)',
            'e = 0',
            'Max KE lost'
        ]
    },

    // ============================================
    // (K) WORK, ENERGY & POWER
    // ============================================
    'work_energy': {
        name: 'Work-Energy Theorem',
        description: 'Work done equals change in kinetic energy: W = ΔKE',
        setup: (world) => {
            const body = new RigidBody(2.0, new Vector2(2, 5.7));
            body.radius = 0.3;
            body.color = '#4fc3f7';
            body.friction.kinetic = 0.0;
            world.addBody(body);
        },
        parameters: ['mass', 'force'],
        equations: [
            'W = F·d·cos(θ)',
            'W_net = ΔKE = ½m(v_f² - v_i²)',
            'P = F·v'
        ]
    },

    'energy_conservation': {
        name: 'Energy Conservation',
        description: 'Total mechanical energy conserved in absence of friction',
        setup: (world) => {
            const body = new RigidBody(1.0, new Vector2(2, 2));
            body.velocity = new Vector2(3, 0);
            body.radius = 0.25;
            body.color = '#81c784';
            world.addBody(body);
        },
        parameters: ['mass', 'velocity', 'gravity'],
        equations: [
            'E = KE + PE',
            'KE = ½mv²',
            'PE = mgh',
            'E_initial = E_final (no friction)'
        ]
    },

    'energy_dissipation': {
        name: 'Energy Dissipation by Friction',
        description: 'Friction converts mechanical energy to heat',
        setup: (world) => {
            const body = new RigidBody(3.0, new Vector2(2, 5.7));
            body.velocity = new Vector2(5, 0);
            body.radius = 0.35;
            body.color = '#f44336';
            body.friction.kinetic = 0.4;
            world.addBody(body);
        },
        parameters: ['mass', 'velocity', 'muk'],
        equations: [
            'W_friction = -f_k·d',
            'E_dissipated = μ_k·m·g·d',
            'v_f² = v_i² - 2μ_k·g·d'
        ]
    }
};

// Scenario categories for organized UI
const SCENARIO_CATEGORIES = {
    'Basic Motion': ['1d_motion', '1d_acceleration', '2d_motion', 'relative_motion'],
    'Forces': ['applied_force', 'weight_and_normal', 'spring_force', 'tension'],
    'Friction': ['static_friction', 'kinetic_friction', 'friction_comparison'],
    'Inclined Planes': ['inclined_plane', 'incline_with_force'],
    'Connected Systems': ['two_blocks_connected', 'atwood_machine'],
    'Circular Motion': ['uniform_circular', 'vertical_circle'],
    'Projectiles': ['projectile', 'projectile_with_drag'],
    'Advanced': ['rocket', 'accelerating_frame', 'elevator'],
    'Collisions': ['elastic_collision', 'inelastic_collision', 'perfectly_inelastic'],
    'Energy & Work': ['work_energy', 'energy_conservation', 'energy_dissipation']
};
