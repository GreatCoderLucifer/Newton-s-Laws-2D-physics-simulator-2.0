/**
 * EDUCATIONAL MODULE
 * Step-by-step explanations and problem generation for NEET/JEE
 */

class EducationalModule {
    constructor(world) {
        this.world = world;
        this.currentScenario = null;
        this.stepByStepMode = false;
        this.problemMode = false;
        this.currentProblem = null;
    }

    // Generate step-by-step explanation for current scenario
    generateExplanation(scenarioKey) {
        const scenario = SCENARIOS[scenarioKey];
        if (!scenario) return null;

        this.currentScenario = scenarioKey;
        
        const explanations = {
            '1d_motion': this.explain1DMotion(),
            '1d_acceleration': this.explain1DAcceleration(),
            'applied_force': this.explainAppliedForce(),
            'static_friction': this.explainStaticFriction(),
            'kinetic_friction': this.explainKineticFriction(),
            'inclined_plane': this.explainInclinedPlane(),
            'two_blocks_connected': this.explainConnectedBlocks(),
            'atwood_machine': this.explainAtwoodMachine(),
            'uniform_circular': this.explainCircularMotion(),
            'vertical_circle': this.explainVerticalCircle(),
            'projectile': this.explainProjectile(),
            'elastic_collision': this.explainElasticCollision(),
            'work_energy': this.explainWorkEnergy(),
            'spring_force': this.explainSpringForce(),
            'rocket': this.explainRocket()
        };

        return explanations[scenarioKey] || this.generateGenericExplanation(scenario);
    }

    explain1DMotion() {
        const body = this.world.bodies[0];
        if (!body) return null;

        return {
            title: "Newton's First Law (Law of Inertia)",
            steps: [
                {
                    title: "Step 1: Identify the State",
                    content: `The object has mass m = ${body.mass.toFixed(2)} kg and initial velocity v = ${body.velocity.x.toFixed(2)} m/s.`,
                    equation: "v = constant (when F_net = 0)"
                },
                {
                    title: "Step 2: Check Net Force",
                    content: `Net force F_net = ${body.netForce.magnitude().toFixed(2)} N. Since there is no net force, the object maintains constant velocity.`,
                    equation: "F_net = 0 → a = 0"
                },
                {
                    title: "Step 3: Predict Motion",
                    content: "The object will continue moving at the same velocity indefinitely. This demonstrates inertia - the tendency to maintain state of motion.",
                    equation: "x(t) = x₀ + v·t"
                }
            ],
            keyTakeaways: [
                "An object at rest stays at rest, an object in motion stays in motion",
                "Unless acted upon by a net external force",
                "Velocity remains constant when F_net = 0"
            ]
        };
    }

    explain1DAcceleration() {
        const body = this.world.bodies[0];
        if (!body) return null;

        const force = body.netForce.magnitude();
        const accel = force / body.mass;

        return {
            title: "Newton's Second Law (F = ma)",
            steps: [
                {
                    title: "Step 1: Given Information",
                    content: `Mass m = ${body.mass.toFixed(2)} kg, Applied Force F = ${force.toFixed(2)} N`,
                    equation: "F = m·a"
                },
                {
                    title: "Step 2: Calculate Acceleration",
                    content: `Using F = ma, we get: a = F/m = ${force.toFixed(2)} / ${body.mass.toFixed(2)}`,
                    equation: `a = ${accel.toFixed(2)} m/s²`
                },
                {
                    title: "Step 3: Predict Velocity",
                    content: `Starting from rest, velocity increases linearly: v(t) = u + at = 0 + ${accel.toFixed(2)}·t`,
                    equation: "v(t) = a·t"
                },
                {
                    title: "Step 4: Predict Position",
                    content: `Position follows: x(t) = ut + ½at² = ½·${accel.toFixed(2)}·t²`,
                    equation: "x(t) = ½at²"
                }
            ],
            keyTakeaways: [
                "Force produces acceleration proportional to force and inversely proportional to mass",
                "Heavier objects need more force for same acceleration",
                "Acceleration is constant if force is constant"
            ]
        };
    }

    explainAppliedForce() {
        const body = this.world.bodies[0];
        if (!body) return null;

        return {
            title: "Applied Force Analysis",
            steps: [
                {
                    title: "Step 1: Identify All Forces",
                    content: "Forces acting: Applied Force, Weight (mg), Normal Force (N)",
                    equation: "ΣF = F_applied + W + N"
                },
                {
                    title: "Step 2: Resolve Components",
                    content: `If force is at angle θ: F_x = F·cos(θ), F_y = F·sin(θ)`,
                    equation: "F_net,x = F·cos(θ) - f"
                },
                {
                    title: "Step 3: Apply Newton's Second Law",
                    content: `Horizontal: F·cos(θ) = m·a_x, Vertical: N = mg + F·sin(θ)`,
                    equation: "a = F·cos(θ) / m"
                }
            ],
            keyTakeaways: [
                "Break forces into components",
                "Normal force adjusts based on vertical component of applied force",
                "Only horizontal component causes horizontal acceleration"
            ]
        };
    }

    explainStaticFriction() {
        const body = this.world.bodies[0];
        if (!body) return null;

        const normal = body.mass * this.world.gravity.y;
        const maxStatic = body.friction.static * normal;

        return {
            title: "Static Friction Analysis",
            steps: [
                {
                    title: "Step 1: Calculate Normal Force",
                    content: `On horizontal surface: N = mg = ${body.mass.toFixed(2)} × ${this.world.gravity.y.toFixed(2)}`,
                    equation: `N = ${normal.toFixed(2)} N`
                },
                {
                    title: "Step 2: Maximum Static Friction",
                    content: `f_s,max = μ_s·N = ${body.friction.static.toFixed(2)} × ${normal.toFixed(2)}`,
                    equation: `f_s,max = ${maxStatic.toFixed(2)} N`
                },
                {
                    title: "Step 3: Comparison with Applied Force",
                    content: `If F_applied < f_s,max: object remains stationary, f_s = F_applied. If F_applied > f_s,max: object starts moving, f becomes kinetic friction.`,
                    equation: "f_s ≤ μ_s·N"
                }
            ],
            keyTakeaways: [
                "Static friction prevents motion up to maximum value",
                "f_s adjusts to match applied force (up to limit)",
                "Once exceeded, kinetic friction takes over"
            ]
        };
    }

    explainKineticFriction() {
        const body = this.world.bodies[0];
        if (!body) return null;

        const normal = body.mass * this.world.gravity.y;
        const fk = body.friction.kinetic * normal;
        const decel = fk / body.mass;

        return {
            title: "Kinetic Friction Analysis",
            steps: [
                {
                    title: "Step 1: Calculate Friction Force",
                    content: `f_k = μ_k·N = ${body.friction.kinetic.toFixed(2)} × ${normal.toFixed(2)}`,
                    equation: `f_k = ${fk.toFixed(2)} N`
                },
                {
                    title: "Step 2: Calculate Deceleration",
                    content: `a = -f_k/m = -${fk.toFixed(2)} / ${body.mass.toFixed(2)}`,
                    equation: `a = ${decel.toFixed(2)} m/s² (negative = deceleration)`
                },
                {
                    title: "Step 3: Stopping Distance",
                    content: `Using v² = u² + 2as with v = 0: s = u²/(2|a|)`,
                    equation: `s = ${body.velocity.x}² / (2 × ${Math.abs(decel).toFixed(2)})`
                }
            ],
            keyTakeaways: [
                "Kinetic friction is constant for given surfaces",
                "Always opposes direction of motion",
                "μ_k < μ_s typically"
            ]
        };
    }

    explainInclinedPlane() {
        return {
            title: "Inclined Plane Analysis",
            steps: [
                {
                    title: "Step 1: Resolve Weight Components",
                    content: "Weight mg acts vertically down. Resolve into parallel and perpendicular components.",
                    equation: "W_∥ = mg·sin(θ), W_⊥ = mg·cos(θ)"
                },
                {
                    title: "Step 2: Normal Force",
                    content: "Normal force balances perpendicular component of weight.",
                    equation: "N = mg·cos(θ)"
                },
                {
                    title: "Step 3: Friction Force",
                    content: "Friction opposes motion down the plane.",
                    equation: "f = μ·N = μ·mg·cos(θ)"
                },
                {
                    title: "Step 4: Net Force Along Plane",
                    content: "Net force determines acceleration down the plane.",
                    equation: "F_net = mg·sin(θ) - f = m·a"
                },
                {
                    title: "Step 5: Solve for Acceleration",
                    content: "Final acceleration along the incline.",
                    equation: "a = g(sin(θ) - μ·cos(θ))"
                }
            ],
            keyTakeaways: [
                "Always resolve forces parallel and perpendicular to surface",
                "Normal force ≠ mg on incline",
                "Object slides if tan(θ) > μ_s"
            ]
        };
    }

    explainConnectedBlocks() {
        return {
            title: "Connected Blocks System",
            steps: [
                {
                    title: "Step 1: System Approach",
                    content: "Treat both blocks as single system with total mass M = m₁ + m₂",
                    equation: "F_net = F_applied (for system)"
                },
                {
                    title: "Step 2: Common Acceleration",
                    content: "Both blocks have same acceleration since connected.",
                    equation: "a = F / (m₁ + m₂)"
                },
                {
                    title: "Step 3: Find Tension (Isolate Block 2)",
                    content: "Apply F = ma to second block alone.",
                    equation: "T = m₂·a = m₂·F/(m₁ + m₂)"
                },
                {
                    title: "Step 4: Verify with Block 1",
                    content: "For first block: F - T = m₁·a",
                    equation: "T = F - m₁·a (should match Step 3)"
                }
            ],
            keyTakeaways: [
                "Connected objects share same acceleration",
                "Use system approach for acceleration",
                "Isolate individual objects to find internal forces (tension)"
            ]
        };
    }

    explainAtwoodMachine() {
        return {
            title: "Atwood Machine (Pulley System)",
            steps: [
                {
                    title: "Step 1: Free Body Diagrams",
                    content: "Block 1 (heavier): mg₁ downward, T upward. Block 2 (lighter): mg₂ downward, T upward.",
                    equation: "Same tension T in massless rope"
                },
                {
                    title: "Step 2: Apply F = ma to Each Block",
                    content: "Block 1: m₁g - T = m₁a, Block 2: T - m₂g = m₂a",
                    equation: "Assuming m₁ > m₂, a is positive downward for m₁"
                },
                {
                    title: "Step 3: Solve for Acceleration",
                    content: "Add both equations to eliminate T: m₁g - m₂g = m₁a + m₂a",
                    equation: "a = (m₁ - m₂)g / (m₁ + m₂)"
                },
                {
                    title: "Step 4: Solve for Tension",
                    content: "Substitute a back into either equation.",
                    equation: "T = 2m₁m₂g / (m₁ + m₂)"
                }
            ],
            keyTakeaways: [
                "Acceleration depends on mass difference",
                "Tension is NOT average of weights",
                "If m₁ = m₂, then a = 0 (equilibrium)"
            ]
        };
    }

    explainCircularMotion() {
        return {
            title: "Uniform Circular Motion",
            steps: [
                {
                    title: "Step 1: Centripetal Acceleration",
                    content: "Object moving in circle requires acceleration toward center.",
                    equation: "a_c = v²/r = ω²r"
                },
                {
                    title: "Step 2: Centripetal Force",
                    content: "Apply F = ma in radial direction.",
                    equation: "F_c = m·a_c = mv²/r = mω²r"
                },
                {
                    title: "Step 3: Source of Force",
                    content: "Tension in string, normal force, gravity, etc. provides the centripetal force.",
                    equation: "T = F_c (for string constraint)"
                }
            ],
            keyTakeaways: [
                "Centripetal force always points to center",
                "Velocity is tangent to circle",
                "Speed constant, but velocity changes (direction)"
            ]
        };
    }

    explainVerticalCircle() {
        return {
            title: "Vertical Circular Motion",
            steps: [
                {
                    title: "Step 1: Forces at Bottom",
                    content: "Tension and weight both act, tension larger.",
                    equation: "T - mg = mv²/r → T = mg + mv²/r"
                },
                {
                    title: "Step 2: Forces at Top",
                    content: "Both tension and weight point to center.",
                    equation: "T + mg = mv²/r → T = mv²/r - mg"
                },
                {
                    title: "Step 3: Minimum Speed at Top",
                    content: "For T ≥ 0 at top: mv²/r ≥ mg",
                    equation: "v_min = √(gr)"
                },
                {
                    title: "Step 4: Energy Consideration",
                    content: "If v_top = v_min, find v_bottom using energy conservation.",
                    equation: "v_bottom = √(5gr)"
                }
            ],
            keyTakeaways: [
                "Tension varies around loop",
                "Minimum speed needed at top to maintain circular motion",
                "T_bottom - T_top = 6mg (when v_top = √(gr))"
            ]
        };
    }

    explainProjectile() {
        return {
            title: "Projectile Motion",
            steps: [
                {
                    title: "Step 1: Resolve Initial Velocity",
                    content: "Break into horizontal and vertical components.",
                    equation: "v_x = v₀·cos(θ), v_y = v₀·sin(θ)"
                },
                {
                    title: "Step 2: Horizontal Motion",
                    content: "No acceleration in horizontal direction (ignoring air resistance).",
                    equation: "x = v₀·cos(θ)·t"
                },
                {
                    title: "Step 3: Vertical Motion",
                    content: "Constant downward acceleration g.",
                    equation: "y = v₀·sin(θ)·t - ½gt²"
                },
                {
                    title: "Step 4: Range Formula",
                    content: "Set y = 0 and solve for t, then find x.",
                    equation: "R = v₀²·sin(2θ)/g, max when θ = 45°"
                }
            ],
            keyTakeaways: [
                "Horizontal and vertical motions are independent",
                "Trajectory is parabolic",
                "Maximum range at 45° launch angle"
            ]
        };
    }

    explainElasticCollision() {
        return {
            title: "Elastic Collision (1D)",
            steps: [
                {
                    title: "Step 1: Conservation of Momentum",
                    content: "Total momentum before = total momentum after.",
                    equation: "m₁v₁ + m₂v₂ = m₁v₁' + m₂v₂'"
                },
                {
                    title: "Step 2: Conservation of Kinetic Energy",
                    content: "In elastic collision, KE is also conserved.",
                    equation: "½m₁v₁² + ½m₂v₂² = ½m₁v₁'² + ½m₂v₂'²"
                },
                {
                    title: "Step 3: Coefficient of Restitution",
                    content: "For elastic collision, e = 1.",
                    equation: "e = (v₂' - v₁')/(v₁ - v₂) = 1"
                },
                {
                    title: "Step 4: Solve System",
                    content: "Two equations, two unknowns (v₁' and v₂').",
                    equation: "v₁' = ((m₁-m₂)v₁ + 2m₂v₂)/(m₁+m₂)"
                }
            ],
            keyTakeaways: [
                "Both momentum and energy conserved in elastic collision",
                "Relative velocity reverses (e = 1)",
                "Special cases: equal masses exchange velocities"
            ]
        };
    }

    explainWorkEnergy() {
        return {
            title: "Work-Energy Theorem",
            steps: [
                {
                    title: "Step 1: Define Work",
                    content: "Work is force applied over distance.",
                    equation: "W = F·d·cos(θ)"
                },
                {
                    title: "Step 2: Work-Energy Theorem",
                    content: "Net work equals change in kinetic energy.",
                    equation: "W_net = ΔKE = KE_f - KE_i"
                },
                {
                    title: "Step 3: Calculate Initial and Final KE",
                    content: "Use KE = ½mv²",
                    equation: "W_net = ½m(v_f² - v_i²)"
                },
                {
                    title: "Step 4: Power",
                    content: "Rate of doing work.",
                    equation: "P = W/t = F·v"
                }
            ],
            keyTakeaways: [
                "Work changes kinetic energy",
                "Only net work matters for ΔKE",
                "Conservative forces: W = -ΔPE"
            ]
        };
    }

    explainSpringForce() {
        return {
            title: "Hooke's Law & Spring Force",
            steps: [
                {
                    title: "Step 1: Hooke's Law",
                    content: "Spring force proportional to displacement from equilibrium.",
                    equation: "F = -kx (negative = restoring force)"
                },
                {
                    title: "Step 2: Spring Potential Energy",
                    content: "Energy stored in compressed/stretched spring.",
                    equation: "PE_spring = ½kx²"
                },
                {
                    title: "Step 3: Simple Harmonic Motion",
                    content: "Mass on spring oscillates with period T.",
                    equation: "T = 2π√(m/k)"
                },
                {
                    title: "Step 4: Energy Conservation",
                    content: "Total energy = KE + PE = constant",
                    equation: "½mv² + ½kx² = constant"
                }
            ],
            keyTakeaways: [
                "Spring force always opposes displacement",
                "Stiffer spring (larger k) = more force",
                "Oscillation frequency independent of amplitude"
            ]
        };
    }

    explainRocket() {
        return {
            title: "Rocket Propulsion (Variable Mass)",
            steps: [
                {
                    title: "Step 1: Thrust Force",
                    content: "Rocket expels mass backward, producing forward thrust.",
                    equation: "F_thrust = v_e·(dm/dt)"
                },
                {
                    title: "Step 2: Equation of Motion",
                    content: "Newton's second law for variable mass system.",
                    equation: "F_thrust - mg = m(dv/dt)"
                },
                {
                    title: "Step 3: Tsiolkovsky Rocket Equation",
                    content: "Ideal velocity change (no gravity).",
                    equation: "Δv = v_e·ln(m_initial/m_final)"
                },
                {
                    title: "Step 4: Escape Velocity",
                    content: "Minimum velocity to escape gravitational field.",
                    equation: "v_escape = √(2GM/R) ≈ 11.2 km/s (Earth)"
                }
            ],
            keyTakeaways: [
                "Mass decreases as fuel burns",
                "Higher exhaust velocity = more efficient",
                "Logarithmic relationship: mass ratio matters exponentially"
            ]
        };
    }

    generateGenericExplanation(scenario) {
        return {
            title: scenario.name,
            steps: [
                {
                    title: "Scenario Description",
                    content: scenario.description,
                    equation: scenario.equations ? scenario.equations[0] : ""
                }
            ],
            keyTakeaways: scenario.equations || []
        };
    }

    // Generate NEET/JEE style numerical problems
    generateProblem(scenarioKey) {
        const problems = {
            '1d_acceleration': this.problem1DAcceleration,
            'static_friction': this.problemStaticFriction,
            'inclined_plane': this.problemInclinedPlane,
            'atwood_machine': this.problemAtwoodMachine,
            'projectile': this.problemProjectile,
            'work_energy': this.problemWorkEnergy,
            'elastic_collision': this.problemElasticCollision
        };

        const generator = problems[scenarioKey];
        if (generator) {
            this.currentProblem = generator.call(this);
            return this.currentProblem;
        }

        return null;
    }

    problem1DAcceleration() {
        const mass = (Math.random() * 10 + 2).toFixed(1);
        const force = (Math.random() * 50 + 10).toFixed(1);
        const acceleration = (force / mass).toFixed(2);

        return {
            question: `A body of mass ${mass} kg is acted upon by a constant force of ${force} N. Calculate the acceleration produced.`,
            options: [
                `${acceleration} m/s²`,
                `${(parseFloat(acceleration) * 1.5).toFixed(2)} m/s²`,
                `${(parseFloat(acceleration) * 0.5).toFixed(2)} m/s²`,
                `${(parseFloat(acceleration) * 2).toFixed(2)} m/s²`
            ],
            correctAnswer: 0,
            solution: `Using Newton's Second Law: F = ma
a = F/m = ${force}/${mass} = ${acceleration} m/s²`,
            formula: "a = F/m"
        };
    }

    problemStaticFriction() {
        const mass = (Math.random() * 20 + 10).toFixed(1);
        const mu = (Math.random() * 0.5 + 0.3).toFixed(2);
        const g = 10;
        const maxFriction = (mass * mu * g).toFixed(1);

        return {
            question: `A block of mass ${mass} kg rests on a horizontal surface with coefficient of static friction μ_s = ${mu}. What is the maximum force that can be applied horizontally without moving the block? (g = 10 m/s²)`,
            options: [
                `${maxFriction} N`,
                `${(parseFloat(maxFriction) * 1.2).toFixed(1)} N`,
                `${(parseFloat(maxFriction) * 0.8).toFixed(1)} N`,
                `${(mass * g).toFixed(1)} N`
            ],
            correctAnswer: 0,
            solution: `Maximum static friction: f_s,max = μ_s·N = μ_s·mg
f_s,max = ${mu} × ${mass} × ${g} = ${maxFriction} N`,
            formula: "f_s,max = μ_s·N"
        };
    }

    problemInclinedPlane() {
        const mass = (Math.random() * 15 + 5).toFixed(1);
        const angle = Math.floor(Math.random() * 30 + 20);
        const g = 10;
        const force = (mass * g * Math.sin(angle * Math.PI / 180)).toFixed(1);

        return {
            question: `A block of mass ${mass} kg is placed on a frictionless incline of angle ${angle}°. Calculate the component of weight parallel to the incline. (g = 10 m/s²)`,
            options: [
                `${force} N`,
                `${(parseFloat(force) * 1.3).toFixed(1)} N`,
                `${(mass * g).toFixed(1)} N`,
                `${(mass * g * Math.cos(angle * Math.PI / 180)).toFixed(1)} N`
            ],
            correctAnswer: 0,
            solution: `Component parallel to incline: F_∥ = mg·sin(θ)
F_∥ = ${mass} × ${g} × sin(${angle}°) = ${force} N`,
            formula: "F_∥ = mg·sin(θ)"
        };
    }

    problemAtwoodMachine() {
        const m1 = (Math.random() * 5 + 5).toFixed(1);
        const m2 = (Math.random() * 3 + 2).toFixed(1);
        const g = 10;
        const accel = ((m1 - m2) * g / (parseFloat(m1) + parseFloat(m2))).toFixed(2);

        return {
            question: `In an Atwood machine, two masses ${m1} kg and ${m2} kg are connected by a massless string over a frictionless pulley. Find the acceleration of the system. (g = 10 m/s²)`,
            options: [
                `${accel} m/s²`,
                `${(parseFloat(accel) * 2).toFixed(2)} m/s²`,
                `${g} m/s²`,
                `${((m1 - m2) / (parseFloat(m1) + parseFloat(m2))).toFixed(2)} m/s²`
            ],
            correctAnswer: 0,
            solution: `Using Atwood machine formula: a = (m₁ - m₂)g/(m₁ + m₂)
a = (${m1} - ${m2}) × ${g} / (${m1} + ${m2}) = ${accel} m/s²`,
            formula: "a = (m₁ - m₂)g/(m₁ + m₂)"
        };
    }

    problemProjectile() {
        const velocity = (Math.random() * 20 + 20).toFixed(1);
        const angle = 45;
        const g = 10;
        const range = (velocity * velocity * Math.sin(2 * angle * Math.PI / 180) / g).toFixed(1);

        return {
            question: `A projectile is launched at ${velocity} m/s at an angle of ${angle}° to the horizontal. Calculate the horizontal range. (g = 10 m/s²)`,
            options: [
                `${range} m`,
                `${(parseFloat(range) * 1.5).toFixed(1)} m`,
                `${(parseFloat(range) * 0.5).toFixed(1)} m`,
                `${(velocity * velocity / g).toFixed(1)} m`
            ],
            correctAnswer: 0,
            solution: `Range formula: R = v₀²·sin(2θ)/g
R = ${velocity}² × sin(${2 * angle}°) / ${g} = ${range} m`,
            formula: "R = v₀²·sin(2θ)/g"
        };
    }

    problemWorkEnergy() {
        const mass = (Math.random() * 10 + 5).toFixed(1);
        const vi = (Math.random() * 10 + 5).toFixed(1);
        const vf = (Math.random() * 15 + 10).toFixed(1);
        const work = (0.5 * mass * (vf * vf - vi * vi)).toFixed(1);

        return {
            question: `A ${mass} kg object's velocity changes from ${vi} m/s to ${vf} m/s. Calculate the work done on the object.`,
            options: [
                `${work} J`,
                `${(parseFloat(work) * 1.2).toFixed(1)} J`,
                `${(0.5 * mass * vf * vf).toFixed(1)} J`,
                `${(mass * (parseFloat(vf) - parseFloat(vi))).toFixed(1)} J`
            ],
            correctAnswer: 0,
            solution: `Work-Energy Theorem: W = ΔKE = ½m(v_f² - v_i²)
W = 0.5 × ${mass} × (${vf}² - ${vi}²) = ${work} J`,
            formula: "W = ½m(v_f² - v_i²)"
        };
    }

    problemElasticCollision() {
        const m1 = (Math.random() * 3 + 2).toFixed(1);
        const m2 = (Math.random() * 3 + 2).toFixed(1);
        const v1 = (Math.random() * 5 + 3).toFixed(1);
        const v2 = 0;
        
        const v1final = (((parseFloat(m1) - parseFloat(m2)) * parseFloat(v1)) / (parseFloat(m1) + parseFloat(m2))).toFixed(2);

        return {
            question: `A ${m1} kg object moving at ${v1} m/s collides elastically with a stationary ${m2} kg object. Find the velocity of the first object after collision.`,
            options: [
                `${v1final} m/s`,
                `${(parseFloat(v1final) * 1.5).toFixed(2)} m/s`,
                `${v1} m/s`,
                `0 m/s`
            ],
            correctAnswer: 0,
            solution: `For elastic collision: v₁' = ((m₁-m₂)v₁ + 2m₂v₂)/(m₁+m₂)
Since v₂ = 0: v₁' = (${m1}-${m2})×${v1}/(${m1}+${m2}) = ${v1final} m/s`,
            formula: "v₁' = ((m₁-m₂)v₁)/(m₁+m₂)"
        };
    }

    checkAnswer(userAnswer) {
        if (!this.currentProblem) return null;

        const isCorrect = userAnswer === this.currentProblem.correctAnswer;
        
        return {
            correct: isCorrect,
            correctAnswer: this.currentProblem.options[this.currentProblem.correctAnswer],
            solution: this.currentProblem.solution
        };
    }
}
