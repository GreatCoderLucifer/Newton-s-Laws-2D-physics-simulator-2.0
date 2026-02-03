# Newton's Laws Physics Simulator
## Complete Educational Tool for NEET/JEE/Olympiad Preparation

A production-grade, fully-featured 2D physics simulator built entirely with **vanilla HTML, CSS, and JavaScript** (no external libraries). This simulator provides accurate, real-time visualization of Newton's Laws of Motion with comprehensive educational features.

---

## 🎯 Features

### Core Physics Engine
- **Vector Mathematics**: Complete 2D vector class with all operations
- **Rigid Body Dynamics**: Full implementation with mass, velocity, acceleration, rotation
- **Multiple Integration Methods**:
  - Euler (basic)
  - Semi-Implicit Euler (recommended, more stable)
  - Runge-Kutta 4th Order (highest accuracy)
- **Force Accumulator System**: Proper implementation of F = ma
- **Constraint System**: Springs, ropes, rods
- **Collision Detection & Resolution**: With coefficient of restitution

### Complete Newton's Laws Coverage

#### (A) Basic Motion
- 1D uniform motion (First Law - Inertia)
- 1D uniform acceleration (Second Law)
- 2D motion with independent components
- Relative motion between frames

#### (B) Forces
- Applied force at any angle
- Weight and normal force (Third Law pairs)
- Spring force (Hooke's Law)
- Tension in strings/ropes
- Pseudo forces in non-inertial frames

#### (C) Friction
- Static friction (f_s ≤ μ_s·N)
- Kinetic friction (f_k = μ_k·N)
- Limiting friction demonstration
- Friction on inclined planes

#### (D) Inclined Planes
- Adjustable angle (0° to 90°)
- Force decomposition (parallel & perpendicular)
- Block sliding with/without friction
- Maximum angle before slipping

#### (E) Connected Systems
- Two blocks connected by string
- Atwood machine (pulley system)
- Tension force calculation
- System acceleration

#### (F) Circular Motion
- Uniform circular motion (centripetal force)
- Vertical circle (variable tension)
- Minimum speed at top of loop
- Breaking tension threshold

#### (G) Projectile Motion
- Parabolic trajectory
- With/without air resistance
- Range, time of flight, max height formulas
- Drag force (linear + quadratic)

#### (H) Variable Mass Systems
- Rocket propulsion
- Tsiolkovsky rocket equation
- Thrust = v_e × dm/dt
- Mass flow rate control

#### (I) Non-Inertial Frames
- Accelerating reference frames
- Elevator problems
- Pseudo force visualization
- Apparent weight changes

#### (J) Collisions
- Elastic collision (e = 1)
- Inelastic collision (0 < e < 1)
- Perfectly inelastic (e = 0)
- Momentum & energy conservation

#### (K) Work, Energy & Power
- Work-energy theorem
- Kinetic energy tracking
- Gravitational potential energy
- Energy dissipation by friction
- Real-time energy graphs

### Educational Features

#### 📚 Step-by-Step Explanations
- Detailed breakdown for each scenario
- Physics equations with actual values
- Solution methodology
- Key takeaways and concepts

#### 🎓 NEET/JEE Problem Generator
- Auto-generated numerical problems
- Multiple choice questions
- Instant answer verification
- Detailed solutions with formulas

#### 📊 Visualization Tools
- **Force Vectors**: Color-coded arrows with magnitudes
- **Free Body Diagrams**: Real-time FBD for each object
- **Motion Trails**: Track object paths
- **Energy Graphs**: KE, PE, and total energy vs time
- **Vector Components**: Show x and y components

### User Interface
- **30+ Pre-built Scenarios** covering all topics
- **Real-time Parameter Control**:
  - Mass, force, angle
  - Gravity, friction coefficients
  - Spring constants, drag coefficients
  - Time step, integration method
- **Interactive Canvas**: Drag objects with mouse
- **Keyboard Shortcuts** for quick control
- **Dark Scientific Theme** for comfortable viewing

---

## 🚀 Quick Start

### Installation
1. Download all files to the same directory:
   - `physics-simulator.html`
   - `physics-engine.js`
   - `scenarios.js`
   - `renderer.js`
   - `educational.js`
   - `ui-controller.js`
   - `main.js`

2. Open `physics-simulator.html` in a modern web browser (Chrome, Firefox, Edge, Safari)

3. No installation, no build process, no dependencies required!

### Basic Usage
1. **Select a Scenario**: Click any scenario button in the sidebar
2. **Play/Pause**: Click "▶ Play" or press Spacebar
3. **Adjust Parameters**: Use sliders to change properties in real-time
4. **Reset**: Click "↻ Reset" or press R
5. **Drag Objects**: Click and drag objects on canvas when paused

---

## ⌨️ Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play/Pause simulation |
| `R` | Reset current scenario |
| `S` | Step one frame (when paused) |
| `G` | Toggle coordinate grid |
| `V` | Toggle force vectors |
| `F` | Toggle free body diagrams |
| `T` | Toggle motion trail |

---

## 📐 Physics Accuracy

All physics calculations follow **first principles** from classical mechanics:

### Newton's Laws
1. **First Law (Inertia)**: Objects maintain velocity when F_net = 0
2. **Second Law**: F = m·a (force produces acceleration)
3. **Third Law**: Action-reaction pairs (F_AB = -F_BA)

### Forces
- **Gravity**: F_g = m·g (9.8 m/s² default)
- **Normal**: Perpendicular to surface contact
- **Friction**: 
  - Static: f_s ≤ μ_s·N
  - Kinetic: f_k = μ_k·N
- **Spring**: F = -k·x (Hooke's Law)
- **Drag**: 
  - Linear: F = -b·v
  - Quadratic: F = -½·ρ·C_d·A·v²

### Energy & Momentum
- **Conservation of Energy**: E = KE + PE = constant (no friction)
- **Conservation of Momentum**: p_total = constant (closed system)
- **Work-Energy Theorem**: W_net = ΔKE

### Collisions
- **Momentum**: Always conserved
- **Energy**: Conserved only in elastic collisions
- **Coefficient of Restitution**: e = (v₂' - v₁')/(v₁ - v₂)

---

## 🎨 Customization

### Adding New Scenarios

Edit `scenarios.js` and add to the `SCENARIOS` object:

```javascript
'my_scenario': {
    name: 'My Custom Scenario',
    description: 'Description of what this demonstrates',
    setup: (world) => {
        // Create objects
        const body = new RigidBody(mass, position);
        body.velocity = new Vector2(vx, vy);
        world.addBody(body);
        
        // Add constraints
        const spring = new SpringConstraint(body1, body2, length, k);
        world.addConstraint(spring);
    },
    parameters: ['mass', 'force', 'gravity'],
    equations: ['F = ma', 'v = u + at']
}
```

### Creating Custom Forces

Add to `PhysicsWorld` class in `physics-engine.js`:

```javascript
applyCustomForce() {
    this.bodies.forEach(body => {
        const force = calculateYourForce(body);
        body.applyForce(force, 'Force Name');
    });
}
```

### Integration Methods

Choose from three methods (UI or code):

```javascript
// Euler (simple, less stable)
body.updateEuler(dt);

// Semi-Implicit Euler (recommended)
body.updateSemiImplicitEuler(dt);

// RK4 (most accurate, slower)
body.updateRK4(dt);
```

---

## 📚 Educational Use Cases

### For Students
- **Visualize Concepts**: See forces, motion, and energy in real-time
- **Practice Problems**: Auto-generated NEET/JEE style questions
- **Step-by-Step Learning**: Detailed explanations for each scenario
- **Experiment Freely**: Adjust parameters and observe results

### For Teachers
- **Classroom Demonstrations**: Project on screen for whole class
- **Interactive Lessons**: Let students control parameters
- **Problem Sets**: Generate unique problems for each student
- **Homework Tool**: Students explore scenarios at home

### For Olympiad Preparation
- **Advanced Scenarios**: Variable mass, non-inertial frames
- **Precise Calculations**: Verify solutions numerically
- **Edge Cases**: Explore limiting conditions
- **Multiple Integration**: Compare numerical methods

---

## 🔧 Technical Architecture

### File Structure
```
physics-simulator.html  - Main HTML structure & CSS
physics-engine.js      - Core physics (Vector2, RigidBody, World)
scenarios.js          - 30+ scenario definitions
renderer.js          - Canvas rendering & visualization
educational.js       - Explanations & problem generator
ui-controller.js     - User interaction & controls
main.js             - Application lifecycle & initialization
```

### Design Principles
1. **Separation of Concerns**: Physics, rendering, and UI are independent
2. **No Magic Numbers**: All constants named and documented
3. **Physically Accurate**: All formulas from first principles
4. **Extensible**: Easy to add scenarios and features
5. **Performance**: Fixed timestep, efficient algorithms
6. **Educational**: Code teaches physics concepts

### Physics Update Loop
```
1. Clear forces from previous frame
2. Apply global forces (gravity, drag)
3. Apply scenario-specific forces
4. Solve constraints (springs, ropes)
5. Detect collisions
6. Integrate motion (Euler/RK4)
7. Resolve collisions
8. Track energy/momentum
```

---

## 📊 Performance

- **60 FPS** on modern browsers
- **Fixed timestep** (16.67ms default) for stability
- **Efficient collision detection** (O(n²) for small n)
- **Optimized rendering** (dirty flag pattern)
- **No memory leaks** (proper cleanup)

Tested on:
- Chrome 120+
- Firefox 120+
- Safari 17+
- Edge 120+

---

## 🎯 NEET/JEE Topic Coverage

### Mechanics Syllabus Alignment

✅ **Kinematics**
- 1D and 2D motion
- Relative motion
- Projectile motion

✅ **Newton's Laws**
- All three laws with examples
- Free body diagrams
- Applications to various scenarios

✅ **Friction**
- Static and kinetic friction
- Angle of repose
- Motion on inclined planes

✅ **Circular Motion**
- Centripetal force
- Vertical circles
- Conical pendulum (can be added)

✅ **Work, Energy, Power**
- Work-energy theorem
- Conservation of energy
- Potential and kinetic energy

✅ **Collisions**
- Elastic and inelastic
- Coefficient of restitution
- Conservation laws

✅ **System of Particles**
- Connected masses
- Pulleys (Atwood machine)
- Variable mass systems

---

## 🔬 Advanced Features

### Numerical Stability
- **Constraint Stabilization**: Prevents drift in rope/rod constraints
- **Velocity Clamping**: Prevents numerical explosions
- **Fixed Timestep**: Independent of frame rate
- **Energy Monitoring**: Detect and correct energy drift

### Debugging Tools
Access via browser console:
```javascript
world           // Physics world object
renderer        // Renderer object
uiController    // UI controller
toggleRocketThrust()  // Control rocket
```

### Extension Points
The architecture supports adding:
- Particle systems
- Soft body physics
- Basic fluid dynamics
- Multi-body chains
- Custom constraint types
- Torque and rotation
- Moment of inertia variations

---

## 📖 Learning Resources

### Recommended Study Path
1. **Start with**: 1D Motion scenarios
2. **Progress to**: Force and friction
3. **Master**: Connected systems and pulleys
4. **Advanced**: Circular motion and collisions
5. **Challenge**: Variable mass and non-inertial frames

### Problem Solving Strategy
1. Draw Free Body Diagram
2. Identify all forces
3. Apply Newton's Second Law
4. Solve for unknowns
5. Verify with simulation

---

## 🐛 Troubleshooting

### Simulation runs slowly
- Close other browser tabs
- Reduce number of objects
- Disable motion trail
- Lower timestep resolution

### Objects behave unexpectedly
- Check mass values (not too small)
- Verify friction coefficients
- Reset scenario
- Check for NaN in console

### Canvas not displaying
- Ensure JavaScript is enabled
- Check browser console for errors
- Try different browser
- Verify all JS files loaded

---

## 🤝 Contributing

This is an educational tool. Contributions welcome:
- Additional scenarios
- Bug fixes
- Educational content improvements
- Documentation enhancements

---

## 📄 License

Educational use only. Free to use for learning, teaching, and non-commercial purposes.

---

## 🎓 Credits

Developed as a comprehensive educational tool for physics students preparing for NEET, JEE, and Physics Olympiads.

**Physics Principles**: Classical Mechanics (Newton, Lagrange, Hamilton)
**Educational Design**: Aligned with Indian competitive exam syllabi
**Code Quality**: Production-grade, well-documented, extensible

---

## 📧 Support

For questions or issues, use browser console for debugging or consult the inline code documentation.

---

## 🚀 Future Enhancements

Potential additions:
- 3D physics engine
- Lagrangian mechanics view
- Hamiltonian formulation
- Chaos theory demonstrations
- Special relativity corrections
- Export simulation data
- Video recording of simulations
- Mobile app version

---

**Version**: 1.0.0  
**Last Updated**: 2026  
**Compatibility**: All modern browsers  
**Dependencies**: None (Pure JavaScript)

---

## Quick Reference Card

### Common Formulas
```
Newton's Second Law:     F = m·a
Friction (static):       f_s ≤ μ_s·N
Friction (kinetic):      f_k = μ_k·N
Centripetal Force:       F_c = m·v²/r
Hooke's Law:            F = -k·x
Work-Energy:            W = ΔKE
Momentum:               p = m·v
Collision (elastic):    e = 1
Energy Conservation:    KE + PE = constant
```

### SI Units
- Mass: kg
- Force: N (Newton)
- Velocity: m/s
- Acceleration: m/s²
- Energy: J (Joule)
- Power: W (Watt)
- Momentum: kg·m/s

---

**Happy Learning! May Newton's Laws be with you! 🍎**
