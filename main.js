/**
 * MAIN APPLICATION
 * Initializes and runs the complete Newton's Laws Physics Simulator
 */

// Global references
let world;
let renderer;
let eduModule;
let uiController;
let canvas;
let graphCanvas;
let animationFrameId;
let lastTime = 0;

// Initialize the application
function init() {
    console.log('Initializing Newton\'s Laws Physics Simulator...');
    
    // Get canvas elements
    canvas = document.getElementById('mainCanvas');
    graphCanvas = document.getElementById('graphCanvas');
    
    if (!canvas) {
        console.error('Main canvas not found!');
        return;
    }

    // Initialize physics world
    world = new PhysicsWorld();
    
    // Set camera offset (coordinate system)
    const canvasWidth = canvas.width;
    const canvasHeight = canvas.height;
    
    // Initialize renderer
    renderer = new Renderer(canvas, world);
    renderer.offsetX = 50; // Left margin
    renderer.offsetY = 50; // Top margin
    
    // Initialize educational module
    eduModule = new EducationalModule(world);
    
    // Initialize UI controller
    uiController = new UIController(world, renderer, eduModule);
    
    // Make uiController global for button callbacks
    window.uiController = uiController;
    
    // Load default scenario
    loadDefaultScenario();
    
    // Start animation loop
    startAnimationLoop();
    
    // Setup canvas interaction
    setupCanvasInteraction();
    
    console.log('Initialization complete!');
    
    // Hide loading screen
    const loading = document.getElementById('loading');
    if (loading) {
        loading.classList.remove('active');
    }
}

function loadDefaultScenario() {
    // Load the first scenario as default
    const defaultScenario = '1d_acceleration';
    uiController.loadScenario(defaultScenario);
}

function setupCanvasInteraction() {
    let isDragging = false;
    let draggedBody = null;
    let mouseOffset = new Vector2();

    canvas.addEventListener('mousedown', (e) => {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const worldPos = renderer.screenToWorld(mouseX, mouseY);

        // Check if clicking on a body
        world.bodies.forEach(body => {
            const dist = body.position.distanceTo(worldPos);
            if (dist < body.radius && !body.isStatic) {
                isDragging = true;
                draggedBody = body;
                mouseOffset = body.position.subtract(worldPos);
                
                // Pause simulation when dragging
                if (!uiController.isPaused) {
                    uiController.controls.playPauseBtn.click();
                }
            }
        });
    });

    canvas.addEventListener('mousemove', (e) => {
        if (isDragging && draggedBody) {
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            const worldPos = renderer.screenToWorld(mouseX, mouseY);
            
            // Update body position
            draggedBody.position = worldPos.add(mouseOffset);
            draggedBody.velocity = new Vector2(); // Reset velocity
        }
    });

    canvas.addEventListener('mouseup', () => {
        isDragging = false;
        draggedBody = null;
    });

    canvas.addEventListener('mouseleave', () => {
        isDragging = false;
        draggedBody = null;
    });

    // Add keyboard controls
    document.addEventListener('keydown', (e) => {
        switch(e.key) {
            case ' ': // Spacebar - play/pause
                e.preventDefault();
                uiController.controls.playPauseBtn.click();
                break;
            case 'r': // R - reset
            case 'R':
                uiController.resetScenario();
                break;
            case 's': // S - step
            case 'S':
                if (uiController.isPaused) {
                    uiController.stepFrame();
                }
                break;
            case 'g': // G - toggle grid
            case 'G':
                uiController.controls.showGrid.checked = !uiController.controls.showGrid.checked;
                uiController.controls.showGrid.dispatchEvent(new Event('change'));
                break;
            case 'v': // V - toggle vectors
            case 'V':
                uiController.controls.showVectors.checked = !uiController.controls.showVectors.checked;
                uiController.controls.showVectors.dispatchEvent(new Event('change'));
                break;
            case 'f': // F - toggle FBD
            case 'F':
                uiController.controls.showFBD.checked = !uiController.controls.showFBD.checked;
                uiController.controls.showFBD.dispatchEvent(new Event('change'));
                break;
            case 't': // T - toggle trail
            case 'T':
                uiController.controls.showTrail.checked = !uiController.controls.showTrail.checked;
                uiController.controls.showTrail.dispatchEvent(new Event('change'));
                break;
        }
    });
}

function startAnimationLoop() {
    function animate(currentTime) {
        // Calculate delta time
        const deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // Update physics if not paused
        if (!uiController.isPaused) {
            // Use fixed timestep for stability
            uiController.stepFrame();
        }

        // Render
        renderer.render();
        
        // Update UI stats
        uiController.update();
        
        // Render graph if visible
        if (uiController.controls.showGraph.checked) {
            renderer.renderGraph(graphCanvas);
        }

        // Continue loop
        animationFrameId = requestAnimationFrame(animate);
    }

    // Start the loop
    animationFrameId = requestAnimationFrame(animate);
}

function stopAnimationLoop() {
    if (animationFrameId) {
        cancelAnimationFrame(animationFrameId);
    }
}

// Additional utility functions for rocket scenarios
function toggleRocketThrust() {
    world.bodies.forEach(body => {
        if (body.shape === 'rocket') {
            body.isThrusting = !body.isThrusting;
        }
    });
}

// Export functions for debugging
window.world = world;
window.renderer = renderer;
window.eduModule = eduModule;
window.toggleRocketThrust = toggleRocketThrust;

// Performance monitoring
const stats = {
    frameCount: 0,
    fps: 0,
    lastFpsUpdate: 0
};

function updatePerformanceStats(currentTime) {
    stats.frameCount++;
    
    if (currentTime - stats.lastFpsUpdate > 1000) {
        stats.fps = stats.frameCount;
        stats.frameCount = 0;
        stats.lastFpsUpdate = currentTime;
        
        // Display FPS in console (optional)
        // console.log(`FPS: ${stats.fps}`);
    }
}

// Initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}

// Clean up on page unload
window.addEventListener('beforeunload', () => {
    stopAnimationLoop();
});

/**
 * EXTENSION POINTS FOR FUTURE DEVELOPMENT
 * 
 * 1. Additional Scenarios:
 *    - Add to scenarios.js SCENARIOS object
 *    - Follow existing pattern with setup function
 * 
 * 2. New Force Types:
 *    - Add to PhysicsWorld.applyCustomForces()
 *    - Create new force calculation methods
 * 
 * 3. Advanced Constraints:
 *    - Create new constraint classes extending base pattern
 *    - Add to PhysicsWorld.constraints array
 * 
 * 4. Particle Systems:
 *    - Create ParticleEmitter class
 *    - Integrate with RigidBody system
 * 
 * 5. Soft Body Physics:
 *    - Implement spring-mass network
 *    - Use existing SpringConstraint
 * 
 * 6. Fluid Dynamics (Basic):
 *    - Add buoyancy force calculation
 *    - Extend drag for Reynolds number
 * 
 * 7. Multi-body Chains:
 *    - Chain constructor in scenarios
 *    - Use RodConstraint or RopeConstraint
 * 
 * 8. Export/Import Scenarios:
 *    - JSON serialization of world state
 *    - Load custom scenarios from file
 * 
 * 9. Video Recording:
 *    - Capture canvas frames
 *    - MediaRecorder API integration
 * 
 * 10. Advanced Integration Methods:
 *     - Verlet integration
 *     - Symplectic integrators
 *     - Adaptive timestep
 * 
 * CODE ORGANIZATION:
 * 
 * physics-engine.js    - Core physics (Vector2, RigidBody, PhysicsWorld)
 * scenarios.js         - Scenario definitions
 * renderer.js          - All visualization
 * educational.js       - Explanations and problems
 * ui-controller.js     - User interaction
 * main.js             - Application lifecycle (this file)
 * 
 * DESIGN PRINCIPLES:
 * 
 * 1. Separation of Concerns: Physics, rendering, and UI are separate
 * 2. No Magic Numbers: All constants are named and documented
 * 3. Physically Accurate: All formulas from first principles
 * 4. Extensible: Easy to add new scenarios and features
 * 5. Educational: Code teaches physics concepts
 * 6. Performance: Fixed timestep, efficient algorithms
 * 7. Debuggable: Clear variable names, console access
 * 
 * PHYSICS ACCURACY NOTES:
 * 
 * - All units are SI (m, kg, s, N, J)
 * - Newton's Laws strictly enforced
 * - Energy conservation verified (within numerical limits)
 * - Momentum conservation in collisions
 * - Realistic friction models
 * - Proper force decomposition
 * - Constraint stabilization
 * 
 * EDUCATIONAL FEATURES:
 * 
 * - Step-by-step explanations
 * - Free body diagrams
 * - Vector visualization
 * - Real-time equation display
 * - NEET/JEE problem generator
 * - Multiple integration methods
 * - Energy graphs
 * 
 * NUMERICAL STABILITY:
 * 
 * - Semi-implicit Euler recommended
 * - Fixed timestep (not frame-dependent)
 * - Constraint stabilization
 * - Velocity clamping for extreme cases
 * - RK4 available for high accuracy
 * 
 * PERFORMANCE OPTIMIZATION:
 * 
 * - Spatial partitioning (for many bodies)
 * - Object pooling (for particles)
 * - Canvas layer separation
 * - Efficient vector operations
 * - Early exits in collision detection
 * 
 * ACCESSIBILITY:
 * 
 * - Keyboard shortcuts
 * - Color-blind friendly palette
 * - High contrast mode option
 * - Screen reader support (aria labels)
 * - Scalable UI elements
 */

console.log(`
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║       NEWTON'S LAWS PHYSICS SIMULATOR                        ║
║       Complete Educational Tool for NEET/JEE                 ║
║                                                              ║
║       Features:                                              ║
║       • Accurate physics simulation                          ║
║       • 30+ scenarios covering all NLM topics                ║
║       • Step-by-step explanations                            ║
║       • Free body diagrams                                   ║
║       • Energy & momentum graphs                             ║
║       • NEET/JEE problem generator                           ║
║       • Multiple integration methods                         ║
║                                                              ║
║       Keyboard Shortcuts:                                    ║
║       Space - Play/Pause                                     ║
║       R - Reset scenario                                     ║
║       S - Step one frame                                     ║
║       G - Toggle grid                                        ║
║       V - Toggle vectors                                     ║
║       F - Toggle free body diagrams                          ║
║       T - Toggle motion trail                                ║
║                                                              ║
║       Ready for physics exploration! 🚀                      ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
`);
