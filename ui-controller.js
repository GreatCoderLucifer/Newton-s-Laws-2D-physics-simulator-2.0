/**
 * UI CONTROLLER
 * Handles all user interactions, parameter updates, and UI state
 */

class UIController {
    constructor(world, renderer, educationalModule) {
        this.world = world;
        this.renderer = renderer;
        this.eduModule = educationalModule;
        
        this.isPaused = true;
        this.currentScenario = null;
        
        this.initializeControls();
        this.setupEventListeners();
    }

    initializeControls() {
        // Control references
        this.controls = {
            // Simulation controls
            playPauseBtn: document.getElementById('playPauseBtn'),
            stepBtn: document.getElementById('stepBtn'),
            resetBtn: document.getElementById('resetBtn'),
            
            // Scenario
            scenarioGrid: document.getElementById('scenarioGrid'),
            
            // Physics parameters
            gravitySlider: document.getElementById('gravitySlider'),
            gravityVal: document.getElementById('gravityVal'),
            dtSlider: document.getElementById('dtSlider'),
            dtVal: document.getElementById('dtVal'),
            integratorSelect: document.getElementById('integratorSelect'),
            
            // Object properties
            massSlider: document.getElementById('massSlider'),
            massVal: document.getElementById('massVal'),
            forceSlider: document.getElementById('forceSlider'),
            forceVal: document.getElementById('forceVal'),
            forceAngleSlider: document.getElementById('forceAngleSlider'),
            forceAngleVal: document.getElementById('forceAngleVal'),
            
            // Friction
            musSlider: document.getElementById('musSlider'),
            musVal: document.getElementById('musVal'),
            mukSlider: document.getElementById('mukSlider'),
            mukVal: document.getElementById('mukVal'),
            inclineSlider: document.getElementById('inclineSlider'),
            inclineVal: document.getElementById('inclineVal'),
            
            // Drag
            dragLinearSlider: document.getElementById('dragLinearSlider'),
            dragLinearVal: document.getElementById('dragLinearVal'),
            dragQuadSlider: document.getElementById('dragQuadSlider'),
            dragQuadVal: document.getElementById('dragQuadVal'),
            
            // Springs
            springKSlider: document.getElementById('springKSlider'),
            springKVal: document.getElementById('springKVal'),
            restLengthSlider: document.getElementById('restLengthSlider'),
            restLengthVal: document.getElementById('restLengthVal'),
            
            // Visualization
            showVectors: document.getElementById('showVectors'),
            showFBD: document.getElementById('showFBD'),
            showGrid: document.getElementById('showGrid'),
            showTrail: document.getElementById('showTrail'),
            showGraph: document.getElementById('showGraph'),
            
            // Educational
            eduMode: document.getElementById('eduMode'),
            problemMode: document.getElementById('problemMode'),
            nonInertialFrame: document.getElementById('nonInertialFrame'),
            
            // Display panels
            infoPanel: document.getElementById('infoPanel'),
            statsGrid: document.getElementById('statsGrid'),
            equationsDisplay: document.getElementById('equationsDisplay'),
            eduContent: document.getElementById('eduContent'),
            problemContent: document.getElementById('problemContent'),
            graphPanel: document.getElementById('graphPanel')
        };
    }

    setupEventListeners() {
        // Play/Pause
        this.controls.playPauseBtn.addEventListener('click', () => {
            this.isPaused = !this.isPaused;
            this.controls.playPauseBtn.textContent = this.isPaused ? '▶ Play' : '⏸ Pause';
            this.controls.playPauseBtn.classList.toggle('active', !this.isPaused);
        });

        // Step frame
        this.controls.stepBtn.addEventListener('click', () => {
            if (this.isPaused) {
                this.stepFrame();
            }
        });

        // Reset
        this.controls.resetBtn.addEventListener('click', () => {
            this.resetScenario();
        });

        // Sliders with live update
        this.setupSlider('gravity', (val) => {
            this.world.gravity.y = parseFloat(val);
        });

        this.setupSlider('dt', (val) => {
            this.world.fixedTimeStep = parseFloat(val);
        });

        this.setupSlider('mass', (val) => {
            this.updateBodyProperty('mass', parseFloat(val));
        });

        this.setupSlider('force', (val) => {
            this.appliedForce = parseFloat(val);
        });

        this.setupSlider('forceAngle', (val) => {
            this.forceAngle = parseFloat(val) * Math.PI / 180;
        });

        this.setupSlider('mus', (val) => {
            this.updateBodyProperty('friction', { static: parseFloat(val) });
        });

        this.setupSlider('muk', (val) => {
            this.updateBodyProperty('friction', { kinetic: parseFloat(val) });
        });

        this.setupSlider('incline', (val) => {
            this.renderer.inclineAngle = parseFloat(val) * Math.PI / 180;
        });

        this.setupSlider('dragLinear', (val) => {
            this.world.dragCoefficients.linear = parseFloat(val);
        });

        this.setupSlider('dragQuad', (val) => {
            this.world.dragCoefficients.quadratic = parseFloat(val);
        });

        this.setupSlider('springK', (val) => {
            this.updateConstraintProperty('stiffness', parseFloat(val));
        });

        this.setupSlider('restLength', (val) => {
            this.updateConstraintProperty('restLength', parseFloat(val));
        });

        // Integrator
        this.controls.integratorSelect.addEventListener('change', (e) => {
            this.integrator = e.target.value;
        });

        // Visualization checkboxes
        this.controls.showVectors.addEventListener('change', (e) => {
            this.renderer.showVectors = e.target.checked;
        });

        this.controls.showFBD.addEventListener('change', (e) => {
            this.renderer.showFBD = e.target.checked;
        });

        this.controls.showGrid.addEventListener('change', (e) => {
            this.renderer.showGrid = e.target.checked;
        });

        this.controls.showTrail.addEventListener('change', (e) => {
            this.renderer.showTrail = e.target.checked;
        });

        this.controls.showGraph.addEventListener('change', (e) => {
            this.controls.graphPanel.style.display = e.target.checked ? 'block' : 'none';
        });

        // Educational mode
        this.controls.eduMode.addEventListener('change', (e) => {
            this.eduModule.stepByStepMode = e.target.checked;
            this.updateEducationalContent();
        });

        this.controls.problemMode.addEventListener('change', (e) => {
            this.eduModule.problemMode = e.target.checked;
            this.updateProblemContent();
        });

        this.controls.nonInertialFrame.addEventListener('change', (e) => {
            if (e.target.checked) {
                this.world.frameAcceleration = new Vector2(0, -2.0);
            } else {
                this.world.frameAcceleration = new Vector2(0, 0);
            }
        });

        // Generate scenario buttons
        this.generateScenarioButtons();
        
        // Initialize values
        this.appliedForce = 0;
        this.forceAngle = 0;
        this.integrator = 'semiImplicit';
    }

    setupSlider(name, callback) {
        const slider = this.controls[`${name}Slider`];
        const display = this.controls[`${name}Val`];
        
        if (!slider || !display) return;
        
        slider.addEventListener('input', (e) => {
            const value = e.target.value;
            display.textContent = parseFloat(value).toFixed(2);
            callback(value);
        });
    }

    generateScenarioButtons() {
        const grid = this.controls.scenarioGrid;
        grid.innerHTML = '';

        // Create buttons for each category
        for (const [category, scenarios] of Object.entries(SCENARIO_CATEGORIES)) {
            // Category header
            const header = document.createElement('div');
            header.style.gridColumn = '1 / -1';
            header.style.color = '#81c784';
            header.style.fontSize = '12px';
            header.style.fontWeight = 'bold';
            header.style.marginTop = '10px';
            header.textContent = category;
            grid.appendChild(header);

            // Scenario buttons
            scenarios.forEach(key => {
                const scenario = SCENARIOS[key];
                const btn = document.createElement('button');
                btn.className = 'scenario-btn';
                btn.textContent = scenario.name;
                btn.addEventListener('click', (e) => {
    this.loadScenario(key, e);
});

                grid.appendChild(btn);
            });
        }
    }

    loadScenario(scenarioKey, event) {

        const scenario = SCENARIOS[scenarioKey];
        if (!scenario) return;

        // Clear world
        this.world.reset();
        
        // Setup scenario
        scenario.setup(this.world);
        
        this.currentScenario = scenarioKey;
        
        // Update UI
        document.querySelectorAll('.scenario-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        if (event && event.target) {
    event.target.classList.add('active');
} else {
    // fallback: activate by scenario key
    document.querySelectorAll('.scenario-btn').forEach(btn => {
        if (btn.textContent === SCENARIOS[scenarioKey].name) {
            btn.classList.add('active');
        }
    });
}


        
        // Update educational content
        this.updateEducationalContent();
        
        // Reset to paused
        this.isPaused = true;
        this.controls.playPauseBtn.textContent = '▶ Play';
        this.controls.playPauseBtn.classList.remove('active');
    }

    resetScenario() {
        if (this.currentScenario) {
            this.loadScenario(this.currentScenario);
        }
    }

    stepFrame() {
        // Apply external forces
        this.applyExternalForces();
        
        // Step physics
        this.world.step(this.world.fixedTimeStep, this.integrator);
        
        // Update rocket-specific logic
        this.updateRockets();
    }

    applyExternalForces() {
        if (Math.abs(this.appliedForce) > 0.01) {
            this.world.bodies.forEach(body => {
                if (!body.isStatic && !body.shape === 'rocket') {
                    const force = Vector2.fromAngle(this.forceAngle, this.appliedForce);
                    body.applyForce(force, 'Applied Force');
                }
            });
        }
    }

    updateRockets() {
        this.world.bodies.forEach(body => {
            if (body.shape === 'rocket' && body.isThrusting && body.fuelMass > 0) {
                // Rocket thrust
                const thrust = body.exhaustVelocity * body.massFlowRate;
                const thrustVector = Vector2.fromAngle(body.angle - Math.PI / 2, thrust);
                body.applyForce(thrustVector, 'Rocket Thrust');
                
                // Decrease fuel mass
                body.fuelMass -= body.massFlowRate * this.world.fixedTimeStep;
                if (body.fuelMass < 0) body.fuelMass = 0;
                
                // Update total mass
                body.mass = body.dryMass + body.fuelMass;
                body.inverseMass = 1 / body.mass;
            }
        });
    }

    updateBodyProperty(property, value) {
        this.world.bodies.forEach(body => {
            if (!body.isStatic) {
                if (property === 'friction') {
                    if (value.static !== undefined) body.friction.static = value.static;
                    if (value.kinetic !== undefined) body.friction.kinetic = value.kinetic;
                } else {
                    body[property] = value;
                    if (property === 'mass') {
                        body.inverseMass = 1 / value;
                    }
                }
            }
        });
    }

    updateConstraintProperty(property, value) {
        this.world.constraints.forEach(constraint => {
            if (constraint[property] !== undefined) {
                constraint[property] = value;
            }
        });
    }

    updateStats() {
        const statsGrid = this.controls.statsGrid;
        statsGrid.innerHTML = '';

        // Calculate total system stats
        let totalMass = 0;
        let totalKE = 0;
        let totalPE = 0;
        let totalMomentum = new Vector2();

        this.world.bodies.forEach(body => {
            if (!body.isStatic) {
                totalMass += body.mass;
                totalKE += body.kineticEnergy();
                totalPE += body.mass * this.world.gravity.y * (this.world.groundLevel - body.position.y);
                totalMomentum = totalMomentum.add(body.momentum());
            }
        });

        const stats = [
            { label: 'Time', value: `${this.world.time.toFixed(2)} s` },
            { label: 'Total Mass', value: `${totalMass.toFixed(2)} kg` },
            { label: 'KE', value: `${totalKE.toFixed(2)} J` },
            { label: 'PE', value: `${totalPE.toFixed(2)} J` },
            { label: 'Total E', value: `${(totalKE + totalPE).toFixed(2)} J` },
            { label: 'Momentum', value: `${totalMomentum.magnitude().toFixed(2)} kg·m/s` }
        ];

        // Add body-specific stats if only one body
        if (this.world.bodies.length === 1) {
            const body = this.world.bodies[0];
            stats.push(
                { label: 'Position', value: `(${body.position.x.toFixed(2)}, ${body.position.y.toFixed(2)}) m` },
                { label: 'Velocity', value: `${body.velocity.magnitude().toFixed(2)} m/s` },
                { label: 'Acceleration', value: `${body.acceleration.magnitude().toFixed(2)} m/s²` }
            );
        }

        stats.forEach(stat => {
            const item = document.createElement('div');
            item.className = 'stat-item';
            item.innerHTML = `
                <div class="stat-label">${stat.label}</div>
                <div class="stat-value">${stat.value}</div>
            `;
            statsGrid.appendChild(item);
        });
    }

    updateEquations() {
        const display = this.controls.equationsDisplay;
        
        if (!this.currentScenario) {
            display.innerHTML = '';
            return;
        }

        const scenario = SCENARIOS[this.currentScenario];
        if (!scenario.equations) {
            display.innerHTML = '';
            return;
        }

        display.innerHTML = '<h3 style="color: #81c784; margin: 10px 0; font-size: 13px;">Key Equations</h3>';
        
        scenario.equations.forEach(eq => {
            const eqDiv = document.createElement('div');
            eqDiv.className = 'equation';
            eqDiv.textContent = eq;
            display.appendChild(eqDiv);
        });
    }

    updateEducationalContent() {
        const content = this.controls.eduContent;
        
        if (!this.eduModule.stepByStepMode || !this.currentScenario) {
            content.innerHTML = '';
            return;
        }

        const explanation = this.eduModule.generateExplanation(this.currentScenario);
        if (!explanation) {
            content.innerHTML = '';
            return;
        }

        let html = '<div class="edu-panel">';
        html += `<h4>${explanation.title}</h4>`;
        
        explanation.steps.forEach((step, index) => {
            html += `<div class="step">
                <strong>${step.title}</strong><br>
                ${step.content}
                ${step.equation ? `<div class="equation">${step.equation}</div>` : ''}
            </div>`;
        });

        if (explanation.keyTakeaways) {
            html += '<div style="margin-top: 10px;"><strong style="color: #81c784;">Key Takeaways:</strong><ul style="margin: 5px 0; padding-left: 20px; font-size: 11px;">';
            explanation.keyTakeaways.forEach(point => {
                html += `<li>${point}</li>`;
            });
            html += '</ul></div>';
        }

        html += '</div>';
        content.innerHTML = html;
    }

    updateProblemContent() {
        const content = this.controls.problemContent;
        
        if (!this.eduModule.problemMode || !this.currentScenario) {
            content.innerHTML = '';
            return;
        }

        const problem = this.eduModule.generateProblem(this.currentScenario);
        if (!problem) {
            content.innerHTML = '';
            return;
        }

        let html = '<div class="problem-box">';
        html += '<h4>🎓 NEET/JEE Practice Problem</h4>';
        html += `<p style="font-size: 13px; margin: 10px 0;">${problem.question}</p>`;
        
        html += '<div style="margin: 10px 0;">';
        problem.options.forEach((option, index) => {
            html += `<div style="margin: 5px 0;">
                <input type="radio" name="answer" value="${index}" id="opt${index}">
                <label for="opt${index}" style="font-size: 12px; margin-left: 5px;">${option}</label>
            </div>`;
        });
        html += '</div>';
        
        html += '<button onclick="uiController.checkProblemAnswer()" style="width: 100%; margin-top: 10px;">Check Answer</button>';
        html += '<div id="problemFeedback"></div>';
        html += '</div>';
        
        content.innerHTML = html;
    }

    checkProblemAnswer() {
        const selected = document.querySelector('input[name="answer"]:checked');
        if (!selected) {
            alert('Please select an answer!');
            return;
        }

        const userAnswer = parseInt(selected.value);
        const result = this.eduModule.checkAnswer(userAnswer);
        
        const feedback = document.getElementById('problemFeedback');
        if (result.correct) {
            feedback.className = 'feedback correct';
            feedback.innerHTML = `✓ Correct!<br><div style="margin-top: 5px; font-size: 11px;">${result.solution}</div>`;
        } else {
            feedback.className = 'feedback incorrect';
            feedback.innerHTML = `✗ Incorrect. The correct answer is: ${result.correctAnswer}<br><div style="margin-top: 5px; font-size: 11px;">${result.solution}</div>`;
        }
    }

    update() {
        this.updateStats();
        this.updateEquations();
    }
}
