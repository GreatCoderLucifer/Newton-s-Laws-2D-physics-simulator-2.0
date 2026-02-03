/**
 * RENDERER - Visualization Engine
 * Handles all drawing: objects, vectors, FBD, grids, graphs
 */

class Renderer {
    constructor(canvas, world) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.world = world;
        
        // Camera/View parameters
        this.scale = 100; // pixels per meter
        this.offsetX = 0;
        this.offsetY = 0;
        
        // Visualization flags
        this.showVectors = true;
        this.showFBD = true;
        this.showGrid = false;
        this.showTrail = false;
        
        // Incline angle for rendering
        this.inclineAngle = 0; // radians
        
        // Colors
        this.colors = {
            background: '#0f1229',
            grid: '#1e2749',
            ground: '#2d3561',
            force: '#f44336',
            velocity: '#4fc3f7',
            acceleration: '#81c784',
            normal: '#ff9800',
            friction: '#9c27b0',
            tension: '#ffeb3b'
        };
    }

    // Convert world coordinates to screen coordinates
    worldToScreen(worldPos) {
        return {
            x: worldPos.x * this.scale + this.offsetX,
            y: worldPos.y * this.scale + this.offsetY
        };
    }

    // Convert screen to world
    screenToWorld(screenX, screenY) {
        return new Vector2(
            (screenX - this.offsetX) / this.scale,
            (screenY - this.offsetY) / this.scale
        );
    }

    clear() {
        this.ctx.fillStyle = this.colors.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }

    render() {
        this.clear();
        
        if (this.showGrid) {
            this.drawGrid();
        }
        
        this.drawGround();
        this.drawConstraints();
        this.drawBodies();
        
        if (this.showVectors) {
            this.drawVectors();
        }
        
        if (this.showFBD) {
            this.drawFreeBoDiagrams();
        }
        
        if (this.showTrail) {
            this.drawTrails();
        }
    }

    drawGrid() {
        const { ctx } = this;
        ctx.strokeStyle = this.colors.grid;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 4]);

        // Vertical lines
        for (let x = 0; x <= this.canvas.width; x += this.scale) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, this.canvas.height);
            ctx.stroke();
        }

        // Horizontal lines
        for (let y = 0; y <= this.canvas.height; y += this.scale) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(this.canvas.width, y);
            ctx.stroke();
        }

        ctx.setLineDash([]);

        // Axes
        ctx.strokeStyle = '#4fc3f7';
        ctx.lineWidth = 2;
        
        // X-axis
        ctx.beginPath();
        ctx.moveTo(0, this.offsetY);
        ctx.lineTo(this.canvas.width, this.offsetY);
        ctx.stroke();
        
        // Y-axis
        ctx.beginPath();
        ctx.moveTo(this.offsetX, 0);
        ctx.lineTo(this.offsetX, this.canvas.height);
        ctx.stroke();

        // Labels
        ctx.fillStyle = '#4fc3f7';
        ctx.font = '12px monospace';
        ctx.fillText('X', this.canvas.width - 20, this.offsetY - 10);
        ctx.fillText('Y', this.offsetX + 10, 20);
    }

    drawGround() {
        const { ctx } = this;
        const groundY = this.world.groundLevel * this.scale + this.offsetY;
        
        // Ground line
        ctx.strokeStyle = this.colors.ground;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, groundY);
        ctx.lineTo(this.canvas.width, groundY);
        ctx.stroke();

        // Inclined plane if angle > 0
        if (this.inclineAngle > 0) {
            ctx.save();
            ctx.strokeStyle = this.colors.ground;
            ctx.fillStyle = 'rgba(45, 53, 97, 0.3)';
            ctx.lineWidth = 3;
            
            const startX = 100;
            const length = 400;
            const height = length * Math.sin(this.inclineAngle);
            
            ctx.beginPath();
            ctx.moveTo(startX, groundY);
            ctx.lineTo(startX + length * Math.cos(this.inclineAngle), groundY - height);
            ctx.lineTo(startX + length * Math.cos(this.inclineAngle), groundY);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            
            // Angle arc
            ctx.strokeStyle = '#81c784';
            ctx.beginPath();
            ctx.arc(startX, groundY, 40, -this.inclineAngle, 0);
            ctx.stroke();
            
            // Angle label
            ctx.fillStyle = '#81c784';
            ctx.font = 'bold 14px monospace';
            ctx.fillText(`θ = ${(this.inclineAngle * 180 / Math.PI).toFixed(1)}°`, startX + 50, groundY - 10);
            
            ctx.restore();
        }

        // Hatching pattern
        ctx.strokeStyle = 'rgba(79, 195, 247, 0.3)';
        ctx.lineWidth = 1;
        for (let x = 0; x < this.canvas.width; x += 20) {
            ctx.beginPath();
            ctx.moveTo(x, groundY);
            ctx.lineTo(x + 10, groundY + 10);
            ctx.stroke();
        }
    }

    drawBodies() {
        this.world.bodies.forEach(body => {
            const pos = this.worldToScreen(body.position);
            const { ctx } = this;

            ctx.save();

            if (body.shape === 'rocket') {
                this.drawRocket(body, pos);
            } else if (body.shape === 'circle') {
                this.drawCircle(body, pos);
            } else {
                this.drawBox(body, pos);
            }

            // Highlight if grounded
            if (body.isGrounded) {
                ctx.strokeStyle = '#81c784';
                ctx.lineWidth = 2;
                ctx.setLineDash([4, 4]);
                ctx.strokeRect(
                    pos.x - body.radius * this.scale - 5,
                    pos.y - body.radius * this.scale - 5,
                    body.radius * this.scale * 2 + 10,
                    body.radius * this.scale * 2 + 10
                );
                ctx.setLineDash([]);
            }

            ctx.restore();
        });
    }

    drawCircle(body, pos) {
        const { ctx } = this;
        const radius = body.radius * this.scale;

        // Body
        ctx.fillStyle = body.color;
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, radius, 0, Math.PI * 2);
        ctx.fill();

        // Border
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Rotation indicator
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(pos.x, pos.y);
        ctx.lineTo(
            pos.x + Math.cos(body.angle) * radius,
            pos.y + Math.sin(body.angle) * radius
        );
        ctx.stroke();

        // Label with mass
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 12px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`${body.mass.toFixed(1)}kg`, pos.x, pos.y + 5);
    }

    drawBox(body, pos) {
        const { ctx } = this;
        const w = body.width * this.scale;
        const h = body.height * this.scale;

        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(body.angle);

        // Body
        ctx.fillStyle = body.color;
        ctx.fillRect(-w / 2, -h / 2, w, h);

        // Border
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.strokeRect(-w / 2, -h / 2, w, h);

        ctx.restore();

        // Label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 12px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`${body.mass.toFixed(1)}kg`, pos.x, pos.y + 5);
    }

    drawRocket(body, pos) {
        const { ctx } = this;
        const size = body.radius * this.scale;

        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(body.angle - Math.PI / 2);

        // Rocket body
        ctx.fillStyle = body.color;
        ctx.beginPath();
        ctx.moveTo(0, -size);
        ctx.lineTo(size / 2, size);
        ctx.lineTo(-size / 2, size);
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Exhaust if thrusting
        if (body.isThrusting && body.fuelMass > 0) {
            ctx.fillStyle = '#ff9800';
            ctx.beginPath();
            ctx.moveTo(-size / 3, size);
            ctx.lineTo(0, size + size / 2);
            ctx.lineTo(size / 3, size);
            ctx.closePath();
            ctx.fill();
        }

        ctx.restore();

        // Fuel indicator
        ctx.fillStyle = '#fff';
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`Fuel: ${body.fuelMass.toFixed(1)}kg`, pos.x, pos.y + size + 15);
    }

    drawConstraints() {
        const { ctx } = this;

        this.world.constraints.forEach(constraint => {
            if (constraint instanceof SpringConstraint) {
                this.drawSpring(constraint);
            } else if (constraint instanceof RopeConstraint) {
                this.drawRope(constraint);
            } else if (constraint instanceof RodConstraint) {
                this.drawRod(constraint);
            }
        });
    }

    drawSpring(spring) {
        const { ctx } = this;
        const posA = this.worldToScreen(spring.bodyA.position);
        const posB = this.worldToScreen(spring.bodyB.position);

        const dx = posB.x - posA.x;
        const dy = posB.y - posA.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const angle = Math.atan2(dy, dx);

        ctx.save();
        ctx.translate(posA.x, posA.y);
        ctx.rotate(angle);

        // Draw spring coils
        ctx.strokeStyle = '#4fc3f7';
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        const coils = 10;
        const amplitude = 8;
        
        for (let i = 0; i <= coils; i++) {
            const x = (i / coils) * dist;
            const y = Math.sin(i * Math.PI) * amplitude;
            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.stroke();

        ctx.restore();

        // Spring info
        const midX = (posA.x + posB.x) / 2;
        const midY = (posA.y + posB.y) / 2;
        ctx.fillStyle = '#4fc3f7';
        ctx.font = '11px monospace';
        ctx.fillText(`k=${spring.stiffness}N/m`, midX, midY - 10);
    }

    drawRope(rope) {
        const { ctx } = this;
        const posA = this.worldToScreen(rope.bodyA.position);
        const posB = this.worldToScreen(rope.bodyB.position);

        const delta = rope.bodyB.position.subtract(rope.bodyA.position);
        const currentLength = delta.magnitude();
        const isTaut = currentLength >= rope.maxLength * 0.99;

        // Rope color changes with tension
        ctx.strokeStyle = isTaut ? this.colors.tension : '#666';
        ctx.lineWidth = isTaut ? 3 : 2;
        ctx.setLineDash(isTaut ? [] : [5, 5]);

        ctx.beginPath();
        ctx.moveTo(posA.x, posA.y);
        ctx.lineTo(posB.x, posB.y);
        ctx.stroke();
        ctx.setLineDash([]);

        // Tension label
        if (isTaut && rope.tension > 0) {
            const midX = (posA.x + posB.x) / 2;
            const midY = (posA.y + posB.y) / 2;
            ctx.fillStyle = this.colors.tension;
            ctx.font = 'bold 11px monospace';
            ctx.fillText(`T=${rope.tension.toFixed(1)}N`, midX, midY - 5);
        }
    }

    drawRod(rod) {
        const { ctx } = this;
        const posA = this.worldToScreen(rod.bodyA.position);
        const posB = this.worldToScreen(rod.bodyB.position);

        ctx.strokeStyle = '#999';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(posA.x, posA.y);
        ctx.lineTo(posB.x, posB.y);
        ctx.stroke();
    }

    drawVectors() {
        this.world.bodies.forEach(body => {
            if (body.isStatic) return;
            
            const pos = this.worldToScreen(body.position);

            // Velocity vector
            if (body.velocity.magnitude() > 0.1) {
                this.drawVector(
                    pos.x, pos.y,
                    body.velocity.x, body.velocity.y,
                    this.colors.velocity, 'v', 20
                );
            }

            // Acceleration vector
            if (body.acceleration.magnitude() > 0.1) {
                this.drawVector(
                    pos.x, pos.y,
                    body.acceleration.x, body.acceleration.y,
                    this.colors.acceleration, 'a', 30
                );
            }
        });
    }

    drawVector(x, y, vx, vy, color, label, scale = 20) {
        const { ctx } = this;
        const mag = Math.sqrt(vx * vx + vy * vy);
        if (mag < 0.01) return;

        const endX = x + vx * scale;
        const endY = y + vy * scale;

        // Arrow shaft
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(endX, endY);
        ctx.stroke();

        // Arrowhead
        const angle = Math.atan2(vy, vx);
        const headLength = 10;
        
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(endX, endY);
        ctx.lineTo(
            endX - headLength * Math.cos(angle - Math.PI / 6),
            endY - headLength * Math.sin(angle - Math.PI / 6)
        );
        ctx.lineTo(
            endX - headLength * Math.cos(angle + Math.PI / 6),
            endY - headLength * Math.sin(angle + Math.PI / 6)
        );
        ctx.closePath();
        ctx.fill();

        // Label
        ctx.fillStyle = color;
        ctx.font = 'bold 12px monospace';
        ctx.fillText(
            `${label}=${mag.toFixed(2)}`,
            endX + 5,
            endY - 5
        );
    }

    drawFreeBoDiagrams() {
        this.world.bodies.forEach((body, index) => {
            if (body.isStatic) return;

            const pos = this.worldToScreen(body.position);
            const offsetX = 200;
            const offsetY = index * 150;
            const fbdX = this.canvas.width - offsetX;
            const fbdY = 100 + offsetY;

            // Skip if off screen
            if (fbdY > this.canvas.height - 100) return;

            this.drawFBD(body, fbdX, fbdY);
        });
    }

    drawFBD(body, x, y) {
        const { ctx } = this;

        // Background
        ctx.fillStyle = 'rgba(20, 24, 48, 0.9)';
        ctx.strokeStyle = '#4fc3f7';
        ctx.lineWidth = 2;
        ctx.fillRect(x - 90, y - 70, 180, 140);
        ctx.strokeRect(x - 90, y - 70, 180, 140);

        // Title
        ctx.fillStyle = '#4fc3f7';
        ctx.font = 'bold 11px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('Free Body Diagram', x, y - 55);

        // Point mass representation
        ctx.fillStyle = body.color;
        ctx.beginPath();
        ctx.arc(x, y, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw all forces
        const scale = 3; // Force visualization scale
        body.forces.forEach(forceData => {
            const force = forceData.force;
            const name = forceData.name;
            
            let color = this.colors.force;
            if (name.includes('Gravity') || name.includes('Weight')) color = '#f44336';
            if (name.includes('Normal')) color = '#ff9800';
            if (name.includes('Friction')) color = '#9c27b0';
            if (name.includes('Spring')) color = '#4fc3f7';
            if (name.includes('Tension')) color = '#ffeb3b';
            if (name.includes('Drag')) color = '#00bcd4';
            
            this.drawFBDVector(x, y, force.x, force.y, color, name, scale);
        });

        // Net force
        if (body.netForce.magnitude() > 0.1) {
            ctx.strokeStyle = '#81c784';
            ctx.lineWidth = 3;
            ctx.setLineDash([5, 3]);
            const endX = x + body.netForce.x * scale;
            const endY = y + body.netForce.y * scale;
            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(endX, endY);
            ctx.stroke();
            ctx.setLineDash([]);

            ctx.fillStyle = '#81c784';
            ctx.font = 'bold 10px monospace';
            ctx.fillText(`F_net=${body.netForce.magnitude().toFixed(1)}N`, x, y + 55);
        }
    }

    drawFBDVector(x, y, fx, fy, color, label, scale) {
        const { ctx } = this;
        const mag = Math.sqrt(fx * fx + fy * fy);
        if (mag < 0.01) return;

        const endX = x + fx * scale;
        const endY = y + fy * scale;

        // Arrow
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(endX, endY);
        ctx.stroke();

        // Arrowhead
        const angle = Math.atan2(fy, fx);
        const headLength = 8;
        
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(endX, endY);
        ctx.lineTo(
            endX - headLength * Math.cos(angle - Math.PI / 6),
            endY - headLength * Math.sin(angle - Math.PI / 6)
        );
        ctx.lineTo(
            endX - headLength * Math.cos(angle + Math.PI / 6),
            endY - headLength * Math.sin(angle + Math.PI / 6)
        );
        ctx.closePath();
        ctx.fill();

        // Label
        ctx.fillStyle = color;
        ctx.font = '9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`${label.substring(0, 12)}`, endX + 5, endY + 3);
        ctx.fillText(`${mag.toFixed(1)}N`, endX + 5, endY + 13);
    }

    drawTrails() {
        this.world.bodies.forEach(body => {
            if (body.trail.length < 2) return;

            const { ctx } = this;
            ctx.strokeStyle = body.color;
            ctx.lineWidth = 2;
            ctx.globalAlpha = 0.3;
            ctx.setLineDash([2, 2]);

            ctx.beginPath();
            const start = this.worldToScreen(body.trail[0]);
            ctx.moveTo(start.x, start.y);

            for (let i = 1; i < body.trail.length; i++) {
                const pos = this.worldToScreen(body.trail[i]);
                ctx.lineTo(pos.x, pos.y);
            }

            ctx.stroke();
            ctx.setLineDash([]);
            ctx.globalAlpha = 1.0;
        });
    }

    // Graph renderer (energy vs time)
    renderGraph(graphCanvas) {
        const ctx = graphCanvas.getContext('2d');
        const width = graphCanvas.width;
        const height = graphCanvas.height;
        
        ctx.fillStyle = '#0f1229';
        ctx.fillRect(0, 0, width, height);

        if (this.world.energyHistory.length < 2) return;

        // Find max energy for scaling
        let maxEnergy = 0;
        this.world.energyHistory.forEach(data => {
            maxEnergy = Math.max(maxEnergy, data.total, data.kinetic, data.potential);
        });

        if (maxEnergy < 0.01) return;

        const margin = 30;
        const graphWidth = width - 2 * margin;
        const graphHeight = height - 2 * margin;

        // Draw axes
        ctx.strokeStyle = '#4fc3f7';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(margin, margin);
        ctx.lineTo(margin, height - margin);
        ctx.lineTo(width - margin, height - margin);
        ctx.stroke();

        // Labels
        ctx.fillStyle = '#b0b0b0';
        ctx.font = '11px monospace';
        ctx.fillText('Energy (J)', margin + 5, margin - 10);
        ctx.fillText('Time (s)', width - margin - 40, height - margin + 20);

        // Draw energy curves
        this.drawEnergyCurve(ctx, this.world.energyHistory, 'kinetic', '#4fc3f7', margin, graphWidth, graphHeight, maxEnergy);
        this.drawEnergyCurve(ctx, this.world.energyHistory, 'potential', '#ff9800', margin, graphWidth, graphHeight, maxEnergy);
        this.drawEnergyCurve(ctx, this.world.energyHistory, 'total', '#81c784', margin, graphWidth, graphHeight, maxEnergy);

        // Legend
        const legends = [
            { label: 'KE', color: '#4fc3f7' },
            { label: 'PE', color: '#ff9800' },
            { label: 'Total', color: '#81c784' }
        ];
        
        legends.forEach((leg, i) => {
            ctx.fillStyle = leg.color;
            ctx.fillRect(width - 100, 10 + i * 20, 15, 3);
            ctx.fillText(leg.label, width - 80, 10 + i * 20 + 5);
        });
    }

    drawEnergyCurve(ctx, data, property, color, marginLeft, graphWidth, graphHeight, maxEnergy) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();

        const margin = 30;
        
        data.forEach((point, index) => {
            const x = marginLeft + (index / (data.length - 1)) * graphWidth;
            const y = margin + graphHeight - (point[property] / maxEnergy) * graphHeight;
            
            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });

        ctx.stroke();
    }
}
