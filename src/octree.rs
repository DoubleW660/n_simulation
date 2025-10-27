use glam::Vec3;

const CAPACITY: usize = 1; // Max bodies per node before subdivision

#[derive(Clone)]
struct Body {
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    mass: f32,
    index: usize, // Original index in bodies array
}

// Physics integration using Velocity Verlet method
impl Body {
    fn new(position: Vec3, velocity: Vec3, mass: f32, index: usize) -> Self {
        Body {
            position,
            velocity,
            acceleration: Vec3::ZERO,
            mass,
            index,
        }
    }

    // Update velocity by half-step (for Velocity Verlet)
    fn update_velocity_half(&mut self, dt: f32) {
        self.velocity += self.acceleration * (dt / 2.0);
    }

    // Update position
    fn update_position(&mut self, dt: f32) {
        self.position += self.velocity * dt;
    }

    // Apply force and calculate acceleration
    fn apply_force(&mut self, force: Vec3) {
        self.acceleration = force / self.mass;
    }
}

#[derive(Clone)]
struct BoundingBox {
    center: Vec3,
    half_size: f32,
}

impl BoundingBox {

    // Does this bounding box contain the point
    fn contains(&self, point: Vec3) -> bool {
        // Returns if the point is contained within the 6 sides of the cube
        (point.x >= self.center.x - self.half_size && point.x <= self.center.x + self.half_size) &&
        (point.y >= self.center.y - self.half_size && point.y <= self.center.y + self.half_size) &&
        (point.z >= self.center.z - self.half_size && point.z <= self.center.z + self.half_size)
    }
    
    fn get_octant(&self, point: Vec3) -> usize {
        let mut octant = 0;
        if point.x >= self.center.x { octant |= 4; }
        if point.y >= self.center.y { octant |= 2; }
        if point.z >= self.center.z { octant |= 1; }
        octant
    }

    fn get_octant_box(&self, octant: usize) -> BoundingBox {
        let quarter = self.half_size / 2.0;
        let offset_x = if octant & 4 != 0 { quarter } else { -quarter };
        let offset_y = if octant & 2 != 0 { quarter } else { -quarter };
        let offset_z = if octant & 1 != 0 { quarter } else { -quarter };

        BoundingBox { center: Vec3::new(
            self.center.x + offset_x,
            self.center.y + offset_y,
            self.center.z + offset_z,
        ), 
            half_size: quarter }
    }
}



struct OctreeNode {
    boundary: BoundingBox,
    center_of_mass: Vec3,
    total_mass: f32,
    body: Option<Body>,
    children: Option<Box<[OctreeNode; 8]>>,   
} 

impl OctreeNode {

    // Constructor 
    fn new(boundary: BoundingBox) -> Self {
        OctreeNode { 
            boundary,
            center_of_mass: Vec3::ZERO,
            total_mass: 0.0,
            body: None,
            children: None }
    }

    // Return If Leaf True
    fn is_leaf(&self) -> bool {
        self.children.is_none()
    }

    fn insert_body(&mut self, new_body: Body) -> bool {
    // Confirm it is within the box
        if !self.boundary.contains(new_body.position) {
            return false;
        }

        // Case 1: Empty leaf node - just store the body
        if self.is_leaf() && self.body.is_none() {
            self.body = Some(new_body.clone());
            self.center_of_mass = new_body.position;
            self.total_mass = new_body.mass;
            return true;
        }

    // Case 2: Leaf node with a body - need to subdivide
        if self.is_leaf() && self.body.is_some() {
            let existing_body = self.body.take().unwrap();
            self.subdivide();
        
            // Re-insert both the existing body and the new body
            let octant_existing = self.boundary.get_octant(existing_body.position);
            self.children.as_mut().unwrap()[octant_existing].insert_body(existing_body);
        
            let octant_new = self.boundary.get_octant(new_body.position);
            self.children.as_mut().unwrap()[octant_new].insert_body(new_body);
        
            // Update center of mass
            self.calc_center_mass();
            return true;
        }

        // Case 3: Internal node - insert into appropriate child
        if !self.is_leaf() {
            let octant = self.boundary.get_octant(new_body.position);
            let success = self.children.as_mut().unwrap()[octant].insert_body(new_body);
            
            if success {
                self.calc_center_mass();
            }
        
            return success;
        }

        false
    }

    

    // Subdivide on greater than 1 body per node
    fn subdivide(&mut self) {
        // Ensure we are not subdividing an empty Node
        if !self.is_leaf() {
            return; 
        }

        let children: [OctreeNode; 8] = [
            OctreeNode::new(self.boundary.get_octant_box(0)),
            OctreeNode::new(self.boundary.get_octant_box(1)),
            OctreeNode::new(self.boundary.get_octant_box(2)),
            OctreeNode::new(self.boundary.get_octant_box(3)),
            OctreeNode::new(self.boundary.get_octant_box(4)),
            OctreeNode::new(self.boundary.get_octant_box(5)),
            OctreeNode::new(self.boundary.get_octant_box(6)),
            OctreeNode::new(self.boundary.get_octant_box(7)),
        ];
        
        self.children = Some(Box::new(children));
    }

    fn calc_center_mass(&mut self){

        
        // Case 1 Is leaf and No Body
        if self.is_leaf(){
            if let Some(ref body) = self.body {
                self.center_of_mass = body.position;
                self.total_mass = body.mass;
            }else {
                self.center_of_mass = Vec3::ZERO;
                self.total_mass = 0.0;
            }
            return
        } 

        let mut total_mass = 0.0;
        let mut weighted_pos = Vec3::ZERO;

        // Case 3 Internal Node with Children
        if let Some(ref children) = self.children {
            for child in children.iter() {
                if child.total_mass > 0.00 {
                    weighted_pos += child.center_of_mass * child.total_mass;
                    total_mass += child.total_mass;
                }
            }
        }
            
        if total_mass > 0.0 {
            self.center_of_mass = weighted_pos / total_mass;
            self.total_mass = total_mass;
        } else {
            self.center_of_mass = Vec3::ZERO;
            self.total_mass = 0.0;
        }


        }

        // Helper function to calculate force on a body using Barnes-Hut
    fn calculate_force(&self, body: &Body, theta: f32, softening: f32, g: f32) -> Vec3 {
           
        // Don't calculate force on itself
        if self.is_leaf() {
           if let Some(ref node_body) = self.body {
               
                if node_body.index == body.index {
                    return Vec3::ZERO;
                }
            }
        }

        // If node is empty
        if self.total_mass == 0.0 {
            return Vec3::ZERO;
        }

        let r = self.center_of_mass - body.position;
        let distance = r.length();
    
        // Barnes-Hut criterion: s/d < theta
        let s = self.boundary.half_size * 2.0;
    
        if self.is_leaf() || (s / distance < theta) {
            // Treat as single body
            if distance < 0.0001 {
                return Vec3::ZERO;
            }
        
            let distance_sq = distance * distance + softening * softening;
            let force_magnitude = g * body.mass * self.total_mass / distance_sq;
            let force = r.normalize() * force_magnitude;
            return force;
        }

        // Otherwise, recurse into children
        let mut force = Vec3::ZERO;
        if let Some(ref children) = self.children {
            for child in children.iter() {
                force += child.calculate_force(body, theta, softening, g);
            }
        }

    force
}
}

fn update_simulation(bodies: &mut Vec<Body>, dt: f32, theta: f32, softening: f32, g: f32) {
    
    // HALF STEP Velocity Update
    for body in bodies.iter_mut(){
        body.update_velocity_half(dt);
    }

    //Update Positions
    for body in bodies.iter_mut() {
        body.update_position(dt);
    }

    // Build New Octree
    let mut min_pos = Vec3::splat(f32::INFINITY);
    let mut max_pos = Vec3::splat(f32::NEG_INFINITY);

    for body in bodies.iter() {
        min_pos = min_pos.min(body.position);
        max_pos = max_pos.max(body.position);
    }

     // Create bounding box with some padding
    let center = (min_pos + max_pos) / 2.0;
    let size = (max_pos - min_pos).max_element();
    let half_size = (size / 2.0) * 1.1; // 10% padding
    
    let boundary = BoundingBox {
        center,
        half_size: half_size.max(1.0), // Minimum size of 1.0
    };

    let mut root = OctreeNode::new(boundary);
    
    // Insert all bodies into octree
    for body in bodies.iter() {
        let success = root.insert_body(body.clone());
    }

    // Calculate forces for all bodies
    for body in bodies.iter_mut() {
        let force = root.calculate_force(body, theta, softening, g);
    body.apply_force(force);
    }

    // Half-step velocity update using new acceleration
    for body in bodies.iter_mut() {
        body.update_velocity_half(dt);
    }
}

fn calculate_energy(bodies: &Vec<Body>, g: f32) -> (f32, f32){
    let mut kinetic = 0.0;
    let mut potential = 0.0;

    // Kinetic energy
    for body in bodies.iter() {
        kinetic += 0.5 * body.mass * body.velocity.length_squared();
    }

    // Potential energy
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            let r = bodies[i].position - bodies[j].position;
            let distance = r.length();
            if distance > 0.0001 {
                potential -= g * bodies[i].mass * bodies[j].mass / distance;
            }
        }
    }

    (kinetic, potential)

}

// Better two-body orbit - both bodies orbit their common center of mass
fn create_two_body_orbit() -> Vec<Body> {
    let m1 = 100.0_f32;  // Mass of body 1
    let m2 = 1.0_f32;    // Mass of body 2
    let separation = 10.0_f32;
    
    // Calculate center of mass position
    let total_mass = m1 + m2;
    let r1 = separation * m2 / total_mass;  // Distance of m1 from COM
    let r2 = separation * m1 / total_mass;  // Distance of m2 from COM
    
    // Orbital velocity: v = sqrt(G * M / r)
    // For circular orbit: v1 * m1 = v2 * m2 (momentum conservation)
    let v_orbit = (1.0 * total_mass / separation).sqrt();
    let v1 = v_orbit * m2 / total_mass;
    let v2 = v_orbit * m1 / total_mass;
    
    vec![
        Body::new(
            Vec3::new(-r1, 0.0, 0.0),
            Vec3::new(0.0, v1, 0.0),
            m1,
            0,
        ),
        Body::new(
            Vec3::new(r2, 0.0, 0.0),
            Vec3::new(0.0, -v2, 0.0),
            m2,
            1,
        ),
    ]
}

// Create a random particle cloud
fn create_particle_cloud(num_particles: usize, radius: f32) -> Vec<Body> {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    let mut bodies = Vec::new();

    for i in 0..num_particles {
        // Random position in sphere
        let theta = rng.gen::<f32>() * 2.0 * std::f32::consts::PI;
        let phi = rng.gen::<f32>() * std::f32::consts::PI;
        let r = rng.gen::<f32>() * radius;

        let x = r * phi.sin() * theta.cos();
        let y = r * phi.sin() * theta.sin();
        let z = r * phi.cos();

        bodies.push(Body::new(
            Vec3::new(x, y, z),
            Vec3::ZERO,  // Start at rest
            rng.gen::<f32>() * 2.0 + 0.1,  // Random mass between 0.1 and 2.1
            i,
        ));
    }

    bodies
}

// Main function with simulation loop
pub fn main() {
    // Simulation parameters
    let dt = 0.01;           // Time step
    let theta = 0.5;         // Barnes-Hut threshold
    let softening = 0.1;     // Softening parameter
    let g = 1.0;             // Gravitational constant (using 1.0 for simplicity)
    let num_steps = 1000;    // Number of simulation steps

    // Create initial conditions
    println!("Creating initial conditions...");
    let mut bodies = create_two_body_orbit();
    // Or use a particle cloud:
    // let mut bodies = create_particle_cloud(100, 50.0);

    println!("Starting simulation with {} bodies", bodies.len());
    println!("dt={}, theta={}, softening={}, g={}", dt, theta, softening, g);
    println!();

    // Initial energy
    let (ke0, pe0) = calculate_energy(&bodies, g);
    let total_energy_initial = ke0 + pe0;
    println!("Initial energy: KE={:.6}, PE={:.6}, Total={:.6}", ke0, pe0, total_energy_initial);
    println!();

    // Run simulation
    for step in 0..num_steps {
        update_simulation(&mut bodies, dt, theta, softening, g);

        // Print progress every 100 steps
        if step % 100 == 0 {
            let (ke, pe) = calculate_energy(&bodies, g);
            let total_energy = ke + pe;
            let energy_error = ((total_energy - total_energy_initial) / total_energy_initial * 100.0).abs();
            
            println!("Step {}: KE={:.6}, PE={:.6}, Total={:.6}, Error={:.4}%", 
                     step, ke, pe, total_energy, energy_error);
            
            // Print first body's position
            println!("  Body 0 position: {:.3}, {:.3}, {:.3}", 
                     bodies[0].position.x, bodies[0].position.y, bodies[0].position.z);
        }
    }

    println!();
    println!("Simulation complete!");
    
    // Final energy
    let (ke_f, pe_f) = calculate_energy(&bodies, g);
    let total_energy_final = ke_f + pe_f;
    let total_error = ((total_energy_final - total_energy_initial) / total_energy_initial * 100.0).abs();
    
    println!("Final energy: KE={:.6}, PE={:.6}, Total={:.6}", ke_f, pe_f, total_energy_final);
    println!("Total energy error: {:.4}%", total_error);
}
