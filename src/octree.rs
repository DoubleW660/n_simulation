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

    // Insert body into OctreeNode
    fn insert_body(&mut self, new_body: Body) -> bool {
        //Confirm it is within the box
        if !self.boundary.contains(new_body.position) {
            return false
        }

        // Case 1 Is leaf and No Body
        if self.is_leaf() && self.body.is_none(){
            self.body = Some(new_body.clone());
            self.center_of_mass = new_body.position;
            self.total_mass = new_body.mass;
            return true            
        } 
        
        // Case 2 Is leaf adding body too
        if  self.is_leaf() && self.body.is_some(){
            // Set body to new
            let existing_body = self.body.take().unwrap();
            self.subdivide();

            let octant = self.boundary.get_octant(new_body.position);
            self.children.as_mut().unwrap()[octant].insert_body(existing_body);
        }

        // Case 3 Internal Node with Children
        if !self.is_leaf(){
            let octant = self.boundary.get_octant(new_body.position);
            self.children.as_mut().unwrap()[octant].insert_body(new_body);
        
        }

        self.calc_center_mass();

        true
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
        let s = self.boundary.half_size * 2.0; // Size of the region
        
        if self.is_leaf() || (s / distance < theta) {
            // Treat as single body
            if distance < 0.0001 {
                return Vec3::ZERO; // Avoid division by zero
            }
            
            let distance_sq = distance * distance + softening * softening;
            let force_magnitude = g * body.mass * self.total_mass / distance_sq;
            return r.normalize() * force_magnitude;
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


