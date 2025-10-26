use glam::Vec3;

const CAPACITY: usize = 1; // Max bodies per node before subdivision

#[derive(Clone)]
struct Body {
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    mass: f32,
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
        let quarter = self.half_size / 2;
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
    body: Option<usize>,
    children: Option<Box<[OctreeNode; 8]>>,   
} 

impl OctreeNode {

    fn new(boundary: BoundingBox) -> Self {
        OctreeNode { 
            boundary,
            center_of_mass: Vec3::ZERO,
            total_mass: 0.0,
            body: None,
            children: None }
    }

    fn is_leaf(&self) -> bool {
        self.children.is_none()
    }

    fn insert_body(&mut self, new_body: Body) -> bool {
        //Confirm it is within the box
        if !self.boundary.contains(new_body.position) {
            return false
        }

        if self.is_leaf() && self.body.is_none(){
            self.body = new_body;
            
        }

    }

}