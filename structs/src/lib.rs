
#[derive(Clone, Debug)]
pub struct PolygonInfo {
    pub points: Vec<(f64, f64)>,
    pub aisle: Option<i64>,
    pub area: Option<String>,
    pub side: Option<String>,
    pub section: Option<i64>,
    // Optional product id associated with this polygon (if available)
    pub product_id: Option<u64>,
}

#[derive(Debug, Clone, Copy)]
pub struct Pt {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ViewBox {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
}