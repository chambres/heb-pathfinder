use common_types::PolygonInfo;
use roxmltree::Document;
use tiny_skia::Pixmap;
use tiny_skia::Transform;
use usvg::{Options, Tree};

use serde::Deserialize;

use common_types::{Pt, ViewBox};

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

#[derive(Debug, Clone, Eq)]
pub struct AttrKey {
    pub aisle: Option<String>,
    pub area: Option<String>,
    pub section: Option<String>,
    pub side: Option<String>,
}

// Implement PartialEq and Hash for AttrKey so it can be a HashMap key
impl PartialEq for AttrKey {
    fn eq(&self, other: &Self) -> bool {
        self.aisle == other.aisle
            && self.area == other.area
            && self.section == other.section
            && self.side == other.side
    }
}

impl Hash for AttrKey {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.aisle.hash(state);
        self.area.hash(state);
        self.section.hash(state);
        self.side.hash(state);
    }
}

// helper for flexible key construction
impl AttrKey {
    pub fn from_opt<TA, TB, TC, TD>(
        aisle: Option<TA>,
        area: Option<TB>,
        section: Option<TC>,
        side: Option<TD>,
    ) -> Self
    where
        TA: ToString,
        TB: ToString,
        TC: ToString,
        TD: ToString,
    {
        Self {
            aisle: aisle.map(|v| v.to_string()),
            area: area.map(|v| v.to_string()),
            section: section.map(|v| v.to_string()),
            side: side.map(|v| v.to_string()),
        }
    }
}

// ---- Serde structs ----
#[derive(Debug, Deserialize)]
struct Root {
    data: Vec<Block>,
}

#[derive(Debug, Deserialize)]
struct Block {
    results: Vec<ResultItem>,
}

#[derive(Debug, Deserialize)]
struct ResultItem {
        #[serde(rename = "productId")]
        product_id: Option<u64>,
    psas: Vec<Psa>,      // keep psas as a Vec
    #[serde(rename = "approximateLocation")]
    approximate_location: Option<ApproximateLocation>,
}

#[derive(Debug, Deserialize)]
struct Psa {
    aisle: Option<u32>,
    area: Option<String>,    // "01", "02", ...
    section: Option<u32>,
    side: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ApproximateLocation {
    aisle: Option<u32>,
    area: Option<String>,
    section: Option<u32>,
    side: Option<String>,
}
// ---------------------------




pub fn rasterize_svg_to_png_bytes(svg_str: &str) -> Result<(Vec<u8>, u32, u32), Box<dyn std::error::Error>> {
    let opt = Options::default();
    let tree = Tree::from_str(svg_str, &opt)
        .map_err(|e| format!("Failed to parse SVG: {e}"))?;


    let size = tree.size().to_int_size();
    let mut pixmap = Pixmap::new(size.width(), size.height())
        .ok_or("Failed to create pixmap")?;

    {
        let mut pm = pixmap.as_mut();
        resvg::render(&tree, Transform::identity(), &mut pm);
    }

    let mut png_bytes = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut png_bytes, pixmap.width(), pixmap.height());
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(pixmap.data())?;
    }

    Ok((png_bytes, pixmap.width(), pixmap.height()))
}

pub fn extract_viewbox(svg_string: &str) -> Result<ViewBox, String> {
    let document = Document::parse(svg_string)
        .map_err(|e| format!("XML parse error: {}", e))?;
    let root = document.root_element();

    let view_box_str = root
        .attribute("viewBox")
        .ok_or("Missing viewBox attribute in <svg> element")?;

    let binding = view_box_str.replace(',', " ");
    let parts: Vec<&str> = binding.split_whitespace().collect();

    if parts.len() != 4 {
        return Err(format!(
            "Expected 4 numbers in viewBox, found {}: '{}'",
            parts.len(),
            view_box_str
        ));
    }

    let x = parts[0]
        .parse::<f64>()
        .map_err(|e| format!("Invalid x '{}': {}", parts[0], e))?;
    let y = parts[1]
        .parse::<f64>()
        .map_err(|e| format!("Invalid y '{}': {}", parts[1], e))?;
    let w = parts[2]
        .parse::<f64>()
        .map_err(|e| format!("Invalid width '{}': {}", parts[2], e))?;
    let h = parts[3]
        .parse::<f64>()
        .map_err(|e| format!("Invalid height '{}': {}", parts[3], e))?;

    Ok(ViewBox { x, y, w, h })
}

pub fn svg_to_pixels(point: &Pt, raster_w: f64, raster_h: f64, vb: ViewBox) -> Pt {
    let rx = raster_w / vb.w;
    let ry = raster_h / vb.h;
    let s = rx.min(ry);

    let dx = (raster_w - vb.w * s) / 2.0;
    let dy = (raster_h - vb.h * s) / 2.0;

    Pt {
        x: (point.x - vb.x) * s + dx,
        y: (point.y - vb.y) * s + dy,
    }
}

pub fn remove_combined_fixtures_from_svg(svg_str: &str, polygons: &Vec<PolygonInfo>) -> Result<String, Box<dyn std::error::Error>> {        
    let document = Document::parse(svg_str)?;


    let mut output_xml = String::new();
    output_xml.push_str("<?xml version='1.0' encoding='utf-8'?>
<svg xmlns=\"http://www.w3.org/2000/svg\" baseProfile=\"full\" height=\"100%\" version=\"1.1\" viewBox=\"273.4,-265.06,4160.6,3501.06\" width=\"100%\">"); // Manually add root tag for simplicity

    for node in document.descendants() {
        if !node.is_element() {
            continue;
        }

        // If the node has class="combined-fixture" skip it entirely
        if node.attribute("class").map(|s| s == "combined-fixture").unwrap_or(false) {
            continue;
        }

        // If this is a polygon, check if it matches any polygon in the provided list.
        if node.tag_name().name() == "polygon" {
            let aisle_attr = node.attribute("aisle");
            let area_attr = node.attribute("area");
            let section_attr = node.attribute("section");
            let side_attr = node.attribute("side");

            let aisle_attr_num: Option<i64> = aisle_attr.and_then(|s| s.trim().parse::<i64>().ok());
            let section_attr_num: Option<i64> = section_attr.and_then(|s| s.trim().parse::<i64>().ok());

            // If any polygon in the list matches this node's attributes, skip adding it.
            let mut matched = false;
            for poly in polygons {
                let key_aisle_num = poly.aisle;
                let key_section_num = poly.section;

                let aisle_eq = if key_aisle_num.is_some() || aisle_attr_num.is_some() {
                    key_aisle_num.map(|v| v as i64) == aisle_attr_num
                } else {
                    poly.aisle.is_none() && aisle_attr.is_none() || poly.aisle.is_none() && aisle_attr.is_none()
                };

                // area and side compare as strings (both optional)
                let area_eq = match (&poly.area, area_attr) {
                    (Some(pa), Some(aa)) => pa.as_str() == aa,
                    (None, None) => true,
                    _ => false,
                };

                let side_eq = match (&poly.side, side_attr) {
                    (Some(ps), Some(sa)) => ps.as_str() == sa,
                    (None, None) => true,
                    _ => false,
                };

                let section_eq = if key_section_num.is_some() || section_attr_num.is_some() {
                    key_section_num.map(|v| v as i64) == section_attr_num
                } else {
                    poly.section.is_none() && section_attr.is_none() || poly.section.is_none() && section_attr.is_none()
                };

                if aisle_eq && area_eq && section_eq && side_eq {
                    matched = true;
                    break;
                }
            }

            if matched {
                // skip this polygon node (do not include it in output_xml)
                continue;
            }
        }

        // Start element tag
        output_xml.push_str(&format!("<{}", node.tag_name().name()));
        for attr in node.attributes() {
            output_xml.push_str(&format!(" {}='{}'", attr.name(), attr.value()));
        }
        output_xml.push_str(">");

        // Text content
        if let Some(text) = node.text() {
            output_xml.push_str(text);
        }

        // End element tag (assuming no nested elements for simplicity in this example)
        output_xml.push_str(&format!("</{}>", node.tag_name().name()));
    }
    output_xml.push_str("</svg>"); // Manually add closing root tag



    Ok(output_xml.to_string())
}
fn node_to_polygon_info(
    node: roxmltree::Node,
    raster_w: f64,
    raster_h: f64,
    vb: ViewBox,
    product_id: Option<u64>,
) -> Result<PolygonInfo, Box<dyn std::error::Error>> {
    let points_attr = node
        .attribute("points")
        .ok_or_else(|| "Polygon missing 'points' attribute".to_string())?;
    let raw_pts = parse_points(points_attr)?;

    let mut pts_px = Vec::with_capacity(raw_pts.len());
    for p in raw_pts {
        let px = svg_to_pixels(&p, raster_w, raster_h, vb);
        pts_px.push((px.x, px.y));
    }

    // Convert numeric attrs where possible
    let aisle_num = node.attribute("aisle").and_then(|v| v.parse::<i64>().ok());
    let section_num = node.attribute("section").and_then(|v| v.parse::<i64>().ok());

    Ok(PolygonInfo {
        points: pts_px,
        aisle: aisle_num,
        area: node.attribute("area").map(|s| s.to_string()),
        section: section_num,
        side: node.attribute("side").map(|s| s.to_string()),
        product_id, // <<— now settable
    })
}

fn polygon_title_text<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
) -> Option<&'input str>
where
    'a: 'input,
{
    for c in node.children() {
        if c.is_element() && c.tag_name().name() == "title" {
            if let Some(t) = c.text() {
                return Some(t.trim());
            }
        }
    }
    None
}

pub fn polygons_to_pixels(
    svg_str: &str,
    raster_w: f64,
    raster_h: f64,
    item_json: &serde_json::Value,
) -> Result<Vec<PolygonInfo>, Box<dyn std::error::Error>> {
    let polygons = create_hashmap(svg_str)?;
    let viewbox = extract_viewbox(svg_str)?;

    // ---- Parse JSON into typed structs ----
    let root: Root = serde_json::from_value(item_json.clone())?;

    // ---- Scan the SVG once for special polygons and add them with product IDs ----
    let doc = roxmltree::Document::parse(svg_str)?;
    let mut out: Vec<PolygonInfo> = Vec::new();
    let mut found_special_predef = false;
    let mut found_special_title = false;

    for node in doc.descendants() {
        if !(node.is_element() && node.tag_name().name() == "polygon") {
            continue;
        }

        // Special by attributes -> product_id = 1
        if !found_special_predef
            && node.attribute("area") == Some("2000")
            && node.attribute("aisle") == Some("2000")
            && node.attribute("side") == Some("A")
            && node.attribute("section") == Some("2000")
        {
            let p = node_to_polygon_info(node, raster_w, raster_h, viewbox, Some(1))?;
            out.push(p);
            found_special_predef = true;
        }

        // Special by <title> text -> product_id = 2
        if !found_special_title {
            if let Some(t) = polygon_title_text(node) {
                if t == "CHECKSTAND MERCHANDISER SELF CHECK OUT" {
                    let p = node_to_polygon_info(node, raster_w, raster_h, viewbox, Some(2))?;
                    out.push(p);
                    found_special_title = true;
                }
            }
        }

        if found_special_predef && found_special_title {
            break; // perf: stop once both specials are found
        }
    }

    // ---- Main product-driven polygons (unchanged logic, just push after specials) ----
    for block in root.data {
        for result in block.results {
            let (aisle_opt, area_opt, section_opt, side_opt) = if !result.psas.is_empty() {
                let psa = &result.psas[0];
                (psa.aisle, psa.area.clone(), psa.section, psa.side.clone())
            } else if let Some(al) = result.approximate_location {
                (al.aisle, al.area, al.section, al.side)
            } else {
                continue;
            };

            let key = AttrKey::from_opt(
                aisle_opt.clone(),
                area_opt.clone(),
                section_opt.clone(),
                side_opt.clone(),
            );

            if let Some(points) = polygons.get(&key) {
                let mut pts_px = Vec::with_capacity(points.len());
                for p in points {
                    let px = svg_to_pixels(&p, raster_w, raster_h, viewbox);
                    pts_px.push((px.x, px.y));
                }

                out.push(PolygonInfo {
                    points: pts_px,
                    aisle: aisle_opt.map(|v| v as i64),
                    area: area_opt.clone(),
                    section: section_opt.map(|v| v as i64),
                    side: side_opt.clone(),
                    product_id: result.product_id, // from data
                });
            }
        }
    }

    Ok(out)
}

//------------------


fn parse_points(points_str: &str) -> Result<Vec<Pt>, String> {
    let binding = points_str.replace(',', " ");
    let tokens: Vec<&str> = binding.split_whitespace().collect();

    if tokens.len() % 2 != 0 {
        return Err(format!(
            "Invalid points list (odd token count {}): '{}'",
            tokens.len(),
            points_str
        ));
    }

    let mut out = Vec::with_capacity(tokens.len() / 2);
    for pair in tokens.chunks_exact(2) {
        let x = pair[0]
            .parse::<f64>()
            .map_err(|e| format!("Invalid x '{}': {}", pair[0], e))?;
        let y = pair[1]
            .parse::<f64>()
            .map_err(|e| format!("Invalid y '{}': {}", pair[1], e))?;
        out.push(Pt { x, y });
    }
    Ok(out)
}

pub fn create_hashmap(
    svg_string: &str,
) -> Result<HashMap<AttrKey, Vec<Pt>>, String> {
    let doc = Document::parse(svg_string).map_err(|e| format!("XML parse error: {}", e))?;
    let mut map: HashMap<AttrKey, Vec<Pt>> = HashMap::new();

    for node in doc
        .descendants()
        .filter(|n| n.is_element() && n.tag_name().name() == "polygon")
    {
        let points_attr = node
            .attribute("points")
            .ok_or_else(|| "Polygon missing 'points' attribute".to_string())?;

        let points = parse_points(points_attr)?;

        let key = AttrKey {
            aisle: node.attribute("aisle").map(|s| s.to_string()),
            area: node.attribute("area").map(|s| s.to_string()),
            section: node.attribute("section").map(|s| s.to_string()),
            side: node.attribute("side").map(|s| s.to_string()),
        };

        map.entry(key).or_default().extend(points);
    }

    Ok(map)
}
