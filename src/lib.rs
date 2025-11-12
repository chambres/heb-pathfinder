use wasm_bindgen::prelude::*;
use serde_json::Value;
use web_sys::console;
use svg_tools::{rasterize_svg_to_png_bytes, polygons_to_pixels};
use pathfinding::{run_pathfinding_given_polygons_return_path, polyline_to_svg_points};




#[wasm_bindgen(start)]
pub fn start() {
    // Better panic messages in dev builds
    #[cfg(debug_assertions)]
    console_error_panic_hook::set_once();
    console::log_1(&"heb_scout wasm loaded".into());
}

/// Entry point you call from JS:
/// - `data` = the big JSON string you pasted (contains `data` array and `svg`)
/// Returns a base64-encoded PNG (string) you can set on an <img>.
#[wasm_bindgen]
pub fn heb_entry(data: String) -> Result<String, JsValue> {
    // Parse JSON (quick validation first)
    let v: Value = serde_json::from_str(&data)
        .map_err(|e| JsValue::from_str(&format!("invalid JSON: {e}")))?;

    // Extract `svg`
    let svg_str = v.get("svg")
        .and_then(|x| x.as_str())
        .ok_or_else(|| JsValue::from_str("missing `svg` string in payload"))?;

    // Rasterize SVG -> PNG bytes
    let (png_bytes, w, h) = rasterize_svg_to_png_bytes(svg_str)
        .map_err(|e| JsValue::from_str(&format!("rasterize_svg_to_png_bytes failed: {e}")))?;

    let (fw, fh) = (w as f64, h as f64);

    // Compute polygons (in pixel space). We also receive the modified SVG string
    // but in wasm we can't write to disk, so we ignore it here.
    let polys = polygons_to_pixels(svg_str, fw, fh, &v)
        .map_err(|e| JsValue::from_str(&format!("polygons_to_pixels failed: {e}")))?;

    // Run your pathfinding overlay / logic; assume it mutates/produces an overlay in-memory.
    // If your function currently writes to disk, refactor it to *return* bytes instead.
    // Here we assume it returns Vec<u8> PNG; adjust according to your actual signature.
    let (polyline, mask) = run_pathfinding_given_polygons_return_path(&png_bytes, polys)
        .map_err(|e| JsValue::from_str(&format!("pathfinding failed: {e}")))?;

    let poly_s = polyline_to_svg_points(&polyline);
    // Build a small JSON result { polyline: "x1,y1 x2,y2 ...", mask: [true,false,...] }
    let result = serde_json::json!({ "polyline": poly_s, "mask": mask });
    Ok(result.to_string())
}
