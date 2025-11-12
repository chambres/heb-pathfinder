#[cfg(not(target_arch = "wasm32"))]
use arboard::Clipboard;


use pathfinding::{polyline_to_svg_points, run_pathfinding_given_polygons_return_path};
use serde_json::Value;
use std::{error::Error, path::Path};
use simplelog::{SimpleLogger, LevelFilter, Config};
use log::{info, debug}; 
//use common_types::PolygonInfo;

use svg_tools::{extract_viewbox, polygons_to_pixels, rasterize_svg_to_png_bytes, remove_combined_fixtures_from_svg};



fn main() -> Result<(), Box<dyn Error>> {
    SimpleLogger::init(LevelFilter::Info, Config::default()).unwrap();

    // read sample_output.json and put its contents in the clipboard (if it exists)
    let json_path = Path::new("C:\\Users\\rahul\\Documents\\prog\\python\\heb\\sample_output.json");
    if json_path.exists() {
        info!("Reading sample_output.json");
        let json_string: String = std::fs::read_to_string(json_path)?;
        info!("Setting clipboard text to sample_output.json contents");
        write_to_clipboard(&json_string)?;
    } else {
        info!("sample_output.json not found — skipping clipboard setup");
    }



    //READ CLIPBOARD ====================================================

    info!("Reading clipboard text");
    let data: String = read_from_clipboard()?;
    
    debug!("Clipboard text was: {}", data);

    match serde_json::from_str::<Value>(&data) {
        Ok(_v) => {}
        Err(e) => {
            println!("Failed to parse clipboard contents as JSON: {}", e);
            return Ok(());
        }
    }

    
    //READ JSON INTO OBJ ================================================

    info!("Parsing clipboard text as JSON");
    let v: Value = serde_json::from_str(&data)?;

    //SVG FROM JSON ================================================
    info!("Extracting `svg` string from JSON");
    let svg_str = v.get("svg").and_then(|x| x.as_str()).ok_or("missing svg")?;         

    //GET DIMENSIONS FROM SVG VIEWBOX ======================================
    info!("Extracting viewbox dimensions from SVG string");
    let viewbox = extract_viewbox(&svg_str)?;
    let (w, h) = (viewbox.w, viewbox.h);

    //FIND POLYGONS =======================================================
    info!("Finding polygons in SVG string");
    let polys = polygons_to_pixels(&svg_str, w, h, &v)?;
    info!("Found {} polygons", polys.len());
    let edited_str = remove_combined_fixtures_from_svg(&svg_str, &polys)?;


    //CONVERT SVG TO PNG ================================================
    info!("Rasterizing SVG to PNG bytes");
    let (png_bytes, _, _) = rasterize_svg_to_png_bytes(&edited_str)?;



    //PATHFINDING ============================================================
    info!("Running pathfinding algorithm on PNG bytes and polygon data");
    //let png_output = run_pathfinding_given_polygons_as_bytes(&png_bytes, polys.clone())?;

    let (polyline, mask) = run_pathfinding_given_polygons_return_path(&png_bytes, polys)?;

    if polyline.is_empty() {
      println!("No path produced (not enough reachable waypoints).");
      return Ok(());
    }

    // Print polyline and a parallel boolean mask (which points correspond to products)
    println!("Polyline: {}", polyline_to_svg_points(&polyline));
    println!("Mask: {:?}", mask);





    Ok(())
}


//clipboard functions
#[cfg(not(target_arch = "wasm32"))]
fn write_to_clipboard(text: &str) -> Result<(), Box<dyn Error>> {
    let mut clipboard = Clipboard::new()?;
    clipboard.set_text(text)?;
    Ok(())
}


#[cfg(not(target_arch = "wasm32"))]
fn read_from_clipboard() -> Result<String, Box<dyn Error>> {
    let mut clipboard = Clipboard::new()?;
    let text = clipboard.get_text()?;
    Ok(text)
}

#[cfg(target_arch = "wasm32")]
fn read_from_clipboard() -> Result<String, Box<dyn Error>> {
    Ok((r#"{}"#).to_string())
}
#[cfg(target_os = "wasi")]
fn write_to_clipboard(text: &str) -> Result<(), Box<dyn Error>> {
    Ok(())
}
