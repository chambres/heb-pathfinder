use image::{GrayImage, Rgba, RgbaImage, GenericImageView};
use imageproc::drawing::{draw_filled_circle_mut, draw_filled_rect_mut, draw_line_segment_mut};
use imageproc::rect::Rect;
use std::cmp::{max, min};
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};
use std::path::Path;

use image::codecs::png::PngEncoder;
use image::ImageEncoder;

use common_types::PolygonInfo;

/// Public settings to run the pipeline
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub k: i32,          // coarse cell size
    pub thresh: f32,     // 0..1 required free ratio per supercell
    pub inflate: i32,    // obstacle inflation in coarse cells
    pub connect8: bool,  // 8-connected vs 4-connected A*
    pub start_px: Pt,    // pixel-space start (y,x)
    pub goal_px: Pt,     // pixel-space goal (y,x)
}

/// A coarse cell (row=i, col=j)
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Cell {
    pub i: i32,
    pub j: i32,
}

/// Pixel point (row=y, col=x)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt {
    pub y: i32,
    pub x: i32,
}

/// Outcome with some useful artifacts
#[derive(Debug)]
pub struct PathOutcome {
    pub image_w: u32,
    pub image_h: u32,
    pub coarse_w: i32,
    pub coarse_h: i32,
    pub coarse_path_cells: Vec<Cell>,
    pub coarse_centers_px: Vec<Pt>,
    pub smoothed_px: Vec<Pt>,
    pub saved_overlay_path: Option<String>,
}

#[inline]
fn clamp_i32(v: i32, lo: i32, hi: i32) -> i32 {
    v.max(lo).min(hi)
}

#[inline]
fn hypot(a: f32, b: f32) -> f32 {
    (a * a + b * b).sqrt()
}

#[inline]
fn pack_key(i: i32, j: i32) -> u64 {
    ((i as i64 as u64) << 32) | ((j as i64 as u64) & 0xFFFF_FFFF)
}

#[inline]
fn unpack_key(k: u64) -> (i32, i32) {
    let i = (k >> 32) as i32;
    let j = (k & 0xFFFF_FFFF) as i32;
    (i, j)
}

/// Route result for multi-waypoint problems
#[derive(Debug)]
pub struct MultiRouteOutcome {
    pub order: Vec<usize>,      // indices into input waypoints in visiting order
    pub total_length: f32,      // sum of segment lengths (in pixels)
    pub polyline: Vec<Pt>,      // stitched pixel path
    pub saved_overlay_path: Option<String>,
}

/// New: Render + path return for multi-waypoint problems
#[derive(Debug)]
pub struct MultiRouteRender {
    pub order: Vec<usize>,     // TSP visiting order (indices into given waypoints)
    pub total_length: f32,     // length of 'polyline' in pixels
    pub polyline: Vec<Pt>,     // the stitched pixel polyline you want
    /// A boolean mask, same length as `polyline`, where `true` marks a point
    /// that corresponds to an actual waypoint/product (the rest are path points).
    /// Instead of booleans, hold the product id (u64) for product points; 0 means not a product
    pub product_point_mask: Vec<u64>,
    pub overlay_png: Vec<u8>,  // overlay PNG bytes (may be empty if not rendered)
    pub saved_overlay_path: Option<String>,
}

/// Euclidean length of a pixel polyline
pub fn polyline_length(pts: &[Pt]) -> f32 {
    let mut sum = 0.0f32;
    for w in pts.windows(2) {
        let a = w[0]; let b = w[1];
        sum += hypot((b.x - a.x) as f32, (b.y - a.y) as f32);
    }
    sum
}

/// Compute a smoothed pixel path between two pixel points using an existing coarse grid
fn path_between_pixels(
    free: &[u8], w: u32, h: u32,
    coarse: &[u8], wc: i32, hc: i32, k: i32, connect8: bool,
    start_px: Pt, goal_px: Pt,
) -> Option<Vec<Pt>> {
    let mut s = Cell { i: start_px.y / k, j: start_px.x / k };
    let mut g = Cell { i: goal_px.y  / k, j: goal_px.x  / k };
    if let Some(ns) = nudge_to_free(s, coarse, wc, hc) { s = ns; }
    if let Some(ng) = nudge_to_free(g, coarse, wc, hc) { g = ng; }

    let cells = astar_path(coarse, wc, hc, connect8, s, g)?;
    if cells.is_empty() { return None; }
    let px_path = coarse_to_pixel_centers(&cells, k);
    let smooth = smooth_path(free, w, h, &px_path);
    Some(smooth)
}

/// Solve TSP order (open path or cycle) via Held–Karp DP (exact) for n ≤ 12.
/// Returns visiting order indices (0..n-1).
fn tsp_held_karp(dist: &Vec<Vec<f32>>, make_cycle: bool) -> Vec<usize> {
    let n = dist.len();
    // anchor at 0 to reduce symmetry
    let full = 1u32 << n;
    let mut dp: Vec<HashMap<usize, (f32, usize)>> = vec![HashMap::new(); full as usize];
    dp[1].insert(0, (0.0, usize::MAX)); // mask with only 0

    for mask in 1..full {
        if (mask & 1) == 0 { continue; } // ensure 0 is in the set
        for last in 0..n {
            if (mask & (1 << last)) == 0 { continue; }
            if let Some(&(cost, _)) = dp[mask as usize].get(&last) {
                for next in 0..n {
                    if (mask & (1 << next)) != 0 { continue; }
                    let new_mask = mask | (1 << next);
                    let new_cost = cost + dist[last][next];
                    let e = dp[new_mask as usize].entry(next).or_insert((f32::INFINITY, usize::MAX));
                    if new_cost < e.0 { *e = (new_cost, last); }
                }
            }
        }
    }

    // Finish
    let last_mask = (full - 1) as usize;
    let (best_last, _best_cost) = if make_cycle {
        // cycle back to 0
        let mut best = (0usize, f32::INFINITY);
        for last in 1..n {
            if let Some(&(cost, _)) = dp[last_mask].get(&last) {
                let total = cost + dist[last][0];
                if total < best.1 { best = (last, total); }
            }
        }
        best
    } else {
        // open path: end anywhere (min cost)
        let mut best = (0usize, f32::INFINITY);
        for last in 0..n {
            if let Some(&(cost, _)) = dp[last_mask].get(&last) {
                if cost < best.1 { best = (last, cost); }
            }
        }
        best
    };

    // Reconstruct
    let mut order = Vec::<usize>::with_capacity(n + make_cycle as usize);
    let mut mask = (full - 1) as usize;
    let mut last = best_last;
    while last != usize::MAX {
        order.push(last);
        let prev = dp[mask].get(&last).unwrap().1;
        if prev == usize::MAX { break; }
        mask &= !(1 << last);
        last = prev;
    }
    order.push(0);
    order.reverse();
    if make_cycle { order.push(0); }
    order
}

/// Simple nearest-neighbor + 2-opt heuristic for n > 12
fn tsp_heuristic(dist: &Vec<Vec<f32>>, make_cycle: bool) -> Vec<usize> {
    let n = dist.len();
    let mut used = vec![false; n];
    let mut order = Vec::<usize>::with_capacity(n + make_cycle as usize);
    let mut cur = 0usize;
    used[cur] = true;
    order.push(cur);

    for _ in 1..n {
        let mut best = None;
        for j in 0..n {
            if !used[j] {
                let c = dist[cur][j];
                if best.map_or(true, |(_bj, bc)| c < bc) {
                    best = Some((j, c));
                }
            }
        }
        let (nx, _) = best.unwrap();
        used[nx] = true;
        order.push(nx);
        cur = nx;
    }
    if make_cycle { order.push(0); }

    // 2-opt improvement (limited)
    let m = if make_cycle { order.len() - 1 } else { order.len() };
    let mut improved = true;
    while improved {
        improved = false;
        for i in 1..m.saturating_sub(2) {
            for k in (i + 1)..m.saturating_sub(1) {
                let a = order[i - 1]; let b = order[i];
                let c = order[k];     let d = order[(k + 1) % order.len()];
                let delta = dist[a][c] + dist[b][d] - dist[a][b] - dist[c][d];
                if delta < -1e-4 {
                    order[i..=k].reverse();
                    improved = true;
                }
            }
        }
    }
    order
}


/// Exact open-path TSP with fixed start/end using Held–Karp (n <= ~12).
/// The route starts at index 0 and ends at index (n-1) in `dist`.
fn tsp_fixed_ends_exact(dist: &Vec<Vec<f32>>) -> Vec<usize> {
    let n = dist.len();
    assert!(n >= 2);

    // Middle nodes are 1..(n-2) (if any)
    let m = if n >= 2 { (n - 2) as usize } else { 0 };
    let full_mask = if m == 0 { 0usize } else { (1usize << m) - 1 };

    // dp[mask][last_mid] = (cost, parent_last_mid)
    // "last_mid" is an index in 0..m-1 referring to the actual node (1 + last_mid)
    let mut dp: Vec<HashMap<usize, (f32, Option<usize>)>> = vec![HashMap::new(); 1usize << m];

    // Base: from start(0) directly to each middle node
    for mid in 0..m {
        let node = 1 + mid;
        dp[1 << mid].insert(mid, (dist[0][node], None));
    }

    if m == 0 {
        // No middle nodes; path is simply 0 -> n-1
        return vec![0, n - 1];
    }

    // Transitions over subsets of middle nodes
    for mask in 1usize..=full_mask {
        for &last_mid in dp[mask].keys().cloned().collect::<Vec<_>>().iter() {
            let (cost, _) = dp[mask][&last_mid];
            let last_node = 1 + last_mid;
            // try to add a new middle node 'nx'
            for nx in 0..m {
                if (mask & (1 << nx)) != 0 { continue; }
                let nx_node = 1 + nx;
                let new_mask = mask | (1 << nx);
                let new_cost = cost + dist[last_node][nx_node];
                let e = dp[new_mask].entry(nx).or_insert((f32::INFINITY, None));
                if new_cost < e.0 {
                    *e = (new_cost, Some(last_mid));
                }
            }
        }
    }

    // Finish at end node (n-1)
    let end = n - 1;
    let mut best_last_mid = 0usize;
    let mut best_total = f32::INFINITY;
    for (&last_mid, &(cost, _)) in dp[full_mask].iter() {
        let last_node = 1 + last_mid;
        let total = cost + dist[last_node][end];
        if total < best_total {
            best_total = total;
            best_last_mid = last_mid;
        }
    }

    // Reconstruct middle sequence
    let mut order_mids: Vec<usize> = Vec::with_capacity(m);
    let mut mask = full_mask;
    let mut cur = best_last_mid;
    loop {
        order_mids.push(cur);
        if let Some((_, parent)) = dp[mask].get(&cur) {
            match *parent {
                Some(p) => {
                    mask &= !(1 << cur);
                    cur = p;
                }
                None => break,
            }
        } else {
            break;
        }
    }
    order_mids.reverse();

    // Build full order in the reindexed space [0 .. n-1]
    let mut order = Vec::with_capacity(n);
    order.push(0);
    for mid in order_mids {
        order.push(1 + mid);
    }
    order.push(end);
    order
}

/// Heuristic open-path with fixed start/end (start=0, end=n-1).
/// Nearest-neighbor over middle nodes, then a constrained 2-opt that keeps endpoints fixed.
fn tsp_fixed_ends_heuristic(dist: &Vec<Vec<f32>>) -> Vec<usize> {
    let n = dist.len();
    assert!(n >= 2);
    if n == 2 {
        return vec![0, 1];
    }

    // Middle nodes: 1..(n-2)
    let mut unused: Vec<usize> = (1..(n - 1)).collect();
    let mut cur = 0usize;
    let mut seq: Vec<usize> = vec![0];

    while !unused.is_empty() {
        // Do not pick the final end (n-1) until the end.
        let mut best = None;
        for &j in &unused {
            let c = dist[cur][j];
            if best.map_or(true, |(_, bc)| c < bc) {
                best = Some((j, c));
            }
        }
        let (nx, _) = best.unwrap();
        seq.push(nx);
        cur = nx;
        unused.retain(|&u| u != nx);
    }

    // Append the fixed end
    seq.push(n - 1);

    // Constrained 2-opt: keep first (0) and last (n-1) fixed
    let m = seq.len();
    let mut improved = true;
    while improved {
        improved = false;
        for i in 1..(m - 3) {
            for k in (i + 1)..(m - 2) {
                let a = seq[i - 1];
                let b = seq[i];
                let c = seq[k];
                let d = seq[k + 1];
                let delta = dist[a][c] + dist[b][d] - dist[a][b] - dist[c][d];
                if delta < -1e-4 {
                    seq[i..=k].reverse();
                    improved = true;
                }
            }
        }
    }

    seq
}


/// New: end-to-end route that RETURNS the polyline (and overlay PNG bytes)
pub fn run_multi_route_render_from_png_bytes(
    bytes: &[u8],
    cfg: Settings,
    waypoints: Vec<Pt>,
    product_ids: Vec<u64>,
    make_cycle: bool,
    overlay_out_path: Option<&str>,
) -> Result<MultiRouteRender, String> {
    // Decode to gray + free[]
    let dyn_img = image::load_from_memory(bytes).map_err(|e| format!("image decode: {e}"))?;
    let gray: GrayImage = dyn_img.to_luma8();
    let (w, h) = gray.dimensions();

    let mut free = vec![0u8; (w * h) as usize];
    for y in 0..h {
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            free[(y * w + x) as usize] = if v > 127 { 1 } else { 0 };
        }
    }

    // Coarse once
    let (coarse0, wc, hc) = build_coarse_occupancy(&free, w, h, cfg.k, cfg.thresh);
    let coarse = inflate_obstacles(coarse0, wc, hc, cfg.inflate);

    // Trivial cases
    if waypoints.len() <= 1 {
        // No real route; return the points (if any) with empty png
        return Ok(MultiRouteRender {
            order: (0..waypoints.len()).collect(),
            total_length: 0.0,
            polyline: waypoints.clone(),
            product_point_mask: waypoints.iter().enumerate().map(|(i, _)| *product_ids.get(i).unwrap_or(&0u64)).collect(),
            overlay_png: Vec::<u8>::new(),
            saved_overlay_path: overlay_out_path.map(|s| s.to_string()),
        });
    }

    // Pairwise shortest paths (pixel polylines) and distances
    let n = waypoints.len();
    let mut paths: Vec<Vec<Option<Vec<Pt>>>> = vec![vec![None; n]; n];
    let mut dist = vec![vec![f32::INFINITY; n]; n];

    for i in 0..n {
        for j in (i + 1)..n {
            let p = path_between_pixels(
                &free, w, h, &coarse, wc, hc, cfg.k, cfg.connect8,
                waypoints[i], waypoints[j],
            ).ok_or_else(|| format!("No path between point {} and {}", i, j))?;
            let d = polyline_length(&p);
            dist[i][j] = d;
            dist[j][i] = d;
            paths[i][j] = Some(p.clone());
            let mut rp = p.clone(); rp.reverse();
            paths[j][i] = Some(rp);
        }
        dist[i][i] = 0.0;
        paths[i][i] = Some(vec![waypoints[i]]);
    }

    let mut order: Vec<usize>;
    let has_fixed = !make_cycle
        && product_ids.iter().any(|&p| p == 1)
        && product_ids.iter().any(|&p| p == 2);

    if has_fixed {
        // Find the unique indices for product_id == 1 (start) and == 2 (end)
        let s_idx = product_ids.iter().position(|&p| p == 1).unwrap();
        let e_idx = product_ids.iter().position(|&p| p == 2).unwrap();

        // Reindex nodes so start -> 0, end -> n-1, others in between
        let n = dist.len();
        let mut map_new_to_old = Vec::<usize>::with_capacity(n);
        map_new_to_old.push(s_idx);
        for i in 0..n {
            if i != s_idx && i != e_idx {
                map_new_to_old.push(i);
            }
        }
        map_new_to_old.push(e_idx);

        // Build reindexed distance matrix
        let mut dist2 = vec![vec![0.0f32; n]; n];
        for ni in 0..n {
            for nj in 0..n {
                dist2[ni][nj] = dist[map_new_to_old[ni]][map_new_to_old[nj]];
            }
        }

        // Solve with fixed endpoints
        let order2 = if n <= 12 {
            tsp_fixed_ends_exact(&dist2)
        } else {
            tsp_fixed_ends_heuristic(&dist2)
        };

        // Map back to original indices
        order = order2.into_iter().map(|ni| map_new_to_old[ni]).collect();
    } else {
        // Original behavior
        order = if n <= 12 {
            tsp_held_karp(&dist, make_cycle)
        } else {
            tsp_heuristic(&dist, make_cycle)
        };
    }

    // Stitch polyline
    let mut full: Vec<Pt> = vec![];
    let mut total = 0.0f32;
    // Also build a parallel boolean mask marking product/waypoint points in `full`.
    let mut product_mask: Vec<u64> = Vec::new();
    for idx in 0..(order.len() - 1) {
        let a = order[idx];
        let b = order[idx + 1];
        let seg = paths[a][b].as_ref().unwrap();
        if full.is_empty() {
            // first segment: append entire segment
            let start_len = full.len();
            full.extend_from_slice(seg);
            // extend mask with zeros for each added point
            product_mask.extend(std::iter::repeat(0u64).take(seg.len()));
            // mark the segment endpoints with product ids: waypoint a (first) and waypoint b (last)
            if seg.len() >= 1 {
                product_mask[start_len] = *product_ids.get(a).unwrap_or(&0u64);
                product_mask[start_len + seg.len() - 1] = *product_ids.get(b).unwrap_or(&0u64);
            }
        } else {
            // subsequent segments: skip duplicate join point at seg[0]
            let start_index = full.len();
            if seg.len() > 1 {
                full.extend_from_slice(&seg[1..]);
                let added = seg.len() - 1;
                product_mask.extend(std::iter::repeat(0u64).take(added));
                // mark the end of this segment (waypoint b) with its product id
                product_mask[start_index + added - 1] = *product_ids.get(b).unwrap_or(&0u64);
            }
        }
        total += dist[a][b];
    }

    // product_mask was built during stitching above; ensure its length matches `full`.
    // product_mask was built during stitching above; ensure its length matches `full`.
    if product_mask.len() != full.len() {
        // Fallback: recreate mask conservatively (mark any exact waypoint matches).
        let mut fallback = vec![0u64; full.len()];
        for (i, wp) in waypoints.iter().enumerate() {
            if let Some(pos) = full.iter().position(|p| p.x == wp.x && p.y == wp.y) {
                fallback[pos] = *product_ids.get(i).unwrap_or(&0u64);
            }
        }
        product_mask = fallback;
    }

    // // >>> NEW: force first/last mask values to 1 and 2 respectively
    // if !full.is_empty() {
    //     product_mask[0] = 1;
    //     if full.len() > 1 {
    //         product_mask[full.len() - 1] = 2;
    //     }
    // }




    // Optional overlay save (draws the waypoints and route) — returns PNG bytes
    let png_bytes = visualize_and_optionally_save(
        &gray,
        &coarse,
        wc,
        hc,
        cfg.k,
        &full,        // draw the stitched path
        &[],          // no extra overlay
        overlay_out_path,
    )?;

    Ok(MultiRouteRender {
        order,
        total_length: total,
        polyline: full,
        product_point_mask: product_mask,
        overlay_png: png_bytes,
        saved_overlay_path: overlay_out_path.map(|s| s.to_string()),
    })
}

/// Backward-compatible: old function now delegates to the new renderer and returns only PNG bytes.
pub fn run_multi_route_from_png_bytes(
    bytes: &[u8],
    cfg: Settings,
    waypoints: Vec<Pt>,
    make_cycle: bool,
    overlay_out_path: Option<&str>,
) -> Result<Vec<u8>, String> {
    // Build a placeholder product_ids vector (zeros) since caller didn't provide product ids
    let product_ids = vec![0u64; waypoints.len()];
    let res = run_multi_route_render_from_png_bytes(bytes, cfg, waypoints, product_ids, make_cycle, overlay_out_path)?;
    Ok(res.overlay_png)
}

/// Load an image (any format `image` supports) and binarize to free/blocked.
/// free[y*w + x] = 1 if >127 else 0
pub fn load_and_binarize<P: AsRef<Path>>(img_path: P) -> Result<(GrayImage, Vec<u8>), String> {
    let img = image::open(&img_path).map_err(|e| e.to_string())?;
    let gray = img.to_luma8();
    let (w, h) = gray.dimensions();

    let mut free = vec![0u8; (w * h) as usize];
    for y in 0..h {
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            free[(y * w + x) as usize] = if v > 127 { 1 } else { 0 };
        }
    }
    Ok((gray, free))
}

/// Build coarse occupancy (1=free supercell, 0=blocked)
pub fn build_coarse_occupancy(
    free: &[u8],
    img_w: u32,
    img_h: u32,
    k: i32,
    thresh: f32,
) -> (Vec<u8>, i32, i32) {
    let w_i = img_w as i32;
    let h_i = img_h as i32;
    let hc = h_i / k;
    let wc = w_i / k;

    let mut coarse = vec![0u8; (hc * wc) as usize];

    let supercell_walkable = |ci: i32, cj: i32| -> bool {
        let y0 = ci * k;
        let x0 = cj * k;
        if y0 < 0 || x0 < 0 || y0 + k > h_i || x0 + k > w_i {
            return false;
        }
        let mut sum: u32 = 0;
        for yy in y0..(y0 + k) {
            let base = yy as u32 * img_w;
            for xx in x0..(x0 + k) {
                sum += free[(base + xx as u32) as usize] as u32;
            }
        }
        let total = (k * k) as u32;
        (sum as f32 / total as f32) >= thresh
    };

    for i in 0..hc {
        for j in 0..wc {
            let idx = (i * wc + j) as usize;
            coarse[idx] = if supercell_walkable(i, j) { 1 } else { 0 };
        }
    }

    (coarse, wc, hc)
}

/// Inflate obstacles in coarse grid
pub fn inflate_obstacles(coarse: Vec<u8>, wc: i32, hc: i32, inflate: i32) -> Vec<u8> {
    if inflate <= 0 {
        return coarse;
    }
    let mut inflated = coarse.clone();
    for i in 0..hc {
        for j in 0..wc {
            if coarse[(i * wc + j) as usize] == 0 {
                let i0 = max(0, i - inflate);
                let i1 = min(hc - 1, i + inflate);
                let j0 = max(0, j - inflate);
                let j1 = min(wc - 1, j + inflate);
                for ii in i0..=i1 {
                    for jj in j0..=j1 {
                        inflated[(ii * wc + jj) as usize] = 0;
                    }
                }
            }
        }
    }
    inflated
}

/// Nudge a coarse cell to the nearest free coarse cell
pub fn nudge_to_free(cell: Cell, coarse: &[u8], wc: i32, hc: i32) -> Option<Cell> {
    let inside = |i: i32, j: i32| -> bool { i >= 0 && i < hc && j >= 0 && j < wc };

    if inside(cell.i, cell.j) && coarse[(cell.i * wc + cell.j) as usize] == 1 {
        return Some(cell);
    }

    let mut q = VecDeque::new();
    let mut seen = HashSet::new();
    q.push_back(cell);
    seen.insert(cell);

    while let Some(c) = q.pop_front() {
        for di in -1..=1 {
            for dj in -1..=1 {
                if di == 0 && dj == 0 {
                    continue;
                }
                let ni = c.i + di;
                let nj = c.j + dj;
                let nc = Cell { i: ni, j: nj };
                if !inside(ni, nj) || seen.contains(&nc) {
                    continue;
                }
                if coarse[(ni * wc + nj) as usize] == 1 {
                    return Some(nc);
                }
                seen.insert(nc);
                q.push_back(nc);
            }
        }
    }
    None
}

/// A* on coarse grid (returns cells from start to goal)
pub fn astar_path(
    coarse: &[u8],
    wc: i32,
    hc: i32,
    connect8: bool,
    start: Cell,
    goal: Cell,
) -> Option<Vec<Cell>> {
    let neighbors = |i: i32, j: i32| -> Vec<(i32, i32, f32)> {
        let dirs8 = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ];
        let dirs4 = [(-1, 0), (0, -1), (0, 1), (1, 0)];
        let dirs = if connect8 { &dirs8[..] } else { &dirs4[..] };
        let mut out = Vec::with_capacity(dirs.len());
        for (di, dj) in dirs.iter().copied() {
            let ni = i + di;
            let nj = j + dj;
            if ni >= 0 && ni < hc && nj >= 0 && nj < wc && coarse[(ni * wc + nj) as usize] == 1 {
                let diag = (2.0_f32).sqrt();
                let cost = if di != 0 && dj != 0 { diag } else { 1.0 };
                out.push((ni, nj, cost));
            }
        }
        out
    };

    let hfun = |a: Cell, b: Cell| -> f32 {
        hypot((a.i - b.i) as f32, (a.j - b.j) as f32)
    };

    #[derive(Clone)]
    struct Node {
        f: f32,
        g: f32,
        i: i32,
        j: i32,
        parent_key: Option<u64>,
    }
    impl PartialEq for Node {
        fn eq(&self, other: &Self) -> bool {
            self.f == other.f && self.i == other.i && self.j == other.j
        }
    }
    impl Eq for Node {}
    impl PartialOrd for Node {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            other.f.partial_cmp(&self.f)
        }
    }
    impl Ord for Node {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            self.partial_cmp(other).unwrap_or(std::cmp::Ordering::Equal)
        }
    }

    let inside = |c: Cell| -> bool {
        c.i >= 0
            && c.i < hc
            && c.j >= 0
            && c.j < wc
            && coarse[(c.i * wc + c.j) as usize] == 1
    };

    if !inside(start) || !inside(goal) {
        return None;
    }

    let mut open = BinaryHeap::new();
    let mut came: HashMap<u64, u64> = HashMap::new();
    let mut bestg: HashMap<u64, f32> = HashMap::new();

    let start_key = pack_key(start.i, start.j);
    let goal_key = pack_key(goal.i, goal.j);

    let h0 = hfun(start, goal);
    open.push(Node {
        f: h0,
        g: 0.0,
        i: start.i,
        j: start.j,
        parent_key: None,
    });
    bestg.insert(start_key, 0.0);

    let mut closed: HashSet<u64> = HashSet::new();

    while let Some(Node { g, i, j, parent_key, .. }) = open.pop() {
        let key = pack_key(i, j);
        if closed.contains(&key) {
            continue;
        }
        if let Some(pk) = parent_key {
            came.insert(key, pk);
        }
        if key == goal_key {
            // reconstruct
            let mut path = Vec::<Cell>::new();
            let mut cur = key;
            loop {
                let (ci, cj) = unpack_key(cur);
                path.push(Cell { i: ci, j: cj });
                if let Some(&p) = came.get(&cur) {
                    cur = p;
                } else {
                    break;
                }
            }
            path.reverse();
            return Some(path);
        }
        closed.insert(key);

        for (ni, nj, cost) in neighbors(i, j) {
            let nkey = pack_key(ni, nj);
            if closed.contains(&nkey) {
                continue;
            }
            let ng = g + cost;
            if ng < *bestg.get(&nkey).unwrap_or(&f32::INFINITY) {
                bestg.insert(nkey, ng);
                let fh = ng + hfun(Cell { i: ni, j: nj }, goal);
                open.push(Node {
                    f: fh,
                    g: ng,
                    i: ni,
                    j: nj,
                    parent_key: Some(key),
                });
            }
        }
    }
    None
}

/// Map coarse cells to pixel centers
pub fn coarse_to_pixel_centers(path: &[Cell], k: i32) -> Vec<Pt> {
    path.iter()
        .map(|c| Pt {
            y: c.i * k + k / 2,
            x: c.j * k + k / 2,
        })
        .collect()
}

/// Bresenham LOS on the original binary grid
pub fn has_los(free: &[u8], w: u32, h: u32, p0: Pt, p1: Pt) -> bool {
    let (w_i, h_i) = (w as i32, h as i32);
    let mut x0 = p0.x;
    let mut y0 = p0.y;
    let x1 = p1.x;
    let y1 = p1.y;

    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;

    loop {
        if x0 < 0 || x0 >= w_i || y0 < 0 || y0 >= h_i {
            return false;
        }
        let idx = (y0 as u32 * w + x0 as u32) as usize;
        if free[idx] == 0 {
            return false;
        }
        if x0 == x1 && y0 == y1 {
            return true;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x0 += sx;
        }
        if e2 < dx {
            err += dx;
            y0 += sy;
        }
    }
}

/// Greedy smoothing over pixel-waypoints using LOS
pub fn smooth_path(free: &[u8], w: u32, h: u32, pts: &[Pt]) -> Vec<Pt> {
    if pts.is_empty() {
        return vec![];
    }
    let mut out = Vec::<Pt>::new();
    out.push(pts[0]);
    let mut i = 0usize;
    let last = pts.len() - 1;
    while i < last {
        let mut lo = i + 1;
        let hi = last;
        let mut step = 1usize;
        let mut j = lo;

        // exponential jump-ahead
        while j <= hi && has_los(free, w, h, pts[i], pts[j]) {
            lo = j;
            j = j.saturating_add(step);
            step <<= 1;
        }
        // binary refine in (lo ..= min(j,hi))
        let mut left = lo;
        let mut right = std::cmp::min(j, hi);
        while left < right {
            let mid = (left + right + 1) / 2;
            if has_los(free, w, h, pts[i], pts[mid]) {
                left = mid;
            } else {
                right = mid - 1;
            }
        }
        out.push(pts[left]);
        i = left;
    }
    out
}

/// Draw an overlay and optionally save to disk (returns saved path if saved)
pub fn visualize_and_optionally_save(
    gray: &GrayImage,
    coarse: &[u8],
    wc: i32,
    hc: i32,
    k: i32,
    path_px: &[Pt],
    smoothed: &[Pt],
    save_to: Option<&str>,
) -> Result<Vec<u8>, String> {
    // Build the RGBA canvas and draw the same overlays as before, then encode to PNG bytes.
    let (w, h) = gray.dimensions();
    let (w_i, h_i) = (w as i32, h as i32);

    let mut canvas: RgbaImage = RgbaImage::new(w, h);
    for y in 0..h {
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            canvas.put_pixel(x, y, Rgba([v, v, v, 255]));
        }
    }

    // grid lines
    let red_a = Rgba([255, 0, 0, 90]);
    for x in (0..w_i).step_by(k as usize) {
        draw_line_segment_mut(&mut canvas, (x as f32, 0.0), (x as f32, (h_i - 1) as f32), red_a);
    }
    for y in (0..h_i).step_by(k as usize) {
        draw_line_segment_mut(&mut canvas, (0.0, y as f32), ((w_i - 1) as f32, y as f32), red_a);
    }

    // blocked coarse cells
    let blocked = Rgba([255, 0, 0, 64]);
    for i in 0..hc {
        for j in 0..wc {
            if coarse[(i * wc + j) as usize] == 0 {
                let x0 = (j * k).max(0) as u32;
                let y0 = (i * k).max(0) as u32;
                let rect = Rect::at(x0 as i32, y0 as i32).of_size(k as u32, k as u32);
                draw_filled_rect_mut(&mut canvas, rect, blocked);
            }
        }
    }

    // start/goal if present
    if let Some(s) = path_px.first() {
        draw_filled_circle_mut(&mut canvas, (s.x, s.y), 4, Rgba([0, 200, 0, 255]));
    }
    if let Some(g) = path_px.last() {
        draw_filled_circle_mut(&mut canvas, (g.x, g.y), 4, Rgba([230, 0, 0, 255]));
    }

    // polylines
    let draw_polyline = |img: &mut RgbaImage, pts: &[Pt], color: Rgba<u8>, width_px: i32| {
        if pts.len() < 2 { return; }
        let w = width_px.max(1);
        for k in -w..=w {
            for t in -w..=w {
                for s in 0..(pts.len() - 1) {
                    let p0 = pts[s];
                    let p1 = pts[s + 1];
                    draw_line_segment_mut(img, ((p0.x + k) as f32, (p0.y + t) as f32), ((p1.x + k) as f32, (p1.y + t) as f32), color);
                }
            }
        }
    };

    draw_polyline(&mut canvas, path_px, Rgba([0, 0, 255, 220]), 1);
    draw_polyline(&mut canvas, smoothed, Rgba([0, 100, 255, 255]), 2);

    // encode to PNG bytes
    let mut buf = Vec::new();
    {
        let encoder = PngEncoder::new(&mut buf);
        encoder.write_image(&canvas, canvas.width(), canvas.height(), image::ColorType::Rgba8.into())
            .map_err(|e| format!("encode png: {e}"))?;
    }

    // If a save path was provided, write the bytes to disk as well
    if let Some(path) = save_to {
        std::fs::write(path, &buf).map_err(|e| format!("Failed to save {}: {}", path, e))?;
    }

    Ok(buf)
}

/// End-to-end helper: runs the whole pipeline and (optionally) saves an overlay PNG.
/// Returns a `PathOutcome` with both coarse and smoothed paths.
pub fn run_pipeline<P: AsRef<Path>>(
    img_path: P,
    cfg: Settings,
    overlay_out_path: Option<&str>,
) -> Result<PathOutcome, String> {
    // Load + binarize
    let (gray, free) = load_and_binarize(&img_path)?;

    let (w, h) = gray.dimensions();
    let (w_i, h_i) = (w as i32, h as i32);

    // Clamp start/goal
    let start_px = Pt {
        y: clamp_i32(cfg.start_px.y, 0, h_i - 1),
        x: clamp_i32(cfg.start_px.x, 0, w_i - 1),
    };
    let goal_px = Pt {
        y: clamp_i32(cfg.goal_px.y, 0, h_i - 1),
        x: clamp_i32(cfg.goal_px.x, 0, w_i - 1),
    };

    // Coarse grid
    let (coarse0, wc, hc) = build_coarse_occupancy(&free, w, h, cfg.k, cfg.thresh);
    let coarse = inflate_obstacles(coarse0, wc, hc, cfg.inflate);

    // Map to coarse cells & nudge
    let mut start_c = Cell {
        i: start_px.y / cfg.k,
        j: start_px.x / cfg.k,
    };
    let mut goal_c = Cell {
        i: goal_px.y / cfg.k,
        j: goal_px.x / cfg.k,
    };
    if let Some(s) = nudge_to_free(start_c, &coarse, wc, hc) {
        start_c = s;
    }
    if let Some(g) = nudge_to_free(goal_c, &coarse, wc, hc) {
        goal_c = g;
    }

    // A*
    let path_cells = astar_path(&coarse, wc, hc, cfg.connect8, start_c, goal_c)
        .unwrap_or_else(|| Vec::new());

    // Pixel paths
    let coarse_centers_px = coarse_to_pixel_centers(&path_cells, cfg.k);
    let smoothed = if !coarse_centers_px.is_empty() {
        smooth_path(&free, w, h, &coarse_centers_px)
    } else {
        vec![]
    };

    // Optional overlay save -> returns PNG bytes; if overlay_out_path is provided the bytes
    // are saved to disk by `visualize_and_optionally_save`. We keep the saved path as Option<String>.
    let _png = visualize_and_optionally_save(
        &gray,
        &coarse,
        wc,
        hc,
        cfg.k,
        &coarse_centers_px,
        &smoothed,
        overlay_out_path,
    )?;
    let saved = overlay_out_path.map(|s| s.to_string());

    Ok(PathOutcome {
        image_w: w,
        image_h: h,
        coarse_w: wc,
        coarse_h: hc,
        coarse_path_cells: path_cells,
        coarse_centers_px,
        smoothed_px: smoothed,
        saved_overlay_path: saved,
    })
}

/// Run the whole pipeline starting from PNG bytes in memory.
pub fn run_pipeline_from_png_bytes(
    bytes: &[u8],
    cfg: Settings,
    overlay_out_path: Option<&str>,
) -> Result<PathOutcome, String> {
    // Decode PNG bytes → GrayImage
    let dyn_img = image::load_from_memory(bytes).map_err(|e| format!("image decode: {e}"))?;
    let gray: GrayImage = dyn_img.to_luma8();

    // Build free[] thresholded mask (white=free)
    let (w, h) = gray.dimensions();
    let mut free = vec![0u8; (w * h) as usize];
    for y in 0..h {
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            free[(y * w + x) as usize] = if v > 127 { 1 } else { 0 };
        }
    }

    // Clamp start/goal (use cfg)
    let (w_i, h_i) = (w as i32, h as i32);
    let start_px = Pt { y: clamp_i32(cfg.start_px.y, 0, h_i - 1),
                        x: clamp_i32(cfg.start_px.x, 0, w_i - 1) };
    let goal_px  = Pt { y: clamp_i32(cfg.goal_px.y, 0, h_i - 1),
                        x: clamp_i32(cfg.goal_px.x, 0, w_i - 1) };

    // Coarse grid + inflation
    let (coarse0, wc, hc) = build_coarse_occupancy(&free, w, h, cfg.k, cfg.thresh);
    let coarse = inflate_obstacles(coarse0, wc, hc, cfg.inflate);

    // Map to coarse cells & nudge
    let mut start_c = Cell { i: start_px.y / cfg.k, j: start_px.x / cfg.k };
    let mut goal_c  = Cell { i: goal_px.y  / cfg.k, j: goal_px.x  / cfg.k };
    if let Some(s) = nudge_to_free(start_c, &coarse, wc, hc) { start_c = s; }
    if let Some(g) = nudge_to_free(goal_c,  &coarse, wc, hc) { goal_c  = g; }

    // A*
    let path_cells = astar_path(&coarse, wc, hc, cfg.connect8, start_c, goal_c)
        .unwrap_or_else(Vec::new);

    // Pixel paths
    let coarse_centers_px = coarse_to_pixel_centers(&path_cells, cfg.k);
    let smoothed = if !coarse_centers_px.is_empty() {
        smooth_path(&free, w, h, &coarse_centers_px)
    } else { vec![] };

    // Optional overlay save -> produce PNG bytes and optionally write to disk
    let _png = visualize_and_optionally_save(
        &gray, &coarse, wc, hc, cfg.k, &coarse_centers_px, &smoothed, overlay_out_path,
    )?;
    let saved = overlay_out_path.map(|s| s.to_string());

    Ok(PathOutcome {
        image_w: w, image_h: h,
        coarse_w: wc, coarse_h: hc,
        coarse_path_cells: path_cells,
        coarse_centers_px,
        smoothed_px: smoothed,
        saved_overlay_path: saved,
    })
}

/// Compute the integer pixel mean (x,y) of a polygon's vertices.
/// Assumes points are (x, y). If yours are (y, x), swap in the sum below.
fn polygon_mean_pt(points: &[(f64, f64)], w: u32, h: u32) -> Option<Pt> {
    if points.is_empty() {
        return None;
    }
    let mut sx = 0.0f64;
    let mut sy = 0.0f64;
    for &(x, y) in points {
        sx += x;
        sy += y;
    }
    let n = points.len() as f64;
    let mx = (sx / n).round() as i32;
    let my = (sy / n).round() as i32;

    // If your tuple is (y, x) instead, do:
    // let my = (sx / n).round() as i32;
    // let mx = (sy / n).round() as i32;

    let w_i = w as i32;
    let h_i = h as i32;
    Some(Pt {
        y: clamp_i32(my, 0, h_i - 1),
        x: clamp_i32(mx, 0, w_i - 1),
    })
}

/// Find nearest white/free pixel (value 1) starting from `seed` using 8-connected BFS.
fn nearest_free_pixel(free: &[u8], w: u32, h: u32, seed: Pt) -> Option<Pt> {
    let w_i = w as i32;
    let h_i = h as i32;
    let inside = |y: i32, x: i32| y >= 0 && y < h_i && x >= 0 && x < w_i;

    let idx = |y: i32, x: i32| -> usize { (y as u32 * w + x as u32) as usize };

    if !inside(seed.y, seed.x) {
        return None;
    }
    if free[idx(seed.y, seed.x)] == 1 {
        return Some(seed);
    }

    let mut q = VecDeque::new();
    let mut seen = vec![false; (w * h) as usize];

    q.push_back(seed);
    seen[idx(seed.y, seed.x)] = true;

    // 8-neighborhood
    let dirs = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    ];

    while let Some(p) = q.pop_front() {
        for (dy, dx) in dirs {
            let ny = p.y + dy;
            let nx = p.x + dx;
            if !inside(ny, nx) {
                continue;
            }
            let id = idx(ny, nx);
            if seen[id] {
                continue;
            }
            seen[id] = true;
            if free[id] == 1 {
                return Some(Pt { y: ny, x: nx });
            }
            q.push_back(Pt { y: ny, x: nx });
        }
    }
    None
}

pub fn default_points_from_image(png_bytes: &[u8]) -> Result<Vec<Pt>, Box<dyn std::error::Error>> {
    let img = image::load_from_memory(png_bytes)?;
    let (w, h) = img.dimensions();
    // Example: 4 waypoints roughly at corners
    Ok(vec![
        Pt { y: 10_i32.min(h as i32 - 1),         x: 10_i32.min(w as i32 - 1) },
        Pt { y: 10_i32.min(h as i32 - 1),         x: (w as i32 - 11).max(0)   },
        Pt { y: (h as i32 - 11).max(0),           x: 10_i32.min(w as i32 - 1) },
        Pt { y: (h as i32 - 11).max(0),           x: (w as i32 - 11).max(0)   },
    ])
}

pub fn run_pathfinding_given_polygons_and_save(
    png_bytes: &[u8],
    polygons: Vec<PolygonInfo>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Build config (unchanged)
    let cfg = Settings {
        k: 4,
        thresh: 0.6,
        inflate: 1,
        connect8: true,
        start_px: Pt { y: 0, x: 0 },  // not used; waypoints drive the route
        goal_px: Pt { y: 0, x: 0 },
    };

    // Decode once here so we can construct the free mask for nearest-white search
    let dyn_img = image::load_from_memory(png_bytes)?;
    let gray: GrayImage = dyn_img.to_luma8();
    let (w, h) = gray.dimensions();

    // Build free mask (white > 127 is free)
    let mut free = vec![0u8; (w * h) as usize];
    for y in 0..h {
        let base = (y * w) as usize;
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            free[base + x as usize] = if v > 127 { 1 } else { 0 };
        }
    }

    // For each polygon: mean of vertices -> nearest white pixel
    let mut found_polygon_points: Vec<Pt> = Vec::new();
    for poly in polygons.iter() {
        if let Some(seed) = polygon_mean_pt(&poly.points, w, h) {
            if let Some(near) = nearest_free_pixel(&free, w, h, seed) {
                found_polygon_points.push(near);
            } else {
                // Fallback: if no white pixel was found (e.g., fully blocked region), keep the seed (clamped)
                found_polygon_points.push(seed);
            }
        }
    }

    // Optional: dedupe consecutive identical points (rare but safe)
    found_polygon_points.dedup();

    // If fewer than 2 valid points, bail gracefully
    if found_polygon_points.len() <= 1 {
        println!("Not enough reachable waypoints were found.");
        return Ok(());
    }

    let make_cycle = false; // true if you want to return to the first point
    let png = run_multi_route_from_png_bytes(
        png_bytes,
        cfg,
        found_polygon_points,
        make_cycle,
        Some("multi_overlay.png"),
    )?;

    if !png.is_empty() {
        // ensure it's present by writing the bytes as well (safe overwrite).
        std::fs::write("multi_overlay.png", &png)?;
        println!("Saved overlay: multi_overlay.png");
    } else {
        println!("No overlay produced.");
    }
    Ok(())
}

pub fn run_pathfinding_given_polygons_as_bytes(
    png_bytes: &[u8],
    polygons: Vec<PolygonInfo>,
) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    // Build config (unchanged)
    let cfg = Settings {
        k: 4,
        thresh: 0.6,
        inflate: 1,
        connect8: true,
        start_px: Pt { y: 0, x: 0 },  // not used; waypoints drive the route
        goal_px: Pt { y: 0, x: 0 },
    };

    // Decode once here so we can construct the free mask for nearest-white search
    let dyn_img = image::load_from_memory(png_bytes)?;
    let gray: GrayImage = dyn_img.to_luma8();
    let (w, h) = gray.dimensions();

    // Build free mask (white > 127 is free)
    let mut free = vec![0u8; (w * h) as usize];
    for y in 0..h {
        let base = (y * w) as usize;
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            free[base + x as usize] = if v > 127 { 1 } else { 0 };
        }
    }

    // For each polygon: mean of vertices -> nearest white pixel
    let mut found_polygon_points: Vec<Pt> = Vec::new();
    for poly in polygons.iter() {
        if let Some(seed) = polygon_mean_pt(&poly.points, w, h) {
            if let Some(near) = nearest_free_pixel(&free, w, h, seed) {
                found_polygon_points.push(near);
            } else {
                // Fallback: if no white pixel was found (e.g., fully blocked region), keep the seed (clamped)
                found_polygon_points.push(seed);
            }
        }
    }

    // Optional: dedupe consecutive identical points (rare but safe)
    found_polygon_points.dedup();

    // If fewer than 2 valid points, bail gracefully
    if found_polygon_points.len() <= 1 {
        println!("Not enough reachable waypoints were found.");
        return Ok(Vec::<u8>::new());
    }

    let make_cycle = false; // true if you want to return to the first point
    let out = run_multi_route_from_png_bytes(
        png_bytes,
        cfg,
        found_polygon_points,
        make_cycle,
        Some("multi_overlay.png"),
    )?;

    Ok(out)
}

/// New: convenience function that returns ONLY the polyline produced from polygons.
pub fn run_pathfinding_given_polygons_return_path(
    png_bytes: &[u8],
    polygons: Vec<PolygonInfo>,
) -> Result<(Vec<Pt>, Vec<u64>), Box<dyn std::error::Error>> {
    // Build config (unchanged)
    let cfg = Settings {
        k: 4,
        thresh: 0.6,
        inflate: 1,
        connect8: true,
        start_px: Pt { y: 0, x: 0 },  // not used; waypoints drive the route
        goal_px: Pt { y: 0, x: 0 },
    };

    // Decode once here so we can construct the free mask for nearest-white search
    let dyn_img = image::load_from_memory(png_bytes)?;
    let gray: GrayImage = dyn_img.to_luma8();
    let (w, h) = gray.dimensions();

    // Build free mask (white > 127 is free)
    let mut free = vec![0u8; (w * h) as usize];
    for y in 0..h {
        let base = (y * w) as usize;
        for x in 0..w {
            let v = gray.get_pixel(x, y)[0];
            free[base + x as usize] = if v > 127 { 1 } else { 0 };
        }
    }

    // For each polygon: mean of vertices -> nearest white pixel
    // Collect matching product ids in the same order as points
    let mut found_polygon_points: Vec<Pt> = Vec::new();
    let mut found_product_ids: Vec<u64> = Vec::new();
    for poly in polygons.iter() {
        if let Some(seed) = polygon_mean_pt(&poly.points, w, h) {
            let chosen = if let Some(near) = nearest_free_pixel(&free, w, h, seed) {
                near
            } else {
                seed
            };
            found_polygon_points.push(chosen);
            // PolygonInfo now includes product_id (Option<u64>) — default to 0 when missing
            found_product_ids.push(poly.product_id.unwrap_or(0u64));
        }
    }

    // Deduplicate consecutive identical points while keeping product ids aligned
    let mut dedup_points: Vec<Pt> = Vec::new();
    let mut dedup_pids: Vec<u64> = Vec::new();
    for (i, p) in found_polygon_points.into_iter().enumerate() {
        if let Some(last) = dedup_points.last() {
            if last.x == p.x && last.y == p.y {
                // skip consecutive duplicate
                continue;
            }
        }
        dedup_points.push(p);
        dedup_pids.push(found_product_ids[i]);
    }
    found_polygon_points = dedup_points;
    found_product_ids = dedup_pids;

    if found_polygon_points.len() <= 1 {
        return Ok((Vec::new(), Vec::new()));
    }

    // Use the new renderer to get the path + mask; pass product ids aligned with waypoints
    let render = run_multi_route_render_from_png_bytes(
        png_bytes,
        cfg,
        found_polygon_points,
        found_product_ids,
        false,  // make_cycle
        None,   // overlay_out_path
    )?;



    Ok((render.polyline, render.product_point_mask))
}

/// New: helper to convert a polyline to an SVG-compatible "points" string.
/// Remember Pt is (y, x); SVG wants "x,y".
pub fn polyline_to_svg_points(pts: &[Pt]) -> String {
    let mut s = String::new();
    for (i, p) in pts.iter().enumerate() {
        if i > 0 { s.push(' '); }
        s.push_str(&format!("{},{}", p.x, p.y));
    }
    s
}

pub fn polyline_to_string(points: Vec<Pt>) -> String {
    points
        .into_iter()
        .map(|p| format!("{},{}", p.x, p.y))
        .collect::<Vec<_>>()
        .join(" ")
}

