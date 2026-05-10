use std::collections::{HashMap, HashSet};
use macroquad::prelude::*;
use crate::constants::*;
use crate::vehicle::Vehicle;

// ── Types ─────────────────────────────────────────────────────────────────────
type TileKey = (u8, u8, u32); // (tile_x, tile_y, time_slot_idx)

// ── ReservationTable ──────────────────────────────────────────────────────────
pub struct ReservationTable {
    table: HashMap<TileKey, u32>, // → vehicle_id
}

impl ReservationTable {
    pub fn new() -> Self {
        ReservationTable { table: HashMap::new() }
    }

    /// Try to reserve all tiles for `vehicle`'s intersection path.
    /// Returns true on success (and commits the reservation), false on conflict.
    pub fn request(&mut self, vehicle: &Vehicle, now: f64) -> bool {
        let slots = self.compute_slots(vehicle, now);
        // Check every slot is free
        for &key in &slots {
            if let Some(&owner) = self.table.get(&key) {
                if owner != vehicle.id {
                    return false;
                }
            }
        }
        // Commit
        for key in slots {
            self.table.insert(key, vehicle.id);
        }
        true
    }

    /// Release all reservations held by `vehicle_id`.
    pub fn release(&mut self, vehicle_id: u32) {
        self.table.retain(|_, &mut vid| vid != vehicle_id);
    }

    /// Remove time slots that have already passed.
    pub fn cleanup(&mut self, now: f64) {
        let current_slot = (now as f32 / TIME_SLOT) as u32;
        self.table.retain(|&(_, _, ts), _| ts + 4 >= current_slot);
    }

    // ── Tile-path computation ─────────────────────────────────────────────────

    /// Walk the vehicle's remaining path, sampling every TILE_PX/2 px.
    /// For each sample point inside the intersection box, record the
    /// (tile_x, tile_y, time_slot) based on the estimated arrival time.
    fn compute_slots(&self, vehicle: &Vehicle, now: f64) -> Vec<TileKey> {
        // Vehicles only request from the stop line (velocity ≈ 0); use SPEED_NORMAL
        // for accurate ETAs, since that is the speed at which they will cross.
        let speed = SPEED_NORMAL;

        // Build segment list: vehicle.pos → all remaining waypoints
        let mut positions = Vec::with_capacity(vehicle.waypoints.len());
        positions.push(vehicle.pos);
        for i in (vehicle.wp_idx + 1)..vehicle.waypoints.len() {
            positions.push(vehicle.waypoints[i]);
        }

        let mut seen: HashSet<TileKey> = HashSet::new();
        let mut result: Vec<TileKey>   = Vec::new();
        let mut cum_dist = 0.0_f32;

        for seg in positions.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            let seg_len = (b - a).length();
            if seg_len < f32::EPSILON { continue; }

            // Sample at half-tile intervals so no tile is ever skipped
            let steps = ((seg_len / (TILE_PX * 0.5)).ceil() as usize).max(1);

            for s in 0..=steps {
                let t      = s as f32 / steps as f32;
                let p      = a.lerp(b, t);
                let dist   = cum_dist + t * seg_len;

                // Only care about points inside the intersection box
                if p.x < INTER_L || p.x > INTER_R || p.y < INTER_T || p.y > INTER_B {
                    continue;
                }

                let eta = now as f32 + dist / speed;
                let ts  = (eta / TIME_SLOT) as u32;

                let tx = ((p.x - INTER_L) / TILE_PX) as i32;
                let ty = ((p.y - INTER_T) / TILE_PX) as i32;

                if tx < 0 || ty < 0 || tx >= GRID_N as i32 || ty >= GRID_N as i32 {
                    continue;
                }

                // Reserve the current slot plus one ahead as a timing buffer
                for offset in 0u32..2 {
                    let key = (tx as u8, ty as u8, ts + offset);
                    if seen.insert(key) {
                        result.push(key);
                    }
                }
            }

            cum_dist += seg_len;
        }

        result
    }

    // ── Debug rendering ───────────────────────────────────────────────────────

    /// Draw the tile grid and highlight tiles reserved at the current time.
    pub fn draw_debug(&self, now: f64) {
        let ts_now = (now as f32 / TIME_SLOT) as u32;
        let grid_clr = Color { r: 1.0, g: 1.0, b: 1.0, a: 0.15 };
        let fill_clr = Color { r: 1.0, g: 0.35, b: 0.1, a: 0.35 };

        // Grid lines
        for i in 0..=GRID_N {
            let x = INTER_L + i as f32 * TILE_PX;
            let y = INTER_T + i as f32 * TILE_PX;
            draw_line(x, INTER_T, x, INTER_B, 0.5, grid_clr);
            draw_line(INTER_L, y, INTER_R, y, 0.5, grid_clr);
        }

        // Highlight tiles that are reserved within ±2 slots of now
        for &(tx, ty, ts) in self.table.keys() {
            if ts.abs_diff(ts_now) <= 2 {
                let x = INTER_L + tx as f32 * TILE_PX;
                let y = INTER_T + ty as f32 * TILE_PX;
                draw_rectangle(x, y, TILE_PX, TILE_PX, fill_clr);
            }
        }
    }
}
