mod constants;
mod intersection;
mod reservation;
mod vehicle;

use macroquad::prelude::*;
use macroquad::rand::gen_range;
use constants::*;
use reservation::ReservationTable;
use vehicle::{Direction, Route, Vehicle, VehicleState, spawn_pos};

// Minimum gap between two vehicles in the same lane at the spawn point.
// Prevents vehicles being created on top of each other when keys are spammed.
const MIN_SPAWN_GAP: f32 = CAR_LENGTH + SAFE_DIST * 3.0;

// How often (in seconds) a new vehicle is spawned while R is held.
const R_INTERVAL: f32 = 0.9;

fn window_conf() -> Conf {
    Conf {
        window_title: "Smart Road".to_owned(),
        window_width:  WIN_W as i32,
        window_height: WIN_H as i32,
        ..Default::default()
    }
}

/// Returns the bumper-to-bumper gap (px) if `leader` is ahead of `follower` on
/// the same waypoint path, or a negative value if `leader` is actually behind.
fn path_gap(follower: &Vehicle, leader: &Vehicle) -> f32 {
    let leader_is_ahead = if leader.wp_idx != follower.wp_idx {
        leader.wp_idx > follower.wp_idx
    } else {
        // Same waypoint index: whoever is closer to the next waypoint is ahead.
        let nf = follower.wp_idx + 1;
        let nl = leader.wp_idx + 1;
        if nf >= follower.waypoints.len() || nl >= leader.waypoints.len() {
            return -1.0;
        }
        let dist_leader   = (leader.pos   - leader.waypoints[nl]).length();
        let dist_follower = (follower.pos  - follower.waypoints[nf]).length();
        dist_leader < dist_follower
    };
    if !leader_is_ahead {
        return -1.0;
    }
    (leader.pos - follower.pos).length() - CAR_LENGTH
}

/// Maps a bumper-to-bumper gap to a target speed.
/// At gap >= SAFE_DIST → full speed; at gap = 0 → stopped.
fn desired_velocity(gap: f32) -> f32 {
    if gap >= SAFE_DIST {
        SPEED_NORMAL
    } else {
        (SPEED_NORMAL * gap / SAFE_DIST).max(0.0)
    }
}

fn random_route() -> Route {
    match gen_range(0u32, 3) {
        0 => Route::Right,
        1 => Route::Straight,
        _ => Route::Left,
    }
}

fn random_direction() -> Direction {
    match gen_range(0u32, 4) {
        0 => Direction::North,
        1 => Direction::South,
        2 => Direction::East,
        _ => Direction::West,
    }
}

/// Tries to add a vehicle at the spawn point for (origin, route).
/// Does nothing if an existing vehicle is still too close to that point.
fn try_spawn(vehicles: &mut Vec<Vehicle>, id: &mut u32, origin: Direction, route: Route) {
    let pos = spawn_pos(origin, route);
    let blocked = vehicles.iter().any(|v| {
        v.origin == origin && v.route == route
            && (v.pos - pos).length() < MIN_SPAWN_GAP
    });
    if !blocked {
        vehicles.push(Vehicle::new(*id, origin, route, SPEED_NORMAL));
        *id += 1;
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let grass = Color { r: 0.13, g: 0.40, b: 0.13, a: 1.0 };

    let mut vehicles:     Vec<Vehicle>     = Vec::new();
    let mut reservations: ReservationTable = ReservationTable::new();
    let mut next_id:  u32  = 0;
    let mut r_timer:  f32  = R_INTERVAL; // start ready so first R press spawns immediately
    let mut show_grid: bool = false;

    loop {
        let dt  = get_frame_time().min(0.05);
        let now = get_time();

        // ── Arrow keys: spawn one vehicle from the matching arm ────────────────
        // Up    → vehicle enters from South heading north
        // Down  → vehicle enters from North heading south
        // Right → vehicle enters from West heading east
        // Left  → vehicle enters from East heading west
        if is_key_pressed(KeyCode::Up) {
            try_spawn(&mut vehicles, &mut next_id, Direction::South, random_route());
        }
        if is_key_pressed(KeyCode::Down) {
            try_spawn(&mut vehicles, &mut next_id, Direction::North, random_route());
        }
        if is_key_pressed(KeyCode::Right) {
            try_spawn(&mut vehicles, &mut next_id, Direction::West, random_route());
        }
        if is_key_pressed(KeyCode::Left) {
            try_spawn(&mut vehicles, &mut next_id, Direction::East, random_route());
        }

        // ── R key: continuously spawn random vehicles on a timer ───────────────
        if is_key_down(KeyCode::R) {
            r_timer += dt;
            if r_timer >= R_INTERVAL {
                r_timer = 0.0;
                try_spawn(&mut vehicles, &mut next_id, random_direction(), random_route());
            }
        } else {
            r_timer = R_INTERVAL; // reset so the next R press spawns immediately
        }

        // ── G: toggle tile-grid debug overlay ────────────────────────────────
        if is_key_pressed(KeyCode::G) {
            show_grid = !show_grid;
        }

        // ── Esc: quit (stats window will be added in step 9) ──────────────────
        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        // ── Compute following distances and set velocities ─────────────────────
        let n = vehicles.len();
        let mut desired_vels: Vec<f32> = vec![SPEED_NORMAL; n];
        for i in 0..n {
            let mut min_gap = f32::MAX;
            for j in 0..n {
                if i == j { continue; }
                if vehicles[i].origin != vehicles[j].origin { continue; }
                if vehicles[i].route  != vehicles[j].route  { continue; }
                let gap = path_gap(&vehicles[i], &vehicles[j]);
                if gap >= 0.0 && gap < min_gap {
                    min_gap = gap;
                }
            }
            desired_vels[i] = desired_velocity(min_gap);
        }

        // ── AIM request/grant/deny + update ───────────────────────────────────
        for (i, v) in vehicles.iter_mut().enumerate() {
            v.velocity = desired_vels[i];

            // Stop at entry line and request a reservation each frame until granted
            if v.state == VehicleState::Approaching && v.wp_idx == 0 {
                let dist_to_entry = (v.waypoints[1] - v.pos).length();
                if dist_to_entry <= STOP_MARGIN {
                    v.velocity = 0.0;
                    if reservations.request(v, now) {
                        v.state = VehicleState::Reserved;
                    }
                }
            }

            let prev_state = v.state;
            v.update(dt, now);

            // Just exited the intersection: free its tiles
            if prev_state == VehicleState::InIntersection
                && v.state == VehicleState::Exiting
            {
                reservations.release(v.id);
            }
        }

        // Release any vehicles that finished without passing through the box
        for v in vehicles.iter().filter(|v| v.is_done()) {
            reservations.release(v.id);
        }
        vehicles.retain(|v| !v.is_done());

        reservations.cleanup(now);

        // ── Draw ───────────────────────────────────────────────────────────────
        clear_background(grass);
        intersection::draw();
        if show_grid {
            reservations.draw_debug(now);
        }
        for v in &vehicles {
            v.draw();
        }

        next_frame().await;
    }
}
