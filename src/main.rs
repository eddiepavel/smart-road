mod constants;
mod intersection;
mod reservation;
mod vehicle;

use std::collections::HashSet;
use macroquad::prelude::*;
use macroquad::rand::gen_range;
use constants::*;
use reservation::ReservationTable;
use vehicle::{Direction, Route, Vehicle, VehicleState, spawn_pos};

// ── Statistics ────────────────────────────────────────────────────────────────

struct Stats {
    vehicles_passed: u32,
    max_vel:         f32,
    min_vel:         f32,
    max_cross_time:  f32,
    min_cross_time:  f32,
    close_calls:     u32,
}

impl Stats {
    fn new() -> Self {
        Stats {
            vehicles_passed: 0,
            max_vel: 0.0,
            min_vel: f32::MAX,
            max_cross_time: 0.0,
            min_cross_time: f32::MAX,
            close_calls: 0,
        }
    }

    /// Record a vehicle that has just become Done.
    /// Only counted if the vehicle received a reservation (reserve_time > 0).
    fn record(&mut self, v: &Vehicle, now: f64) {
        if v.reserve_time <= 0.0 { return; }
        self.vehicles_passed += 1;
        self.max_vel = self.max_vel.max(v.max_vel);
        self.min_vel = self.min_vel.min(v.min_vel);
        let cross = (now - v.reserve_time) as f32;
        self.max_cross_time = self.max_cross_time.max(cross);
        self.min_cross_time = self.min_cross_time.min(cross);
    }
}

fn draw_stats(stats: &Stats) {
    draw_rectangle(0.0, 0.0, WIN_W, WIN_H, Color { r: 0.0, g: 0.0, b: 0.0, a: 0.72 });

    let pw = 440.0_f32;
    let ph = 310.0_f32;
    let px = (WIN_W - pw) * 0.5;
    let py = (WIN_H - ph) * 0.5;

    draw_rectangle(px, py, pw, ph, Color { r: 0.08, g: 0.10, b: 0.14, a: 0.97 });
    draw_rectangle_lines(px, py, pw, ph, 2.0, Color { r: 0.35, g: 0.55, b: 1.0, a: 1.0 });

    let title = "SIMULATION STATISTICS";
    let ts = measure_text(title, None, 24, 1.0);
    draw_text(title, px + (pw - ts.width) * 0.5, py + 40.0, 24.0, WHITE);
    draw_line(px + 20.0, py + 52.0, px + pw - 20.0, py + 52.0, 1.0,
        Color { r: 0.35, g: 0.55, b: 1.0, a: 0.5 });

    let label_clr = Color { r: 0.70, g: 0.82, b: 1.0, a: 1.0 };
    let lx = px + 28.0;
    let vx = px + pw - 28.0;
    let y0 = py + 88.0;
    let rh = 36.0_f32;

    let fmt_vel  = |v: f32| format!("{:.0} px/s", v);
    let fmt_time = |t: f32| format!("{:.2} s",    t);
    let na       = || "—".to_string();

    let rows: [(&str, String); 6] = [
        ("Vehicles passed",   format!("{}", stats.vehicles_passed)),
        ("Max velocity",      if stats.max_vel > 0.0        { fmt_vel(stats.max_vel) }        else { na() }),
        ("Min velocity",      if stats.min_vel < f32::MAX   { fmt_vel(stats.min_vel) }        else { na() }),
        ("Max crossing time", if stats.max_cross_time > 0.0 { fmt_time(stats.max_cross_time) } else { na() }),
        ("Min crossing time", if stats.min_cross_time < f32::MAX { fmt_time(stats.min_cross_time) } else { na() }),
        ("Close calls",       format!("{}", stats.close_calls)),
    ];

    for (i, (label, value)) in rows.iter().enumerate() {
        let y = y0 + i as f32 * rh;
        draw_text(label, lx, y, 21.0, label_clr);
        let vw = measure_text(value, None, 21, 1.0).width;
        draw_text(value, vx - vw, y, 21.0, WHITE);
    }

    let footer = "Press  ESC  or  ENTER  to exit";
    let fw = measure_text(footer, None, 17, 1.0).width;
    draw_text(footer, px + (pw - fw) * 0.5, py + ph - 14.0, 17.0,
        Color { r: 0.50, g: 0.50, b: 0.50, a: 1.0 });
}

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

/// Maximum safe speed given a bumper-to-bumper gap and the follower's decel rate.
/// Uses the kinematic formula so the car can always stop at SAFE_DIST.
fn desired_velocity(gap: f32, decel: f32) -> f32 {
    if gap <= SAFE_DIST {
        0.0
    } else {
        (2.0 * decel * (gap - SAFE_DIST)).sqrt().min(SPEED_NORMAL)
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
        vehicles.push(Vehicle::new(*id, origin, route, SPEED_SLOW));
        *id += 1;
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let grass = Color { r: 0.13, g: 0.40, b: 0.13, a: 1.0 };

    let mut vehicles:     Vec<Vehicle>     = Vec::new();
    let mut reservations: ReservationTable = ReservationTable::new();
    let mut next_id:   u32  = 0;
    let mut r_timer:   f32  = R_INTERVAL;
    let mut show_grid: bool = false;
    let mut stats:       Stats               = Stats::new();
    let mut close_pairs: HashSet<(u32, u32)> = HashSet::new();

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

        // ── Esc: stop simulation, show stats ─────────────────────────────
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
            desired_vels[i] = desired_velocity(min_gap, vehicles[i].decel);
        }

        // ── Smooth velocity + AIM request/grant/deny + update ────────────────
        for (i, v) in vehicles.iter_mut().enumerate() {
            // Target from following-distance logic; AIM may override to 0
            let mut target = desired_vels[i];

            if v.state == VehicleState::Approaching && v.wp_idx == 0 {
                let dist = (v.waypoints[1] - v.pos).length();

                // Start braking early enough to stop exactly at STOP_MARGIN
                let stopping_dist = v.velocity * v.velocity / (2.0 * v.decel);
                if dist <= stopping_dist + STOP_MARGIN {
                    target = 0.0;
                }

                // Request reservation once essentially stopped at the entry zone
                if v.velocity < 2.0 && dist <= STOP_MARGIN + 6.0 {
                    if reservations.request(v, now) {
                        v.state        = VehicleState::Reserved;
                        v.reserve_time = now;
                    }
                }
            }

            // Exiting vehicles are past the intersection — no conflicts, accelerate freely
            if v.state == VehicleState::Exiting {
                target = SPEED_FAST;
            }

            // Smooth ramp: accelerate or brake toward target
            if v.velocity < target {
                v.velocity = (v.velocity + v.accel * dt).min(target);
            } else if v.velocity > target {
                v.velocity = (v.velocity - v.decel * dt).max(target);
            }

            let prev_state = v.state;
            v.update(dt, now);

            if prev_state == VehicleState::InIntersection
                && v.state == VehicleState::Exiting
            {
                reservations.release(v.id);
            }
        }

        // ── Close call detection ──────────────────────────────────────────
        let cc_dist = CAR_LENGTH + SAFE_DIST;
        let mut new_close: HashSet<(u32, u32)> = HashSet::new();
        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                if (vehicles[i].pos - vehicles[j].pos).length() < cc_dist {
                    let pair = (vehicles[i].id.min(vehicles[j].id),
                                vehicles[i].id.max(vehicles[j].id));
                    new_close.insert(pair);
                    if !close_pairs.contains(&pair) {
                        stats.close_calls += 1;
                    }
                }
            }
        }
        close_pairs = new_close;

        // ── Stats + cleanup for finished vehicles ─────────────────────────
        for v in vehicles.iter().filter(|v| v.is_done()) {
            reservations.release(v.id);
            stats.record(v, now);
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

    // ── Stats window ───────────────────────────────────────────────────────────
    loop {
        clear_background(grass);
        intersection::draw();
        for v in &vehicles {
            v.draw();
        }
        draw_stats(&stats);
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Enter) {
            break;
        }
        next_frame().await;
    }
}
