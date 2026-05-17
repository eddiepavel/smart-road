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
    max_distance:    f32,
    min_distance:    f32,
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
            max_distance: 0.0,
            min_distance: f32::MAX,
            close_calls: 0,
        }
    }

    /// Record a vehicle that has just become Done.
    /// Only counted if the vehicle received a reservation (reserve_time > 0).
    fn record(&mut self, v: &Vehicle, now: f64) {
        if v.reserve_time <= 0.0 { return; }
        self.vehicles_passed += 1;
        self.max_vel      = self.max_vel.max(v.max_vel);
        self.min_vel      = self.min_vel.min(v.min_vel);
        self.max_distance = self.max_distance.max(v.distance);
        self.min_distance = self.min_distance.min(v.distance);
        let cross = (now - v.reserve_time) as f32;
        self.max_cross_time = self.max_cross_time.max(cross);
        self.min_cross_time = self.min_cross_time.min(cross);
    }
}

fn draw_stats(stats: &Stats) {
    draw_rectangle(0.0, 0.0, WIN_W, WIN_H, Color { r: 0.0, g: 0.0, b: 0.0, a: 0.72 });

    let pw = 440.0_f32;
    let ph = 390.0_f32;
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

    let rows: [(&str, String); 8] = [
        ("Vehicles passed",   format!("{}", stats.vehicles_passed)),
        ("Max velocity",      if stats.max_vel > 0.0           { fmt_vel(stats.max_vel) }        else { na() }),
        ("Min velocity",      if stats.min_vel < f32::MAX      { fmt_vel(stats.min_vel) }        else { na() }),
        ("Max crossing time", if stats.max_cross_time > 0.0    { fmt_time(stats.max_cross_time) } else { na() }),
        ("Min crossing time", if stats.min_cross_time < f32::MAX { fmt_time(stats.min_cross_time) } else { na() }),
        ("Max travel distance", if stats.max_distance > 0.0      { format!("{:.0} px", stats.max_distance) } else { na() }),
        ("Min travel distance", if stats.min_distance < f32::MAX { format!("{:.0} px", stats.min_distance) } else { na() }),
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

/// Maximum safe speed given a bumper-to-bumper gap, decel rate, and speed cap.
/// Uses the kinematic formula so the car can always stop at SAFE_DIST.
fn desired_velocity(gap: f32, decel: f32, max_speed: f32) -> f32 {
    if gap <= SAFE_DIST {
        0.0
    } else {
        (2.0 * decel * (gap - SAFE_DIST)).sqrt().min(max_speed)
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
    // Try to load a car sprite from `assets/car.png`. If missing, we'll
    // generate a neutral white car sprite into a render target and tint it.
    let car_tex = match macroquad::texture::load_texture("assets/car.png").await {
        Ok(t) => { let mut tt = t; tt.set_filter(macroquad::texture::FilterMode::Nearest); Some(tt) },
        Err(_)    => None,
    };

    let mut vehicles:     Vec<Vehicle>     = Vec::new();
    let mut reservations: ReservationTable = ReservationTable::new();
    let mut next_id:   u32  = 0;
    let mut r_timer:   f32  = R_INTERVAL;
    let mut show_grid: bool = false;
    let mut stats:       Stats               = Stats::new();
    // Track pairs that are currently in a close-call state so we only count
    // the transition into the close zone once per encounter.
    let mut close_pairs: HashSet<(u32, u32)> = HashSet::new();
    // Simulation time advances by dt each frame (not wall-clock time).
    // This keeps reservation time-slots in sync with physics even when the
    // window is paused, minimised, or the system stutters.
    let mut sim_time: f64 = 0.0;

    loop {
        let dt  = get_frame_time().min(0.05);
        sim_time += dt as f64;
        let now = sim_time;

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
            // Exiting vehicles are past the intersection: let them run up to SPEED_FAST,
            // but still respect the kinematic following distance to any vehicle ahead.
            let max_spd = if vehicles[i].state == VehicleState::Exiting {
                SPEED_FAST
            } else {
                SPEED_NORMAL
            };
            desired_vels[i] = desired_velocity(min_gap, vehicles[i].decel, max_spd);
        }

        // ── Smooth velocity + AIM request/grant/deny + update ────────────────
        for (i, v) in vehicles.iter_mut().enumerate() {
            // Target from following-distance logic; AIM may override to 0
            let mut target = desired_vels[i];

            if v.state == VehicleState::Approaching && v.wp_idx == 0 {
                let dist = (v.waypoints[1] - v.pos).length();

                // Start braking early enough to stop at STOP_MARGIN.
                // Second condition: once stopped near the line, stay stopped —
                // prevents oscillation when the car halts slightly past STOP_MARGIN.
                let stopping_dist = v.velocity * v.velocity / (2.0 * v.decel);
                if dist <= stopping_dist + STOP_MARGIN || dist <= STOP_MARGIN + 8.0 {
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

            // Smooth ramp: accelerate or brake toward target
            if v.velocity < target {
                v.velocity = (v.velocity + v.accel * dt).min(target);
            } else if v.velocity > target {
                v.velocity = (v.velocity - v.decel * dt).max(target);
            }
            // Hard clamp: velocity must always be in [0, SPEED_FAST]
            v.velocity = v.velocity.clamp(0.0, SPEED_FAST);

            let prev_state = v.state;
            v.update(dt, now);

            if prev_state == VehicleState::InIntersection
                && v.state == VehicleState::Exiting
            {
                reservations.release(v.id);
            }
        }

        // ── Close call detection (edge-triggered + approach check) ─────────
        let cc_dist = CAR_LENGTH + SAFE_DIST;
        let mut current_close: HashSet<(u32, u32)> = HashSet::new();

        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                let vi = &vehicles[i];
                let vj = &vehicles[j];

                // Skip vehicles that share the exact same path (same origin+route)
                if vi.origin == vj.origin && vi.route == vj.route {
                    continue;
                }

                // Only consider pairs that are in or near the intersection area.
                let in_box_i = vi.pos.x >= INTER_L && vi.pos.x <= INTER_R
                            && vi.pos.y >= INTER_T && vi.pos.y <= INTER_B;
                let in_box_j = vj.pos.x >= INTER_L && vj.pos.x <= INTER_R
                            && vj.pos.y >= INTER_T && vj.pos.y <= INTER_B;
                if !(in_box_i || in_box_j
                    || vi.state == VehicleState::InIntersection
                    || vj.state == VehicleState::InIntersection)
                {
                    continue;
                }

                let dist = (vi.pos - vj.pos).length();
                if dist >= cc_dist { continue; }

                // Approach check: are they closing in on each other?
                let vel_i = Vec2::new(vi.heading.cos(), vi.heading.sin()) * vi.velocity;
                let vel_j = Vec2::new(vj.heading.cos(), vj.heading.sin()) * vj.velocity;
                let rel_pos = vj.pos - vi.pos;
                let rel_vel = vel_j - vel_i;
                // dot < 0 => distance decreasing
                if rel_pos.dot(rel_vel) >= 0.0 {
                    continue;
                }

                let pair = (vi.id.min(vj.id), vi.id.max(vj.id));
                current_close.insert(pair);

                // Count only the moment the pair enters the close-call state.
                if !close_pairs.contains(&pair) {
                    stats.close_calls += 1;
                }
            }
        }

        // Keep only pairs that are still close this frame.
        close_pairs = current_close;

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
            v.draw(car_tex.as_ref());
        }

        next_frame().await;
    }

    // Flush the ESC key press so the stats loop doesn't exit immediately
    next_frame().await;

    // ── Stats window ───────────────────────────────────────────────────────────
    loop {
        clear_background(grass);
        intersection::draw();
        for v in &vehicles {
            v.draw(car_tex.as_ref());
        }
        draw_stats(&stats);
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Enter) {
            break;
        }
        next_frame().await;
    }
}
