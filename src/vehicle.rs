use macroquad::prelude::*;
use macroquad::rand::gen_range;
use std::f32::consts::PI;
use crate::constants::*;

// ── Turn-arc radii ─────────────────────────────────────────────────────────────
// A right turn stays near the nearest corner → small radius (half a lane width).
// A left turn swings to the far corner → large radius.
const RIGHT_R: f32 = LANE_W / 2.0;              // 20  px
const LEFT_R:  f32 = ROAD_HALF + LANE_W / 2.0;  // 140 px

// Waypoint resolution per arc
const RIGHT_STEPS: usize = 8;
const LEFT_STEPS:  usize = 16;

// ── Public enumerations ────────────────────────────────────────────────────────

/// Which arm the vehicle enters from (the direction it is *travelling away from*).
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Direction { North, South, East, West }

/// The in-lane route the vehicle follows through the intersection.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Route { Right, Straight, Left }

/// Lifecycle stages used by the intersection manager and statistics.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum VehicleState {
    Approaching,      // outside intersection box, heading toward it
    Reserved,         // reservation granted, cleared to enter
    InIntersection,   // inside the intersection box
    Exiting,          // has left the box, travelling to screen edge
    Done,             // off-screen, ready to be removed
}

// ── Vehicle ────────────────────────────────────────────────────────────────────

pub struct Vehicle {
    pub id:       u32,
    pub origin:   Direction,
    pub route:    Route,
    pub pos:      Vec2,
    pub heading:  f32,     // radians; east = 0, south = PI/2
    pub velocity: f32,     // current speed in px/s
    pub state:    VehicleState,

    // Pre-computed path through the world
    pub waypoints: Vec<Vec2>,
    pub wp_idx:    usize,

    // Physics — randomised per vehicle for varied acceleration/braking behaviour
    pub accel: f32,  // acceleration rate  (px/s²)
    pub decel: f32,  // deceleration rate  (px/s²)

    // Statistics
    pub distance:     f32,   // total distance travelled (px)
    pub reserve_time: f64,
    pub entry_time:   f64,
    pub exit_time:    Option<f64>,
    pub max_vel:      f32,
    pub min_vel:      f32,
}

impl Vehicle {
    pub fn new(id: u32, origin: Direction, route: Route, velocity: f32) -> Self {
        let waypoints = build_path(origin, route);
        let pos       = waypoints[0];
        let heading   = initial_heading(origin);
        Vehicle {
            id, origin, route, pos, heading, velocity,
            state: VehicleState::Approaching,
            waypoints, wp_idx: 0,
            accel: gen_range(100.0_f32, 220.0_f32),
            decel: gen_range(150.0_f32, 350.0_f32),
            distance: 0.0,
            reserve_time: 0.0, entry_time: 0.0, exit_time: None,
            max_vel: velocity, min_vel: velocity,
        }
    }

    /// Advance the vehicle along its waypoint path by `dt` seconds.
    pub fn update(&mut self, dt: f32, now: f64) {
        if self.wp_idx + 1 >= self.waypoints.len() {
            self.state = VehicleState::Done;
            return;
        }

        // Track velocity and distance
        self.max_vel  = self.max_vel.max(self.velocity);
        self.min_vel  = self.min_vel.min(self.velocity);
        self.distance += self.velocity * dt;

        let mut remaining = self.velocity * dt;

        while remaining > 0.0 {
            let next_idx = self.wp_idx + 1;
            if next_idx >= self.waypoints.len() {
                self.state = VehicleState::Done;
                break;
            }
            let next = self.waypoints[next_idx];
            let diff = next - self.pos;
            let dist = diff.length();

            if dist < f32::EPSILON {
                self.wp_idx += 1;
                continue;
            }

            let dir = diff / dist;
            if dist <= remaining {
                remaining  -= dist;
                self.pos    = next;
                self.wp_idx = next_idx;
            } else {
                self.pos += dir * remaining;
                self.heading = dir.y.atan2(dir.x);
                remaining = 0.0;
            }
        }

        // Keep heading pointed toward the next waypoint
        if self.wp_idx + 1 < self.waypoints.len() {
            let diff = self.waypoints[self.wp_idx + 1] - self.pos;
            if diff.length_squared() > f32::EPSILON {
                self.heading = diff.y.atan2(diff.x);
            }
        }

        self.update_state(now);
    }

    fn update_state(&mut self, now: f64) {
        let in_box = self.pos.x >= INTER_L && self.pos.x <= INTER_R
                  && self.pos.y >= INTER_T && self.pos.y <= INTER_B;

        match self.state {
            VehicleState::Approaching => {
                if in_box {
                    // Fallback: shouldn't normally fire; stop logic in main prevents this
                    self.state      = VehicleState::InIntersection;
                    self.entry_time = now;
                }
            }
            VehicleState::Reserved => {
                if in_box {
                    self.state      = VehicleState::InIntersection;
                    self.entry_time = now;
                }
            }
            VehicleState::InIntersection => {
                if !in_box {
                    self.state     = VehicleState::Exiting;
                    self.exit_time = Some(now);
                }
            }
            _ => {}
        }

        if self.wp_idx + 1 >= self.waypoints.len() {
            self.state = VehicleState::Done;
        }
    }

    pub fn is_done(&self) -> bool {
        self.state == VehicleState::Done
    }

    /// Draw the vehicle: wheels → body → windshield → headlights, all rotated by heading.
    /// If `tex` is `Some`, render the sprite; otherwise fall back to primitives.
    pub fn draw(&self, tex: Option<&Texture2D>) {
        if let Some(tex) = tex {
            // Draw highlight for reservation
            if self.state == VehicleState::Reserved {
                draw_rectangle_ex(
                    self.pos.x, self.pos.y, CAR_LENGTH + 6.0, CAR_WIDTH + 6.0,
                    DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: self.heading, color: WHITE },
                );
            }

            // Draw the texture centered at `self.pos` and scaled to CAR_LENGTH x CAR_WIDTH.
            let w = CAR_LENGTH;
            let h = CAR_WIDTH;
            let x = self.pos.x - w * 0.5;
            let y = self.pos.y - h * 0.5;
            draw_texture_ex(tex, x, y, origin_color(self.origin),
                DrawTextureParams { dest_size: Some(vec2(w, h)), rotation: self.heading, ..Default::default() });
            return;
        }

        // --- Fallback primitive rendering ---
        let body_color  = origin_color(self.origin);
        let win_color   = Color { r: 0.88, g: 0.94, b: 1.0,  a: 0.90 };
        let wheel_color = Color { r: 0.18, g: 0.18, b: 0.18, a: 1.0  };
        let head_color  = Color { r: 1.0,  g: 1.0,  b: 0.65, a: 0.88 };

        // Local axes in screen space
        let fwd = Vec2::new(self.heading.cos(),  self.heading.sin());
        let rgt = Vec2::new(-self.heading.sin(), self.heading.cos());

        // White outline when reservation is granted
        if self.state == VehicleState::Reserved {
            draw_rectangle_ex(
                self.pos.x, self.pos.y, CAR_LENGTH + 6.0, CAR_WIDTH + 6.0,
                DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: self.heading, color: WHITE },
            );
        }

        // Four wheels (drawn before body so body overlaps their inner edges)
        for &(fs, rs) in &[(1.0_f32, 1.0_f32), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
            let wp = self.pos
                + fwd * (CAR_LENGTH * 0.38 * fs)
                + rgt * (CAR_WIDTH  * 0.52 * rs);
            draw_rectangle_ex(wp.x, wp.y, 8.0, 4.0,
                DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: self.heading, color: wheel_color },
            );
        }

        // Body
        draw_rectangle_ex(
            self.pos.x, self.pos.y, CAR_LENGTH, CAR_WIDTH,
            DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: self.heading, color: body_color },
        );

        // Windshield (front third of body, lighter)
        let win_off = fwd * (CAR_LENGTH * 0.18);
        draw_rectangle_ex(
            self.pos.x + win_off.x, self.pos.y + win_off.y,
            CAR_LENGTH * 0.28, CAR_WIDTH * 0.70,
            DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: self.heading, color: win_color },
        );

        // Headlights — thin yellow strip at nose
        let nose_off = fwd * (CAR_LENGTH * 0.47);
        draw_rectangle_ex(
            self.pos.x + nose_off.x, self.pos.y + nose_off.y,
            CAR_LENGTH * 0.08, CAR_WIDTH * 0.70,
            DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: self.heading, color: head_color },
        );
    }
}

// ── Colour by origin ──────────────────────────────────────────────────────────
fn origin_color(dir: Direction) -> Color {
    match dir {
        Direction::North => Color { r: 0.25, g: 0.55, b: 1.00, a: 1.0 }, // blue
        Direction::South => Color { r: 1.00, g: 0.30, b: 0.30, a: 1.0 }, // red
        Direction::East  => Color { r: 1.00, g: 0.80, b: 0.10, a: 1.0 }, // yellow
        Direction::West  => Color { r: 0.25, g: 0.85, b: 0.35, a: 1.0 }, // green
    }
}

// ── Initial heading per origin ────────────────────────────────────────────────
fn initial_heading(origin: Direction) -> f32 {
    match origin {
        Direction::North =>  PI / 2.0, // facing south (↓)
        Direction::South => -PI / 2.0, // facing north (↑)
        Direction::East  =>  PI,       // facing west  (←)
        Direction::West  =>  0.0,      // facing east  (→)
    }
}

// ── Path builder ──────────────────────────────────────────────────────────────

/// Returns the off-screen spawn position for the given origin/route.
/// Used by the spawner to check whether a new vehicle would overlap an existing one.
pub fn spawn_pos(origin: Direction, route: Route) -> Vec2 {
    let entry = entry_point(origin, route);
    spawn_point(origin, entry)
}

/// Generates the complete waypoint list from off-screen spawn to off-screen despawn.
fn build_path(origin: Direction, route: Route) -> Vec<Vec2> {
    let mut pts: Vec<Vec2> = Vec::new();

    // Spawn: off-screen, centred on the correct lane
    let entry = entry_point(origin, route);
    pts.push(spawn_point(origin, entry));

    // Through the intersection
    match route {
        Route::Straight => {
            pts.push(entry);
            pts.push(straight_exit(origin));
        }
        Route::Right => {
            let (center, start_a, sweep) = right_arc_params(origin);
            pts.extend(arc_waypoints(center, RIGHT_R, start_a, sweep, RIGHT_STEPS));
        }
        Route::Left => {
            let (center, start_a, sweep) = left_arc_params(origin);
            pts.extend(arc_waypoints(center, LEFT_R, start_a, sweep, LEFT_STEPS));
        }
    }

    // Despawn: off-screen, in the exit direction
    let exit = *pts.last().unwrap();
    pts.push(despawn_point(origin, route, exit));

    pts
}

// ── Waypoint helpers ──────────────────────────────────────────────────────────

/// N+1 evenly-spaced points along a circular arc (includes both endpoints).
fn arc_waypoints(center: Vec2, radius: f32, start: f32, sweep: f32, n: usize) -> Vec<Vec2> {
    (0..=n)
        .map(|i| {
            let a = start + (i as f32 / n as f32) * sweep;
            center + Vec2::new(a.cos(), a.sin()) * radius
        })
        .collect()
}

// ── Entry / exit geometry ──────────────────────────────────────────────────────

/// Point on the intersection boundary where a vehicle from `origin` in `route` enters.
fn entry_point(origin: Direction, route: Route) -> Vec2 {
    use Direction::*; use Route::*;
    match (origin, route) {
        // Southbound (from north): west half, x increases westward for r
        (North, Right)    => vec2(SB_R_X, INTER_T),
        (North, Straight) => vec2(SB_S_X, INTER_T),
        (North, Left)     => vec2(SB_L_X, INTER_T),
        // Northbound (from south): east half
        (South, Left)     => vec2(NB_L_X, INTER_B),
        (South, Straight) => vec2(NB_S_X, INTER_B),
        (South, Right)    => vec2(NB_R_X, INTER_B),
        // Westbound (from east): north half
        (East,  Right)    => vec2(INTER_R, WB_R_Y),
        (East,  Straight) => vec2(INTER_R, WB_S_Y),
        (East,  Left)     => vec2(INTER_R, WB_L_Y),
        // Eastbound (from west): south half
        (West,  Left)     => vec2(INTER_L, EB_L_Y),
        (West,  Straight) => vec2(INTER_L, EB_S_Y),
        (West,  Right)    => vec2(INTER_L, EB_R_Y),
    }
}

/// Exit point for a straight route (opposite boundary, same lane centre).
fn straight_exit(origin: Direction) -> Vec2 {
    match origin {
        Direction::North => vec2(SB_S_X, INTER_B),
        Direction::South => vec2(NB_S_X, INTER_T),
        Direction::East  => vec2(INTER_L, WB_S_Y),
        Direction::West  => vec2(INTER_R, EB_S_Y),
    }
}

/// Arc parameters (center, start_angle, sweep) for a right turn.
/// All right turns sweep +PI/2 (counter-clockwise in standard math = clockwise on screen).
fn right_arc_params(origin: Direction) -> (Vec2, f32, f32) {
    let sweep = PI / 2.0;
    match origin {
        Direction::North => (vec2(INTER_L, INTER_T),  0.0,           sweep), // SW corner
        Direction::South => (vec2(INTER_R, INTER_B),  PI,            sweep), // NE corner
        Direction::East  => (vec2(INTER_R, INTER_T),  PI / 2.0,      sweep), // SE corner
        Direction::West  => (vec2(INTER_L, INTER_B), -PI / 2.0,      sweep), // NW corner
    }
}

/// Arc parameters for a left turn.
/// All left turns sweep -PI/2 (clockwise in standard math = counter-clockwise on screen).
fn left_arc_params(origin: Direction) -> (Vec2, f32, f32) {
    let sweep = -PI / 2.0;
    match origin {
        Direction::North => (vec2(INTER_R, INTER_T),  PI,            sweep),
        Direction::South => (vec2(INTER_L, INTER_B),  0.0,           sweep),
        Direction::East  => (vec2(INTER_R, INTER_B),  3.0 * PI / 2.0, sweep),
        Direction::West  => (vec2(INTER_L, INTER_T),  PI / 2.0,      sweep),
    }
}

// ── Spawn / despawn off-screen ────────────────────────────────────────────────

fn spawn_point(origin: Direction, entry: Vec2) -> Vec2 {
    match origin {
        Direction::North => vec2(entry.x, -60.0),
        Direction::South => vec2(entry.x, WIN_H + 60.0),
        Direction::East  => vec2(WIN_W + 60.0, entry.y),
        Direction::West  => vec2(-60.0,         entry.y),
    }
}

fn despawn_point(origin: Direction, route: Route, exit: Vec2) -> Vec2 {
    match exit_direction(origin, route) {
        Direction::North => vec2(exit.x, -60.0),
        Direction::South => vec2(exit.x, WIN_H + 60.0),
        Direction::East  => vec2(WIN_W + 60.0, exit.y),
        Direction::West  => vec2(-60.0,         exit.y),
    }
}

/// The compass direction a vehicle heads after leaving the intersection.
fn exit_direction(origin: Direction, route: Route) -> Direction {
    use Direction::*; use Route::*;
    match (origin, route) {
        (North, Right)    => West,
        (North, Straight) => South,
        (North, Left)     => East,
        (South, Right)    => East,
        (South, Straight) => North,
        (South, Left)     => West,
        (East,  Right)    => North,
        (East,  Straight) => West,
        (East,  Left)     => South,
        (West,  Right)    => South,
        (West,  Straight) => East,
        (West,  Left)     => North,
    }
}
