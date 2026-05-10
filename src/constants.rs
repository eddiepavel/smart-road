pub const WIN_W: f32 = 800.0;
pub const WIN_H: f32 = 800.0;

// Vehicle dimensions (pixels)
pub const CAR_LENGTH: f32 = 28.0;
pub const CAR_WIDTH:  f32 = 18.0;

// Speed levels (pixels / second)
pub const SPEED_SLOW:   f32 = 80.0;
pub const SPEED_NORMAL: f32 = 150.0;
pub const SPEED_FAST:   f32 = 230.0;

// Minimum gap a vehicle must keep from the one ahead (pixels)
pub const SAFE_DIST: f32 = 24.0;

// How far before the intersection box edge a vehicle stops to wait for a reservation (pixels)
pub const STOP_MARGIN: f32 = 40.0;

// Intersection tile grid
pub const GRID_N:    usize = 10;   // tiles per side
pub const TILE_PX:   f32   = 24.0; // pixels per tile (ROAD_W / GRID_N)
pub const TIME_SLOT: f32   = 0.1;  // seconds per reservation slot

pub const LANE_W: f32 = 40.0;
pub const CENTER_X: f32 = WIN_W / 2.0;   // 400.0
pub const CENTER_Y: f32 = WIN_H / 2.0;   // 400.0
pub const ROAD_HALF: f32 = LANE_W * 3.0; // 120.0  (3 lanes per direction)
pub const ROAD_W: f32 = ROAD_HALF * 2.0; // 240.0  (6 lanes total per road)

// Intersection box boundaries
pub const INTER_L: f32 = CENTER_X - ROAD_HALF; // 280.0
pub const INTER_R: f32 = CENTER_X + ROAD_HALF; // 520.0
pub const INTER_T: f32 = CENTER_Y - ROAD_HALF; // 280.0
pub const INTER_B: f32 = CENTER_Y + ROAD_HALF; // 520.0

// ── Lane centre X positions (vertical road) ──────────────────────────────────
// Southbound occupies west half  (x: 280–400). Driver faces south → right = west.
// Rightmost lane (r) is westmost; leftmost lane (l) is eastmost (closest to centre).
pub const SB_R_X: f32 = INTER_L + LANE_W * 0.5; // 300  turns right → west
pub const SB_S_X: f32 = INTER_L + LANE_W * 1.5; // 340  straight   → south
pub const SB_L_X: f32 = INTER_L + LANE_W * 2.5; // 380  turns left → east

// Northbound occupies east half  (x: 400–520). Driver faces north → right = east.
pub const NB_L_X: f32 = CENTER_X + LANE_W * 0.5; // 420  turns left  → west
pub const NB_S_X: f32 = CENTER_X + LANE_W * 1.5; // 460  straight    → north
pub const NB_R_X: f32 = CENTER_X + LANE_W * 2.5; // 500  turns right → east

// ── Lane centre Y positions (horizontal road) ─────────────────────────────────
// Westbound occupies north half  (y: 280–400). Driver faces west → right = north.
pub const WB_R_Y: f32 = INTER_T + LANE_W * 0.5; // 300  turns right → north
pub const WB_S_Y: f32 = INTER_T + LANE_W * 1.5; // 340  straight    → west
pub const WB_L_Y: f32 = INTER_T + LANE_W * 2.5; // 380  turns left  → south

// Eastbound occupies south half  (y: 400–520). Driver faces east → right = south.
pub const EB_L_Y: f32 = CENTER_Y + LANE_W * 0.5; // 420  turns left  → north
pub const EB_S_Y: f32 = CENTER_Y + LANE_W * 1.5; // 460  straight    → east
pub const EB_R_Y: f32 = CENTER_Y + LANE_W * 2.5; // 500  turns right → south
