#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>

/*
 * Player-car model attribution:
 *
 * Adapted from "Skyline R32 - Super Drift 3D" by Aeroux Games 3D:
 * https://sketchfab.com/3d-models/skyline-r32-super-drift-3d-c498ca03fa8b4758996c76b26760d5b9
 *
 * Original model licensed under Creative Commons Attribution 4.0:
 * https://creativecommons.org/licenses/by/4.0/
 *
 * Converted and modified for Courier32's low-poly software renderer.
 * Changes include revised geometry, rebuilt wheels, glass/pillars,
 * colors, lighting details, and conversion to embedded C arrays.
 */

// User Config
// Leave false for the normal T-Display-S3 orientation. Set true to rotate the screen 180 degrees.
#define DISPLAY_FLIP_180 false
#define DISPLAY_ROTATION (DISPLAY_FLIP_180 ? 1 : 3)
// HUD performance readout (bottom-left corner).
#define SHOW_FPS true           // show the frames-per-second counter
#define SHOW_FRAME_TIMING true // also show CPU render time per frame in ms

// Game Constants
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 170
#define BTN_LEFT_PIN 0
#define BTN_RIGHT_PIN 14
#define BUTTON_DEBOUNCE_MS 18UL
#define MAX_SPEED 110.0f // km/h, city concept speed cap
// Frame pacing target (microseconds per frame). The dense city renderer does
// not reliably hit 60 fps, and an UNEVEN rate is what reads as stutter -- so we
// lock to a rate the hardware can hold and deliver it evenly. The pacing only
// ADDS wait to early frames; a frame already over budget is never slowed, so a
// higher target is never worse than a lower one, just less even where headroom
// runs out. 22222 = 45 fps, now reachable in the city centre after the
// view-cone culling pass (was 33333 = a rock-steady 30 fps). Watch the HUD
// "worst ms": under ~22 ms means 45 fps is stable; if the densest downtown view
// spikes past it and reads as a hitch, drop back to 33333. 18000 = ~55 fps,
// 16666 = 60 fps; 0UL = uncapped benchmark.
#define FRAME_TIME_US 22222UL
#define TURN_ROLL_AMOUNT 0.04f
#define TURN_STEER_VISUAL_AMOUNT 0.16f
#define TURN_VISUAL_RESPONSE 7.5f
// Chase camera: sits CHASE_CAM_DIST behind the car along the (smoothed) car
// heading, CHASE_CAM_HEIGHT above the ground, and turns with the car.
#define CHASE_CAM_DIST 5.4f
#define CHASE_CAM_HEIGHT 1.55f
#define CHASE_CAM_YAW_RESPONSE 4.5f
#define CAR_TURN_RATE 2.2f   // rad/s at parking speed; tapers off with speed
// Crash knockback: hitting another car throws the player along the contact
// normal with a DECAYING impulse velocity, kept separate from engine speed, so
// a crash physically shoves the car instead of just scrubbing speed. Magnitude
// is tiered to the impact speed (km/h), matching the brief:
//   10-40  light bounce  (~0.6-1.6 m)
//   41-70  firmer shove  (~1.6-2.9 m)
//   71+    violent throw (several metres, capped)
// An impulse's total displacement is ~ v0 / CRASH_DAMP (exponential decay).
#define CRASH_MIN_KPH 10.0f  // below this a tap doesn't throw the car
#define CRASH_DAMP 4.0f      // knockback velocity decay rate (1/s); ~0.25 s feel
#define CRASH_MAX_VEL 20.0f  // cap on accumulated knockback speed (m/s)
// The OTHER car (NPC traffic) gets the visible shove -- the chase camera is
// glued to the player, so launching the thing you hit is what reads on screen.
// It's knocked harder than the player, slides further (slower decay), then
// eases back onto its lane so it rejoins traffic.
#define NPC_KNOCK_SCALE 1.7f  // NPC shove vs the player's own knockback
#define NPC_KNOCK_DAMP 2.6f   // NPC knockback velocity decay (1/s); slides further
#define NPC_KNOCK_RETURN 0.7f // how fast a knocked car eases back to its lane (1/s)
#define NPC_KNOCK_MAX 28.0f   // cap on NPC knockback speed (m/s)
// NPC car LOD: full player-grade mesh within _IN metres, drop to the LOD coupe
// past _OUT (hysteresis band kills boundary flip-flop), at most _MAX full meshes
// per frame. June 2026: extended the full-mesh range 3x (was 24/30, cap 3) and
// lifted the per-frame cap to all cars. Verified 30 fps stable on HW at 4x
// (96/110); pulled back to 3x to leave headroom -- the extra range past ~72 m
// isn't really noticeable. Revert to 24.0f / 30.0f / 3 to restore old behavior.
#define NPC_FULLMESH_IN  72.0f          // full mesh within this depth (was 24)
#define NPC_FULLMESH_OUT 90.0f          // fall back to LOD coupe past this (was 30)
#define NPC_FULLMESH_MAX CITY_NPC_COUNT // full meshes per frame (was 3 = all cars)
#define MODEL_MAX_VERTICES 256
#define MODEL_MAX_FACES 384
// Panel write clock. 20 MHz is the known-good speed for the T-Display-S3's
// ST7789. Higher values (e.g. 24-32 MHz) may work on some units but can show
// as shimmering static / missing screen regions -- raise only in small steps.
#define LCD_BUS_WRITE_HZ 20000000
// Async DMA frame push (double buffered). Set to 0 to fall back to blocking
// pushSprite if the display ever misbehaves -- the game then runs slower but
// uses the exact same proven transfer path as the original code.
#define USE_DMA_PUSH 1
#define FOG_COLOR 0xAE7C            // hazy horizon blue-grey (RGB565)
#define CITY_ROAD_COUNT 24
#define CITY_INTERSECTION_COUNT 22
#define CITY_LAYOUT_BUILDING_COUNT 27
#define CITY_SIDEWALK_COUNT 4
#define CITY_PARK_COUNT 2
#define CITY_POND_COUNT 2
#define CITY_PROP_COUNT 85
#define CITY_NPC_COUNT 10
#define CITY_ROUTE_POINTS 8
#define MAX_PARTICLES 44
// City ambience: one full sunrise-to-sunrise day/night cycle takes this many
// real seconds. The clock drives the sky palette, the sun/moon/stars, street
// and vehicle lighting, and lit building windows.
#define CITY_DAY_SECONDS 240.0f
#define CITY_START_TOD 0.17f        // 0.0 sunrise, 0.25 noon, 0.5 sunset, 0.75 midnight
#define CITY_NIGHT_TINT 0x0863      // world colors blend toward this dark blue at night
#define CITY_PED_COUNT 14
#define CITY_STAR_COUNT 42

// 3D Math Structures
struct Point3D {
    float x, y, z;
};

struct Point2D {
    float x, y;
};

struct Face {
    uint8_t indices[4];
    uint8_t num_vertices;
    uint8_t flags;  // bit0: double-sided (exempt from backface culling)
    uint16_t color; // 0xFFFF means use base color
};

// Global Rendering Variables
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_Parallel8 _bus;
    lgfx::Light_PWM _light;

public:
    LGFX() {
        auto bus_cfg = _bus.config();
        bus_cfg.freq_write = LCD_BUS_WRITE_HZ;
        bus_cfg.pin_wr = 8;
        bus_cfg.pin_rd = 9;
        bus_cfg.pin_rs = 7;
        bus_cfg.pin_d0 = 39;
        bus_cfg.pin_d1 = 40;
        bus_cfg.pin_d2 = 41;
        bus_cfg.pin_d3 = 42;
        bus_cfg.pin_d4 = 45;
        bus_cfg.pin_d5 = 46;
        bus_cfg.pin_d6 = 47;
        bus_cfg.pin_d7 = 48;
        _bus.config(bus_cfg);
        _panel.setBus(&_bus);

        auto panel_cfg = _panel.config();
        panel_cfg.pin_cs = 6;
        panel_cfg.pin_rst = 5;
        panel_cfg.pin_busy = -1;
        panel_cfg.panel_width = 170;
        panel_cfg.panel_height = 320;
        panel_cfg.memory_width = 240;
        panel_cfg.memory_height = 320;
        panel_cfg.offset_x = 35;
        panel_cfg.offset_y = 0;
        panel_cfg.offset_rotation = 0;
        panel_cfg.dummy_read_pixel = 8;
        panel_cfg.dummy_read_bits = 1;
        panel_cfg.readable = false;
        panel_cfg.rgb_order = false;
        panel_cfg.invert = true;
        panel_cfg.dlen_16bit = false;
        panel_cfg.bus_shared = false; // bus is exclusive to the panel; keeps DMA pushes async
        _panel.config(panel_cfg);

        auto light_cfg = _light.config();
        light_cfg.pin_bl = 38;
        light_cfg.invert = false;
        light_cfg.freq = 44100;
        light_cfg.pwm_channel = 0;
        _light.config(light_cfg);
        _panel.setLight(&_light);

        setPanel(&_panel);
    }
};

LGFX tft;
// Two full-screen framebuffers: while one streams to the panel over DMA, the
// next frame renders into the other. All draw code keeps using the name
// "sprite" through the alias below; loop() flips fb_idx after each push.
LGFX_Sprite fb[2] = { LGFX_Sprite(&tft), LGFX_Sprite(&tft) };
uint8_t fb_idx = 0;
bool use_dma = false;
#define sprite fb[fb_idx]

float cam_x = 0.0f;
float cam_y = 0.0f;
float cam_z = 0.0f;
float cam_yaw = 0.0f;
float cam_pitch = 0.0f;
float cam_cos_yaw = 1.0f;
float cam_sin_yaw = 0.0f;
float cam_cos_pitch = 1.0f;
float cam_sin_pitch = 0.0f;

float center_x = 160.0f;
float center_y = 85.0f;
float fov = 180.0f;

// Active light direction in WORLD space (the sun during the race, animated in
// the menu). draw3DModel rotates it into model space once per call, so each
// face is lit with a single dot product against its precomputed normal.
float g_light_x = 0.577f;
float g_light_y = 0.707f;
float g_light_z = -0.408f;

// 3D model data: winding-corrected, outward-facing, with precomputed unit
// face normals for one-dot-product lighting and exact backface culling.
// The data tables themselves live at the BOTTOM of this file so the game
// code stays together; regenerate them with tools/gen_models.py.
// ===== GENERATED MODEL DECLARATIONS (tools/gen_models.py) -- do not hand-edit =====
#define CAR_NUM_VERTICES 144
#define CAR_NUM_FACES 232
extern const Point3D car_vertices[CAR_NUM_VERTICES];
extern const Face car_faces[CAR_NUM_FACES];
extern const Point3D car_normals[CAR_NUM_FACES];
#define WHEEL_NUM_VERTICES 31
#define WHEEL_NUM_FACES 30
extern const Point3D wheel_vertices[WHEEL_NUM_VERTICES];
extern const Face wheel_faces[WHEEL_NUM_FACES];
extern const Point3D wheel_normals[WHEEL_NUM_FACES];
#define STATUE_NUM_VERTICES 80
#define STATUE_NUM_FACES 51
extern const Point3D statue_vertices[STATUE_NUM_VERTICES];
extern const Face statue_faces[STATUE_NUM_FACES];
extern const Point3D statue_normals[STATUE_NUM_FACES];
#define RAILING_NUM_VERTICES 56
#define RAILING_NUM_FACES 35
extern const Point3D railing_vertices[RAILING_NUM_VERTICES];
extern const Face railing_faces[RAILING_NUM_FACES];
extern const Point3D railing_normals[RAILING_NUM_FACES];
#define LOD_CAR_NUM_VERTICES 50
#define LOD_CAR_NUM_FACES 30
extern const Point3D lod_car_vertices[LOD_CAR_NUM_VERTICES];
extern const Face lod_car_faces[LOD_CAR_NUM_FACES];
extern const Point3D lod_car_normals[LOD_CAR_NUM_FACES];
#define BILLBOARD_NUM_VERTICES 8
#define BILLBOARD_NUM_FACES 1
extern const Point3D billboard_vertices[BILLBOARD_NUM_VERTICES];
extern const Face billboard_faces[BILLBOARD_NUM_FACES];
extern const Point3D billboard_normals[BILLBOARD_NUM_FACES];
#define BRIDGE_NUM_VERTICES 24
#define BRIDGE_NUM_FACES 12
extern const Point3D bridge_vertices[BRIDGE_NUM_VERTICES];
extern const Face bridge_faces[BRIDGE_NUM_FACES];
extern const Point3D bridge_normals[BRIDGE_NUM_FACES];
// ===== END GENERATED MODEL DECLARATIONS =====

// Player State Variables
// The car roams freely in world space: position + heading are the source of
// truth; player_w (lateral offset from the nearest road) is derived each frame.
float player_x = 0.0f;
float player_y = 0.0f;          // road surface height under the car
float player_z = 0.0f;
float player_heading = 0.0f;    // world yaw; 0 faces +Z (start straight)
float travel_heading = 0.0f;    // velocity direction; lags the nose on grass (drift)
float g_slip = 0.0f;            // heading minus travel direction (drift angle)
float steer_input = 0.0f;       // smoothed steering value (-1..1)
float cam_chase_yaw = 0.0f;     // smoothed camera heading (lags the car)
float cam_chase_pitch = -0.065f;
float cam_chase_x = 0.0f;
float cam_chase_y = 0.0f;
float cam_chase_z = 0.0f;
float player_w = 0.0f;          // derived lateral offset (-2.5 to 2.5 on road)
float player_speed = 0.0f;      // km/h
float player_roll = 0.0f;       // Visual tilt when turning
float player_pitch = 0.0f;      // Visual nose up/down following the road grade
float player_steer_angle = 0.0f;// Front wheel steer angle
bool player_braking = false;    // both buttons held -> show brake lights
bool btn_left_down = false;
bool btn_right_down = false;
bool btn_left_raw_last = false;
bool btn_right_raw_last = false;
unsigned long btn_left_change_ms = 0;
unsigned long btn_right_change_ms = 0;

int screen_shake_timer = 0;
float g_crash_vx = 0.0f;        // crash knockback velocity (m/s), world +X
float g_crash_vz = 0.0f;        // crash knockback velocity (m/s), world +Z
unsigned long last_time = 0;
unsigned long last_frame_us = 0;
unsigned long fps_window_start_us = 0;
int fps_window_frames = 0;
float measured_fps = 0.0f;
float perf_frame_ms = 0.0f;
float perf_render_ms = 0.0f;
float perf_push_ms = 0.0f;
float perf_worst_ms = 0.0f;   // worst render ms in the last 1 s window
float perf_worst_acc = 0.0f;  // accumulator for the current window

// Screen-space particles: dirt spray off-road, sparks on collisions.
struct Particle {
    float x, y, vx, vy;
    int16_t life;
    uint16_t color;
};
Particle particles[MAX_PARTICLES];
int particle_cursor = 0;

// City minimap: road network projected into a round radar in the top-right
// corner at startup. Round + smaller than the old 70x58 box; the road-net
// bounding box is fit inside the disc and the far corners clip at the rim.
#define CMM_CX 288   // radar centre x (screen is 320 wide)
#define CMM_CY 31    // radar centre y
#define CMM_R  26    // radar radius (disc spans x 262..314, y 5..57)
int16_t cmm_x0[CITY_ROAD_COUNT], cmm_y0[CITY_ROAD_COUNT];
int16_t cmm_x1[CITY_ROAD_COUNT], cmm_y1[CITY_ROAD_COUNT];
float cmm_scale = 1.0f;
float cmm_wx = 0.0f, cmm_wz = 0.0f; // world center of the map bounds

struct CityRoad {
    float x0, z0, x1, z1;
    float width;
    uint16_t color;
};

// City plan: a 12 m grand avenue runs north-south (x=0) from the harbor
// promenade up past the central park. East of it sits a 3x3 downtown grid
// with a public plaza and a construction block; west, an old-town crescent
// of houses; south, the working docks along the harbor bay.
// Roads 0 and 1 (avenue + main cross street) get the wide concrete sidewalks.
const CityRoad city_roads[CITY_ROAD_COUNT] = {
    {    0.0f, -130.0f,    0.0f,  188.0f, 12.0f, 0x3A29 }, //  0 grand avenue (extended N to the rally track)
    { -120.0f,   10.0f,  140.0f,   10.0f,  8.8f, 0x3A29 }, //  1 main cross street
    {   40.0f,  -40.0f,   40.0f,  110.0f,  8.8f, 0x3A29 }, //  2 downtown 1st ave
    {   90.0f,  -40.0f,   90.0f,  110.0f,  8.8f, 0x3A29 }, //  3 downtown 2nd ave
    {  140.0f,  -40.0f,  140.0f,  110.0f,  8.8f, 0x3A29 }, //  4 downtown 3rd ave
    {    0.0f,  -40.0f,  140.0f,  -40.0f,  8.8f, 0x3A29 }, //  5 downtown south st
    {    0.0f,   60.0f,  140.0f,   60.0f,  8.8f, 0x3A29 }, //  6 downtown mid st
    {    0.0f,  110.0f,  140.0f,  110.0f,  8.8f, 0x3A29 }, //  7 downtown north st
    { -120.0f,   10.0f, -125.0f,   45.0f,  8.8f, 0x3A29 }, //  8 old town west arc
    { -125.0f,   45.0f, -110.0f,   80.0f,  8.8f, 0x3A29 }, //  9
    { -110.0f,   80.0f,  -80.0f,  100.0f,  8.8f, 0x3A29 }, // 10
    {  -80.0f,  100.0f,  -45.0f,  108.0f,  8.8f, 0x3A29 }, // 11
    {  -45.0f,  108.0f,    0.0f,  110.0f,  8.8f, 0x3A29 }, // 12 joins avenue north
    { -120.0f,   10.0f, -105.0f,  -30.0f,  8.8f, 0x3A29 }, // 13 old town south arc
    { -105.0f,  -30.0f,  -70.0f,  -50.0f,  8.8f, 0x3A29 }, // 14
    {  -70.0f,  -50.0f,  -30.0f,  -55.0f,  8.8f, 0x3A29 }, // 15
    {  -30.0f,  -55.0f,    0.0f,  -40.0f,  8.8f, 0x3A29 }, // 16 joins avenue south
    {    0.0f, -100.0f,  120.0f, -100.0f,  8.8f, 0x3A29 }, // 17 dock road east
    {  -80.0f, -100.0f,    0.0f, -100.0f,  8.8f, 0x3A29 }, // 18 dock road west
    {   90.0f,  -40.0f,   90.0f, -100.0f,  8.8f, 0x3A29 }, // 19 dock connector
    {  120.0f, -100.0f,  120.0f,  -40.0f,  8.8f, 0x3A29 }, // 20 east dock connector
    {  -60.0f, -130.0f,  100.0f, -130.0f,  8.8f, 0x3A29 }, // 21 harbor promenade
    {  100.0f, -130.0f,  120.0f, -100.0f,  8.8f, 0x3A29 }, // 22 promenade ramp east
    {  -60.0f, -130.0f,  -80.0f, -100.0f,  8.8f, 0x3A29 }  // 23 promenade ramp west
};

struct CityIntersection {
    float x, z;
    float radius;
    bool signals;
};

const CityIntersection city_intersections[CITY_INTERSECTION_COUNT] = {
    // Signalized (avenue gateway + downtown core)
    {    0.0f,   10.0f, 9.5f, true },
    {   90.0f,   10.0f, 8.0f, true },
    {   40.0f,   60.0f, 8.0f, true },
    {   90.0f,  110.0f, 8.0f, true },
    // Plain junctions
    {   40.0f,   10.0f, 8.0f, false },
    {  140.0f,   10.0f, 8.0f, false },
    { -120.0f,   10.0f, 8.0f, false },
    {    0.0f,  -40.0f, 9.5f, false },
    {   40.0f,  -40.0f, 8.0f, false },
    {   90.0f,  -40.0f, 8.0f, false },
    {  120.0f,  -40.0f, 8.0f, false },
    {  140.0f,  -40.0f, 8.0f, false },
    {    0.0f,   60.0f, 9.5f, false },
    {   90.0f,   60.0f, 8.0f, false },
    {  140.0f,   60.0f, 8.0f, false },
    {    0.0f,  110.0f, 9.5f, false },
    {   40.0f,  110.0f, 8.0f, false },
    {  140.0f,  110.0f, 8.0f, false },
    {    0.0f, -100.0f, 9.5f, false },
    {   90.0f, -100.0f, 8.0f, false },
    {  120.0f, -100.0f, 8.0f, false },
    {    0.0f, -130.0f, 9.5f, false }
};

struct CityLot {
    float x, z;
    float w, d, h;
    uint16_t color;
    uint8_t type; // 0 tower, 1 mid-rise, 2 house, 3 warehouse
};

struct CityRect {
    float x, z;
    float w, d;
    uint16_t color;
};

struct CityProp {
    float x, z;
    float heading;
    uint8_t type; // 0 signal, 1 lamp, 2 tree, 3 parked car, 4 sign
    uint16_t color;
};

const CityLot city_lots[CITY_LAYOUT_BUILDING_COUNT] = {
    // Downtown towers (varied heights -- the 30 m one anchors the skyline)
    {   57.0f,  -20.0f, 16.0f, 16.0f, 22.0f, 0x6B6F, 0 },
    {   78.0f,   -8.0f, 13.0f, 13.0f, 15.0f, 0x7C34, 0 },
    {  115.0f,  -15.0f, 24.0f, 20.0f, 26.0f, 0x5B4E, 0 },
    {  106.0f,   27.0f, 13.0f, 13.0f, 19.0f, 0x8C96, 0 },
    {  124.0f,   44.0f, 13.0f, 13.0f, 30.0f, 0x6C32, 0 },
    {   65.0f,   85.0f, 22.0f, 17.0f, 12.0f, 0x7BEF, 1 },
    {  115.0f,   85.0f, 18.0f, 14.0f,  4.2f, 0x632C, 3 }, // construction slab
    // Mid-rise blocks lining the grand avenue
    {  -17.0f,  -15.0f, 12.0f, 16.0f,  9.0f, 0x8C51, 1 },
    {  -17.0f,   35.0f, 12.0f, 18.0f, 11.0f, 0x9CB3, 1 },
    {  -17.0f,   85.0f, 12.0f, 16.0f,  8.5f, 0x8C51, 1 },
    {   18.0f,  -18.0f, 13.0f, 14.0f, 10.0f, 0x7BEF, 1 },
    {   18.0f,   88.0f, 13.0f, 13.0f,  9.0f, 0x9CB3, 1 },
    // Old town houses inside the crescent
    {  -95.0f,   -2.0f,  8.0f,  8.0f,  3.4f, 0xD615, 2 },
    { -100.0f,   38.0f,  8.0f,  9.0f,  3.7f, 0xBDF7, 2 },
    {  -90.0f,   64.0f,  9.0f,  8.0f,  3.4f, 0xC618, 2 },
    {  -70.0f,   84.0f,  8.0f,  8.0f,  3.8f, 0xAD75, 2 },
    {  -48.0f,   90.0f,  9.0f,  8.0f,  3.5f, 0xCE59, 2 },
    {  -55.0f,   70.0f,  8.0f,  8.0f,  3.6f, 0xD615, 2 },
    {  -75.0f,   22.0f,  8.0f,  8.0f,  3.5f, 0xBDF7, 2 },
    {  -84.0f,  -14.0f,  8.0f,  8.0f,  3.4f, 0xC618, 2 },
    {  -58.0f,  -32.0f,  9.0f,  8.0f,  3.7f, 0xAD75, 2 },
    {  -34.0f,  -38.0f,  8.0f,  8.0f,  3.4f, 0xCE59, 2 },
    // Dock warehouses between the dock road and the promenade
    {  -48.0f, -115.0f, 18.0f, 12.0f,  6.0f, 0x5AEB, 3 },
    {  -20.0f, -115.0f, 14.0f, 12.0f,  5.5f, 0x632C, 3 },
    {   25.0f, -115.0f, 18.0f, 12.0f,  6.2f, 0x6B4D, 3 },
    {   55.0f, -115.0f, 16.0f, 12.0f,  5.0f, 0x73AE, 3 },
    {   85.0f, -114.0f, 18.0f, 13.0f,  6.8f, 0x5AEB, 3 }
};

const CityRect city_sidewalks[CITY_SIDEWALK_COUNT] = {
    {  65.0f,   35.0f,  36.0f, 36.0f, 0x9D35 }, // downtown public plaza
    {  62.0f,  -88.0f,  30.0f, 16.0f, 0x94B2 }, // dockside parking lot
    {  20.0f, -138.0f, 150.0f,  9.0f, 0xA552 }, // harbor boardwalk
    {  95.0f,  140.0f,  16.0f, 16.0f, 0x9D35 }  // park fountain plaza
};

const CityRect city_parks[CITY_PARK_COUNT] = {
    {  70.0f, 140.0f, 110.0f, 50.0f, 0x2C8A },  // central park
    { -78.0f,  45.0f,  22.0f, 26.0f, 0x2C8A }   // old town green
};

// Water bodies (game-side assets, not part of the map-editor format):
// drawn as shore-ringed ellipses with animated shimmer, and solid to drive
// into -- the car stops at the waterline with a splash.
struct CityPond {
    float x, z;
    float rx, rz;
};

const CityPond city_ponds[CITY_POND_COUNT] = {
    {  45.0f,  138.0f,  14.0f, 11.0f },  // park pond
    {  20.0f, -167.0f, 105.0f, 22.0f }   // harbor bay
};

// Downtown plaza furniture (game-side assets like ponds -- NOT emitted by the
// map editor, so they survive a map re-import). The 36x36 plaza at (65,35) is
// ringed by a low Roman balustrade with four wide entry gaps (one per side,
// ~12 m / 3 cars), and a posing marble statue stands on each corner. The rail
// runs double as AABB collision so you can only drive in through the gaps.
struct CityRail { float x, z, w, d, heading; };  // centre, half-less footprint, yaw
#define CITY_PLAZA_RAIL_COUNT 8
const CityRail city_plaza_rails[CITY_PLAZA_RAIL_COUNT] = {
    {  53.0f, 17.0f, 12.0f,  0.5f, 0.0f },     // south, west run
    {  77.0f, 17.0f, 12.0f,  0.5f, 0.0f },     // south, east run
    {  53.0f, 53.0f, 12.0f,  0.5f, 0.0f },     // north, west run
    {  77.0f, 53.0f, 12.0f,  0.5f, 0.0f },     // north, east run
    {  47.0f, 23.0f,  0.5f, 12.0f, 1.5708f },  // west, south run
    {  47.0f, 47.0f,  0.5f, 12.0f, 1.5708f },  // west, north run
    {  83.0f, 23.0f,  0.5f, 12.0f, 1.5708f },  // east, south run
    {  83.0f, 47.0f,  0.5f, 12.0f, 1.5708f }   // east, north run
};
struct CityStatue { float x, z, heading; };   // heading faces the plaza centre
#define CITY_STATUE_COUNT 4
const CityStatue city_statues[CITY_STATUE_COUNT] = {
    {  47.0f, 17.0f,  0.7854f },   // SW corner, faces (65,35)
    {  83.0f, 17.0f, -0.7854f },   // SE corner
    {  47.0f, 53.0f,  2.3562f },   // NW corner
    {  83.0f, 53.0f, -2.3562f }    // NE corner
};

// Prop types: 0 signal, 1 lamp, 2 leafy tree, 3 parked car, 4 sign,
// 5 construction barrier, 6 fountain, 7 pine tree, 8 avenue arch.
const CityProp city_props[CITY_PROP_COUNT] = {
    // -- Traffic signals at the four signalized junctions (6)
    {    8.0f,   18.0f, 0.0f, 0, 0xFFFF },
    {   -8.0f,    2.0f, 0.0f, 0, 0xFFFF },
    {   98.0f,   18.0f, 0.0f, 0, 0xFFFF },
    {   82.0f,    2.0f, 0.0f, 0, 0xFFFF },
    {   48.0f,   68.0f, 0.0f, 0, 0xFFFF },
    {   98.0f,  118.0f, 0.0f, 0, 0xFFFF },
    // -- Avenue gateway arches (2)
    {    0.0f, -118.0f, 0.0f, 8, 0xFFFF },
    {    0.0f,  152.0f, 0.0f, 8, 0xFFFF },
    // -- Fountains: downtown plaza + park plaza (2)
    {   65.0f,   35.0f, 0.0f, 6, 0xFFFF },
    {   95.0f,  140.0f, 0.0f, 6, 0xFFFF },
    // -- Construction barriers ringing the building site at (115,85) (8)
    {  106.0f,   77.0f, 0.0f, 5, 0xFFFF },
    {  115.0f,   77.0f, 0.0f, 5, 0xFFFF },
    {  124.0f,   77.0f, 0.0f, 5, 0xFFFF },
    {  106.0f,   93.0f, 0.0f, 5, 0xFFFF },
    {  115.0f,   93.0f, 0.0f, 5, 0xFFFF },
    {  124.0f,   93.0f, 0.0f, 5, 0xFFFF },
    {  105.0f,   85.0f, 1.5708f, 5, 0xFFFF },
    {  125.0f,   85.0f, 1.5708f, 5, 0xFFFF },
    // -- Street lamps: grand avenue both sides (12)
    {   -8.5f, -118.0f, 0.0f, 1, 0xFFFF },
    {   -8.5f,  -70.0f, 0.0f, 1, 0xFFFF },
    {   -8.5f,  -20.0f, 0.0f, 1, 0xFFFF },
    {   -8.5f,   35.0f, 0.0f, 1, 0xFFFF },
    {   -8.5f,   85.0f, 0.0f, 1, 0xFFFF },
    {   -8.5f,  130.0f, 0.0f, 1, 0xFFFF },
    {    8.5f, -118.0f, 0.0f, 1, 0xFFFF },
    {    8.5f,  -70.0f, 0.0f, 1, 0xFFFF },
    {    8.5f,  -20.0f, 0.0f, 1, 0xFFFF },
    {    8.5f,   35.0f, 0.0f, 1, 0xFFFF },
    {    8.5f,   85.0f, 0.0f, 1, 0xFFFF },
    {    8.5f,  130.0f, 0.0f, 1, 0xFFFF },
    // -- Street lamps: downtown + promenade (10)
    {   30.0f,    2.0f, 0.0f, 1, 0xFFFF },
    {   55.0f,   18.0f, 0.0f, 1, 0xFFFF },
    {   75.0f,    2.0f, 0.0f, 1, 0xFFFF },
    {  105.0f,   18.0f, 0.0f, 1, 0xFFFF },
    {   82.0f,   40.0f, 0.0f, 1, 0xFFFF },
    {   98.0f,   80.0f, 0.0f, 1, 0xFFFF },
    {  125.0f,    2.0f, 0.0f, 1, 0xFFFF },
    {  -50.0f, -138.0f, 0.0f, 1, 0xFFFF },
    {   40.0f, -138.0f, 0.0f, 1, 0xFFFF },
    {   90.0f, -138.0f, 0.0f, 1, 0xFFFF },
    // -- Leafy trees: central park + avenue + plaza (16)
    {   30.0f,  150.0f, 0.0f, 2, 0xFFFF },
    {   55.0f,  162.0f, 0.0f, 2, 0xFFFF },
    {   88.0f,  156.0f, 0.0f, 2, 0xFFFF },
    {  108.0f,  148.0f, 0.0f, 2, 0xFFFF },
    {  118.0f,  162.0f, 0.0f, 2, 0xFFFF },
    {   40.0f,  120.0f, 0.0f, 2, 0xFFFF },
    {   72.0f,  118.0f, 0.0f, 2, 0xFFFF },
    {  100.0f,  124.0f, 0.0f, 2, 0xFFFF },
    {  -72.0f,   40.0f, 0.0f, 2, 0xFFFF },
    {  -84.0f,   52.0f, 0.0f, 2, 0xFFFF },
    {  -78.0f,   30.0f, 0.0f, 2, 0xFFFF },
    {   10.0f,  -90.0f, 0.0f, 2, 0xFFFF },
    {  -10.0f,  -60.0f, 0.0f, 2, 0xFFFF },
    {   10.0f,  120.0f, 0.0f, 2, 0xFFFF },
    {  -10.0f,  135.0f, 0.0f, 2, 0xFFFF },
    {   52.0f,   52.0f, 0.0f, 2, 0xFFFF },
    // -- Pines: old town green + park + harborfront (9)
    {  -74.0f,   55.0f, 0.0f, 7, 0xFFFF },
    {  -82.0f,   38.0f, 0.0f, 7, 0xFFFF },
    {   20.0f,  165.0f, 0.0f, 7, 0xFFFF },
    {  125.0f,  118.0f, 0.0f, 7, 0xFFFF },
    {   15.0f,  160.0f, 0.0f, 7, 0xFFFF },
    {  -55.0f, -125.0f, 0.0f, 7, 0xFFFF },
    {  -20.0f, -125.0f, 0.0f, 7, 0xFFFF },
    {   55.0f, -125.0f, 0.0f, 7, 0xFFFF },
    {   95.0f, -125.0f, 0.0f, 7, 0xFFFF },
    // -- Parked cars: dockside lot rows + avenue curb (10)
    {   50.0f,  -92.0f, 0.0f, 3, 0xFFFF },
    {   58.0f,  -92.0f, 0.0f, 3, 0xFFFF },
    {   66.0f,  -92.0f, 0.0f, 3, 0xFFFF },
    {   74.0f,  -92.0f, 0.0f, 3, 0xFFFF },
    {   50.0f,  -84.0f, 3.1416f, 3, 0xFFFF },
    {   58.0f,  -84.0f, 3.1416f, 3, 0xFFFF },
    {   66.0f,  -84.0f, 3.1416f, 3, 0xFFFF },
    {   74.0f,  -84.0f, 3.1416f, 3, 0xFFFF },
    {    9.0f,  -30.0f, 0.0f, 3, 0xFFFF },
    {   -9.0f,   45.0f, 3.1416f, 3, 0xFFFF },
    // -- Signs (2)
    {  -40.0f, -120.0f, 0.0f, 4, 0xFFFF },
    {  130.0f,   30.0f, 1.5708f, 4, 0xFFFF },
    // -- Grove filling the barren downtown green block (x0-40, z10-60) that sits
    // next to the plaza, just off the grand avenue. Leafy + a couple of pines,
    // all inset well clear of the surrounding streets. (8)
    {   13.0f,   21.0f, 0.0f, 2, 0xFFFF },
    {   27.0f,   23.0f, 0.0f, 2, 0xFFFF },
    {   19.0f,   31.0f, 0.0f, 7, 0xFFFF },
    {   32.0f,   33.0f, 0.0f, 2, 0xFFFF },
    {   12.0f,   41.0f, 0.0f, 2, 0xFFFF },
    {   24.0f,   44.0f, 0.0f, 7, 0xFFFF },
    {   16.0f,   51.0f, 0.0f, 2, 0xFFFF },
    {   30.0f,   50.0f, 0.0f, 2, 0xFFFF }
};

struct CityNPC {
    uint8_t route;
    float progress;
    float speed_ms;     // cruise speed
    uint16_t color;
    float x, z, heading;
    float cur_speed;    // live speed: drops for red lights, the player, crashes
    uint8_t hi_lod;     // 1 = currently drawing the full mesh (hysteresis latch)
    float knock_x, knock_z;   // crash knockback displacement off the lane (m)
    float knock_vx, knock_vz; // crash knockback velocity (m/s); decays to 0
};

CityNPC city_npcs[CITY_NPC_COUNT] = {
    { 0,   0.0f,  8.0f, 0xFFE0, 0, 0, 0 },
    { 1,  34.0f,  7.2f, 0x07FF, 0, 0, 0 },
    { 2,  72.0f,  8.6f, 0xF81F, 0, 0, 0 },
    { 3,  12.0f,  6.8f, 0xFD20, 0, 0, 0 },
    { 4,  95.0f,  7.6f, 0xFFFF, 0, 0, 0 },
    { 5,  48.0f,  8.2f, 0xF800, 0, 0, 0 },
    { 6,  26.0f,  6.4f, 0x07E0, 0, 0, 0 },
    { 7,  18.0f,  9.0f, 0x9CD3, 0, 0, 0 },
    { 8,  62.0f,  7.0f, 0x001F, 0, 0, 0 },
    { 9, 110.0f,  8.4f, 0xC618, 0, 0, 0 }
};

// NPC routes, each a loop of nodes on the real road grid (see the road table
// above). Cars drive these with a right-hand lane offset and smoothed
// cornering applied at runtime, so turns look driven rather than snapped.
const float city_route_x[CITY_NPC_COUNT][CITY_ROUTE_POINTS] = {
    {    0,   40,  140,  140,   40,    0,    0,    0 }, // 0 downtown perimeter
    {   40,   90,  140,  140,   90,   40,   40,   40 }, // 1 downtown inner block
    {    0,    0,    0,    0,   40,   40,   40,    0 }, // 2 avenue + mid st
    {   90,  140,  140,   90,   90,   40,   40,   90 }, // 3 east downtown
    {    0,   90,  120,  120,  140,  140,    0,    0 }, // 4 dock + east loop
    { -120,    0,   40,  140,  140,    0,    0, -120 }, // 5 cross-town sweep
    { -120, -125, -110,  -80,  -45,    0,    0, -120 }, // 6 old town north arc
    { -120, -105,  -70,  -30,    0,    0,    0, -120 }, // 7 old town south arc
    {    0,   40,   90,  140,  140,   90,    0,    0 }, // 8 park & north loop
    {    0,    0,    0,    0,    0,    0,    0,    0 }  // 9 avenue cruise
};

const float city_route_z[CITY_NPC_COUNT][CITY_ROUTE_POINTS] = {
    {   10,   10,   10,  110,  110,  110,   60,   10 }, // 0
    {  -40,  -40,  -40,   60,   60,   60,   10,  -40 }, // 1
    {  110,   60,   10,  -40, -100,  -40,   60,  110 }, // 2
    {  -40,  -40,   60,   60,  110,  110,   10,  -40 }, // 3
    { -100, -100, -100,  -40,  -40,   10,   10, -100 }, // 4
    {   10,   10,   10,   10,   60,   60,   10,   10 }, // 5
    {   10,   45,   80,  100,  108,  110,   10,   10 }, // 6
    {   10,  -30,  -50,  -55,  -40, -100,   10,   10 }, // 7
    {  110,  110,  110,  110,   60,   60,   60,  110 }, // 8
    { -130, -100,  -40,   60,  110,   60,  -40, -130 }  // 9
};

// Per-route arc lengths, cached at startup (these never change). Recomputing
// them every NPC every frame was pure waste -- and a per-frame stutter source.
float g_route_len[CITY_NPC_COUNT];

// Time-of-day state, updated once per frame in city mode. All world draw
// calls read these instead of fixed colors, so the whole city slides through
// sunrise / noon / sunset / night continuously.
float g_wheel_spin = 0.0f;      // accumulated wheel rotation (radians) for spinning wheels
float city_tod = CITY_START_TOD;
float g_sun_el = 1.0f;          // sun elevation: +1 noon, 0 horizon, negative at night
float g_night = 0.0f;           // 0 = full day, 1 = deep night (eased)
uint8_t g_city_dim = 0;         // 0..20 world blend toward CITY_NIGHT_TINT
bool g_lights_on = false;       // street lamps, headlights, lit windows
uint16_t g_horizon565 = FOG_COLOR; // current haze color; distant objects fade to it
int g_sky_top_r = 10, g_sky_top_g = 64, g_sky_top_b = 200;
int g_sky_hor_r = 172, g_sky_hor_g = 211, g_sky_hor_b = 235;

// Sun-cast shadow state, derived from the time of day. Direction points away
// from the sun on the ground plane; length scales object height into shadow
// reach; alpha fades the shadows out through dusk.
float g_shadow_dx = 0.0f, g_shadow_dz = -1.0f;
float g_shadow_len = 1.2f;
uint8_t g_shadow_a = 7;

// Sun screen position from the last background pass, for the lens flare.
// x stays far negative while the sun is off-screen or below the horizon.
int g_sun_sx = -1000, g_sun_sy = 0;
uint16_t g_sun_core565 = 0xFFFF;

// Per-frame budget of NPC cars allowed to draw the full 368-face mesh. The
// scene queue draws far-to-near, so near cars (the only ones that qualify for
// the full mesh) come last and naturally claim this budget -- capping it
// bounds the worst-case frame when traffic clusters at a junction.
int g_fullmesh_left = 0;

// Pedestrians wander the sidewalk band beside a road, bouncing between its
// ends. Purely ambient: they hop aside if the player bears down on them.
struct CityPed {
    uint8_t road;       // index into city_roads
    int8_t dir;         // +1 / -1 along the road
    int8_t side;        // sidewalk side of the road
    float t;            // 0..1 position along the road
    float speed;        // m/s walking pace
    float phase;        // leg-swing animation phase
    float dodge;        // extra lateral offset after dodging the player
    uint16_t color;     // shirt color
    float x, z;         // derived world position
    float down;         // >0: knocked over, lying in the street for this long
};
CityPed city_peds[CITY_PED_COUNT];

enum GameState {
    START_SCREEN,
    PLAYING,
    FINISHED
};

GameState current_state = START_SCREEN;

// ---- City Courier gameplay loop -------------------------------------------
// A delivery job: drive to the glowing pickup beacon, stop, then haul the fare
// to the drop-off beacon before the clock runs out. Each drop pays a fare and
// buys more time; running the clock to zero ends the run with a score.
#define COURIER_RUN_SECONDS   75.0f   // starting time bank
#define COURIER_RADIUS        10.0f   // stop within this of the beacon to trigger
#define COURIER_STOP_KMH      9.0f    // ...and be slower than this
#define COURIER_MIN_TRIP      45.0f   // targets spawn at least this far apart
// Hand-placed pickup/drop spots in the map's centre (downtown core, plaza, mid
// streets). The road-point sampler in courierPickTarget leans toward the long
// perimeter roads, so the middle of the city almost never won the draw -- in
// random mode we now sometimes pull a target from this list so deliveries
// actually happen downtown. Each sits on a drivable road so the beacon is
// reachable; verified against city_roads (road id in the comment).
#define COURIER_CENTER_COUNT 6
#define COURIER_CENTER_X    45.0f  // downtown anchor (~centroid of the centre spots)
#define COURIER_CENTER_Z    35.0f
#define COURIER_EDGE_DIST   85.0f  // a centre target is allowed only when the OTHER
                                   // trip end is at least this far from the anchor,
                                   // so fares read as centre<->edge, never centre<->centre
const float courier_center_spots[COURIER_CENTER_COUNT][2] = {
    {  0.0f, 35.0f },  // grand avenue, mid    (road 0)
    { 40.0f, 35.0f },  // downtown 1st ave core (road 2)
    { 90.0f, 35.0f },  // downtown 2nd ave core (road 3)
    { 20.0f, 60.0f },  // downtown mid st, west (road 6)
    { 65.0f, 60.0f },  // downtown mid st, plaza (road 6)
    { 40.0f, 10.0f },  // central crossroads    (roads 1 & 2)
};
bool  courier_started = false;        // clock is frozen until the first pickup
bool  courier_has_fare = false;       // false: heading to pickup, true: carrying
float courier_tx = 0.0f, courier_tz = 0.0f;   // current target (beacon) position
float courier_px = 0.0f, courier_pz = 0.0f;   // where the fare was picked up
float courier_time_left = COURIER_RUN_SECONDS;
float courier_fare_clock = 0.0f;      // seconds since pickup (payout scales down)
long  courier_money = 0;
int   courier_deliveries = 0;
int   courier_streak = 0;             // consecutive clean drops -> fare multiplier
int   courier_popup_ms = 0;          // brief "+$NN" / "FARE!" flash timer (frames)
long  courier_popup_amount = 0;
unsigned long courier_best = 0;       // best run earnings this power-cycle

// Function Declarations
void resetRaceState();
void updateButtonInputs();
void updatePhysics(float dt);
void renderTrackAndObjects();
void drawHUD();
void drawFinished();
void drawStartScreen();
void drawMenuGarage(float t);
void drawMenuGarageProps(float t);
void drawMenuQuad3D(float x0, float y0, float z0, float x1, float y1, float z1,
                    float x2, float y2, float z2, float x3, float y3, float z3,
                    uint16_t color);
void drawMenuEllipse(int cx, int cy, int w, int h, uint16_t color);
void drawMenuShadow(int cx, int cy, int w, int h);
void drawQuad(float sx0, float sy0, float sx1, float sy1, float sx2, float sy2, float sx3, float sy3, uint16_t color);
void draw3DModel(const Point3D* vertices, int num_vertices,
                 const Face* faces, const Point3D* normals, int num_faces,
                 float pos_x, float pos_y, float pos_z,
                 float rot_x, float rot_y, float rot_z,
                 float scale, uint16_t base_color, uint8_t fog_a);
void drawCarRearGlass(float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z,
                      float scale, bool braking);
void drawCarWheels(float pos_x, float pos_y, float pos_z,
                   float heading, float roll, float scale, float spin);
void drawBillboard(float pos_x, float pos_y, float pos_z, float rot_y, float scale, uint16_t color, uint8_t fog_a);
void drawTreeImpostor(float pos_x, float pos_y, float pos_z, float scale, uint8_t fog_a);
void updatePlayerCityPosition();
float cityRoadSignedDistance(float x, float z);
void updateCityNPCs(float dt);
void updateCityTimeOfDay(float dt);
void drawCityBackground(int horizon_y, float yaw);
void resetCityPeds();
void updateCityPeds(float dt);
void drawCityPed(int i, uint8_t fog_a);
void drawCityNPCCar(int i, uint8_t fog_a);
void drawCityVehicleLights(float x, float z, float heading, float speed_ratio, bool braking);
void drawHeadlightBeam(float x, float z, float heading, float nose, float near_hw, float len, float far_hw, uint8_t add);
void brightenEllipse(int cx, int cy, int rx, int ry, uint16_t tint, uint8_t add);
void rmwTriangle(float fx0, float fy0, float fx1, float fy1, float fx2, float fy2,
                 uint16_t tint, uint8_t amt, bool darken);
void drawCityPanorama(int horizon_y, float yaw);
void drawCityClouds(int horizon_y, float yaw);
void drawSunFlare();
void drawCityBuildingShadows();
void shadowWorldQuad(float x0, float y0, float z0, float x1, float y1, float z1,
                     float x2, float y2, float z2, float x3, float y3, float z3, uint8_t darken);
void cityParkedCarCollision();
int cityTrafficPhase(bool ns_axis); // 0 green, 1 amber, 2 red
void buildCityMinimap();
void drawCityMinimap();
void courierStartRun();
void courierPickTarget(bool first);
void courierUpdate(float dt);
void drawCourierBeacon();
void drawCourierGameOver();
void getCityRoutePose(uint8_t route, float progress, float& x, float& z, float& heading);
void renderCityWorld();
void drawCityGround();
void drawCityRoads();
void drawCityPark();
void drawCitySceneObjects();
void drawCityPlayerCar();
void drawTrafficLight(float x, float z, uint8_t state);
void drawStreetLamp(float x, float z);
void drawParkedCar(float x, float z, float heading, uint16_t color, uint8_t fog_a);
void buildCityRoutes();
void drawCityPonds();
void drawCityPond(int i, uint8_t fog_a);
void drawCityParkPaths();
void drawCityTrack();
void drawConstructionBarrier(float x, float z, float heading, uint8_t fog_a);
void drawFountain(float x, float z, uint8_t fog_a);
void drawStatue(float x, float z, float heading, uint8_t fog_a);
void drawCityRailing(const CityRail& r, uint8_t fog_a);
void drawPineTree(float x, float z, float scale, uint8_t fog_a);
void drawAvenueArch(float x, float z, float heading, uint8_t fog_a);
bool cityPondBlocked(float x, float z, float& out_nx, float& out_nz);
bool cityBuildingCollision(float& x, float& z, float prev_x, float prev_z, float& out_nx, float& out_nz);
bool cityOnTrack(float x, float z);
bool cityTrackFenceHit(float& x, float& z, float px, float pz, float& out_nx, float& out_nz);
bool cityPointOccludedByBuilding(float x, float z, float height);
void drawWorldQuadClipped(float x0, float y0, float z0, float x1, float y1, float z1,
                          float x2, float y2, float z2, float x3, float y3, float z3,
                          uint16_t color);
void drawCityRect(float x0, float z0, float x1, float z1, uint16_t color);
void drawCityStrip(float x0, float z0, float x1, float z1, float width, uint16_t color);
void drawCityPlainStrip(float x0, float z0, float x1, float z1, float width, uint16_t color, float lift);
void drawCityBox(float cx, float cz, float w, float d, float h, uint16_t color, uint8_t fog_a, uint16_t seed, bool house = false);
void drawCityHouse(float cx, float cz, float w, float d, float h, uint16_t color, uint8_t fog_a, uint16_t seed);
void drawCityFacadeGrid(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy,
                        uint16_t day_color, uint16_t seed, uint8_t fog_a, int rows, int cols);
bool projectPoint(float wx, float wy, float wz, float& sx, float& sy, float& sz);
void updateCameraTrig();
float approachFloat(float current, float target, float response, float dt);
void spawnParticle(float x, float y, float vx, float vy, int life, uint16_t color);
void spawnImpactSparks(int cx, int cy, float dir);
float crashKnockV0(float impact_kph);
void applyCrashImpulse(float nx, float nz, float impact_kph);
void updateAndDrawParticles();
void shadowEllipse(int cx, int cy, int rx, int ry, uint8_t darken);
void fillGradientRows(int y_start, int y_end, int span_start, int span_len,
                      int r0, int g0, int b0, int r1, int g1, int b1);

// Integer RGB565 shading: i32 is 0..32 (32 = full brightness).
static inline uint16_t shade565(uint16_t c, uint8_t i32) {
    uint32_t rb = ((uint32_t)(c & 0xF81F) * i32) >> 5;
    uint32_t g  = ((uint32_t)(c & 0x07E0) * i32) >> 5;
    return (uint16_t)((rb & 0xF81F) | (g & 0x07E0));
}

// Integer RGB565 blend: alpha32 is 0..32 toward color b.
static inline uint16_t blend565(uint16_t a, uint16_t b, uint8_t alpha32) {
    uint32_t ia = 32 - alpha32;
    uint32_t rb = ((uint32_t)(a & 0xF81F) * ia + (uint32_t)(b & 0xF81F) * alpha32) >> 5;
    uint32_t g  = ((uint32_t)(a & 0x07E0) * ia + (uint32_t)(b & 0x07E0) * alpha32) >> 5;
    return (uint16_t)((rb & 0xF81F) | (g & 0x07E0));
}

static inline uint16_t pack565(int r, int g, int b) {
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

// Time-of-day grading for non-emissive world colors: darken and blue-shift
// toward CITY_NIGHT_TINT as night falls. Emissive surfaces (lit windows,
// lamps, signals, headlights) intentionally skip this.
static inline uint16_t cityGrade(uint16_t c) {
    return g_city_dim ? blend565(c, CITY_NIGHT_TINT, g_city_dim) : c;
}

// Blob shadows shift away from the sun like the cast building shadows do
// (h = rough object height). Zero offset once the sun is down.
static inline void sunBlobOffset(float h, float& ox, float& oz) {
    if (g_shadow_a == 0) { ox = 0.0f; oz = 0.0f; return; }
    ox = g_shadow_dx * h * g_shadow_len * 0.55f;
    oz = g_shadow_dz * h * g_shadow_len * 0.55f;
}

// Stable 32-bit hash for procedural details that must not flicker
// (lit windows, star positions, pedestrian variety).
static inline uint32_t cityHash(uint32_t a, uint32_t b) {
    uint32_t h = a * 1103515245UL + b * 2654435761UL;
    h ^= h >> 16;
    h *= 2246822519UL;
    h ^= h >> 13;
    return h;
}

// Direction-preserving pull-in of a projected point toward the screen
// center. Used to shrink stretched off-screen geometry whose edges are
// visually tolerant (e.g. grass-on-grass), so the rasterizer walks less.
static inline void pullInRadial(float& sx, float& sy, float limit) {
    float ox = sx - center_x;
    float oy = sy - center_y;
    float ax = fabsf(ox);
    float ay = fabsf(oy);
    float m = (ax > ay) ? ax : ay;
    if (m > limit) {
        float k = limit / m;
        sx = center_x + ox * k;
        sy = center_y + oy * k;
    }
}

// Conservative horizontal view-cone test in camera space (pitch only moves
// things vertically, so the pre-pitch forward/lateral split is an exact proxy
// for screen-x). `margin` is the object's lateral half-extent in metres; the
// 1.6 slope is far wider than the true ~0.89 screen edge, so an object is only
// rejected when it is clearly behind or well off to the side -- nothing that
// could touch the screen is culled. Lets callers skip the projection + fill
// work for everything outside the frustum, which is most of the city when you
// face down one street with blocks behind and beside you.
static inline bool cityInViewCone(float dx, float dz, float margin) {
    float zc = dx * cam_sin_yaw + dz * cam_cos_yaw;   // forward depth
    if (zc < -margin) return false;                   // behind the camera
    float xc = dx * cam_cos_yaw - dz * cam_sin_yaw;   // lateral offset
    return fabsf(xc) <= zc * 1.6f + margin;
}

void updateButtonInputs() {
    unsigned long now = millis();
    bool left_raw = (digitalRead(BTN_LEFT_PIN) == LOW);
    bool right_raw = (digitalRead(BTN_RIGHT_PIN) == LOW);

    if (left_raw != btn_left_raw_last) {
        btn_left_raw_last = left_raw;
        btn_left_change_ms = now;
    } else if (now - btn_left_change_ms >= BUTTON_DEBOUNCE_MS) {
        btn_left_down = left_raw;
    }

    if (right_raw != btn_right_raw_last) {
        btn_right_raw_last = right_raw;
        btn_right_change_ms = now;
    } else if (now - btn_right_change_ms >= BUTTON_DEBOUNCE_MS) {
        btn_right_down = right_raw;
    }
}

// Setup Function
void setup() {
    setCpuFrequencyMhz(240);

    // Enable Screen Power
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);

    // Enable Screen Backlight
    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH);

    // Set active-low buttons
    pinMode(BTN_LEFT_PIN, INPUT_PULLUP);
    pinMode(BTN_RIGHT_PIN, INPUT_PULLUP);
    btn_left_raw_last = (digitalRead(BTN_LEFT_PIN) == LOW);
    btn_right_raw_last = (digitalRead(BTN_RIGHT_PIN) == LOW);
    btn_left_down = btn_left_raw_last;
    btn_right_down = btn_right_raw_last;
    btn_left_change_ms = millis();
    btn_right_change_ms = btn_left_change_ms;

    // Initialize Display
    tft.init();
    tft.setRotation(DISPLAY_ROTATION); // Landscape mode, optionally flipped 180 degrees.
    tft.setBrightness(255);

    // Two framebuffers in internal (DMA-capable) RAM. If the second
    // allocation fails we fall back to single-buffer blocking pushes.
    fb[0].setColorDepth(16);
    fb[1].setColorDepth(16);
    bool fb0_ok = (fb[0].createSprite(SCREEN_WIDTH, SCREEN_HEIGHT) != nullptr);
    use_dma = USE_DMA_PUSH && fb0_ok && (fb[1].createSprite(SCREEN_WIDTH, SCREEN_HEIGHT) != nullptr);
    if (use_dma) {
        tft.initDMA();
        tft.startWrite(); // hold the bus transaction so DMA pushes stay asynchronous
    }

    // Build the city + its radar, then spawn the courier and start the shift.
    buildCityRoutes();
    buildCityMinimap();
    resetRaceState();

    // Timers
    last_time = millis();
    last_frame_us = micros();
    fps_window_start_us = last_frame_us;
}

// Main Game Loop
void loop() {
    unsigned long now_us = micros();

    // Frame pacing. Render times vary with scene content, and UNEVEN delivery
    // is what reads as microstutter even when the average rate is fine. Pace
    // every frame to a fixed interval so delivery is even: sleep the bulk of
    // the wait in 1 ms chunks, then busy-spin the final approach for exact
    // landing (delay()'s 1 ms granularity alone would reintroduce jitter).
    // FRAME_TIME_US sets the lock rate; 0 disables pacing for a benchmark.
    if (FRAME_TIME_US > 0) {
        unsigned long target = last_frame_us + FRAME_TIME_US;
        while ((long)(target - micros()) > 1500) delay(1);
        while ((long)(target - micros()) > 0) { /* precise spin to the target */ }
        now_us = micros();
    }

    unsigned long elapsed_us = now_us - last_frame_us;
    last_frame_us = now_us;
    last_time = millis();
    float dt = elapsed_us / 1000000.0f;

    // Limit dt to prevent massive jumps during delays
    if (dt > 0.1f) dt = 0.1f;
    updateButtonInputs();

    if (current_state == START_SCREEN) {
        drawStartScreen();

        bool left = btn_left_down;
        bool right = btn_right_down;
        if (left || right) {
            // Drive straight in -- fresh clock + first fare, no 3-2-1 countdown.
            courierStartRun();
            current_state = PLAYING;
        }
    } else {
        // Update Game Physics
        updatePhysics(dt);

        // Render 3D City Scene
        renderTrackAndObjects();

        // Draw HUD Overlays
        if (current_state == PLAYING || current_state == FINISHED) {
            drawHUD();
        }

        if (current_state == FINISHED) {
            drawFinished();
        }
    }

    // Push the frame buffer to the physical screen. With DMA the call only
    // kicks off the transfer; the next frame renders into the other buffer
    // while this one streams out, so the push costs ~0 CPU time.
    unsigned long before_push_us = micros();
    if (use_dma) {
        tft.waitDMA(); // previous frame must be fully out before reusing the channel
        tft.pushImageDMA(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT,
                         (const lgfx::swap565_t*)sprite.getBuffer());
        fb_idx ^= 1;
    } else {
        sprite.pushSprite(0, 0);
    }
    unsigned long after_push_us = micros();

    perf_frame_ms = elapsed_us * 0.001f;
    perf_render_ms = (before_push_us - now_us) * 0.001f;
    perf_push_ms = (after_push_us - before_push_us) * 0.001f;

    // Track the worst CPU render time across the FPS window: a steady average
    // can hide a single 30 ms frame that reads as a visible hitch. This is the
    // number to watch when hunting stutter on hardware (shown after the FPS).
    if (perf_render_ms > perf_worst_acc) perf_worst_acc = perf_render_ms;

    fps_window_frames++;
    unsigned long fps_elapsed_us = after_push_us - fps_window_start_us;
    if (fps_elapsed_us >= 1000000UL) {
        measured_fps = (fps_window_frames * 1000000.0f) / (float)fps_elapsed_us;
        perf_worst_ms = perf_worst_acc;
        perf_worst_acc = 0.0f;
        fps_window_frames = 0;
        fps_window_start_us = after_push_us;
    }
}

void resetRaceState() {
    player_w = 0.0f;
    // Spawn on the grand avenue near the harbor, facing north up the
    // boulevard toward the downtown gateway arch and the park beyond.
    player_x = 0.0f;
    player_y = 0.0f;
    player_z = -90.0f;
    player_heading = 0.0f;
    travel_heading = player_heading;
    g_slip = 0.0f;
    steer_input = 0.0f;
    cam_chase_yaw = player_heading;
    cam_chase_pitch = atan2f(-0.8f, CHASE_CAM_DIST + 6.0f);
    cam_chase_x = player_x - sinf(player_heading) * CHASE_CAM_DIST;
    cam_chase_z = player_z - cosf(player_heading) * CHASE_CAM_DIST;
    cam_chase_y = CHASE_CAM_HEIGHT;
    player_speed = 0.0f;
    player_roll = 0.0f;
    player_pitch = 0.0f;
    player_steer_angle = 0.0f;
    screen_shake_timer = 0;
    player_braking = false;

    for (int i = 0; i < CITY_NPC_COUNT; i++) {
        getCityRoutePose(city_npcs[i].route, city_npcs[i].progress,
                         city_npcs[i].x, city_npcs[i].z, city_npcs[i].heading);
        city_npcs[i].cur_speed = 0.0f;
    }
    resetCityPeds();
    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].life = 0;
    }
    courierStartRun();
}

// Physics Loop
void updatePhysics(float dt) {
    updateCityTimeOfDay(dt);
    updateCityPeds(dt);
    if (current_state == PLAYING) {
        // Read active-low buttons
        bool steer_left = btn_left_down;
        bool steer_right = btn_right_down;

        // Off the asphalt (in a park/plaza) the car bogs down -- EXCEPT on the
        // northern rally track, which runs free at full speed (just loose grip).
        // off_road still drives the dirt spray + drift posture on the track.
        bool off_road = (player_w > 0.0f);
        bool on_track = cityOnTrack(player_x, player_z);
        // Brake lights track the brake input itself, so they stay lit while
        // holding both buttons even at a standstill (0 km/h), not just while
        // still moving.
        player_braking = (steer_left && steer_right);
        if (steer_left && steer_right) {
            float speed_ratio = player_speed / MAX_SPEED;
            if (speed_ratio < 0.0f) speed_ratio = 0.0f;
            if (speed_ratio > 1.0f) speed_ratio = 1.0f;
            float brake_decel = 82.0f + 65.0f * speed_ratio;
            player_speed -= brake_decel * dt;
            if (player_speed < 0.0f) player_speed = 0.0f;
        } else if (off_road && !on_track) {
            if (player_speed > 45.0f) {
                player_speed -= 45.0f * dt;
            } else {
                player_speed += 6.0f * dt;
                if (player_speed > 45.0f) player_speed = 45.0f;
            }
        } else {
            bool cornering = (steer_left != steer_right);
            float accel_rate = (player_speed < 35.0f) ? 15.0f : 10.0f;
            if (player_speed >= 80.0f) accel_rate *= 0.45f;
            if (cornering) accel_rate *= 0.62f;
            player_speed += accel_rate * dt;
            if (player_speed > MAX_SPEED) player_speed = MAX_SPEED;
        }

        // Dirt spray while off-road
        if (off_road && player_speed > 30.0f) {
            for (int d = 0; d < 2; d++) {
                spawnParticle(160 + random(-42, 43), 124 + random(0, 12),
                              random(-12, 13) * 0.1f, -random(8, 28) * 0.1f,
                              random(12, 26), (d & 1) ? 0x9367 : 0x4DE8);
            }
        }

        // Steering rotates the car itself: the car can swing fully around.
        // Off the asphalt the body leans far over and the nose swings wider
        // (drift posture).
        float steer_dir = 0.0f;
        float target_roll = 0.0f;
        float target_steer_angle = 0.0f;
        if (steer_left && !steer_right) {
            steer_dir = -1.0f;
            target_roll = -TURN_ROLL_AMOUNT;
            target_steer_angle = -TURN_STEER_VISUAL_AMOUNT;
        } else if (steer_right && !steer_left) {
            steer_dir = 1.0f;
            target_roll = TURN_ROLL_AMOUNT;
            target_steer_angle = TURN_STEER_VISUAL_AMOUNT;
        }
        if (off_road) {
            target_roll *= 3.0f;
            target_steer_angle *= 1.6f;
        } else {
            target_roll *= 0.5f; // calmer body roll on city asphalt
        }
        // Visual-only: at a crawl the full body lean looks like the car is
        // drifting at 10 km/h. Fade the roll in with speed so slow driving
        // sits flat, reaching the normal amount by ~25 km/h. Steering/grip are
        // untouched -- this only scales the cosmetic tilt.
        float roll_speed_scale = player_speed * (1.0f / 25.0f);
        if (roll_speed_scale > 1.0f) roll_speed_scale = 1.0f;
        target_roll *= roll_speed_scale;
        player_roll = approachFloat(player_roll, target_roll, TURN_VISUAL_RESPONSE, dt);
        player_steer_angle = approachFloat(player_steer_angle, target_steer_angle, TURN_VISUAL_RESPONSE, dt);

        // Steering ramps in over ~0.3 s instead of snapping to full lock, so
        // short button taps give fine corrections (raw on/off was too twitchy
        // with only two buttons). Release recenters faster than attack.
        float steer_resp = (steer_dir == 0.0f) ? 10.0f : 5.5f;
        steer_input = approachFloat(steer_input, steer_dir, steer_resp, dt);

        // Yaw rate: quick at parking speeds (tight U-turns), tapering with
        // speed so flat-out corners still demand braking. The floor keeps the
        // car steerable while crawling (e.g. nosed against the boundary) so
        // it can always swing itself free.
        float speed_factor = player_speed / 20.0f;
        if (speed_factor > 1.0f) speed_factor = 1.0f;
        if (player_speed > 0.5f && speed_factor < 0.35f) speed_factor = 0.35f;
        float turn_rate = CAR_TURN_RATE / (1.0f + player_speed / 120.0f);
        player_heading += steer_input * turn_rate * speed_factor * dt;
        while (player_heading > PI) player_heading -= 2.0f * PI;
        while (player_heading < -PI) player_heading += 2.0f * PI;

        // Grip model: the velocity direction chases the nose. On asphalt it
        // keeps up almost instantly; on grass it lags far behind -- which IS
        // the drift: the car travels at an angle to where it points.
        float grip = on_track ? 3.1f : (off_road ? 2.0f : 5.6f);
        float slip = player_heading - travel_heading;
        while (slip > PI) slip -= 2.0f * PI;
        while (slip < -PI) slip += 2.0f * PI;
        float grip_blend = grip * dt;
        if (grip_blend > 1.0f) grip_blend = 1.0f;
        travel_heading += slip * grip_blend;
        while (travel_heading > PI) travel_heading -= 2.0f * PI;
        while (travel_heading < -PI) travel_heading += 2.0f * PI;
        g_slip = slip * (1.0f - grip_blend);

        // Move the car along its travel direction (not the nose).
        float prev_player_x = player_x;
        float prev_player_z = player_z;
        float v_ms = player_speed / 3.6f;
        player_x += sinf(travel_heading) * v_ms * dt;
        player_z += cosf(travel_heading) * v_ms * dt;
        g_wheel_spin += v_ms * dt / 0.25f;   // angular = distance / wheel radius
        if (g_wheel_spin > 6283.0f) g_wheel_spin -= 6283.0f;

        // Crash knockback: a decaying impulse velocity layered on top of normal
        // driving, so a car-vs-car hit physically throws the body in the impact
        // direction. Set by applyCrashImpulse at the collision sites; the wall /
        // parked-car / NPC pushouts that run later in the frame still clamp the
        // car back out of anything solid it gets flung into.
        if (g_crash_vx != 0.0f || g_crash_vz != 0.0f) {
            player_x += g_crash_vx * dt;
            player_z += g_crash_vz * dt;
            float decay = expf(-CRASH_DAMP * dt);
            g_crash_vx *= decay;
            g_crash_vz *= decay;
            if (g_crash_vx * g_crash_vx + g_crash_vz * g_crash_vz < 0.09f) {
                g_crash_vx = 0.0f; // below ~0.3 m/s: stop drifting
                g_crash_vz = 0.0f;
            }
        }

        // Re-derive world surface state.
        {
            updatePlayerCityPosition();
            float bounce_nx = 0.0f;
            float bounce_nz = 0.0f;
            if (cityBuildingCollision(player_x, player_z, prev_player_x, prev_player_z, bounce_nx, bounce_nz)) {
                float vx = sinf(travel_heading);
                float vz = cosf(travel_heading);
                float into_wall = vx * bounce_nx + vz * bounce_nz;
                if (into_wall < 0.0f) {
                    vx -= 1.55f * into_wall * bounce_nx;
                    vz -= 1.55f * into_wall * bounce_nz;
                    float vlen = sqrtf(vx * vx + vz * vz);
                    if (vlen > 0.01f) {
                        travel_heading = atan2f(vx / vlen, vz / vlen);
                        float hd = travel_heading - player_heading;
                        while (hd > PI) hd -= 2.0f * PI;
                        while (hd < -PI) hd += 2.0f * PI;
                        player_heading += hd * 0.22f;
                        while (player_heading > PI) player_heading -= 2.0f * PI;
                        while (player_heading < -PI) player_heading += 2.0f * PI;
                    }
                }
                if (player_speed > 2.0f) {
                    player_speed *= 0.34f;
                    if (player_speed < 7.0f) player_speed = 7.0f;
                } else {
                    player_speed = 0.0f;
                }
                screen_shake_timer = 8;
                courier_streak = 0; // crashing into a building loses the streak
                float spark_dir = (fabsf(bounce_nx) > 0.01f) ? bounce_nx : ((random(0, 2) == 0) ? -1.0f : 1.0f);
                spawnImpactSparks(160, 118, spark_dir);
                updatePlayerCityPosition();
            }
            cityParkedCarCollision();
            // Water is solid: the car stops at the shoreline with a splash.
            float pond_nx, pond_nz;
            if (cityPondBlocked(player_x, player_z, pond_nx, pond_nz)) {
                if (player_speed > 14.0f) {
                    for (int d = 0; d < 4; d++) {
                        spawnParticle(160 + random(-30, 31), 120 + random(-6, 10),
                                      random(-14, 15) * 0.1f, -random(8, 26) * 0.1f,
                                      random(12, 24), (d & 1) ? 0x5DBF : 0xBEFF);
                    }
                }
                player_speed *= 0.30f;
                screen_shake_timer = 6;
                updatePlayerCityPosition();
            }
            // Rally-track fences are solid walls: bounce off, losing speed in
            // proportion to how head-on the hit is, so a glancing scrape barely
            // slows you but a square hit really stings. Position is already
            // corrected by the resolver; no streak reset (this is a fun lap, not
            // a delivery). It runs only near the northern track.
            float fnx, fnz;
            if (cityTrackFenceHit(player_x, player_z, prev_player_x, prev_player_z, fnx, fnz)) {
                float vx = sinf(travel_heading), vz = cosf(travel_heading);
                float into = vx * fnx + vz * fnz;          // <0: driving into the fence
                if (into < 0.0f) {
                    vx -= 1.7f * into * fnx;
                    vz -= 1.7f * into * fnz;
                    float vl = sqrtf(vx * vx + vz * vz);
                    if (vl > 0.01f) {
                        travel_heading = atan2f(vx / vl, vz / vl);
                        float hd = travel_heading - player_heading;
                        while (hd > PI) hd -= 2.0f * PI;
                        while (hd < -PI) hd += 2.0f * PI;
                        player_heading += hd * 0.25f;
                        while (player_heading > PI) player_heading -= 2.0f * PI;
                        while (player_heading < -PI) player_heading += 2.0f * PI;
                    }
                    float headon = -into;                  // 0 glance .. 1 square
                    player_speed *= (1.0f - 0.55f * headon);
                    if (player_speed < 6.0f) player_speed = 6.0f;
                    if (headon > 0.45f) {
                        screen_shake_timer = 6;
                        spawnImpactSparks(160, 118, (random(0, 2) == 0) ? -1.0f : 1.0f);
                    }
                }
                updatePlayerCityPosition();
            }
            player_pitch = approachFloat(player_pitch, 0.0f, 6.0f, dt);
        }

        // Extra wheel spray while drifting on grass: dirt and torn-up turf
        // thrown against the slide direction.
        if (off_road && fabsf(g_slip) > 0.15f && player_speed > 20.0f) {
            for (int d = 0; d < 3; d++) {
                spawnParticle(160 + random(-64, 65), 118 + random(0, 18),
                              (g_slip > 0.0f ? 1.0f : -1.0f) * random(6, 26) * 0.1f,
                              -random(10, 36) * 0.1f,
                              random(14, 30),
                              (d == 0) ? 0x05E0 : ((d & 1) ? 0x9367 : 0x4DE8));
            }
        }

    }

    if (current_state == PLAYING) {
        updateCityNPCs(dt);
        courierUpdate(dt);
    }

    // Chase camera dynamics, run in every race state: the camera heading
    // chases the car's heading with a short lag (so the world visibly swings
    // when the car turns), while height and pitch ease to soften hill crests.
    float dyaw = player_heading - cam_chase_yaw;
    while (dyaw > PI) dyaw -= 2.0f * PI;
    while (dyaw < -PI) dyaw += 2.0f * PI;
    float yaw_blend = CHASE_CAM_YAW_RESPONSE * dt;
    if (yaw_blend > 1.0f) yaw_blend = 1.0f;
    cam_chase_yaw += dyaw * yaw_blend;
    while (cam_chase_yaw > PI) cam_chase_yaw -= 2.0f * PI;
    while (cam_chase_yaw < -PI) cam_chase_yaw += 2.0f * PI;

    // Camera position behind the car; its height eases to a fixed offset above
    // the flat city ground.
    cam_chase_x = player_x - sinf(cam_chase_yaw) * CHASE_CAM_DIST;
    cam_chase_z = player_z - cosf(cam_chase_yaw) * CHASE_CAM_DIST;
    cam_chase_y = approachFloat(cam_chase_y, CHASE_CAM_HEIGHT, 10.0f, dt);

    // Pitch: aim at a point just above the car.
    float look_y = player_y + 0.55f;
    float pitch_target = atan2f(look_y - cam_chase_y, CHASE_CAM_DIST + 6.0f);
    cam_chase_pitch = approachFloat(cam_chase_pitch, pitch_target, 6.0f, dt);
}

// 3D Scene Rendering Engine
void renderTrackAndObjects() {
    // Chase camera: behind the car along its smoothed heading, turning with
    // the car -- when the car turns left, the whole world swings right.
    // Position/height/pitch are integrated in updatePhysics.
    cam_yaw = cam_chase_yaw;
    cam_pitch = cam_chase_pitch;
    cam_x = cam_chase_x;
    cam_y = cam_chase_y;
    cam_z = cam_chase_z;
    updateCameraTrig();

    // Apply Crash Camera Shake
    if (screen_shake_timer > 0) {
        center_x = 160.0f + random(-3, 4);
        center_y = 85.0f + random(-3, 4);
        screen_shake_timer--;
    } else {
        center_x = 160.0f;
        center_y = 85.0f;
    }

    int horizon_y = (int)(center_y + cam_pitch * fov);

    // City light direction follows the time-of-day sun (set in
    // updateCityTimeOfDay); the background is fully palette-driven.
    drawCityBackground(horizon_y, cam_yaw);
    renderCityWorld();
    drawCityPlayerCar();
    drawSunFlare();
    drawCityMinimap();
}

float cityRoadSignedDistance(float x, float z) {
    float best = 1e9f;
    for (int i = 0; i < CITY_ROAD_COUNT; i++) {
        const CityRoad& r = city_roads[i];
        float dx = r.x1 - r.x0;
        float dz = r.z1 - r.z0;
        float len2 = dx * dx + dz * dz;
        if (len2 < 0.0001f) len2 = 0.0001f; // guard degenerate map-editor segments
        float t = ((x - r.x0) * dx + (z - r.z0) * dz) / len2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        float px = r.x0 + dx * t;
        float pz = r.z0 + dz * t;
        float dist = sqrtf((x - px) * (x - px) + (z - pz) * (z - pz)) - r.width * 0.5f;
        if (dist < best) best = dist;
    }
    return best;
}

void updatePlayerCityPosition() {
    player_y = 0.0f;
    player_w = cityRoadSignedDistance(player_x, player_z);
}

void buildCityRoutes() {
    for (int r = 0; r < CITY_NPC_COUNT; r++) {
        float total = 0.0f;
        for (int i = 0; i < CITY_ROUTE_POINTS - 1; i++) {
            float dx = city_route_x[r][i + 1] - city_route_x[r][i];
            float dz = city_route_z[r][i + 1] - city_route_z[r][i];
            total += sqrtf(dx * dx + dz * dz);
        }
        g_route_len[r] = (total > 1.0f) ? total : 1.0f;
    }
}

void getCityRoutePose(uint8_t route, float progress, float& x, float& z, float& heading) {
    float total = g_route_len[route];
    while (progress >= total) progress -= total;
    while (progress < 0.0f) progress += total;

    for (int i = 0; i < CITY_ROUTE_POINTS - 1; i++) {
        float x0 = city_route_x[route][i];
        float z0 = city_route_z[route][i];
        float x1 = city_route_x[route][i + 1];
        float z1 = city_route_z[route][i + 1];
        float dx = x1 - x0;
        float dz = z1 - z0;
        float len = sqrtf(dx * dx + dz * dz);
        if (progress <= len) {
            float t = (len > 0.0f) ? progress / len : 0.0f;
            x = x0 + dx * t;
            z = z0 + dz * t;
            heading = atan2f(dx, dz);
            return;
        }
        progress -= len;
    }
    x = city_route_x[route][0];
    z = city_route_z[route][0];
    heading = 0.0f;
}

// Parked cars are solid: shove the player out along the contact normal,
// scrub speed, and spark -- same treatment as hitting moving traffic.
// Throw the player along the contact normal (nx,nz point AWAY from whatever was
// hit, toward the car) with a knockback velocity set by the impact speed. The
// impulse is accumulated into g_crash_v* and integrated/decayed in updatePhysics
// so the car keeps sliding for a beat after the bang. Tiered to the brief: a
// 30 km/h clip nudges, a 100 km/h smash flings the car several metres.
// Impact speed (km/h) -> base knockback velocity (m/s), piecewise across the
// three bands in the brief. Returns 0 below the threshold. Shared by the
// player's self-knockback and the (scaled-up) NPC shove.
float crashKnockV0(float impact_kph) {
    if (impact_kph < CRASH_MIN_KPH) return 0.0f;             // too slow to matter
    if (impact_kph <= 40.0f) return 2.5f + (impact_kph - 10.0f) * (4.0f / 30.0f);   // 2.5 -> 6.5  light
    if (impact_kph <= 70.0f) return 6.5f + (impact_kph - 40.0f) * (5.0f / 30.0f);   // 6.5 -> 11.5 firmer
    float v0 = 11.5f + (impact_kph - 70.0f) * (8.5f / 40.0f);                        // 11.5 -> 20 @110 violent
    return (v0 > CRASH_MAX_VEL) ? CRASH_MAX_VEL : v0;
}

void applyCrashImpulse(float nx, float nz, float impact_kph) {
    float v0 = crashKnockV0(impact_kph);
    if (v0 <= 0.0f) return;  // too slow to throw the car

    // Normalize the contact normal defensively, then accumulate the impulse so
    // simultaneous / chained hits stack (capped below).
    float nl = sqrtf(nx * nx + nz * nz);
    if (nl < 1e-4f) return;
    g_crash_vx += (nx / nl) * v0;
    g_crash_vz += (nz / nl) * v0;
    float m2 = g_crash_vx * g_crash_vx + g_crash_vz * g_crash_vz;
    if (m2 > CRASH_MAX_VEL * CRASH_MAX_VEL) {
        float s = CRASH_MAX_VEL / sqrtf(m2);
        g_crash_vx *= s;
        g_crash_vz *= s;
    }

    // Camera shake scales with violence: light tap ~6 frames, big smash ~18.
    int shake = 5 + (int)(v0 * 0.7f);
    if (shake > screen_shake_timer) screen_shake_timer = shake;
}

void cityParkedCarCollision() {
    for (int i = 0; i < CITY_PROP_COUNT; i++) {
        const CityProp& p = city_props[i];
        if (p.type != 3) continue;
        float dx = player_x - p.x;
        float dz = player_z - p.z;
        float d2 = dx * dx + dz * dz;
        if (d2 >= 4.2f) continue;
        float d = sqrtf(d2);
        if (d < 0.05f) { d = 0.05f; dx = 0.05f; }
        float nx = dx / d, nz = dz / d;
        player_x = p.x + nx * 2.05f;
        player_z = p.z + nz * 2.05f;
        if (player_speed > 4.0f) {
            applyCrashImpulse(nx, nz, player_speed); // throw the car off the wreck
            player_speed *= 0.5f;
            if (screen_shake_timer < 6) screen_shake_timer = 6;
            courier_streak = 0; // clipped a parked car
            spawnImpactSparks(160 + ((dx > 0.0f) ? 18 : -18), 118, (dx > 0.0f) ? 1.0f : -1.0f);
        }
    }
}

// Keep the car out of the water. Tests the player against each pond ellipse
// (with a small shoreline margin) and, if inside, pushes it back to the rim
// along the outward ellipse normal. Returns true on contact.
bool cityPondBlocked(float x, float z, float& out_nx, float& out_nz) {
    out_nx = 0.0f;
    out_nz = 0.0f;
    for (int i = 0; i < CITY_POND_COUNT; i++) {
        const CityPond& p = city_ponds[i];
        float rx = p.rx + 0.6f;   // stop a touch before the visual edge
        float rz = p.rz + 0.6f;
        float ex = (x - p.x) / rx;
        float ez = (z - p.z) / rz;
        float e2 = ex * ex + ez * ez;
        if (e2 >= 1.0f) continue;
        // Outward normal of the ellipse at this point.
        float nx = ex / rx, nz = ez / rz;
        float nl = sqrtf(nx * nx + nz * nz);
        if (nl < 1e-4f) { nx = 1.0f; nz = 0.0f; nl = 1.0f; }
        nx /= nl; nz /= nl;
        // March outward to the rim along the normal (cheap, converges in 1-2
        // steps for the shapes here).
        float k = (1.0f - sqrtf(e2));
        player_x = x + nx * (k * rx + 0.2f);
        player_z = z + nz * (k * rz + 0.2f);
        out_nx = nx;
        out_nz = nz;
        return true;
    }
    return false;
}

// Choose a fresh courier target somewhere on the road network, a decent
// distance from the reference point (player for pickups, pickup for drops) so
// every leg is a real drive. `first` skips the distance test for the opener.
void courierPickTarget(bool first) {
    float refx = courier_has_fare ? courier_px : player_x;
    float refz = courier_has_fare ? courier_pz : player_z;
    float bx = courier_tx, bz = courier_tz;  // fallback: keep current if nothing qualifies
    float min2 = COURIER_MIN_TRIP * COURIER_MIN_TRIP;
    // A weighted roll per target keeps the route mix from feeling rote: 60% go
    // for the FARTHEST qualifying road point (the classic cross-map "long haul"),
    // 40% pick a nearer random point. Either way the candidate must clear
    // COURIER_MIN_TRIP, so it's never a spawn-on-top-of-you non-trip.
    bool farthest = (random(0, 10) < 6);   // 60% long haul, 40% random

    // In random mode, often pull the target from the curated centre list so
    // downtown actually sees fares -- but ONLY when the OTHER end of the trip is
    // out toward the edge (player for a pickup, pickup point for a drop). That
    // gate makes every centre fare read as centre<->outskirts (docks, old town,
    // the rally stub) and never centre->centre. Walk the list from a random
    // start and take the first spot that clears the min trip; if the reference
    // is itself downtown, skip straight to the road sampler.
    if (!farthest && random(0, 10) < 6) {
        float rcx = refx - COURIER_CENTER_X, rcz = refz - COURIER_CENTER_Z;
        bool ref_at_edge = (rcx * rcx + rcz * rcz) >= (COURIER_EDGE_DIST * COURIER_EDGE_DIST);
        if (first || ref_at_edge) {
            int s0 = random(0, COURIER_CENTER_COUNT);
            for (int k = 0; k < COURIER_CENTER_COUNT; k++) {
                const float* sp = courier_center_spots[(s0 + k) % COURIER_CENTER_COUNT];
                float dx = sp[0] - refx, dz = sp[1] - refz;
                if (first || dx * dx + dz * dz >= min2) {
                    courier_tx = sp[0];
                    courier_tz = sp[1];
                    return;
                }
            }
        }
    }

    float best_d2 = -1.0f;
    for (int tries = 0; tries < 12; tries++) {
        int ri = random(0, CITY_ROAD_COUNT);
        const CityRoad& r = city_roads[ri];
        float t = 0.12f + (random(0, 1000) / 1000.0f) * 0.76f;
        float cx = r.x0 + (r.x1 - r.x0) * t;
        float cz = r.z0 + (r.z1 - r.z0) * t;
        float dx = cx - refx, dz = cz - refz;
        float d2 = dx * dx + dz * dz;
        if (!first && d2 < min2) continue;
        if (d2 > best_d2) { best_d2 = d2; bx = cx; bz = cz; }
        if (!farthest) break;  // random mode: first qualifying point wins
    }
    courier_tx = bx;
    courier_tz = bz;
}

void courierStartRun() {
    courier_started = false;
    courier_has_fare = false;
    courier_time_left = COURIER_RUN_SECONDS;
    courier_fare_clock = 0.0f;
    courier_money = 0;
    courier_deliveries = 0;
    courier_streak = 0;
    courier_popup_ms = 0;
    courier_popup_amount = 0;
    courierPickTarget(true);
}

// Per-frame courier logic while driving: tick the clock, and when the car
// stops inside the target zone, board a fare or complete a delivery.
void courierUpdate(float dt) {
    if (courier_popup_ms > 0) courier_popup_ms--;

    // Free-roam grace: the countdown does not run until the first fare is
    // aboard, so the player can explore the open city as long as they like.
    if (courier_started) {
        courier_time_left -= dt;
        if (courier_has_fare) courier_fare_clock += dt;
        if (courier_time_left <= 0.0f) {
            courier_time_left = 0.0f;
            if (courier_money > (long)courier_best) courier_best = courier_money;
            current_state = FINISHED;
            player_speed = 0.0f;
            return;
        }
    }

    float dx = player_x - courier_tx;
    float dz = player_z - courier_tz;
    bool in_zone = (dx * dx + dz * dz < COURIER_RADIUS * COURIER_RADIUS);
    bool stopped = (player_speed < COURIER_STOP_KMH);
    if (!in_zone || !stopped) return;

    if (!courier_has_fare) {
        // Board the fare; the drop-off lights up elsewhere in the city. The
        // very first pickup is what starts the clock running.
        courier_started = true;
        courier_has_fare = true;
        courier_px = player_x;
        courier_pz = player_z;
        courier_fare_clock = 0.0f;
        courier_popup_ms = 70;
        courier_popup_amount = 0; // 0 -> render as "FARE ABOARD"
        courierPickTarget(false);
        for (int s = 0; s < 6; s++)
            spawnParticle(160 + random(-26, 27), 96 + random(-10, 11),
                          random(-12, 13) * 0.1f, -random(6, 22) * 0.1f,
                          random(14, 26), 0xFFE0);
    } else {
        // Deliver: pay a fare scaled by trip length and promptness, times the
        // clean-driving streak, and bank extra time to keep the run alive.
        float tdx = courier_tx - courier_px, tdz = courier_tz - courier_pz;
        float trip = sqrtf(tdx * tdx + tdz * tdz);
        int base = 8 + (int)(trip * 0.35f);
        int promptness = (int)(28.0f - courier_fare_clock * 1.2f);
        if (promptness < 0) promptness = 0;
        int mult = 1 + courier_streak / 3;          // x1, x2 at streak 3, x3 at 6...
        if (mult > 5) mult = 5;
        long fare = (long)(base + promptness) * mult;

        courier_money += fare;
        courier_deliveries++;
        courier_streak++;
        courier_popup_ms = 80;
        courier_popup_amount = fare;

        float time_bonus = 14.0f + trip * 0.16f;
        if (time_bonus > 32.0f) time_bonus = 32.0f;
        courier_time_left += time_bonus;
        if (courier_time_left > 120.0f) courier_time_left = 120.0f;

        courier_has_fare = false;
        courierPickTarget(false);
        for (int s = 0; s < 10; s++)
            spawnParticle(160 + random(-34, 35), 92 + random(-12, 13),
                          random(-16, 17) * 0.1f, -random(8, 28) * 0.1f,
                          random(16, 30), (s & 1) ? 0x07E0 : 0xFFE0);
    }
}

// Signal timing shared by the light props and the NPC traffic brain.
// 11 s full cycle: each axis gets 4 s green, 1.5 s amber, then yields.
// Returns 0 green, 1 amber, 2 red for the given axis.
int cityTrafficPhase(bool ns_axis) {
    unsigned long t = millis() % 11000UL;
    bool first_half = (t < 5500UL);
    unsigned long ph = t % 5500UL;
    if (ns_axis == first_half) {
        return (ph < 4000UL) ? 0 : 1;
    }
    return 2;
}

void updateCityNPCs(float dt) {
    for (int i = 0; i < CITY_NPC_COUNT; i++) {
        CityNPC& npc = city_npcs[i];
        float hx = sinf(npc.heading);
        float hz = cosf(npc.heading);

        // Pick a target speed: cruise, unless a red/amber signal or the
        // player's car sits in the lane ahead.
        float target = npc.speed_ms;
        for (int s = 0; s < CITY_INTERSECTION_COUNT; s++) {
            if (!city_intersections[s].signals) continue;
            float dx = city_intersections[s].x - npc.x;
            float dz = city_intersections[s].z - npc.z;
            float dist = sqrtf(dx * dx + dz * dz);
            float r = city_intersections[s].radius;
            if (dist > r + 14.0f || dist < r - 1.0f) continue; // outside the approach band, or already committed
            if (hx * dx + hz * dz < dist * 0.7f) continue;     // not actually heading at it
            bool ns_axis = (fabsf(hz) >= fabsf(hx));
            if (cityTrafficPhase(ns_axis) != 0) target = 0.0f;
            break;
        }
        {
            float pdx = player_x - npc.x;
            float pdz = player_z - npc.z;
            if (pdx * pdx + pdz * pdz < 81.0f) {
                float ahead = hx * pdx + hz * pdz;
                float side = fabsf(hx * pdz - hz * pdx);
                if (ahead > 0.5f && side < 2.3f) target = 0.0f; // yield, don't rear-end the player
            }
        }

        // Gentle throttle, firm brakes.
        if (npc.cur_speed < target) {
            npc.cur_speed += 4.5f * dt;
            if (npc.cur_speed > target) npc.cur_speed = target;
        } else {
            npc.cur_speed -= 16.0f * dt;
            if (npc.cur_speed < target) npc.cur_speed = target;
        }

        npc.progress += npc.cur_speed * dt;
        float route_len = g_route_len[npc.route];
        while (npc.progress >= route_len) npc.progress -= route_len;

        // Centerline pose, then a right-hand lane offset so cars keep to their
        // side of the road instead of riding the middle.
        float cx, cz, raw_head;
        getCityRoutePose(npc.route, npc.progress, cx, cz, raw_head);
        float rhx = cosf(raw_head);   // right vector of the travel direction
        float rhz = -sinf(raw_head);
        const float lane = 2.6f;

        // Crash knockback: a hit gives the car a decaying velocity that slides
        // it off its lane (the visible "rammed it across the street" shove);
        // the offset then eases back so the car rejoins traffic. Integrated here
        // every frame and added on top of the route pose below.
        npc.knock_x += npc.knock_vx * dt;
        npc.knock_z += npc.knock_vz * dt;
        float k_damp = expf(-NPC_KNOCK_DAMP * dt);
        npc.knock_vx *= k_damp;
        npc.knock_vz *= k_damp;
        float k_ret = expf(-NPC_KNOCK_RETURN * dt);
        npc.knock_x *= k_ret;
        npc.knock_z *= k_ret;

        npc.x = cx + rhx * lane + npc.knock_x;
        npc.z = cz + rhz * lane + npc.knock_z;

        // Smooth the heading toward the segment direction: corners between
        // route nodes now sweep instead of snapping the car instantly.
        float dh = raw_head - npc.heading;
        while (dh > PI) dh -= 2.0f * PI;
        while (dh < -PI) dh += 2.0f * PI;
        float hb = 7.0f * dt;
        if (hb > 1.0f) hb = 1.0f;
        npc.heading += dh * hb;
        while (npc.heading > PI) npc.heading -= 2.0f * PI;
        while (npc.heading < -PI) npc.heading += 2.0f * PI;

        // Solid collision with the player: push the car out along the contact
        // normal instead of letting it grind through the NPC.
        float dx = player_x - npc.x;
        float dz = player_z - npc.z;
        float d2 = dx * dx + dz * dz;
        if (d2 < 4.4f) {
            float d = sqrtf(d2);
            if (d < 0.05f) { d = 0.05f; dx = 0.05f; }
            float nx = dx / d;
            float nz = dz / d;
            player_x = npc.x + nx * 2.1f;
            player_z = npc.z + nz * 2.1f;
            if (player_speed > 4.0f) {
                applyCrashImpulse(nx, nz, player_speed); // throw the player off the hit
                // Launch the OTHER car away from the player, in the crash
                // direction (-n points from the player through the NPC). This is
                // the dramatic, on-screen part: ram a car and it gets shoved.
                float kv = crashKnockV0(player_speed) * NPC_KNOCK_SCALE;
                if (kv > NPC_KNOCK_MAX) kv = NPC_KNOCK_MAX;
                npc.knock_vx += -nx * kv;
                npc.knock_vz += -nz * kv;
                player_speed *= 0.45f;
                if (screen_shake_timer < 6) screen_shake_timer = 6;
                courier_streak = 0; // rear-ended traffic
                spawnImpactSparks(160 + (nx > 0.0f ? 18 : -18), 118, nx > 0.0f ? 1.0f : -1.0f);
            }
            npc.cur_speed = 0.0f;
        }
    }
}

// Advance the city clock and derive everything the renderer needs: sky
// palette (keyframed across the day), sun elevation, night strength, the
// haze color distant objects fade into, and whether lights are on.
void updateCityTimeOfDay(float dt) {
    // Day (city_tod 0..0.5) advances 2.3x slower than night (0.5..1.0), so the
    // daylight portion lasts ~2.3x longer while the night keeps its old length.
    float tod_rate = dt / CITY_DAY_SECONDS;
    if (city_tod < 0.5f) tod_rate /= 2.3f;
    city_tod += tod_rate;
    while (city_tod >= 1.0f) city_tod -= 1.0f;

    g_sun_el = sinf(city_tod * 2.0f * PI);

    // Night eases in shortly after the sun dips and out again before dawn.
    float night = (-g_sun_el - 0.02f) * 3.4f;
    if (night < 0.0f) night = 0.0f;
    if (night > 1.0f) night = 1.0f;
    g_night = night * night * (3.0f - 2.0f * night); // smoothstep
    g_city_dim = (uint8_t)(g_night * 19.0f + 0.5f);
    g_lights_on = (g_sun_el < 0.10f);

    // Sky palette keyframes: {tod, top RGB, horizon RGB}.
    struct SkyKey { float t; uint8_t tr, tg, tb, hr, hg, hb; };
    static const SkyKey keys[] = {
        { 0.000f,  26, 38,  80, 255, 138,  74 }, // sunrise glow
        { 0.060f,  16, 70, 190, 185, 205, 232 }, // morning
        { 0.250f,  10, 64, 200, 172, 211, 235 }, // noon
        { 0.420f,  22, 58, 170, 214, 186, 148 }, // late afternoon
        { 0.500f,  56, 34,  96, 255, 112,  48 }, // sunset
        { 0.575f,   6, 10,  34,  44,  44,  86 }, // dusk
        { 0.700f,   3,  6,  22,  14,  20,  44 }, // night
        { 0.870f,   3,  6,  22,  14,  20,  44 }, // deep night
        { 0.955f,   8, 14,  40,  46,  36,  60 }, // pre-dawn
        { 1.000f,  26, 38,  80, 255, 138,  74 }  // wraps to sunrise
    };
    const int nk = sizeof(keys) / sizeof(keys[0]);
    for (int i = 0; i < nk - 1; i++) {
        if (city_tod <= keys[i + 1].t) {
            float span = keys[i + 1].t - keys[i].t;
            float f = (span > 0.0f) ? (city_tod - keys[i].t) / span : 0.0f;
            g_sky_top_r = keys[i].tr + (int)((keys[i + 1].tr - keys[i].tr) * f);
            g_sky_top_g = keys[i].tg + (int)((keys[i + 1].tg - keys[i].tg) * f);
            g_sky_top_b = keys[i].tb + (int)((keys[i + 1].tb - keys[i].tb) * f);
            g_sky_hor_r = keys[i].hr + (int)((keys[i + 1].hr - keys[i].hr) * f);
            g_sky_hor_g = keys[i].hg + (int)((keys[i + 1].hg - keys[i].hg) * f);
            g_sky_hor_b = keys[i].hb + (int)((keys[i + 1].hb - keys[i].hb) * f);
            break;
        }
    }
    g_horizon565 = pack565(g_sky_hor_r, g_sky_hor_g, g_sky_hor_b);

    // Sun direction tracks elevation; shading detail stays readable at night
    // (darkness comes from the grading, not from zeroing the light).
    float el = (g_sun_el > 0.30f) ? g_sun_el : 0.30f;
    float horiz = sqrtf(1.0f - el * el * 0.6f);
    g_light_x = 0.577f * horiz;
    g_light_y = 0.35f + 0.65f * el;
    g_light_z = -0.408f * horiz;
    float ll = sqrtf(g_light_x * g_light_x + g_light_y * g_light_y + g_light_z * g_light_z);
    g_light_x /= ll; g_light_y /= ll; g_light_z /= ll;

    // Cast shadows away from the drawn sun (compass bearing 0.46). Length
    // grows as the sun drops; strength fades through dusk and at high noon
    // shadows pull in tight under the objects.
    if (g_sun_el > 0.04f) {
        const float sun_bearing = 0.46f;
        g_shadow_dx = -sinf(sun_bearing);
        g_shadow_dz = -cosf(sun_bearing);
        float se = (g_sun_el > 0.999f) ? 0.999f : g_sun_el;
        g_shadow_len = sqrtf(1.0f - se * se) / se;
        if (g_shadow_len > 2.4f) g_shadow_len = 2.4f;
        if (g_shadow_len < 0.30f) g_shadow_len = 0.30f;
        float fade = (g_sun_el - 0.04f) * 8.0f;
        if (fade > 1.0f) fade = 1.0f;
        g_shadow_a = (uint8_t)(7.0f * fade);
    } else {
        g_shadow_a = 0;
    }
}

// City sky: dynamic gradient, sun or moon on a fixed compass bearing,
// hash-placed starfield at night, hazy mountain panorama. The horizon color
// is shared with distance fog so the world fades seamlessly into the sky.
void drawCityBackground(int horizon_y, float yaw) {
    int sky_height = horizon_y;
    if (sky_height < 0) sky_height = 0;
    if (sky_height > SCREEN_HEIGHT) sky_height = SCREEN_HEIGHT;
    if (sky_height > 0) {
        fillGradientRows(0, sky_height, 0, (horizon_y > 0 ? horizon_y : 1),
                         g_sky_top_r, g_sky_top_g, g_sky_top_b,
                         g_sky_hor_r, g_sky_hor_g, g_sky_hor_b);
    }

    // Everything on the horizon is anchored to a fixed compass bearing and
    // projected through the SAME perspective the 3D world uses:
    // sx = center + tan(bearing - yaw) * fov. A linear pixels-per-radian
    // factor (the old approach) can never match tan-projection everywhere,
    // which is why the backdrop used to creep against the world when the
    // camera turned. This way it is pixel-locked.
    const float sun_world_yaw = 0.46f;

    // Stars: fixed bearings, faded in by night strength.
    if (g_night > 0.30f && horizon_y > 6) {
        bool partial = (g_night <= 0.75f);
        for (int i = 0; i < CITY_STAR_COUNT; i++) {
            uint32_t h = cityHash(i * 7349u + 11u, 0x5EED5u);
            if (partial && (h & 3) != 0) continue; // stars come out a few at a time
            float d = (float)(h % 6283u) * 0.001f - yaw;
            while (d > PI) d -= 2.0f * PI;
            while (d < -PI) d += 2.0f * PI;
            if (fabsf(d) > 1.25f) continue;
            int sx = (int)(center_x + tanf(d) * fov);
            if (sx < 0 || sx >= SCREEN_WIDTH) continue;
            int fy = (h >> 10) % 100;
            int sy = 4 + (fy * (horizon_y - 12)) / 100;
            if (sy < 2 || sy >= horizon_y - 4) continue;
            uint16_t c = (h & 4) ? 0xFFFF : 0xBDF7;
            if (((h >> 6) & 7) == 0 && ((millis() >> 9) & 1) == (h & 1)) c = 0x630C;
            sprite.drawPixel(sx, sy, c);
            if ((h & 24) == 0) sprite.drawPixel(sx + 1, sy, c);
        }
    }

    // Sun (day) and moon (night); the moon rises opposite the sun.
    g_sun_sx = -1000;
    float sun_delta = sun_world_yaw - yaw;
    while (sun_delta > PI) sun_delta -= 2.0f * PI;
    while (sun_delta < -PI) sun_delta += 2.0f * PI;
    if (g_sun_el > -0.08f && fabsf(sun_delta) < 1.25f) {
        int sun_x = (int)(center_x + tanf(sun_delta) * fov);
        int sun_y = horizon_y - (int)(g_sun_el * 110.0f) - 6;
        if (sun_y > -30 && sun_y < horizon_y + 14 && sun_x > -60 && sun_x < 380) {
            // Low sun goes big and orange; high sun is compact and white-hot.
            float low = 1.0f - g_sun_el;
            if (low < 0.0f) low = 0.0f;
            if (low > 1.0f) low = 1.0f;
            int r_out = 13 + (int)(6.0f * low);
            uint16_t outer = pack565(255, 190 - (int)(90 * low), 40);
            uint16_t inner = pack565(255, 255 - (int)(105 * low), 210 - (int)(160 * low));
            sprite.fillCircle(sun_x, sun_y, r_out, outer);
            sprite.fillCircle(sun_x, sun_y, r_out - 4, inner);
            g_sun_sx = sun_x;
            g_sun_sy = sun_y;
            g_sun_core565 = inner;
        }
    }
    if (g_night > 0.25f) {
        float moon_delta = sun_delta + PI;
        while (moon_delta > PI) moon_delta -= 2.0f * PI;
        float moon_el = -g_sun_el;
        if (moon_el > 0.02f && fabsf(moon_delta) < 1.25f) {
            int moon_x = (int)(center_x + tanf(moon_delta) * fov);
            int moon_y = horizon_y - (int)(moon_el * 110.0f) - 6;
            if (moon_x > -20 && moon_x < 340 && moon_y > 2) {
                sprite.fillCircle(moon_x, moon_y, 9, 0xCE9C);
                sprite.fillCircle(moon_x, moon_y, 7, 0xEF7D);
                // Offset dark disc carves the crescent; craters when full-ish.
                sprite.fillCircle(moon_x - 4, moon_y - 2, 7, pack565(g_sky_top_r, g_sky_top_g, g_sky_top_b));
                sprite.fillCircle(moon_x + 3, moon_y + 1, 1, 0xCE7B);
                sprite.fillCircle(moon_x + 1, moon_y + 4, 1, 0xCE7B);
            }
        }
    }

    drawCityClouds(horizon_y, yaw);
    drawCityPanorama(horizon_y, yaw);

    // Ground beyond the drawn city tiles: graded grass plain.
    int grass_y = horizon_y;
    if (grass_y < 0) grass_y = 0;
    if (grass_y < SCREEN_HEIGHT) {
        int r0 = 150 + (int)((10 - 150) * g_night), g0 = 188 + (int)((14 - 188) * g_night), b0 = 165 + (int)((30 - 165) * g_night);
        int r1 = 56 + (int)((6 - 56) * g_night), g1 = 134 + (int)((10 - 134) * g_night), b1 = 40 + (int)((18 - 40) * g_night);
        fillGradientRows(grass_y, SCREEN_HEIGHT, grass_y, SCREEN_HEIGHT - grass_y,
                         r0, g0, b0, r1, g1, b1);
    }
}

// Clamp-and-project a horizon bearing delta through the world projection.
static inline int panoramaScreenX(float delta) {
    if (delta > 1.30f) delta = 1.30f;
    if (delta < -1.30f) delta = -1.30f;
    return (int)(center_x + tanf(delta) * fov);
}

// Fixed world panorama: a mountain range across the north-to-west arc and a
// distant downtown skyline filling the south-east, both sunk in the horizon
// haze. The skyline lights up window-by-window after dark.
void drawCityPanorama(int horizon_y, float yaw) {
    if (horizon_y < 2) return;

    struct Peak { float bearing, half_w; uint8_t h; uint16_t color, shadow; uint8_t snow; };
    static const Peak peaks[] = {
        { -2.05f, 0.16f, 22, 0x8498, 0x63B3, 0 },
        { -1.76f, 0.20f, 40, 0x7497, 0x5B92, 1 },
        { -1.47f, 0.13f, 28, 0x7C99, 0x5BB4, 0 },
        { -1.20f, 0.22f, 56, 0x6C58, 0x4B31, 2 },
        { -0.93f, 0.16f, 36, 0x84FA, 0x63D5, 0 },
        { -0.64f, 0.21f, 50, 0x6C78, 0x4B52, 3 },
        { -0.36f, 0.14f, 30, 0x7478, 0x5332, 0 },
        { -0.08f, 0.18f, 44, 0x74B8, 0x5B93, 1 },
        {  0.26f, 0.13f, 24, 0x8D1B, 0x6C16, 0 }
    };
    const int n_peaks = sizeof(peaks) / sizeof(peaks[0]);
    for (int i = 0; i < n_peaks; i++) {
        float dc = peaks[i].bearing - yaw;
        while (dc > PI) dc -= 2.0f * PI;
        while (dc < -PI) dc += 2.0f * PI;
        if (fabsf(dc) - peaks[i].half_w > 1.30f) continue;
        int x0 = panoramaScreenX(dc - peaks[i].half_w);
        int x1 = panoramaScreenX(dc);
        int x2 = panoramaScreenX(dc + peaks[i].half_w);
        if (x2 < 0 || x0 >= SCREEN_WIDTH || x2 <= x0) continue;

        int y0 = horizon_y;
        int y1 = horizon_y - peaks[i].h;
        uint16_t m_color = cityGrade(blend565(peaks[i].color, g_horizon565, 10));
        uint16_t m_shadow = cityGrade(blend565(peaks[i].shadow, g_horizon565, 10));
        sprite.fillTriangle(x0, y0, x1, y1, x2, y0, m_color);
        if (i & 1) {
            sprite.fillTriangle(x0, y0, x1, y1, x1, y0, m_shadow);
        } else {
            sprite.fillTriangle(x1, y1, x2, y0, x1, y0, m_shadow);
        }
        if (peaks[i].snow != 0) {
            uint16_t snow = cityGrade(blend565(0xFFFF, g_horizon565, 8));
            int sx0 = x1 - (int)((x1 - x0) * 0.35f);
            int sy0 = y1 + (int)(peaks[i].h * 0.35f);
            int sx2 = x1 + (int)((x2 - x1) * 0.35f);
            if (peaks[i].snow == 1) {
                sprite.fillTriangle(sx0, sy0, x1, y1, x1, sy0, snow);
            } else if (peaks[i].snow == 2) {
                sprite.fillTriangle(x1, sy0, x1, y1, sx2, sy0, snow);
            } else {
                sprite.fillTriangle(sx0, sy0, x1, y1, sx2, sy0, snow);
            }
        }
    }

    // Downtown skyline on the south-east horizon.
    struct Tower { float bearing, half_w; uint8_t h; };
    static const Tower towers[] = {
        { 1.38f, 0.045f, 18 }, { 1.50f, 0.060f, 30 }, { 1.63f, 0.040f, 24 },
        { 1.74f, 0.070f, 42 }, { 1.88f, 0.050f, 34 }, { 2.00f, 0.060f, 26 },
        { 2.12f, 0.045f, 48 }, { 2.25f, 0.065f, 36 }, { 2.39f, 0.050f, 28 },
        { 2.52f, 0.070f, 40 }, { 2.66f, 0.045f, 20 }, { 2.78f, 0.055f, 32 },
        { 2.92f, 0.040f, 16 }
    };
    const int n_towers = sizeof(towers) / sizeof(towers[0]);
    // Day: pale haze silhouettes. Night: dark slabs that read by their windows.
    uint8_t haze = g_lights_on ? 5 : 15;
    for (int i = 0; i < n_towers; i++) {
        float dc = towers[i].bearing - yaw;
        while (dc > PI) dc -= 2.0f * PI;
        while (dc < -PI) dc += 2.0f * PI;
        if (fabsf(dc) - towers[i].half_w > 1.30f) continue;
        int xl = panoramaScreenX(dc - towers[i].half_w);
        int xr = panoramaScreenX(dc + towers[i].half_w);
        if (xr < 0 || xl >= SCREEN_WIDTH || xr <= xl) continue;
        if (xl < 0) xl = 0;
        if (xr >= SCREEN_WIDTH) xr = SCREEN_WIDTH - 1;

        int hpix = towers[i].h;
        if (hpix > horizon_y) hpix = horizon_y;
        int top = horizon_y - hpix;
        uint16_t body = cityGrade(blend565(0x4A8F, g_horizon565, haze));
        sprite.fillRect(xl, top, xr - xl + 1, hpix, body);
        sprite.drawFastHLine(xl, top, xr - xl + 1, cityGrade(blend565(0x6B91, g_horizon565, haze)));

        // Stable lit-window grid: column/row counts are fixed per tower so
        // the lights never crawl as the projection widens at screen edges.
        if (g_lights_on && hpix > 8) {
            int ncols = (int)(towers[i].half_w * 2.0f * fov / 3.0f);
            if (ncols < 2) ncols = 2;
            if (ncols > 8) ncols = 8;
            int nrows = hpix / 4;
            float wpx = (float)(xr - xl + 1);
            for (int r = 0; r < nrows; r++) {
                for (int c = 0; c < ncols; c++) {
                    uint32_t hsh = cityHash(i * 73u + r * 16u + c, 0x5C1F1u);
                    if ((hsh % 100u) >= 38u) continue;
                    int wx = xl + (int)(wpx * (c + 0.5f) / ncols);
                    int wy = top + 3 + r * 4;
                    if (wy >= horizon_y - 1) continue;
                    sprite.drawPixel(wx, wy, (hsh & 2) ? 0xFE60 : 0xFFF2);
                }
            }
        }
        // Aircraft beacons crown the two tallest towers.
        if (g_lights_on && towers[i].h >= 42 && ((millis() / 700u + i) & 1)) {
            sprite.drawPixel((xl + xr) / 2, top - 1, 0xF800);
        }
    }
}

// Clouds spread around the full compass, tinted by the sky palette so they
// catch fire at sunset and dim to slate at dusk.
void drawCityClouds(int horizon_y, float yaw) {
    if (horizon_y < 34 || g_night > 0.80f) return;

    struct Cloud { float bearing; int8_t y_off; uint8_t w, h; };
    static const Cloud clouds[] = {
        { -2.30f, 44, 36, 10 }, { -1.65f, 58, 48, 13 }, { -1.05f, 38, 34, 9 },
        { -0.45f, 52, 44, 12 }, {  0.10f, 40, 30, 8 },  {  0.70f, 60, 50, 14 },
        {  1.45f, 42, 36, 10 }, {  2.10f, 54, 40, 11 }, {  2.80f, 36, 30, 8 }
    };
    const int n_clouds = sizeof(clouds) / sizeof(clouds[0]);

    // Triangle-wave wind: drift out, ease to a stop, drift back.
    float phase = fmodf(millis() * 0.000035f, 0.56f);
    float wind = (phase < 0.28f) ? phase : (0.56f - phase);

    uint16_t white = cityGrade(blend565(0xFFFF, g_horizon565, 7));
    uint16_t shadow = cityGrade(blend565(0xE73C, g_horizon565, 9));

    for (int i = 0; i < n_clouds; i++) {
        float d = clouds[i].bearing + wind - yaw;
        while (d > PI) d -= 2.0f * PI;
        while (d < -PI) d += 2.0f * PI;
        if (fabsf(d) > 1.28f) continue;
        int cx = panoramaScreenX(d);
        int cy = horizon_y - clouds[i].y_off;
        int cw = clouds[i].w;
        int ch = clouds[i].h;
        if (cy < 8 || cy > horizon_y - 6) continue;
        if (cx + cw < 0 || cx - cw > SCREEN_WIDTH) continue;

        sprite.fillRect(cx - cw / 2, cy, cw, ch / 2, shadow);
        sprite.fillCircle(cx - cw / 3, cy, ch / 2, shadow);
        sprite.fillCircle(cx, cy - ch / 4, ch * 2 / 3, white);
        sprite.fillCircle(cx + cw / 3, cy, ch / 2, shadow);
        sprite.fillRect(cx - cw / 3, cy - ch / 3, cw * 2 / 3, ch / 2, white);
    }
}

// Lens flare ghosts along the sun-to-center axis, only when the sun's core
// pixel actually made it to the final frame (a building or cloud in the way
// kills the flare, exactly like a real lens).
void drawSunFlare() {
    if (g_night > 0.15f) return;
    if (g_sun_sx < 0 || g_sun_sx >= SCREEN_WIDTH) return;
    if (g_sun_sy < 0 || g_sun_sy >= SCREEN_HEIGHT) return;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    uint16_t at = __builtin_bswap16(buf[g_sun_sy * SCREEN_WIDTH + g_sun_sx]);
    if (at != g_sun_core565) return;

    brightenEllipse(g_sun_sx, g_sun_sy, 26, 26, 0xFFF1, 3);
    float vx = center_x - g_sun_sx;
    float vy = center_y - g_sun_sy;
    static const float ts[4] = { 0.35f, 0.70f, 1.10f, 1.45f };
    static const int rs[4] = { 5, 3, 4, 2 };
    for (int i = 0; i < 4; i++) {
        int fx = g_sun_sx + (int)(vx * ts[i]);
        int fy = g_sun_sy + (int)(vy * ts[i]);
        brightenEllipse(fx, fy, rs[i], rs[i], (i & 1) ? 0xFFE0 : 0xFFFF, 4);
    }
}

// Pedestrians: deterministic spawn along the road network's sidewalks.
void resetCityPeds() {
    static const uint16_t shirts[] = {
        0xF800, 0x021F, 0xFFE0, 0x07E0, 0xFD20, 0xF81F, 0xFFFF, 0x6B4D
    };
    for (int i = 0; i < CITY_PED_COUNT; i++) {
        uint32_t h = cityHash(i * 131u + 7u, 0xBEDD5u);
        CityPed& p = city_peds[i];
        p.road = (uint8_t)(h % CITY_ROAD_COUNT);
        p.dir = (h & 8) ? 1 : -1;
        p.side = (h & 16) ? 1 : -1;
        p.t = ((h >> 5) % 100) * 0.01f;
        p.speed = 0.9f + ((h >> 12) % 60) * 0.01f;
        p.phase = (h >> 3) % 7;
        p.dodge = 0.0f;
        p.down = 0.0f;
        p.color = shirts[h % 8];
        const CityRoad& r = city_roads[p.road];
        p.x = r.x0 + (r.x1 - r.x0) * p.t;
        p.z = r.z0 + (r.z1 - r.z0) * p.t;
    }
}

void updateCityPeds(float dt) {
    for (int i = 0; i < CITY_PED_COUNT; i++) {
        CityPed& p = city_peds[i];

        // Knocked over: lie still where we fell, then pick ourselves back up
        // and rejoin the sidewalk. No walking/dodging while down.
        if (p.down > 0.0f) {
            p.down -= dt;
            continue;
        }

        const CityRoad& r = city_roads[p.road];
        float dx = r.x1 - r.x0;
        float dz = r.z1 - r.z0;
        float len = sqrtf(dx * dx + dz * dz);
        if (len < 1.0f) len = 1.0f;

        p.t += p.dir * p.speed * dt / len;
        if (p.t > 1.0f) { p.t = 1.0f; p.dir = -1; }
        if (p.t < 0.0f) { p.t = 0.0f; p.dir = 1; }

        // Sidewalk offset, plus a hop away from the curb when the player
        // comes in hot. Eases back to the curb afterwards.
        float pdx = player_x - p.x;
        float pdz = player_z - p.z;
        float pd2 = pdx * pdx + pdz * pdz;
        if (pd2 < 22.0f && player_speed > 25.0f) {
            p.dodge = approachFloat(p.dodge, 2.6f, 9.0f, dt);
        } else {
            p.dodge = approachFloat(p.dodge, 0.0f, 0.7f, dt);
        }

        float off = r.width * 0.5f + 1.7f + p.dodge;
        float inv = 1.0f / len;
        p.x = r.x0 + dx * p.t + (-dz * inv) * p.side * off;
        p.z = r.z0 + dz * p.t + (dx * inv) * p.side * off;
        p.phase += p.speed * dt * 5.5f;

        // Clip them: drive into a walker at speed and they go down. They keep
        // the spot they fell on (no further updates) and get up after a bit.
        if (pd2 < 1.7f && player_speed > 12.0f) {
            p.down = 7.5f;
            if (screen_shake_timer < 6) screen_shake_timer = 6;
        }
    }
}

// Tiny articulated walker: shadow, scissoring legs, torso, head, swinging
// arms once close enough. Reads correctly from 3 px tall up to ~46 px.
void drawCityPed(int i, uint8_t fog_a) {
    const CityPed& p = city_peds[i];
    float sx, sy, sz;
    projectPoint(p.x, 0.0f, p.z, sx, sy, sz);
    if (sz < 1.2f || sz > 80.0f) return;
    int x = (int)sx;
    int y = (int)sy;
    if (x < -12 || x > SCREEN_WIDTH + 12 || y < -12 || y > SCREEN_HEIGHT + 12) return;

    int ph = (int)(1.62f * fov / sz);
    uint16_t shirt = cityGrade(p.color);
    uint16_t pants = cityGrade(0x29A6);
    uint16_t skin = cityGrade(0xE54F);
    if (fog_a) {
        shirt = blend565(shirt, g_horizon565, fog_a);
        pants = blend565(pants, g_horizon565, fog_a);
        skin = blend565(skin, g_horizon565, fog_a);
    }
    if (ph > 46) ph = 46;

    // Knocked over: a body laid flat on the road, head to one side, with a
    // small red mark beneath it. Reads at any size.
    if (p.down > 0.0f) {
        shadowEllipse(x, y, ph / 4 + 2, ph / 10 + 1, 6);
        int bl = ph / 2; if (bl < 2) bl = 2;        // body half-length
        int th = ph / 7 + 1;                        // body thickness
        uint16_t red = cityGrade(0xC000);
        if (fog_a) red = blend565(red, g_horizon565, fog_a);
        sprite.fillRect(x - bl, y - th, bl * 2, th, shirt);            // torso/legs
        int hr = ph / 9 + 1;
        sprite.fillCircle(x - bl - hr / 2, y - th / 2 - 1, hr, skin);  // head
        sprite.fillRect(x - bl - hr, y - 1, hr + 2, 2, red);          // small red pool
        return;
    }

    if (ph < 4) {
        sprite.fillRect(x, y - 2, 1, 2, shirt);
        return;
    }

    shadowEllipse(x, y, ph / 5 + 1, ph / 14 + 1, 6);

    int head_r = ph / 9 + 1;
    int head_cy = y - ph + head_r;
    int hip = y - (ph * 9) / 20;
    int torso_top = head_cy + head_r;
    int tw = ph / 5 + 1;
    int spread = (int)(sinf(p.phase) * ph * 0.14f);

    sprite.drawLine(x, hip, x - spread, y, pants);
    sprite.drawLine(x, hip, x + spread, y, pants);
    sprite.fillRect(x - tw / 2, torso_top, tw, hip - torso_top, shirt);
    if (ph > 14) {
        sprite.drawLine(x - tw / 2, torso_top + 1, x - tw / 2 - spread / 2, hip - 1, shirt);
        sprite.drawLine(x + tw / 2, torso_top + 1, x + tw / 2 + spread / 2, hip - 1, shirt);
    }
    sprite.fillCircle(x, head_cy, head_r, skin);
}

// Scanline read-modify-write triangle: per covered pixel either darkens the
// framebuffer (real shadow) or blends it toward a tint (real light). This is
// what beams and cast shadows are painted with -- fillTriangle can't read
// what's underneath it.
void rmwTriangle(float fx0, float fy0, float fx1, float fy1, float fx2, float fy2,
                 uint16_t tint, uint8_t amt, bool darken) {
    if (amt == 0) return;
    float t;
    if (fy1 < fy0) { t = fx0; fx0 = fx1; fx1 = t; t = fy0; fy0 = fy1; fy1 = t; }
    if (fy2 < fy0) { t = fx0; fx0 = fx2; fx2 = t; t = fy0; fy0 = fy2; fy2 = t; }
    if (fy2 < fy1) { t = fx1; fx1 = fx2; fx2 = t; t = fy1; fy1 = fy2; fy2 = t; }
    float dy02 = fy2 - fy0;
    if (dy02 < 0.001f) return;

    int y_top = (int)ceilf(fy0);
    int y_bot = (int)floorf(fy2);
    if (y_top < 0) y_top = 0;
    if (y_bot >= SCREEN_HEIGHT) y_bot = SCREEN_HEIGHT - 1;
    if (y_top > y_bot) return;

    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    uint8_t keep = 32 - amt;
    float dy01 = fy1 - fy0;
    float dy12 = fy2 - fy1;
    for (int y = y_top; y <= y_bot; y++) {
        float xa = fx0 + (fx2 - fx0) * ((y - fy0) / dy02);
        float xb;
        if (y < fy1) {
            xb = (dy01 > 0.001f) ? fx0 + (fx1 - fx0) * ((y - fy0) / dy01) : fx1;
        } else {
            xb = (dy12 > 0.001f) ? fx1 + (fx2 - fx1) * ((y - fy1) / dy12) : fx1;
        }
        int xl = (int)((xa < xb) ? xa : xb);
        int xr = (int)((xa > xb) ? xa : xb);
        if (xl < 0) xl = 0;
        if (xr >= SCREEN_WIDTH) xr = SCREEN_WIDTH - 1;
        if (xl > xr) continue;
        uint16_t* p = buf + y * SCREEN_WIDTH + xl;
        if (darken) {
            for (int x = xl; x <= xr; x++, p++) {
                uint16_t c = __builtin_bswap16(*p);
                *p = __builtin_bswap16(shade565(c, keep));
            }
        } else {
            for (int x = xl; x <= xr; x++, p++) {
                uint16_t c = __builtin_bswap16(*p);
                *p = __builtin_bswap16(blend565(c, tint, amt));
            }
        }
    }
}

// Additive light splash: blends framebuffer pixels toward a warm tint.
// The exact counterpart of shadowEllipse, used for headlights and lamps.
void brightenEllipse(int cx, int cy, int rx, int ry, uint16_t tint, uint8_t add) {
    if (rx <= 0 || ry <= 0 || add == 0) return;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    for (int dy = -ry; dy <= ry; dy++) {
        int y = cy + dy;
        if (y < 0 || y >= SCREEN_HEIGHT) continue;
        float fy = (float)dy / (float)ry;
        int span = (int)(rx * sqrtf(1.0f - fy * fy));
        int x0 = cx - span; if (x0 < 0) x0 = 0;
        int x1 = cx + span; if (x1 >= SCREEN_WIDTH) x1 = SCREEN_WIDTH - 1;
        uint16_t* px = buf + y * SCREEN_WIDTH + x0;
        for (int x = x0; x <= x1; x++, px++) {
            uint16_t c = __builtin_bswap16(*px);
            *px = __builtin_bswap16(blend565(c, tint, add));
        }
    }
}

// A real projected cone of light: a quad on the road surface running from
// the bumper out ahead of the car, widening as it goes, blended additively
// so lane markings stay readable inside the beam.
void drawHeadlightBeam(float x, float z, float heading, float nose, float near_hw, float len, float far_hw, uint8_t add) {
    float cy = cosf(heading);
    float sy = sinf(heading);
    float side_v[4] = { -near_hw, near_hw, far_hw, -far_hw };
    float fwd_v[4] = { nose, nose, nose + len, nose + len };
    float sxp[4], syp[4];
    for (int i = 0; i < 4; i++) {
        float wx = x + side_v[i] * cy + fwd_v[i] * sy;
        float wz = z - side_v[i] * sy + fwd_v[i] * cy;
        float sx, sy2, sz;
        projectPoint(wx, 0.02f, wz, sx, sy2, sz);
        if (sz < 0.30f) return; // reaches behind the camera: skip this frame
        pullInRadial(sx, sy2, 1500.0f);
        sxp[i] = sx;
        syp[i] = sy2;
    }
    rmwTriangle(sxp[0], syp[0], sxp[1], syp[1], sxp[2], syp[2], 0xFFB5, add, false);
    rmwTriangle(sxp[0], syp[0], sxp[2], syp[2], sxp[3], syp[3], 0xFFB5, add, false);
}

// Headlight lenses, taillights, and the beam pool for a vehicle. Lenses are
// shown only from the side that can see them; the pool shows from anywhere.
void drawCityVehicleLights(float x, float z, float heading, float speed_ratio, bool braking) {
    float cy = cosf(heading);
    float sy = sinf(heading);
    float cam_dx = cam_x - x;
    float cam_dz = cam_z - z;
    float cam_local_z = cam_dx * sy + cam_dz * cy; // + = camera ahead of the car

    if (g_lights_on) {
        drawHeadlightBeam(x, z, heading, 1.55f, 0.85f, 8.0f, 2.3f, 4);
    }

    if (cam_local_z < 0.35f && (braking || g_lights_on)) {
        uint16_t lamp = braking ? 0xF800 : 0xB000;
        for (int side = -1; side <= 1; side += 2) {
            float lx = 0.42f * side, ly = 0.34f, lz = -1.55f;
            float wx = x + lx * cy + lz * sy;
            float wz = z - lx * sy + lz * cy;
            float sx, syp, sz;
            projectPoint(wx, ly, wz, sx, syp, sz);
            if (sz <= 0.4f || sz > 60.0f) continue;
            int r = (int)(16.0f / sz);
            if (r < 1) r = 1;
            if (r > 3) r = 3;
            sprite.fillRect((int)sx - r, (int)syp - r, r * 2 + 1, r * 2 + 1, lamp);
        }
    }
    if (cam_local_z > -0.35f && g_lights_on) {
        for (int side = -1; side <= 1; side += 2) {
            float lx = 0.44f * side, ly = 0.36f, lz = 1.55f;
            float wx = x + lx * cy + lz * sy;
            float wz = z - lx * sy + lz * cy;
            float sx, syp, sz;
            projectPoint(wx, ly, wz, sx, syp, sz);
            if (sz <= 0.4f || sz > 80.0f) continue;
            int r = (int)(20.0f / sz);
            if (r < 1) r = 1;
            if (r > 3) r = 3;
            sprite.fillRect((int)sx - r, (int)syp - r, r * 2 + 1, r * 2 + 1, 0xFFF6);
            sprite.drawPixel((int)sx, (int)syp, 0xFFFF);
        }
    }
    (void)speed_ratio;
}

// Near-clip a world quad and project it to screen (up to 5 points after the
// clip). Shared by the solid painter and the RMW shadow painter.
static int clipWorldQuadToScreen(const float* wx, const float* wy, const float* wz,
                                 Point2D* screen, float pull_limit) {
    const float NEAR_Z = 0.42f;
    Point3D poly[8];
    Point3D clipped[8];

    int n = 4;
    for (int i = 0; i < 4; i++) {
        float dx = wx[i] - cam_x;
        float dy = wy[i] - cam_y;
        float dz = wz[i] - cam_z;
        float rz1 = dx * cam_sin_yaw + dz * cam_cos_yaw;
        poly[i].x = dx * cam_cos_yaw - dz * cam_sin_yaw;
        float ry1 = dy;
        poly[i].y = ry1 * cam_cos_pitch - rz1 * cam_sin_pitch;
        poly[i].z = ry1 * cam_sin_pitch + rz1 * cam_cos_pitch;
    }

    int out_n = 0;
    for (int i = 0; i < n; i++) {
        Point3D a = poly[i];
        Point3D b = poly[(i + 1) % n];
        bool a_in = a.z >= NEAR_Z;
        bool b_in = b.z >= NEAR_Z;
        if (a_in && b_in) {
            clipped[out_n++] = b;
        } else if (a_in && !b_in) {
            float t = (NEAR_Z - a.z) / (b.z - a.z);
            clipped[out_n++] = Point3D{ a.x + (b.x - a.x) * t,
                                        a.y + (b.y - a.y) * t,
                                        NEAR_Z };
        } else if (!a_in && b_in) {
            float t = (NEAR_Z - a.z) / (b.z - a.z);
            clipped[out_n++] = Point3D{ a.x + (b.x - a.x) * t,
                                        a.y + (b.y - a.y) * t,
                                        NEAR_Z };
            clipped[out_n++] = b;
        }
    }
    if (out_n < 3) return 0;

    for (int i = 0; i < out_n; i++) {
        float inv = fov / clipped[i].z;
        float sx = center_x + clipped[i].x * inv;
        float sy = center_y - clipped[i].y * inv;
        pullInRadial(sx, sy, pull_limit);
        screen[i].x = sx;
        screen[i].y = sy;
    }
    return out_n;
}

void drawWorldQuadClipped(float x0, float y0, float z0, float x1, float y1, float z1,
                          float x2, float y2, float z2, float x3, float y3, float z3,
                          uint16_t color) {
    float wx[4] = { x0, x1, x2, x3 };
    float wy[4] = { y0, y1, y2, y3 };
    float wz[4] = { z0, z1, z2, z3 };
    Point2D screen[8];
    int out_n = clipWorldQuadToScreen(wx, wy, wz, screen, 1800.0f);
    for (int i = 1; i < out_n - 1; i++) {
        sprite.fillTriangle((int16_t)screen[0].x, (int16_t)screen[0].y,
                            (int16_t)screen[i].x, (int16_t)screen[i].y,
                            (int16_t)screen[i + 1].x, (int16_t)screen[i + 1].y,
                            color);
    }
}

// Single-pass convex-polygon RMW darken: each scanline span is darkened exactly
// once. A triangle fan (the old approach) re-darkened the shared diagonal edge,
// which showed as a faint darker line across the shadow quad -- this kills it.
static void rmwConvexDarken(const Point2D* pts, int n, uint8_t amt) {
    if (amt == 0 || n < 3) return;
    float ymin = 1e9f, ymax = -1e9f;
    for (int i = 0; i < n; i++) {
        if (pts[i].y < ymin) ymin = pts[i].y;
        if (pts[i].y > ymax) ymax = pts[i].y;
    }
    int y0 = (int)ceilf(ymin);  if (y0 < 0) y0 = 0;
    int y1 = (int)floorf(ymax); if (y1 >= SCREEN_HEIGHT) y1 = SCREEN_HEIGHT - 1;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    uint8_t keep = 32 - amt;
    for (int y = y0; y <= y1; y++) {
        float fy = (float)y + 0.5f;
        float xl = 1e9f, xr = -1e9f;
        for (int i = 0; i < n; i++) {
            const Point2D& a = pts[i];
            const Point2D& b = pts[(i + 1) % n];
            if ((a.y <= fy) == (b.y <= fy)) continue; // edge doesn't straddle this row
            float t = (fy - a.y) / (b.y - a.y);
            float x = a.x + (b.x - a.x) * t;
            if (x < xl) xl = x;
            if (x > xr) xr = x;
        }
        if (xr < xl) continue;
        int ixl = (int)ceilf(xl - 0.5f);  if (ixl < 0) ixl = 0;
        int ixr = (int)floorf(xr - 0.5f); if (ixr >= SCREEN_WIDTH) ixr = SCREEN_WIDTH - 1;
        uint16_t* p = buf + y * SCREEN_WIDTH + ixl;
        for (int x = ixl; x <= ixr; x++, p++) {
            uint16_t c = __builtin_bswap16(*p);
            *p = __builtin_bswap16(shade565(c, keep));
        }
    }
}

// Translucent shadow quad on the ground: same clipping as the solid painter
// but darkens what's already there instead of overwriting it. Filled as one
// convex span-per-row so there's no internal seam.
void shadowWorldQuad(float x0, float y0, float z0, float x1, float y1, float z1,
                     float x2, float y2, float z2, float x3, float y3, float z3, uint8_t darken) {
    float wx[4] = { x0, x1, x2, x3 };
    float wy[4] = { y0, y1, y2, y3 };
    float wz[4] = { z0, z1, z2, z3 };
    Point2D screen[8];
    int out_n = clipWorldQuadToScreen(wx, wy, wz, screen, 1500.0f);
    rmwConvexDarken(screen, out_n, darken);
}

// Buildings cast real ground shadows away from the sun: they stretch out
// through the morning, pull in tight at noon, swing east and grow again
// toward sunset, then fade through dusk.
void drawCityBuildingShadows() {
    if (g_shadow_a == 0) return;
    // Cast shadows are read-modify-write fills (2x the cost of a flat fill) and
    // a long tower shadow can blanket a big patch of ground, so they are the
    // priciest optional layer. Only the nearest few cast -- past ~70 m the
    // ground haze swallows them anyway.
    int budget = 7;
    for (int i = 0; i < CITY_LAYOUT_BUILDING_COUNT && budget > 0; i++) {
        const CityLot& lot = city_lots[i];
        float ddx = lot.x - cam_x;
        float ddz = lot.z - cam_z;
        if (ddx * ddx + ddz * ddz > 5000.0f) continue; // ~70 m
        // Behind the camera? its shadow can't be on screen either.
        if (ddx * cam_sin_yaw + ddz * cam_cos_yaw < -6.0f) continue;
        budget--;

        float hh = lot.h;
        if (hh > 12.0f) hh = 12.0f;
        float svx = g_shadow_dx * hh * g_shadow_len;
        float svz = g_shadow_dz * hh * g_shadow_len;

        // Silhouette ends: the footprint corners extreme perpendicular to the
        // shadow direction; the cast quad spans between them.
        float hw = lot.w * 0.5f, hd = lot.d * 0.5f;
        float best = -1e9f, worst = 1e9f;
        float ax = 0, az = 0, bx = 0, bz = 0;
        for (int c = 0; c < 4; c++) {
            float cxw = (c & 1) ? hw : -hw;
            float czw = (c & 2) ? hd : -hd;
            float cross = cxw * g_shadow_dz - czw * g_shadow_dx;
            if (cross > best) { best = cross; ax = lot.x + cxw; az = lot.z + czw; }
            if (cross < worst) { worst = cross; bx = lot.x + cxw; bz = lot.z + czw; }
        }
        shadowWorldQuad(ax, 0.015f, az,
                        bx, 0.015f, bz,
                        bx + svx, 0.015f, bz + svz,
                        ax + svx, 0.015f, az + svz,
                        g_shadow_a);
    }
}

bool cityResolveLotCollision(const CityLot& lot, float& x, float& z, float prev_x, float prev_z, float& out_nx, float& out_nz) {
    float pad = (lot.type == 2) ? 1.05f : 1.25f;
    float half_w = lot.w * 0.5f + pad;
    float half_d = lot.d * 0.5f + pad;
    float dx = x - lot.x;
    float dz = z - lot.z;
    if (fabsf(dx) >= half_w || fabsf(dz) >= half_d) return false;

    float prev_dx = prev_x - lot.x;
    float prev_dz = prev_z - lot.z;
    float pen_x = half_w - fabsf(dx);
    float pen_z = half_d - fabsf(dz);
    const float escape_pad = 1.85f;

    if (pen_x < pen_z) {
        out_nx = (dx != 0.0f) ? ((dx > 0.0f) ? 1.0f : -1.0f)
                              : ((prev_dx > 0.0f) ? 1.0f : -1.0f);
        out_nz = 0.0f;
        x = lot.x + out_nx * (half_w + escape_pad);
    } else {
        out_nx = 0.0f;
        out_nz = (dz != 0.0f) ? ((dz > 0.0f) ? 1.0f : -1.0f)
                              : ((prev_dz > 0.0f) ? 1.0f : -1.0f);
        z = lot.z + out_nz * (half_d + escape_pad);
    }
    return true;
}

bool cityLineHitsLot(const CityLot& lot, float x, float z) {
    float dx = x - cam_x;
    float dz = z - cam_z;
    float min_x = lot.x - lot.w * 0.5f - 0.35f;
    float max_x = lot.x + lot.w * 0.5f + 0.35f;
    float min_z = lot.z - lot.d * 0.5f - 0.35f;
    float max_z = lot.z + lot.d * 0.5f + 0.35f;
    float t0 = 0.02f;
    float t1 = 0.965f;

    if (fabsf(dx) < 0.001f) {
        if (cam_x < min_x || cam_x > max_x) return false;
    } else {
        float inv = 1.0f / dx;
        float a = (min_x - cam_x) * inv;
        float b = (max_x - cam_x) * inv;
        if (a > b) { float t = a; a = b; b = t; }
        if (a > t0) t0 = a;
        if (b < t1) t1 = b;
        if (t0 > t1) return false;
    }

    if (fabsf(dz) < 0.001f) {
        if (cam_z < min_z || cam_z > max_z) return false;
    } else {
        float inv = 1.0f / dz;
        float a = (min_z - cam_z) * inv;
        float b = (max_z - cam_z) * inv;
        if (a > b) { float t = a; a = b; b = t; }
        if (a > t0) t0 = a;
        if (b < t1) t1 = b;
    }
    return t0 <= t1;
}

// --- Northern dirt rally circuit -----------------------------------------
// A flowing ~580 m loop (generated by a star-shaped radius(theta), so it can
// never self-intersect) on the far-north grass, fed by the extended avenue.
// The drivable surface is the band within CITY_TRACK_HALFW of this centreline;
// the surface isn't a registered road, so the off-road model gives it a loose
// rally feel, and a wood post-and-rail fence lines both edges. Everything is
// gated on distance from the track centre, so it costs nothing in the city.
struct TrackPt { float x, z; };
#define CITY_TRACK_PTS 30
#define CITY_TRACK_CX 0.0f
#define CITY_TRACK_CZ 286.0f
#define CITY_TRACK_HALFW 6.0f       // half track width (~12 m, 3-4 cars abreast)
#define CITY_TRACK_ENTRY 15         // south vertex: the avenue tees in here
static const TrackPt track_pts[CITY_TRACK_PTS] = {
    {   0.0f, 390.8f}, {  21.6f, 387.8f}, {  40.1f, 376.0f}, {  54.6f, 361.1f}, {  65.3f, 344.8f},
    {  69.1f, 325.9f}, {  63.7f, 306.7f}, {  53.1f, 291.6f}, {  47.4f, 281.0f}, {  52.7f, 268.9f},
    {  64.1f, 249.0f}, {  69.5f, 223.4f}, {  61.4f, 201.5f}, {  42.4f, 190.8f}, {  20.4f, 189.9f},
    {   0.0f, 192.8f}, { -18.9f, 197.0f}, { -35.9f, 205.4f}, { -48.0f, 220.0f}, { -54.0f, 237.4f},
    { -58.2f, 252.4f}, { -66.1f, 264.5f}, { -76.9f, 277.9f}, { -82.6f, 294.7f}, { -77.1f, 311.1f},
    { -63.2f, 322.5f}, { -49.7f, 330.7f}, { -41.2f, 342.7f}, { -33.6f, 361.4f}, { -20.1f, 380.7f}
};

// Resolve the car against ONE thin fence segment A->B. It's a true wall: the
// car can sit on either side (track or grass) and only gets stopped when it
// overlaps the line or just crossed it -- no magnet pull from a distance. The
// car is pushed back to whichever side its PREVIOUS position was on. `onx/onz`
// is the outward normal (toward that open side).
static bool trackWallResolve(float ax, float az, float bx, float bz,
                             float& x, float& z, float px, float pz,
                             float thick, float& onx, float& onz) {
    float ex = bx - ax, ez = bz - az;
    float len2 = ex * ex + ez * ez;
    if (len2 < 0.0001f) return false;
    float len = sqrtf(len2), inv = 1.0f / len;
    float dx = ex * inv, dz = ez * inv;          // unit direction
    float nx = -dz, nz = dx;                      // unit left normal
    float along = (x - ax) * dx + (z - az) * dz;
    if (along < -thick || along > len + thick) return false;      // off the segment ends
    float ac = along < 0.0f ? 0.0f : (along > len ? len : along);
    float qx = ax + dx * ac, qz = az + dz * ac;
    float perp2 = (x - qx) * (x - qx) + (z - qz) * (z - qz);
    float s_cur = (x - ax) * nx + (z - az) * nz;
    float s_prev = (px - ax) * nx + (pz - az) * nz;
    bool crossed = (s_cur * s_prev < 0.0f) && along > 0.0f && along < len;
    if (!crossed && perp2 >= thick * thick) return false;          // not touching the wall
    float side = (s_prev >= 0.0f) ? 1.0f : -1.0f;                  // keep the car on its old side
    x = qx + nx * side * thick;
    z = qz + nz * side * thick;
    onx = nx * side; onz = nz * side;                             // normal toward the open side
    return true;
}

// The rally fences as solid walls. The car may roam freely on the grass outside
// the outer fence or on the track inside it; it just can't pass through either
// fence line (with a gap in the outer fence at the south entry). Only corrects
// position + returns the wall normal -- the caller scales the speed penalty by
// how head-on the hit was, so a glancing scrape barely slows you.
bool cityTrackFenceHit(float& x, float& z, float px, float pz, float& out_nx, float& out_nz) {
    float dxc = x - CITY_TRACK_CX, dzc = z - CITY_TRACK_CZ;
    if (dxc * dxc + dzc * dzc > 16000.0f) return false;           // nowhere near the track
    float ox[CITY_TRACK_PTS], oz[CITY_TRACK_PTS], inx[CITY_TRACK_PTS], inz[CITY_TRACK_PTS];
    for (int i = 0; i < CITY_TRACK_PTS; i++) {
        int p = (i - 1 + CITY_TRACK_PTS) % CITY_TRACK_PTS, n = (i + 1) % CITY_TRACK_PTS;
        float ax = track_pts[i].x - track_pts[p].x, az = track_pts[i].z - track_pts[p].z;
        float bx = track_pts[n].x - track_pts[i].x, bz = track_pts[n].z - track_pts[i].z;
        float al = sqrtf(ax * ax + az * az), bl = sqrtf(bx * bx + bz * bz);
        if (al > 0.001f) { ax /= al; az /= al; }
        if (bl > 0.001f) { bx /= bl; bz /= bl; }
        float tx = ax + bx, tz = az + bz, tl = sqrtf(tx * tx + tz * tz);
        if (tl > 0.001f) { tx /= tl; tz /= tl; }
        float nx = tz, nz = -tx;
        if ((track_pts[i].x - CITY_TRACK_CX) * nx + (track_pts[i].z - CITY_TRACK_CZ) * nz < 0.0f) { nx = -nx; nz = -nz; }
        ox[i] = track_pts[i].x + nx * CITY_TRACK_HALFW; oz[i] = track_pts[i].z + nz * CITY_TRACK_HALFW;
        inx[i] = track_pts[i].x - nx * CITY_TRACK_HALFW; inz[i] = track_pts[i].z - nz * CITY_TRACK_HALFW;
    }
    const float THICK = 1.0f;
    for (int i = 0; i < CITY_TRACK_PTS; i++) {
        int j = (i + 1) % CITY_TRACK_PTS;
        if (trackWallResolve(inx[i], inz[i], inx[j], inz[j], x, z, px, pz, THICK, out_nx, out_nz))
            return true;                                          // inner fence (full loop)
        float mx = (track_pts[i].x + track_pts[j].x) * 0.5f;
        float mz = (track_pts[i].z + track_pts[j].z) * 0.5f;
        if (fabsf(mx) < 12.0f && mz < 200.0f) continue;           // outer fence has a gap here (entry)
        if (trackWallResolve(ox[i], oz[i], ox[j], oz[j], x, z, px, pz, THICK, out_nx, out_nz))
            return true;
    }
    return false;
}

// True when the car is on the rally band (within ~HALFW of the centreline, in
// the track annulus). The physics uses this to let the dirt track run free at
// full speed, while city grass/dirt stays capped to a crawl.
bool cityOnTrack(float x, float z) {
    float dxc = x - CITY_TRACK_CX, dzc = z - CITY_TRACK_CZ;
    float cd2 = dxc * dxc + dzc * dzc;
    if (cd2 < 1225.0f || cd2 > 14400.0f) return false;
    float best = 1e9f;
    for (int i = 0; i < CITY_TRACK_PTS; i++) {
        float ax = track_pts[i].x, az = track_pts[i].z;
        int j = (i + 1) % CITY_TRACK_PTS;
        float ex = track_pts[j].x - ax, ez = track_pts[j].z - az;
        float t = ((x - ax) * ex + (z - az) * ez) / (ex * ex + ez * ez);
        if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
        float px = ax + ex * t, pz = az + ez * t;
        float dd = (x - px) * (x - px) + (z - pz) * (z - pz);
        if (dd < best) best = dd;
    }
    float lim = CITY_TRACK_HALFW + 2.0f;            // a touch wider than the collision band
    return best <= lim * lim;
}

bool cityBuildingCollision(float& x, float& z, float prev_x, float prev_z, float& out_nx, float& out_nz) {
    out_nx = 0.0f;
    out_nz = 0.0f;
    for (int i = 0; i < CITY_LAYOUT_BUILDING_COUNT; i++) {
        if (cityResolveLotCollision(city_lots[i], x, z, prev_x, prev_z, out_nx, out_nz)) {
            return true;
        }
    }
    // Plaza balustrade runs are solid (you can only drive in through the gaps);
    // the corner statues' plinths are solid too. Both reuse the lot AABB solver.
    for (int i = 0; i < CITY_PLAZA_RAIL_COUNT; i++) {
        const CityRail& r = city_plaza_rails[i];
        CityLot rl = { r.x, r.z, r.w, r.d, 1.0f, 0, 0 };
        if (cityResolveLotCollision(rl, x, z, prev_x, prev_z, out_nx, out_nz)) return true;
    }
    for (int i = 0; i < CITY_STATUE_COUNT; i++) {
        CityLot sl = { city_statues[i].x, city_statues[i].z, 1.1f, 1.1f, 1.0f, 0, 0 };
        if (cityResolveLotCollision(sl, x, z, prev_x, prev_z, out_nx, out_nz)) return true;
    }
    return false;
}

bool cityPointOccludedByBuilding(float x, float z, float height) {
    float dx = x - cam_x;
    float dz = z - cam_z;
    if (dx * dx + dz * dz < 9.0f) return false;

    for (int i = 0; i < CITY_LAYOUT_BUILDING_COUNT; i++) {
        const CityLot& lot = city_lots[i];
        if (lot.h > height * 0.65f && cityLineHitsLot(lot, x, z)) return true;
    }
    return false;
}

// Lowest world height at which the vertical courier beam at (tx,tz) clears every
// building between it and the camera. The beam is one (x,z) column, so all of
// its heights share the same horizontal sight line and only the vertical angle
// changes -- a single threshold covers the whole beam (true 3D occlusion, not
// the old "lift the base 16 m" fudge). For a building whose footprint the sight
// line enters at parameter t0 (0 at the camera, 1 at the beacon), the rising ray
// is lowest at that near edge; it clears the roof once cam_y + t0*(y-cam_y) > h,
// i.e. y > cam_y + (h-cam_y)/t0. The max over every crossed building is where
// the beam reaches open sky. Returns 0 when nothing blocks the base. Buildings
// are the only occluders tall enough to matter.
static float cityBeamVisibleAboveY(float tx, float tz) {
    float dx = tx - cam_x;
    float dz = tz - cam_z;
    float vis_y = 0.0f;
    for (int i = 0; i < CITY_LAYOUT_BUILDING_COUNT; i++) {
        const CityLot& lot = city_lots[i];
        float min_x = lot.x - lot.w * 0.5f - 0.35f;
        float max_x = lot.x + lot.w * 0.5f + 0.35f;
        float min_z = lot.z - lot.d * 0.5f - 0.35f;
        float max_z = lot.z + lot.d * 0.5f + 0.35f;
        float t0 = 0.0f, t1 = 1.0f;
        if (fabsf(dx) < 0.0001f) {
            if (cam_x < min_x || cam_x > max_x) continue;
        } else {
            float inv = 1.0f / dx;
            float a = (min_x - cam_x) * inv;
            float b = (max_x - cam_x) * inv;
            if (a > b) { float t = a; a = b; b = t; }
            if (a > t0) t0 = a;
            if (b < t1) t1 = b;
        }
        if (fabsf(dz) < 0.0001f) {
            if (cam_z < min_z || cam_z > max_z) continue;
        } else {
            float inv = 1.0f / dz;
            float a = (min_z - cam_z) * inv;
            float b = (max_z - cam_z) * inv;
            if (a > b) { float t = a; a = b; b = t; }
            if (a > t0) t0 = a;
            if (b < t1) t1 = b;
        }
        if (t0 > t1) continue;                       // sight line misses the footprint
        if (t0 <= 0.001f || t0 >= 1.0f) continue;    // at the camera, or past the beacon
        float yb = cam_y + (lot.h - cam_y) / t0;     // height that clears this roof
        if (yb > vis_y) vis_y = yb;
    }
    return vis_y;
}

void drawCityRect(float x0, float z0, float x1, float z1, uint16_t color) {
    drawWorldQuadClipped(x0, 0.0f, z0, x1, 0.0f, z0, x1, 0.0f, z1, x0, 0.0f, z1, color);
}

void drawCityStrip(float x0, float z0, float x1, float z1, float width, uint16_t color) {
    float dx = x1 - x0;
    float dz = z1 - z0;
    float len = sqrtf(dx * dx + dz * dz);
    if (len < 0.1f) return;
    float px = -dz / len * width * 0.5f;
    float pz =  dx / len * width * 0.5f;
    drawWorldQuadClipped(x0 + px, 0.018f, z0 + pz,
                         x1 + px, 0.018f, z1 + pz,
                         x1 - px, 0.018f, z1 - pz,
                         x0 - px, 0.018f, z0 - pz,
                         color);

    // Dashed center markings, only on the stretch near the camera -- distant
    // dashes are a sub-pixel shimmer that still cost a projection each. Walk
    // outward from the closest point on the strip and stop once we pass the
    // visible range, instead of projecting every dash on a 300 m avenue.
    float tnear = (((cam_x - x0) * dx + (cam_z - z0) * dz) / (len * len));
    if (tnear < 0.0f) tnear = 0.0f;
    if (tnear > 1.0f) tnear = 1.0f;
    float step = 11.0f / len;            // dash period as a fraction of the strip
    if (step < 0.02f) step = 0.02f;
    uint16_t dash_col = cityGrade(0xD69A);
    for (int dir = -1; dir <= 1; dir += 2) {
        for (int k = (dir < 0 ? 0 : 1); k < 40; k++) {
            float t0 = tnear + dir * k * step;
            if (t0 < 0.0f || t0 > 1.0f) break;
            float t1 = t0 + step * 0.5f;
            if (t1 > 1.0f) t1 = 1.0f;
            float l0x, l0y, l0z, l1x, l1y, l1z;
            projectPoint(x0 + dx * t0, 0.04f, z0 + dz * t0, l0x, l0y, l0z);
            if (l0z > 80.0f) break;       // marched past the visible range
            if (l0z < 1.0f) continue;
            projectPoint(x0 + dx * t1, 0.04f, z0 + dz * t1, l1x, l1y, l1z);
            if (l1z < 1.0f) continue;
            sprite.drawLine((int16_t)l0x, (int16_t)l0y, (int16_t)l1x, (int16_t)l1y, dash_col);
        }
    }
}

void drawCityPlainStrip(float x0, float z0, float x1, float z1, float width, uint16_t color, float lift) {
    float dx = x1 - x0;
    float dz = z1 - z0;
    float len = sqrtf(dx * dx + dz * dz);
    if (len < 0.1f) return;
    float px = -dz / len * width * 0.5f;
    float pz =  dx / len * width * 0.5f;
    drawWorldQuadClipped(x0 + px, lift, z0 + pz,
                         x1 + px, lift, z1 + pz,
                         x1 - px, lift, z1 - pz,
                         x0 - px, lift, z0 - pz,
                         color);
}

// Distance haze strength for free-standing city objects (buildings, cars,
// trees): nothing under ~90 m, fading toward the horizon color at the
// building cull radius so pop-in is invisible.
static inline uint8_t cityFogFromD2(float d2) {
    if (d2 < 8000.0f) return 0;
    float f = (d2 - 8000.0f) * (1.0f / 22000.0f);
    if (f > 1.0f) f = 1.0f;
    return (uint8_t)(f * 22.0f);
}

// Window grid on a building face. By day all windows share a cool glass
// color; after dark each window keeps a stable on/off state (hashed from the
// building seed and its grid cell) so the skyline lights up convincingly.
void drawCityFacadeGrid(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy,
                        uint16_t day_color, uint16_t seed, uint8_t fog_a, int rows, int cols) {
    uint16_t day_win = cityGrade(day_color);
    if (fog_a) day_win = blend565(day_win, g_horizon565, fog_a);
    uint16_t dark_win = cityGrade(0x10A2);
    if (fog_a) dark_win = blend565(dark_win, g_horizon565, fog_a);

    float inv_rows = 1.0f / (float)rows;
    float inv_cols = 1.0f / (float)cols;
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            float u = (col + 0.5f) * inv_cols;
            float v = (row + 0.5f) * 0.9f * inv_rows + 0.06f;
            float uw = inv_cols * 0.48f;   // window half-cell spans scale with grid density
            float vh = inv_rows * 0.40f;
            float px = ax * (1.0f - u) * (1.0f - v) + bx * u * (1.0f - v) + cx * u * v + dx * (1.0f - u) * v;
            float py = ay * (1.0f - u) * (1.0f - v) + by * u * (1.0f - v) + cy * u * v + dy * (1.0f - u) * v;
            float nx = ax * (1.0f - (u + uw)) * (1.0f - v) + bx * (u + uw) * (1.0f - v) +
                       cx * (u + uw) * v + dx * (1.0f - (u + uw)) * v;
            float ny = ay * (1.0f - u) * (1.0f - (v + vh)) + by * u * (1.0f - (v + vh)) +
                       cy * u * (v + vh) + dy * (1.0f - u) * (v + vh);
            int ww = (int)(fabsf(nx - px));
            int hh = (int)(fabsf(ny - py));
            if (ww < 2) ww = 2;
            if (hh < 1) hh = 1;
            if (ww > 8) ww = 8;
            if (hh > 5) hh = 5;

            int rx = (int)px - ww / 2;
            int ry = (int)py - hh / 2;
            // Off-screen window: fillRect would clip it to nothing anyway, so
            // skip the call (and the hash). Common when driving close past a
            // facade that runs off the screen edge -- half its grid is unseen.
            if (rx + ww < 0 || rx > SCREEN_WIDTH || ry + hh < 0 || ry > SCREEN_HEIGHT) continue;

            uint16_t win;
            if (g_lights_on) {
                uint32_t hsh = cityHash(seed * 23u + row * 5u + col, 0xA11CE5u);
                if ((hsh % 100u) < 54u) {
                    win = (hsh & 2) ? 0xFE60 : 0xFF2B; // warm interior light (emissive)
                    if (fog_a) win = blend565(win, g_horizon565, fog_a);
                } else {
                    win = dark_win;
                }
            } else {
                win = day_win;
            }
            sprite.fillRect(rx, ry, ww, hh, win);
        }
    }
}

// A suburban facade: two windows and (on the front) a door, instead of the
// dense office-tower window grid. Corner order matches drawCityFacadeGrid
// (a/b = base left/right, c/d = top right/left; u left->right, v bottom->top).
static void drawHouseFacade(float ax, float ay, float bx, float by,
                            float cx, float cy, float dx, float dy,
                            uint16_t seed, uint8_t fog_a, bool door) {
    uint16_t win;
    if (g_lights_on) {
        uint32_t hsh = cityHash(seed * 23u + 7u, 0xA11CE5u);
        win = ((hsh % 100u) < 60u) ? 0xFE60 : cityGrade(0x2104);
    } else {
        win = cityGrade(0x9DDF);    // pale daytime glass
    }
    if (fog_a) win = blend565(win, g_horizon565, fog_a);
    uint16_t frame = cityGrade(0x6B4D);
    if (fog_a) frame = blend565(frame, g_horizon565, fog_a);

    const float wu[2] = { 0.27f, 0.73f };
    float wv = door ? 0.56f : 0.50f;
    for (int k = 0; k < 2; k++) {
        float u = wu[k], v = wv, uw = 0.14f, vh = 0.20f;
        float px = ax*(1-u)*(1-v) + bx*u*(1-v) + cx*u*v + dx*(1-u)*v;
        float py = ay*(1-u)*(1-v) + by*u*(1-v) + cy*u*v + dy*(1-u)*v;
        float nx = ax*(1-(u+uw))*(1-v) + bx*(u+uw)*(1-v) + cx*(u+uw)*v + dx*(1-(u+uw))*v;
        float ny = ay*(1-u)*(1-(v+vh)) + by*u*(1-(v+vh)) + cy*u*(v+vh) + dy*(1-u)*(v+vh);
        int ww = (int)fabsf(nx - px); int hh = (int)fabsf(ny - py);
        if (ww < 2) ww = 2; if (ww > 12) ww = 12;
        if (hh < 2) hh = 2; if (hh > 12) hh = 12;
        sprite.fillRect((int)px - ww/2 - 1, (int)py - hh/2 - 1, ww + 2, hh + 2, frame);
        sprite.fillRect((int)px - ww/2, (int)py - hh/2, ww, hh, win);
    }
    if (door) {
        float u = 0.5f, uw = 0.11f, vh = 0.46f;
        float px = ax*(1-u) + bx*u;     // base centre (v = 0)
        float tx = ax*(1-u)*(1-vh) + bx*u*(1-vh) + cx*u*vh + dx*(1-u)*vh;
        float ty = ay*(1-u)*(1-vh) + by*u*(1-vh) + cy*u*vh + dy*(1-u)*vh;
        float py = ay*(1-u) + by*u;
        float wx2 = ax*(1-(u+uw)) + bx*(u+uw);
        int dw = (int)fabsf(wx2 - px); if (dw < 2) dw = 2; if (dw > 12) dw = 12;
        int dh = (int)fabsf(py - ty); if (dh < 3) dh = 3;
        uint16_t door_c = cityGrade(0x5326);
        if (fog_a) door_c = blend565(door_c, g_horizon565, fog_a);
        sprite.fillRect((int)px - dw/2 - 1, (int)ty, dw + 2, dh, frame);
        sprite.fillRect((int)px - dw/2, (int)ty + 1, dw, dh - 1, door_c);
    }
}

void drawCityBox(float cx, float cz, float w, float d, float h, uint16_t color, uint8_t fog_a, uint16_t seed, bool house) {
    float d2 = (cx - cam_x) * (cx - cam_x) + (cz - cam_z) * (cz - cam_z);
    if (d2 > 30000.0f) return;

    float x0 = cx - w * 0.5f, x1 = cx + w * 0.5f;
    float z0 = cz - d * 0.5f, z1 = cz + d * 0.5f;
    float p[8][3] = {
        {x0, 0.0f, z0}, {x1, 0.0f, z0}, {x1, 0.0f, z1}, {x0, 0.0f, z1},
        {x0, h,    z0}, {x1, h,    z0}, {x1, h,    z1}, {x0, h,    z1}
    };
    float sx[8], sy[8], sz[8];
    for (int i = 0; i < 8; i++) projectPoint(p[i][0], p[i][1], p[i][2], sx[i], sy[i], sz[i]);

    uint16_t side_a = cityGrade(shade565(color, 24));
    uint16_t side_b = cityGrade(shade565(color, 18));
    uint16_t roof = cityGrade(shade565(color, 30));
    if (fog_a) {
        side_a = blend565(side_a, g_horizon565, fog_a);
        side_b = blend565(side_b, g_horizon565, fog_a);
        roof = blend565(roof, g_horizon565, fog_a);
    }
    // Window grid runs at ONE fixed resolution at every distance it's drawn, so
    // the grid never changes density as you approach. The old 3-tier distance
    // LOD (3x2 -> 4x3 -> 5x4) made windows visibly "multiply" as a tower came
    // closer, which read as the building swapping to a more detailed model.
    // Windows still cut out entirely past ~96 m (sub-pixel and hazed by then),
    // which is the only transition left and is unnoticeable at that range.
    // Houses ignore wrows/wcols (drawHouseFacade draws a fixed 2 windows + door),
    // so for them this just gates whether the facade draws at all.
    int wrows = 0, wcols = 0;
    if (d2 < 9200.0f) { wrows = 5; wcols = 4; }  // <96 m, constant grid (no LOD pop)
    bool windows = (wrows > 0);

    // Roof is a top face. With the chase cam down at ~1.5 m it is back-facing
    // for anything taller than the camera, so painting it AFTER the walls (no
    // shared depth buffer here) laid the roof band back down over the near
    // wall -- the "roof showing through the side walls". Draw it FIRST when
    // we're below it so the nearer walls paint over it; only draw it last
    // (when it is genuinely the top surface we can see) if the camera is above.
    bool roof_from_above = (cam_y > h);
    if (!roof_from_above)
        drawWorldQuadClipped(x0, h, z0, x1, h, z0, x1, h, z1, x0, h, z1, roof);

    if (cam_z < cz) {
        drawWorldQuadClipped(x0, 0.0f, z0, x1, 0.0f, z0, x1, h, z0, x0, h, z0, side_a);
        if (windows && sz[0] > 0.6f && sz[1] > 0.6f && sz[5] > 0.6f && sz[4] > 0.6f) {
            if (house) drawHouseFacade(sx[0], sy[0], sx[1], sy[1], sx[5], sy[5], sx[4], sy[4], seed, fog_a, true);
            else drawCityFacadeGrid(sx[0], sy[0], sx[1], sy[1], sx[5], sy[5], sx[4], sy[4], 0x45BF, seed, fog_a, wrows, wcols);
        }
    } else {
        drawWorldQuadClipped(x1, 0.0f, z1, x0, 0.0f, z1, x0, h, z1, x1, h, z1, side_a);
        if (windows && sz[2] > 0.6f && sz[3] > 0.6f && sz[7] > 0.6f && sz[6] > 0.6f) {
            if (house) drawHouseFacade(sx[2], sy[2], sx[3], sy[3], sx[7], sy[7], sx[6], sy[6], seed, fog_a, true);
            else drawCityFacadeGrid(sx[2], sy[2], sx[3], sy[3], sx[7], sy[7], sx[6], sy[6], 0x45BF, seed, fog_a, wrows, wcols);
        }
    }
    if (cam_x < cx) {
        drawWorldQuadClipped(x0, 0.0f, z1, x0, 0.0f, z0, x0, h, z0, x0, h, z1, side_b);
        if (windows && sz[3] > 0.6f && sz[0] > 0.6f && sz[4] > 0.6f && sz[7] > 0.6f) {
            if (house) drawHouseFacade(sx[3], sy[3], sx[0], sy[0], sx[4], sy[4], sx[7], sy[7], seed + 97, fog_a, false);
            else drawCityFacadeGrid(sx[3], sy[3], sx[0], sy[0], sx[4], sy[4], sx[7], sy[7], 0x34DA, seed + 97, fog_a, wrows, wcols);
        }
    } else {
        drawWorldQuadClipped(x1, 0.0f, z0, x1, 0.0f, z1, x1, h, z1, x1, h, z0, side_b);
        if (windows && sz[1] > 0.6f && sz[2] > 0.6f && sz[6] > 0.6f && sz[5] > 0.6f) {
            if (house) drawHouseFacade(sx[1], sy[1], sx[2], sy[2], sx[6], sy[6], sx[5], sy[5], seed + 97, fog_a, false);
            else drawCityFacadeGrid(sx[1], sy[1], sx[2], sy[2], sx[6], sy[6], sx[5], sy[5], 0x34DA, seed + 97, fog_a, wrows, wcols);
        }
    }
    if (roof_from_above)
        drawWorldQuadClipped(x0, h, z0, x1, h, z0, x1, h, z1, x0, h, z1, roof);

    // Tall towers carry a blinking red aircraft beacon after dark.
    if (g_lights_on && h >= 11.0f) {
        float bx, by, bz;
        projectPoint(cx, h + 0.6f, cz, bx, by, bz);
        if (bz > 1.0f && bz < 170.0f && ((millis() + (uint32_t)seed * 517u) / 650u) & 1) {
            int r = (bz < 60.0f) ? 2 : 1;
            sprite.fillCircle((int)bx, (int)by, r, 0xF800);
            sprite.drawPixel((int)bx, (int)by, 0xFD20);
        }
    }
}

void drawCityHouse(float cx, float cz, float w, float d, float h, uint16_t color, uint8_t fog_a, uint16_t seed) {
    drawCityBox(cx, cz, w, d, h, color, fog_a, seed, true);

    float x0 = cx - w * 0.55f;
    float x1 = cx + w * 0.55f;
    float z0 = cz - d * 0.55f;
    float z1 = cz + d * 0.55f;
    float roof_h = 1.25f;
    uint16_t roof_a = cityGrade(shade565(0xA1E7, 26));
    uint16_t roof_b = cityGrade(shade565(0x8144, 22));
    if (fog_a) {
        roof_a = blend565(roof_a, g_horizon565, fog_a);
        roof_b = blend565(roof_b, g_horizon565, fog_a);
    }
    drawWorldQuadClipped(x0, h, z0, x1, h, z0, x1, h + roof_h, cz, x0, h + roof_h, cz, roof_a);
    drawWorldQuadClipped(x1, h, z1, x0, h, z1, x0, h + roof_h, cz, x1, h + roof_h, cz, roof_b);
}

void drawCityGround() {
    const float tile = 42.0f;
    int cx = (int)floorf(player_x / tile);
    int cz = (int)floorf(player_z / tile);
    const float half = tile * 0.71f; // reaches the tile corners
    // 4-tile radius (~168 m) is past the haze cutoff; the view-wedge reject
    // below drops the bulk of these before they cost a transform.
    for (int ix = cx - 4; ix <= cx + 4; ix++) {
        for (int iz = cz - 4; iz <= cz + 4; iz++) {
            float x0 = ix * tile;
            float z0 = iz * tile;
            // Cheap camera-space reject of the tile center before paying for
            // the full 4-corner transform + clip: behind the camera, or far
            // outside the view wedge.
            float mx = x0 + tile * 0.5f - cam_x;
            float mz = z0 + tile * 0.5f - cam_z;
            float zc = mx * cam_sin_yaw + mz * cam_cos_yaw;
            if (zc < -half) continue;
            float xc = mx * cam_cos_yaw - mz * cam_sin_yaw;
            if (fabsf(xc) - half > zc * 3.0f + 8.0f) continue;
            uint16_t color = ((ix + iz) & 1) ? 0x4C67 : 0x43E6;
            if (x0 > 55.0f) color = 0x43A5;
            drawCityRect(x0, z0, x0 + tile, z0 + tile, cityGrade(color));
        }
    }
}

// Quick visibility reject for a road strip: skip when every part of it is
// far away or the whole strip lies behind the camera.
static bool cityStripOutOfRange(float x0, float z0, float x1, float z1, float max_dist) {
    float dx = x1 - x0;
    float dz = z1 - z0;
    float len2 = dx * dx + dz * dz;
    float t = (len2 > 0.0001f) ? (((cam_x - x0) * dx + (cam_z - z0) * dz) / len2) : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float px = x0 + dx * t - cam_x;
    float pz = z0 + dz * t - cam_z;
    if (px * px + pz * pz > max_dist * max_dist) return true;
    // Both endpoints (and the closest point) behind the camera plane?
    float za = (x0 - cam_x) * cam_sin_yaw + (z0 - cam_z) * cam_cos_yaw;
    float zb = (x1 - cam_x) * cam_sin_yaw + (z1 - cam_z) * cam_cos_yaw;
    return (za < -2.0f && zb < -2.0f);
}

void drawCityRoads() {
    for (int i = 0; i < CITY_SIDEWALK_COUNT; i++) {
        const CityRect& s = city_sidewalks[i];
        drawCityRect(s.x - s.w * 0.5f, s.z - s.d * 0.5f,
                     s.x + s.w * 0.5f, s.z + s.d * 0.5f,
                     cityGrade(s.color));
    }

    for (int i = 0; i < CITY_ROAD_COUNT; i++) {
        if (cityStripOutOfRange(city_roads[i].x0, city_roads[i].z0,
                                city_roads[i].x1, city_roads[i].z1, 230.0f)) continue;
        float sidewalk_w = city_roads[i].width + ((i < 2) ? 5.0f : 4.0f);
        uint16_t concrete = (i < 2) ? 0xA514 : 0x9492;
        drawCityPlainStrip(city_roads[i].x0, city_roads[i].z0,
                           city_roads[i].x1, city_roads[i].z1,
                           sidewalk_w, cityGrade(concrete), 0.012f);
    }
    for (int i = 0; i < CITY_INTERSECTION_COUNT; i++) {
        if (!cityInViewCone(city_intersections[i].x - cam_x, city_intersections[i].z - cam_z,
                            city_intersections[i].radius + 8.0f)) continue;
        float r = city_intersections[i].radius + 3.4f;
        drawCityRect(city_intersections[i].x - r, city_intersections[i].z - r,
                     city_intersections[i].x + r, city_intersections[i].z + r,
                     cityGrade(city_intersections[i].signals ? 0xA514 : 0x9492));
    }

    for (int i = 0; i < CITY_ROAD_COUNT; i++) {
        if (cityStripOutOfRange(city_roads[i].x0, city_roads[i].z0,
                                city_roads[i].x1, city_roads[i].z1, 230.0f)) continue;
        drawCityStrip(city_roads[i].x0, city_roads[i].z0,
                      city_roads[i].x1, city_roads[i].z1,
                      city_roads[i].width, cityGrade(city_roads[i].color));
    }
    for (int i = 0; i < CITY_INTERSECTION_COUNT; i++) {
        if (!cityInViewCone(city_intersections[i].x - cam_x, city_intersections[i].z - cam_z,
                            city_intersections[i].radius + 8.0f)) continue;
        float r = city_intersections[i].radius;
        uint16_t asphalt = city_intersections[i].signals ? 0x39E7 : 0x3186;
        drawCityRect(city_intersections[i].x - r, city_intersections[i].z - r,
                     city_intersections[i].x + r, city_intersections[i].z + r,
                     cityGrade(asphalt));
        if (!city_intersections[i].signals) continue;
        float x = city_intersections[i].x;
        float z = city_intersections[i].z;
        uint16_t paint = cityGrade(0xDEDB); // off-white (between muted grey and pure white)
        drawCityPlainStrip(x - 6.2f, z - r - 2.1f, x + 6.2f, z - r - 2.1f, 0.62f, paint, 0.055f);
        drawCityPlainStrip(x - 6.2f, z + r + 2.1f, x + 6.2f, z + r + 2.1f, 0.62f, paint, 0.055f);
        drawCityPlainStrip(x - r - 2.1f, z - 6.2f, x - r - 2.1f, z + 6.2f, 0.62f, paint, 0.055f);
        drawCityPlainStrip(x + r + 2.1f, z - 6.2f, x + r + 2.1f, z + 6.2f, 0.62f, paint, 0.055f);
    }
}

void drawCityPark() {
    for (int i = 0; i < CITY_PARK_COUNT; i++) {
        const CityRect& p = city_parks[i];
        drawCityRect(p.x - p.w * 0.5f, p.z - p.d * 0.5f,
                     p.x + p.w * 0.5f, p.z + p.d * 0.5f,
                     cityGrade(p.color));
    }
}

// Footpaths tying the park's features to the street so they don't float in the
// grass. Drawn after the lawn and before the pond/scene objects: the streets
// (drawn later) cap the near ends and the water/fountain cap the far ends, so
// each path runs cleanly between road and feature.
void drawCityParkPaths() {
    // Packed-earth path + shore landing out to the north park pond (45,138).
    uint16_t dirt = cityGrade(0xA46A);   // tan, well clear of the lawn green
    drawCityPlainStrip(45.0f, 110.0f, 45.0f, 128.0f, 2.6f, dirt, 0.014f);
    drawCityPlainStrip(40.5f, 125.5f, 49.5f, 125.5f, 3.0f, dirt, 0.015f);
    // Concrete sidewalk + apron out to the north-east park fountain (95,140).
    uint16_t paved = cityGrade(0xA514);
    drawCityPlainStrip(95.0f, 110.0f, 95.0f, 137.0f, 2.6f, paved, 0.014f);
    drawCityRect(90.5f, 135.0f, 99.5f, 143.5f, paved);   // paved apron under the fountain
}

// Outward unit normal at track vertex i (averaged tangent of its two segments,
// flipped to point away from the loop centre). Used to lay the dirt ribbon and
// fences along the edges.
static void trackEdgeNormal(int i, float& nx, float& nz) {
    int p = (i - 1 + CITY_TRACK_PTS) % CITY_TRACK_PTS, n = (i + 1) % CITY_TRACK_PTS;
    float ix = track_pts[i].x - track_pts[p].x, iz = track_pts[i].z - track_pts[p].z;
    float ox = track_pts[n].x - track_pts[i].x, oz = track_pts[n].z - track_pts[i].z;
    float il = sqrtf(ix * ix + iz * iz), ol = sqrtf(ox * ox + oz * oz);
    if (il > 0.001f) { ix /= il; iz /= il; }
    if (ol > 0.001f) { ox /= ol; oz /= ol; }
    float tx = ix + ox, tz = iz + oz, tl = sqrtf(tx * tx + tz * tz);
    if (tl > 0.001f) { tx /= tl; tz /= tl; }
    nx = tz; nz = -tx;                                   // perpendicular to the tangent
    if ((track_pts[i].x - CITY_TRACK_CX) * nx + (track_pts[i].z - CITY_TRACK_CZ) * nz < 0.0f) {
        nx = -nx; nz = -nz;                              // make it point outward
    }
}

// A wood post-and-rail fence run between two edge points: two horizontal rails
// + posts every few metres. Open, so the loop reads as a track from across the
// infield (the old solid barriers just looked like a wall). Per-run distance +
// cone cull keep it cheap; posts thin out with distance.
void drawTrackFence(float x0, float z0, float x1, float z1, uint8_t fog_a) {
    float mx = (x0 + x1) * 0.5f - cam_x, mz = (z0 + z1) * 0.5f - cam_z;
    float d2 = mx * mx + mz * mz;
    if (d2 > 20000.0f || !cityInViewCone(mx, mz, 14.0f)) return;
    uint16_t rail = cityGrade(0x9B47), post = cityGrade(0x7A23);   // weathered wood
    if (fog_a) { rail = blend565(rail, g_horizon565, fog_a); post = blend565(post, g_horizon565, fog_a); }
    float ax, ay, az, bx, by, bz;
    for (int r = 0; r < 2; r++) {                                  // top + middle rail
        float ry = (r == 0) ? 1.05f : 0.58f;
        if (projectPoint(x0, ry, z0, ax, ay, az) && az > 0.5f &&
            projectPoint(x1, ry, z1, bx, by, bz) && bz > 0.5f)
            sprite.drawLine((int)ax, (int)ay, (int)bx, (int)by, rail);
    }
    float dx = x1 - x0, dz = z1 - z0, len = sqrtf(dx * dx + dz * dz);
    int np = (int)(len / ((d2 > 9000.0f) ? 7.0f : 3.5f));         // posts thin out far off
    if (np < 1) np = 1;
    for (int k = 0; k <= np; k++) {
        float t = (float)k / np, px = x0 + dx * t, pz = z0 + dz * t;
        if (projectPoint(px, 0.0f, pz, ax, ay, az) && az > 0.5f &&
            projectPoint(px, 1.2f, pz, bx, by, bz) && bz > 0.5f)
            sprite.drawLine((int)ax, (int)ay, (int)bx, (int)by, post);
    }
}

// The northern dirt rally circuit: a dirt ribbon following the centreline, with
// a wood post-and-rail fence on each edge. One distance gate => zero cost until
// you drive up north; edge points are recomputed locally (cheap, ~30 verts).
void drawCityTrack() {
    float dxc = cam_x - CITY_TRACK_CX, dzc = cam_z - CITY_TRACK_CZ;
    if (dxc * dxc + dzc * dzc > 21000.0f) return;        // only on the northern outskirts
    float inx[CITY_TRACK_PTS], inz[CITY_TRACK_PTS], outx[CITY_TRACK_PTS], outz[CITY_TRACK_PTS];
    for (int i = 0; i < CITY_TRACK_PTS; i++) {
        float nx, nz; trackEdgeNormal(i, nx, nz);
        outx[i] = track_pts[i].x + nx * CITY_TRACK_HALFW; outz[i] = track_pts[i].z + nz * CITY_TRACK_HALFW;
        inx[i]  = track_pts[i].x - nx * CITY_TRACK_HALFW; inz[i]  = track_pts[i].z - nz * CITY_TRACK_HALFW;
    }
    uint16_t dirt = cityGrade(0xA366);                   // reddish packed rally dirt
    for (int i = 0; i < CITY_TRACK_PTS; i++) {
        int j = (i + 1) % CITY_TRACK_PTS;
        float mx = (track_pts[i].x + track_pts[j].x) * 0.5f - cam_x;
        float mz = (track_pts[i].z + track_pts[j].z) * 0.5f - cam_z;
        float d2 = mx * mx + mz * mz;
        if (d2 > 20000.0f || !cityInViewCone(mx, mz, 16.0f)) continue;
        uint8_t fog = cityFogFromD2(d2);
        uint16_t dcol = fog ? blend565(dirt, g_horizon565, fog) : dirt;
        drawWorldQuadClipped(inx[i], 0.0f, inz[i], outx[i], 0.0f, outz[i],
                             outx[j], 0.0f, outz[j], inx[j], 0.0f, inz[j], dcol);
        drawTrackFence(inx[i], inz[i], inx[j], inz[j], fog);       // inner fence (full loop)
        bool entry = fabsf((track_pts[i].x + track_pts[j].x) * 0.5f) < 12.0f &&
                     (track_pts[i].z + track_pts[j].z) * 0.5f < 200.0f;
        if (!entry) drawTrackFence(outx[i], outz[i], outx[j], outz[j], fog);  // outer (gap at entry)
    }
    drawCityRect(-6.0f, 195.0f, 6.0f, 196.2f, cityGrade(0xDEDB));  // start/finish stripe
}

void drawCityNPCCar(int i, uint8_t fog_a) {
    CityNPC& npc = city_npcs[i];
    float ssx, ssy, ssz;
    projectPoint(npc.x, 0.02f, npc.z, ssx, ssy, ssz);
    if (ssz <= 1.0f || ssz > 140.0f) return;
    if (ssz < 90.0f) {
        float sox, soz;
        sunBlobOffset(0.5f, sox, soz);
        float shx, shy, shz;
        projectPoint(npc.x + sox, 0.02f, npc.z + soz, shx, shy, shz);
        int srx = (int)(0.80f * fov / ssz);
        int sry = srx / 3;
        if (sry < 2) sry = 2;
        shadowEllipse((int)shx, (int)shy, srx, sry, 8);
    }
    // Close traffic gets the full player-grade car mesh; the LOD coupe takes
    // over with distance. A hysteresis band (24 m in / 30 m out) stops cars
    // hovering at the boundary from flipping the ~1 ms mesh cost every frame
    // -- that flip-flop was a measurable per-frame stutter. A per-frame budget
    // then caps how many full meshes a single frame can pay for, so a cluster
    // of cars at a junction can't blow the frame time.
    bool want_full = npc.hi_lod ? (ssz < NPC_FULLMESH_OUT) : (ssz < NPC_FULLMESH_IN);
    if (want_full && g_fullmesh_left <= 0) want_full = false;
    npc.hi_lod = want_full;
    if (want_full) {
        g_fullmesh_left--;
        // Wheels before body (body occludes the wheel tops -- see drawCarWheels).
        drawCarWheels(npc.x, 0.08f, npc.z, npc.heading, 0.0f, 1.0f, g_wheel_spin);
        draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, car_normals, CAR_NUM_FACES,
                    npc.x, 0.08f, npc.z,
                    0.0f, npc.heading, 0.0f,
                    1.0f, npc.color, fog_a);
    } else {
        draw3DModel(lod_car_vertices, LOD_CAR_NUM_VERTICES, lod_car_faces, lod_car_normals, LOD_CAR_NUM_FACES,
                    npc.x, 0.08f, npc.z,
                    0.0f, npc.heading, 0.0f,
                    1.15f, npc.color, fog_a);
    }
    bool braking = (npc.cur_speed < npc.speed_ms * 0.85f);
    drawCityVehicleLights(npc.x, npc.z, npc.heading,
                          npc.cur_speed / npc.speed_ms, braking);
}

void drawTrafficLight(float x, float z, uint8_t state) {
    float d2 = (x - cam_x) * (x - cam_x) + (z - cam_z) * (z - cam_z);
    if (d2 > 18000.0f) return;
    if (cityPointOccludedByBuilding(x, z, 3.45f)) return;

    float bx, by, bz, tx, ty, tz;
    projectPoint(x, 0.0f, z, bx, by, bz);
    projectPoint(x, 3.45f, z, tx, ty, tz);
    if (bz < 0.6f || tz < 0.6f || bz > 120.0f) return;
    sprite.drawLine((int16_t)bx, (int16_t)by, (int16_t)tx, (int16_t)ty, cityGrade(0x8410));

    float hx, hy, hz;
    projectPoint(x, 3.05f, z, hx, hy, hz);
    if (hz < 0.6f || hz > 95.0f) return;
    int s = (int)(42.0f / hz);
    if (s < 2) s = 2;
    if (s > 7) s = 7;

    sprite.fillRect((int)hx - s, (int)hy - s * 2, s * 2 + 1, s * 4 + 1, 0x0000);

    // Glow around the active lens after dark (over the housing, under the lens).
    if (g_lights_on) {
        static const uint16_t halo_col[3] = { 0x0560, 0xB400, 0xB000 };
        int ly = (state == 2) ? -s : ((state == 1) ? 0 : s);
        sprite.fillCircle((int)hx, (int)hy + ly, s / 2 + 2, halo_col[state]);
    }
    uint16_t red = (state == 2) ? 0xF800 : 0x4000;
    uint16_t amber = (state == 1) ? 0xFD60 : 0x5180;
    uint16_t green = (state == 0) ? 0x07E0 : 0x0180;
    sprite.fillCircle((int)hx, (int)hy - s, s / 2 + 1, red);
    sprite.fillCircle((int)hx, (int)hy, s / 2 + 1, amber);
    sprite.fillCircle((int)hx, (int)hy + s, s / 2 + 1, green);
}

void drawStreetLamp(float x, float z) {
    float d2 = (x - cam_x) * (x - cam_x) + (z - cam_z) * (z - cam_z);
    if (d2 > 16000.0f) return;
    float bx, by, bz, tx, ty, tz;
    projectPoint(x, 0.0f, z, bx, by, bz);
    projectPoint(x, 4.2f, z, tx, ty, tz);
    if (bz < 0.6f || tz < 0.6f || bz > 110.0f) return;

    // Cone of light on the pavement below, only while the lamps are lit and
    // near enough to matter.
    if (g_lights_on && bz < 60.0f) {
        int prx = (int)(2.4f * fov / bz);
        if (prx > 2) {
            if (prx > 70) prx = 70;
            int pry = prx / 3 + 1;
            if (pry > 20) pry = 20;
            brightenEllipse((int)bx, (int)by, prx, pry, 0xFEED, 4);
        }
    }

    sprite.drawLine((int16_t)bx, (int16_t)by, (int16_t)tx, (int16_t)ty, cityGrade(0x5AEB));
    int r = (int)(30.0f / tz);
    if (r < 1) r = 1;
    if (r > 4) r = 4;
    if (g_lights_on) {
        sprite.fillCircle((int)tx, (int)ty, r + 1, 0xC600);
        sprite.fillCircle((int)tx, (int)ty, r, 0xFFE0);
    } else {
        sprite.fillCircle((int)tx, (int)ty, r, cityGrade(0xC618));
    }
}

void drawParkedCar(float x, float z, float heading, uint16_t color, uint8_t fog_a) {
    float d2 = (x - cam_x) * (x - cam_x) + (z - cam_z) * (z - cam_z);
    if (d2 > 18000.0f) return;
    float ssx, ssy, ssz;
    projectPoint(x, 0.02f, z, ssx, ssy, ssz);
    if (ssz < 1.0f || ssz > 95.0f) return;
    float sox, soz;
    sunBlobOffset(0.5f, sox, soz);
    float shx, shy, shz;
    projectPoint(x + sox, 0.02f, z + soz, shx, shy, shz);
    int srx = (int)(0.74f * fov / ssz);
    int sry = srx / 3;
    if (sry < 2) sry = 2;
    shadowEllipse((int)shx, (int)shy, srx, sry, 8);
    draw3DModel(lod_car_vertices, LOD_CAR_NUM_VERTICES, lod_car_faces, lod_car_normals, LOD_CAR_NUM_FACES,
                x, 0.08f, z,
                0.0f, heading, 0.0f,
                1.08f, color, fog_a);
}

// ---- New city assets -------------------------------------------------------

// Water body: a sunken disc of animated water ringed by a sandy shore.
// Built from world-space ring sectors fed through drawWorldQuadClipped so the
// huge harbor bay clips correctly when it straddles the near plane.
void drawCityPond(int i, uint8_t fog_a) {
    const CityPond& p = city_ponds[i];
    // Sector count scales with apparent size: the big harbor bay needs detail
    // up close but only a coarse ring when it's a sliver near the horizon.
    float dxp = p.x - cam_x, dzp = p.z - cam_z;
    float pdist = sqrtf(dxp * dxp + dzp * dzp);
    int N = 14;
    if (pdist > 90.0f) N = 10;
    if (pdist > 150.0f) N = 8;
    float phase = millis() * 0.0011f;

    // Water reflects the sky, so it tracks the horizon haze and night grading.
    uint16_t deep = blend565(0x0A3F, g_horizon565, 6);
    uint16_t shallow = blend565(0x4DBF, g_horizon565, 4);
    deep = cityGrade(deep);
    shallow = cityGrade(shallow);
    uint16_t shore = cityGrade(0xB4ED);
    if (fog_a) {
        deep = blend565(deep, g_horizon565, fog_a);
        shallow = blend565(shallow, g_horizon565, fog_a);
        shore = blend565(shore, g_horizon565, fog_a);
    }

    float prevx = p.x + p.rx, prevz = p.z;
    float pswx = p.x + (p.rx + 1.8f), pswz = p.z;
    for (int k = 1; k <= N; k++) {
        float a = k * (2.0f * PI / N);
        float ca = cosf(a), sa = sinf(a);
        float wx = p.x + ca * p.rx;
        float wz = p.z + sa * p.rz;
        float swx = p.x + ca * (p.rx + 1.8f);
        float swz = p.z + sa * (p.rz + 1.8f);

        // Shore band.
        drawWorldQuadClipped(prevx, -0.02f, prevz, wx, -0.02f, wz,
                             swx, -0.02f, swz, pswx, -0.02f, pswz, shore);
        // Water sector (sunken; banded shimmer animates around the ring).
        int band = ((int)(k + phase * 6.0f)) & 1;
        uint16_t wcol = band ? deep : shallow;
        drawWorldQuadClipped(p.x, -0.10f, p.z, prevx, -0.06f, prevz,
                             wx, -0.06f, wz, p.x, -0.10f, p.z, wcol);
        prevx = wx; prevz = wz;
        pswx = swx; pswz = swz;
    }

    // Sun/moon glint: a bright streak on the water nearest the camera.
    if (g_sun_el > 0.04f || g_night > 0.3f) {
        float gx, gy, gz;
        if (projectPoint(p.x, -0.05f, p.z, gx, gy, gz) && gz > 2.0f && gz < 70.0f) {
            uint16_t glint = (g_night > 0.3f) ? 0xCE9C : 0xFFEE;
            int gw = (int)(p.rx * 0.5f * fov / gz);
            if (gw > 60) gw = 60;
            if (gw > 3) {
                for (int s = -gw / 2; s <= gw / 2; s += 3) {
                    int yy = (int)gy + ((s & 2) ? 1 : 0);
                    sprite.drawPixel((int)gx + s, yy, glint);
                }
            }
        }
    }
}

void drawCityPonds() {
    for (int i = 0; i < CITY_POND_COUNT; i++) {
        float dx = city_ponds[i].x - cam_x;
        float dz = city_ponds[i].z - cam_z;
        float reach = (city_ponds[i].rx > city_ponds[i].rz ? city_ponds[i].rx : city_ponds[i].rz);
        float d2 = dx * dx + dz * dz;
        // Distance reject to the nearest edge.
        if (d2 > (175.0f + reach) * (175.0f + reach)) continue;
        // Facing reject: if the whole water body is well behind the camera,
        // skip it before tessellating any sectors.
        float zc = dx * cam_sin_yaw + dz * cam_cos_yaw;
        if (zc < -(reach + 6.0f)) continue;
        drawCityPond(i, cityFogFromD2(d2));
    }
}

// Striped construction barrier: an A-frame board with two legs and, after
// dark, a blinking amber hazard lamp.
void drawConstructionBarrier(float x, float z, float heading, uint8_t fog_a) {
    float d2 = (x - cam_x) * (x - cam_x) + (z - cam_z) * (z - cam_z);
    if (d2 > 16000.0f) return;
    float c = cosf(heading), s = sinf(heading);
    float hw = 1.4f;          // half width of the board
    float y0 = 0.45f, y1 = 1.05f;

    // Board ends in world space.
    float lx = x - c * hw, lz = z + s * hw;
    float rx = x + c * hw, rz = z - s * hw;

    uint16_t orange = cityGrade(0xFC60);
    uint16_t white = cityGrade(0xFFFF);
    if (fog_a) {
        orange = blend565(orange, g_horizon565, fog_a);
        white = blend565(white, g_horizon565, fog_a);
    }
    // Six diagonal stripes across the board.
    const int ST = 6;
    for (int k = 0; k < ST; k++) {
        float t0 = (float)k / ST, t1 = (float)(k + 1) / ST;
        float ax = lx + (rx - lx) * t0, az = lz + (rz - lz) * t0;
        float bx = lx + (rx - lx) * t1, bz = lz + (rz - lz) * t1;
        drawWorldQuadClipped(ax, y0, az, bx, y0, bz, bx, y1, bz, ax, y1, az,
                             (k & 1) ? orange : white);
    }
    // Legs.
    uint16_t leg = cityGrade(0x52AA);
    float lsx, lsy, lsz, lex, ley, lez;
    projectPoint(lx, 0.0f, lz, lsx, lsy, lsz);
    projectPoint(lx, y0, lz, lex, ley, lez);
    if (lsz > 0.5f && lez > 0.5f) sprite.drawLine((int16_t)lsx, (int16_t)lsy, (int16_t)lex, (int16_t)ley, leg);
    projectPoint(rx, 0.0f, rz, lsx, lsy, lsz);
    projectPoint(rx, y0, rz, lex, ley, lez);
    if (lsz > 0.5f && lez > 0.5f) sprite.drawLine((int16_t)lsx, (int16_t)lsy, (int16_t)lex, (int16_t)ley, leg);

    if (g_lights_on && ((millis() / 500u) & 1)) {
        float hx, hy, hz;
        projectPoint(x, y1 + 0.15f, z, hx, hy, hz);
        if (hz > 0.6f && hz < 60.0f) sprite.fillCircle((int)hx, (int)hy, 2, 0xFD20);
    }
}

// Fountain: a stone basin, central pillar, and animated water jets. Sits on
// the plaza tiles; the spray is screen-space droplets arcing off the apex.
void drawFountain(float x, float z, uint8_t fog_a) {
    float d2 = (x - cam_x) * (x - cam_x) + (z - cam_z) * (z - cam_z);
    if (d2 > 22000.0f) return;
    const int N = 14;
    uint16_t stone = cityGrade(0x9CD3);
    uint16_t water = cityGrade(blend565(0x4DBF, g_horizon565, 4));
    if (fog_a) {
        stone = blend565(stone, g_horizon565, fog_a);
        water = blend565(water, g_horizon565, fog_a);
    }
    float rb = 2.6f;   // basin radius
    float prevx = x + rb, prevz = z;
    float pix = x + rb * 0.74f, piz = z;
    for (int k = 1; k <= N; k++) {
        float a = k * (2.0f * PI / N);
        float wx = x + cosf(a) * rb, wz = z + sinf(a) * rb;
        float ix = x + cosf(a) * rb * 0.74f, iz = z + sinf(a) * rb * 0.74f;
        // Stone rim.
        drawWorldQuadClipped(pix, 0.30f, piz, prevx, 0.30f, prevz,
                             wx, 0.30f, wz, ix, 0.30f, iz, stone);
        // Inner water.
        drawWorldQuadClipped(x, 0.18f, z, pix, 0.22f, piz, ix, 0.22f, iz, x, 0.18f, z, water);
        prevx = wx; prevz = wz; pix = ix; piz = iz;
    }
    // Central pillar.
    drawWorldQuadClipped(x - 0.25f, 0.30f, z, x + 0.25f, 0.30f, z,
                         x + 0.25f, 1.4f, z, x - 0.25f, 1.4f, z, stone);

    // Water jets: droplets on parabolic arcs from the apex.
    float apx, apy, apz;
    if (projectPoint(x, 1.7f, z, apx, apy, apz) && apz > 1.5f && apz < 55.0f) {
        float t = millis() * 0.004f;
        int drops = (apz < 18.0f) ? 14 : 7;
        float scale = fov / apz;
        for (int k = 0; k < drops; k++) {
            float a = k * (2.0f * PI / drops);
            float ph = t + k * 0.7f;
            float fall = fmodf(ph, 1.0f);            // 0..1 along the arc
            float reach = 0.9f * scale * fall;
            float rise = (1.6f * fall - fall * fall * 2.4f); // up then down
            int px = (int)(apx + cosf(a) * reach);
            int py = (int)(apy - rise * scale * 0.9f);
            uint16_t c = (k & 1) ? 0xCEFF : 0xFFFF;
            sprite.drawPixel(px, py, c);
            if (apz < 14.0f) sprite.drawPixel(px, py + 1, c);
        }
    }
}

// Roman posing-man marble statue on a plinth (corner of the downtown plaza).
// Low-poly box figure; `heading` turns it to overlook the plaza centre.
void drawStatue(float x, float z, float heading, uint8_t fog_a) {
    draw3DModel(statue_vertices, STATUE_NUM_VERTICES, statue_faces, statue_normals,
                STATUE_NUM_FACES, x, 0.0f, z, 0.0f, heading, 0.0f, 1.0f, 0xE71C, fog_a);
}

// One run of the plaza balustrade. The model is built at the real run length
// (12 m), so it draws at scale 1; `heading` aligns it along the plaza edge.
void drawCityRailing(const CityRail& r, uint8_t fog_a) {
    draw3DModel(railing_vertices, RAILING_NUM_VERTICES, railing_faces, railing_normals,
                RAILING_NUM_FACES, r.x, 0.0f, r.z, 0.0f, r.heading, 0.0f, 1.0f, 0x9CD3, fog_a);
}

// Layered conifer: stacked dark-green skirts on a short trunk. A denser,
// pointier counterpart to the round leafy impostor.
void drawPineTree(float x, float z, float scale, uint8_t fog_a) {
    float sx, sy, sz;
    projectPoint(x, 0.0f, z, sx, sy, sz);
    if (sz <= 2.0f || sz > 60.0f) return;
    int h = (int)(520.0f * scale / sz);
    if (h < 5) return;
    if (h > 100) h = 100;
    int px = (int)sx, py = (int)sy;
    if (px < -h || px > SCREEN_WIDTH + h || py < -h || py > SCREEN_HEIGHT + h) return;

    uint16_t trunk = cityGrade(0x4204);
    uint16_t dark = cityGrade(0x0240);
    uint16_t mid = cityGrade(0x0360);
    uint16_t lite = cityGrade(0x2480);
    if (fog_a) {
        trunk = blend565(trunk, g_horizon565, fog_a);
        dark = blend565(dark, g_horizon565, fog_a);
        mid = blend565(mid, g_horizon565, fog_a);
        lite = blend565(lite, g_horizon565, fog_a);
    }
    int tw = h / 9; if (tw < 1) tw = 1;
    sprite.fillRect(px - tw / 2, py - h / 5, tw, h / 5, trunk);

    // Three skirts from the ground up, each narrower; light from the sun side.
    int base = py - h / 6;
    uint16_t tones[3] = { dark, mid, lite };
    for (int tier = 0; tier < 3; tier++) {
        int ty = base - (h * tier) / 3;
        int tipy = ty - h / 2;
        int half = (h * (3 - tier)) / 9;
        sprite.fillTriangle(px - half, ty, px + half, ty, px, tipy, tones[tier]);
        // Sunlit right edge highlight.
        if (h > 16) sprite.fillTriangle(px, ty, px + half, ty, px, tipy, blend565(tones[tier], lite, 8));
    }
}

// Grand avenue gateway arch: reuse the bridge mesh, spanning the 12 m avenue.
void drawAvenueArch(float x, float z, float heading, uint8_t fog_a) {
    float d2 = (x - cam_x) * (x - cam_x) + (z - cam_z) * (z - cam_z);
    if (d2 > 42000.0f) return;
    draw3DModel(bridge_vertices, BRIDGE_NUM_VERTICES, bridge_faces, bridge_normals, BRIDGE_NUM_FACES,
                x, 0.0f, z,
                0.0f, heading, 0.0f,
                1.28f, 0x8430, fog_a);
}

// One depth-sorted painter's pass over everything that stands on the ground:
// buildings, props, traffic, pedestrians. Sorting them together is what stops
// cars and lamps from drawing through the buildings in front of them.
void drawCitySceneObjects() {
    struct SceneItem {
        float d2;
        uint8_t type;  // 0 building, 1 prop, 2 NPC car, 3 pedestrian
        uint8_t index;
    };
    static SceneItem items[CITY_LAYOUT_BUILDING_COUNT + CITY_PROP_COUNT + CITY_NPC_COUNT +
                           CITY_PED_COUNT + CITY_PLAZA_RAIL_COUNT + CITY_STATUE_COUNT];
    int count = 0;

    for (int i = 0; i < CITY_LAYOUT_BUILDING_COUNT; i++) {
        float dx = city_lots[i].x - cam_x;
        float dz = city_lots[i].z - cam_z;
        float d2 = dx * dx + dz * dz;
        if (d2 <= 30000.0f && cityInViewCone(dx, dz, 18.0f)) items[count++] = SceneItem{ d2, 0, (uint8_t)i };
    }
    for (int i = 0; i < CITY_PROP_COUNT; i++) {
        float dx = city_props[i].x - cam_x;
        float dz = city_props[i].z - cam_z;
        float d2 = dx * dx + dz * dz;
        if (d2 <= 19000.0f && cityInViewCone(dx, dz, 8.0f)) items[count++] = SceneItem{ d2, 1, (uint8_t)i };
    }
    for (int i = 0; i < CITY_NPC_COUNT; i++) {
        float dx = city_npcs[i].x - cam_x;
        float dz = city_npcs[i].z - cam_z;
        float d2 = dx * dx + dz * dz;
        if (d2 <= 20000.0f && cityInViewCone(dx, dz, 8.0f)) items[count++] = SceneItem{ d2, 2, (uint8_t)i };
    }
    for (int i = 0; i < CITY_PED_COUNT; i++) {
        float dx = city_peds[i].x - cam_x;
        float dz = city_peds[i].z - cam_z;
        float d2 = dx * dx + dz * dz;
        if (d2 <= 6400.0f && cityInViewCone(dx, dz, 5.0f)) items[count++] = SceneItem{ d2, 3, (uint8_t)i };
    }
    for (int i = 0; i < CITY_PLAZA_RAIL_COUNT; i++) {
        float dx = city_plaza_rails[i].x - cam_x;
        float dz = city_plaza_rails[i].z - cam_z;
        float d2 = dx * dx + dz * dz;
        if (d2 <= 16000.0f && cityInViewCone(dx, dz, 14.0f)) items[count++] = SceneItem{ d2, 4, (uint8_t)i };
    }
    for (int i = 0; i < CITY_STATUE_COUNT; i++) {
        float dx = city_statues[i].x - cam_x;
        float dz = city_statues[i].z - cam_z;
        float d2 = dx * dx + dz * dz;
        if (d2 <= 16000.0f && cityInViewCone(dx, dz, 6.0f)) items[count++] = SceneItem{ d2, 5, (uint8_t)i };
    }

    // Shell sort, far to near.
    for (int gap = count / 2; gap > 0; gap /= 2) {
        for (int i = gap; i < count; i++) {
            SceneItem tmp = items[i];
            int j = i;
            while (j >= gap && items[j - gap].d2 < tmp.d2) {
                items[j] = items[j - gap];
                j -= gap;
            }
            items[j] = tmp;
        }
    }

    g_fullmesh_left = NPC_FULLMESH_MAX; // full-mesh budget per frame (see NPC_FULLMESH_*)
    static const uint16_t car_col[] = { 0xF800, 0x07FF, 0xFFE0, 0xFFFF, 0xFD20, 0x9CD3, 0xF81F, 0x07E0 };
    for (int n = 0; n < count; n++) {
        const SceneItem& it = items[n];
        uint8_t fog_a = cityFogFromD2(it.d2);
        if (it.type == 0) {
            const CityLot& lot = city_lots[it.index];
            uint16_t seed = (uint16_t)(it.index * 7 + 3);
            if (lot.type == 2) {
                drawCityHouse(lot.x, lot.z, lot.w, lot.d, lot.h, lot.color, fog_a, seed);
            } else {
                drawCityBox(lot.x, lot.z, lot.w, lot.d, lot.h, lot.color, fog_a, seed);
            }
        } else if (it.type == 1) {
            const CityProp& p = city_props[it.index];
            if (p.type == 0) {
                drawTrafficLight(p.x, p.z, (uint8_t)cityTrafficPhase((it.index & 1) == 0));
            } else if (p.type == 1) {
                drawStreetLamp(p.x, p.z);
            } else if (p.type == 2) {
                drawTreeImpostor(p.x, 0.0f, p.z, 1.24f + 0.10f * (it.index & 3), fog_a);
            } else if (p.type == 3) {
                uint16_t color = (p.color == 0xFFFF) ? car_col[it.index & 7] : p.color;
                drawParkedCar(p.x, p.z, p.heading, color, fog_a);
            } else if (p.type == 5) {
                drawConstructionBarrier(p.x, p.z, p.heading, fog_a);
            } else if (p.type == 6) {
                drawFountain(p.x, p.z, fog_a);
            } else if (p.type == 7) {
                drawPineTree(p.x, p.z, 2.0f + 0.24f * (it.index & 3), fog_a);
            } else if (p.type == 8) {
                drawAvenueArch(p.x, p.z, p.heading, fog_a);
            } else {
                uint16_t color = (p.color == 0xFFFF) ? 0x001F : p.color;
                drawBillboard(p.x, 0.25f, p.z, p.heading, 0.28f, color, fog_a);
            }
        } else if (it.type == 2) {
            drawCityNPCCar(it.index, fog_a);
        } else if (it.type == 3) {
            drawCityPed(it.index, fog_a);
        } else if (it.type == 4) {
            drawCityRailing(city_plaza_rails[it.index], fog_a);
        } else {
            const CityStatue& s = city_statues[it.index];
            drawStatue(s.x, s.z, s.heading, fog_a);
        }
    }
}

// Courier waypoint: a tall searchlight beam shooting far up into the sky, so
// you spot it OVER the skyline rather than seeing a marker bleed through the
// buildings. Gold for a pickup, green for a drop-off. The beam draws ONLY from
// the height where its sight line clears every intervening roofline up to the
// sky (cityBeamVisibleAboveY) -- so it never shows through a building, yet a far
// beacon still reads as the upper length of beam standing over the skyline.
void drawCourierBeacon() {
    if (current_state != PLAYING) return;
    uint16_t col = courier_has_fare ? 0x07E0 : 0xFE60;   // green drop / gold pickup
    float pulse = 0.6f + 0.4f * sinf(millis() * 0.006f);

    // Height above which the beam has a clear line of sight to the sky. 0 when
    // nothing blocks the base; above the rooflines when buildings stand between
    // the camera and the beacon.
    const float BEAM_TOP = 60.0f;
    float y_lo = cityBeamVisibleAboveY(courier_tx, courier_tz);
    bool base_visible = (y_lo < 0.5f);

    // Ground corona only when the base itself is in the open.
    if (base_visible) {
        const int N = 10;
        float ring_r = 2.6f;
        int prevx = 0, prevy = 0; bool prevok = false;
        for (int k = 0; k <= N; k++) {
            float a = k * (2.0f * PI / N);
            float rx, ry, rz;
            bool ok = projectPoint(courier_tx + cosf(a) * ring_r, 0.05f,
                                   courier_tz + sinf(a) * ring_r, rx, ry, rz) && rz > 0.5f;
            if (ok && prevok && (k & 1))
                sprite.drawLine(prevx, prevy, (int)rx, (int)ry, col);
            prevx = (int)rx; prevy = (int)ry; prevok = ok;
        }
    }

    // A near or tall enough building hides the beam all the way to its top: no
    // line of sight to the sky above this beacon, so draw nothing.
    if (y_lo >= BEAM_TOP - 1.0f) return;

    // Tall thin beam, from the roofline-clearance height up to the sky. Built
    // bottom-up from world points so it tracks perspective.
    const int steps = 12;
    int px = 0, py = 0; bool pok = false;
    int core_x0 = -1, core_y0 = 0, core_x1 = 0, core_y1 = 0; bool core = false;
    for (int s = 0; s <= steps; s++) {
        float f = (float)s / steps;
        float wy = y_lo + (BEAM_TOP - y_lo) * f;
        float sx, sy, sz;
        bool ok = projectPoint(courier_tx, wy, courier_tz, sx, sy, sz) && sz > 0.5f && sz < 260.0f;
        if (ok && pok) {
            int w = 3 - (int)(f * 2.0f);            // tapers from ~3 px to 1 px
            if (w < 1) w = 1;
            uint8_t a = (uint8_t)((9.0f - f * 6.0f) * pulse);  // brighter low, fades up
            rmwTriangle(px - w, py, px + w, py, (int)sx + w, (int)sy, col, a, false);
            rmwTriangle(px - w, py, (int)sx + w, (int)sy, (int)sx - w, (int)sy, col, a, false);
            if (!core) { core_x0 = px; core_y0 = py; core = true; }
            core_x1 = (int)sx; core_y1 = (int)sy;
        }
        px = (int)sx; py = (int)sy; pok = ok;
    }
    if (core) sprite.drawLine(core_x0, core_y0, core_x1, core_y1, col);
}

void renderCityWorld() {
    drawCityGround();
    drawCityPark();
    drawCityParkPaths();
    drawCityPonds();
    drawCityRoads();
    drawCityTrack();
    drawCityBuildingShadows();
    drawCitySceneObjects();
    drawCourierBeacon();
}

void drawCityPlayerCar() {
    if (current_state != PLAYING && current_state != FINISHED) return;
    float bounce_y = 0.0f;
    if (player_speed > 0.0f && current_state == PLAYING) {
        bounce_y = 0.012f * sinf(millis() * 0.050f * (player_speed / MAX_SPEED + 0.3f));
    }
    float car_y = player_y + 0.08f + bounce_y;
    float behind = sinf(player_heading) * (player_x - cam_x) +
                   cosf(player_heading) * (player_z - cam_z);

    // Night driving: a real projected headlight cone (wide spill + hot core,
    // steering slightly with the front wheels). Drawn BEFORE the car body so
    // the car occludes the near part -- otherwise the additive beam brightens
    // the car's own pixels and the light reads as shining through it. The
    // pool starts just ahead of the bumper and throws well down the road.
    if (g_lights_on) {
        float beam_yaw = player_heading + player_steer_angle * 0.5f;
        drawHeadlightBeam(player_x, player_z, beam_yaw, 2.4f, 0.95f, 24.0f, 4.2f, 4);
        drawHeadlightBeam(player_x, player_z, beam_yaw, 2.4f, 0.70f, 14.0f, 2.4f, 4);
    }

    // Wheels before body (body occludes the wheel tops -- see drawCarWheels).
    drawCarWheels(player_x, car_y, player_z, player_heading + player_steer_angle,
                  player_roll, 1.0f, g_wheel_spin);
    draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, car_normals, CAR_NUM_FACES,
                player_x, car_y, player_z,
                player_pitch, player_heading + player_steer_angle, player_roll,
                1.0f, 0x021F, 0);
    if (behind > 0.0f) {
        drawCarRearGlass(player_x, car_y, player_z,
                         player_pitch, player_heading + player_steer_angle, player_roll,
                         1.0f, player_braking && current_state == PLAYING);
    }

    // (Tail lights -- dim always, bright when braking -- are drawn by
    // drawCarRearGlass above.)
    updateAndDrawParticles();
}

// 3D Object Renderer.
// - One fused matrix A = Camera * ModelRotation * scale per call, so each
//   vertex is a single 3x3 multiply-add instead of two chained rotations.
// - Exact backface culling using the precomputed model-space face normals:
//   a face is visible iff the eye is on its front side (dot(n, eye - v0) > 0).
//   Double-sided faces (open sheets like the billboard) are never culled.
// - Lighting is one dot product per face against the model-space light, and
//   shading/fog use integer RGB565 math. Culled/behind faces never get sorted.
void draw3DModel(const Point3D* vertices, int num_vertices,
                 const Face* faces, const Point3D* normals, int num_faces,
                 float pos_x, float pos_y, float pos_z,
                 float rot_x, float rot_y, float rot_z,
                 float scale, uint16_t base_color, uint8_t fog_a) {
    static Point2D projected[MODEL_MAX_VERTICES];
    static float camera_z[MODEL_MAX_VERTICES];

    struct FaceRenderData {
        float avg_z;
        uint16_t index;
        uint16_t color;
    };
    static FaceRenderData face_data[MODEL_MAX_FACES];

    if (num_vertices > MODEL_MAX_VERTICES || num_faces > MODEL_MAX_FACES) return;

    float cx = 1.0f, sx = 0.0f, cy = 1.0f, sy = 0.0f, cz = 1.0f, sz = 0.0f;
    if (rot_x != 0.0f) { cx = cosf(rot_x); sx = sinf(rot_x); }
    if (rot_y != 0.0f) { cy = cosf(rot_y); sy = sinf(rot_y); }
    if (rot_z != 0.0f) { cz = cosf(rot_z); sz = sinf(rot_z); }

    // Model rotation matrix M = Ry * Rx * Rz (roll, then pitch, then yaw).
    float m00 = cy * cz + sy * sx * sz, m01 = -cy * sz + sy * sx * cz, m02 = sy * cx;
    float m10 = cx * sz,                m11 = cx * cz,                 m12 = -sx;
    float m20 = -sy * cz + cy * sx * sz, m21 = sy * sz + cy * sx * cz, m22 = cy * cx;

    // Camera matrix C (yaw then pitch, same convention as projectPoint).
    float cyc = cam_cos_yaw, syc = cam_sin_yaw, cp = cam_cos_pitch, sp = cam_sin_pitch;
    float c00 = cyc,       c02 = -syc;
    float c10 = -syc * sp, c11 = cp, c12 = -cyc * sp;
    float c20 = syc * cp,  c21 = sp, c22 = cyc * cp;

    // Fused vertex transform: cam = A * v + b, with A = C * M * scale.
    float a00 = (c00 * m00 + c02 * m20) * scale;
    float a01 = (c00 * m01 + c02 * m21) * scale;
    float a02 = (c00 * m02 + c02 * m22) * scale;
    float a10 = (c10 * m00 + c11 * m10 + c12 * m20) * scale;
    float a11 = (c10 * m01 + c11 * m11 + c12 * m21) * scale;
    float a12 = (c10 * m02 + c11 * m12 + c12 * m22) * scale;
    float a20 = (c20 * m00 + c21 * m10 + c22 * m20) * scale;
    float a21 = (c20 * m01 + c21 * m11 + c22 * m21) * scale;
    float a22 = (c20 * m02 + c21 * m12 + c22 * m22) * scale;

    float dx = pos_x - cam_x, dy = pos_y - cam_y, dz = pos_z - cam_z;
    float b0 = c00 * dx + c02 * dz;
    float b1 = c10 * dx + c11 * dy + c12 * dz;
    float b2 = c20 * dx + c21 * dy + c22 * dz;

    // Whole model at/behind the camera plane: every on-screen part would be
    // clipped face-by-face anyway, and anything this close but off-axis
    // projects far off-screen. Skip the entire transform.
    if (b2 < 0.4f) return;

    // Eye position and light direction in model space (M is orthonormal, so
    // its inverse is the transpose; uniform scale only affects the eye).
    float inv_s = 1.0f / scale;
    float ex = (m00 * -dx + m10 * -dy + m20 * -dz) * inv_s;
    float ey = (m01 * -dx + m11 * -dy + m21 * -dz) * inv_s;
    float ez = (m02 * -dx + m12 * -dy + m22 * -dz) * inv_s;
    float lx = m00 * g_light_x + m10 * g_light_y + m20 * g_light_z;
    float ly = m01 * g_light_x + m11 * g_light_y + m21 * g_light_z;
    float lz = m02 * g_light_x + m12 * g_light_y + m22 * g_light_z;

    for (int i = 0; i < num_vertices; i++) {
        float vx = vertices[i].x, vy = vertices[i].y, vz = vertices[i].z;
        float zc = a20 * vx + a21 * vy + a22 * vz + b2;
        camera_z[i] = zc;
        if (zc > 0.1f) {
            float inv = fov / zc;
            float sx = center_x + (a00 * vx + a01 * vy + a02 * vz + b0) * inv;
            float sy = center_y - (a10 * vx + a11 * vy + a12 * vz + b1) * inv;
            // A vertex just past the near plane (zc ~0.1) and off-axis projects
            // to tens of thousands of pixels; fed to the UNCLIPPED fillTriangle
            // that is a multi-hundred-ms single-frame freeze (worst on big
            // models the camera passes through, like the avenue arch). Clamp
            // the off-screen reach the same way projectPoint/world quads do.
            pullInRadial(sx, sy, 2000.0f);
            projected[i].x = sx;
            projected[i].y = sy;
        }
    }

    // Cull, light, and gather visible faces.
    int count = 0;
    for (int i = 0; i < num_faces; i++) {
        const Face& f = faces[i];
        const Point3D& n = normals[i];
        const Point3D& p0 = vertices[f.indices[0]];

        float facing = n.x * (ex - p0.x) + n.y * (ey - p0.y) + n.z * (ez - p0.z);
        float light_sign = 1.0f;
        if (facing <= 0.0f) {
            if (!(f.flags & 1)) continue; // backface of a closed surface: invisible
            light_sign = -1.0f;           // double-sided sheet: light the visible side
        }

        float sum_z = 0.0f;
        bool behind = false;
        for (int v = 0; v < f.num_vertices; v++) {
            float z = camera_z[f.indices[v]];
            if (z <= 0.1f) { behind = true; break; }
            sum_z += z;
        }
        if (behind) continue;

        float diff = (n.x * lx + n.y * ly + n.z * lz) * light_sign;
        if (diff < 0.0f) diff = 0.0f;
        int i32 = (int)(19.2f + 13.4f * diff); // ambient 0.60 + diffuse 0.42, in 1/32 steps
        if (i32 > 32) i32 = 32;

        uint16_t col = (f.color == 0xFFFF) ? base_color : f.color;
        col = shade565(col, (uint8_t)i32);
        if (g_city_dim) col = blend565(col, CITY_NIGHT_TINT, g_city_dim);
        if (fog_a) col = blend565(col, g_horizon565, fog_a);

        face_data[count].avg_z = sum_z / f.num_vertices;
        face_data[count].index = (uint16_t)i;
        face_data[count].color = col;
        count++;
    }

    // Painter's sort: furthest faces first.
    for (int gap = count / 2; gap > 0; gap /= 2) {
        for (int i = gap; i < count; i++) {
            FaceRenderData temp = face_data[i];
            int j = i;
            while (j >= gap && face_data[j - gap].avg_z < temp.avg_z) {
                face_data[j] = face_data[j - gap];
                j -= gap;
            }
            face_data[j] = temp;
        }
    }

    for (int fi = 0; fi < count; fi++) {
        const Face& f = faces[face_data[fi].index];
        uint16_t shaded = face_data[fi].color;
        int i0 = f.indices[0];
        int i1 = f.indices[1];
        int i2 = f.indices[2];

        sprite.fillTriangle((int16_t)(projected[i0].x + 0.5f), (int16_t)(projected[i0].y + 0.5f),
                            (int16_t)(projected[i1].x + 0.5f), (int16_t)(projected[i1].y + 0.5f),
                            (int16_t)(projected[i2].x + 0.5f), (int16_t)(projected[i2].y + 0.5f),
                            shaded);
        if (f.num_vertices == 4) {
            int i3 = f.indices[3];
            sprite.fillTriangle((int16_t)(projected[i0].x + 0.5f), (int16_t)(projected[i0].y + 0.5f),
                                (int16_t)(projected[i2].x + 0.5f), (int16_t)(projected[i2].y + 0.5f),
                                (int16_t)(projected[i3].x + 0.5f), (int16_t)(projected[i3].y + 0.5f),
                                shaded);
        }
    }
}

// Four wheels drawn as a separate spinning model so they roll with travel
// (the car body mesh no longer contains wheels). `spin` is the accumulated
// axle rotation in radians; left wheels are flipped 180 deg so their gold face
// points outboard. IMPORTANT: this is called BEFORE the body draw at every
// site -- the body has no shared depth buffer with the wheel pass, so drawing
// wheels first lets the body's side panels occlude each wheel's upper half and
// hide the far wheels entirely (only the part below the sill / past the body
// silhouette survives). That replaced the old far-side cull, which had to skip
// the far wheels wholesale and so dropped the front wheels in the chase view.
void drawCarWheels(float pos_x, float pos_y, float pos_z,
                   float heading, float roll, float scale, float spin) {
    static const float wx[4] = {  0.571f,  0.585f, -0.569f, -0.576f };
    static const float wy[4] = {  0.225f,  0.225f,  0.225f,  0.225f };
    static const float wz[4] = {  0.989f, -0.986f,  1.002f, -1.008f };
    float ch = cosf(heading), sh = sinf(heading);
    for (int i = 0; i < 4; i++) {
        float lx = wx[i] * scale, ly = wy[i] * scale, lz = wz[i] * scale;
        float px = pos_x + lx * ch + lz * sh;
        float pz = pos_z - lx * sh + lz * ch;
        float py = pos_y + ly;
        bool left = wx[i] < 0.0f;
        float ry = left ? heading + 3.14159f : heading;
        float rx = left ? -spin : spin;
        draw3DModel(wheel_vertices, WHEEL_NUM_VERTICES, wheel_faces, wheel_normals, WHEEL_NUM_FACES,
                    px, py, pz, rx, ry, roll, scale, 0xC4A6, 0);
    }
}

void drawCarRearGlass(float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z,
                      float scale, bool braking) {
    // Classic R32 GT-R quad round tail lights (two per side). Dim red lenses
    // are always shown; braking lights them bright. Only drawn when the rear
    // faces the camera (so the menu's front view never bleeds them through).
    float rear_dx = -sinf(rot_y), rear_dz = -cosf(rot_y);
    if (rear_dx * (cam_x - pos_x) + rear_dz * (cam_z - pos_z) <= 0.0f) return;
    // Two round lamps per side, kept as a tight separate pair (R32 GT-R look).
    static const Point3D tail_points[4] = {
        { -0.48f, 0.52f, -1.63f },
        { -0.31f, 0.52f, -1.63f },
        {  0.31f, 0.52f, -1.63f },
        {  0.48f, 0.52f, -1.63f }
    };

    float cx = cosf(rot_x), sx = sinf(rot_x);
    float cy = cosf(rot_y), sy = sinf(rot_y);
    float cz = cosf(rot_z), sz = sinf(rot_z);

    for (int i = 0; i < 4; i++) {
        float lx = tail_points[i].x * scale;
        float ly = tail_points[i].y * scale;
        float lz = tail_points[i].z * scale;

        float x1 = lx * cz - ly * sz;
        float y1 = lx * sz + ly * cz;
        float z1 = lz;

        float x2 = x1;
        float y2 = y1 * cx - z1 * sx;
        float z2 = y1 * sx + z1 * cx;

        float wx = x2 * cy + z2 * sy + pos_x;
        float wy = y2 + pos_y;
        float wz = -x2 * sy + z2 * cy + pos_z;

        float cx_cam = wx - cam_x;
        float cy_cam = wy - cam_y;
        float cz_cam = wz - cam_z;

        float rx_cam1 = cx_cam * cam_cos_yaw - cz_cam * cam_sin_yaw;
        float rz_cam1 = cx_cam * cam_sin_yaw + cz_cam * cam_cos_yaw;
        float ry_cam1 = cy_cam;

        float rx_cam = rx_cam1;
        float ry_cam = ry_cam1 * cam_cos_pitch - rz_cam1 * cam_sin_pitch;
        float rz_cam = ry_cam1 * cam_sin_pitch + rz_cam1 * cam_cos_pitch;

        if (rz_cam <= 0.1f) continue;
        int x = (int)(center_x + (rx_cam * fov / rz_cam));
        int y = (int)(center_y - (ry_cam * fov / rz_cam));
        int r = (int)(0.076f * fov / rz_cam);
        if (r < 1) r = 1;
        if (r > 5) r = 5;
        if (braking) {
            sprite.fillCircle(x, y, r, 0xF800);
            sprite.fillCircle(x, y, (r > 1 ? r - 1 : 1), 0xFD20); // hot center
        } else {
            sprite.fillCircle(x, y, r, 0x7000);                   // dim red lens
            sprite.drawCircle(x, y, r, 0xA800);                   // housing rim
        }
    }
}

void drawTreeImpostor(float pos_x, float pos_y, float pos_z, float scale, uint8_t fog_a) {
    float sx, sy, sz;
    projectPoint(pos_x, pos_y, pos_z, sx, sy, sz);
    if (sz <= 2.0f || sz > 44.0f) return;

    int h = (int)(460.0f * scale / sz);
    if (h < 4) return;
    if (h > 80) h = 80;

    int x = (int)sx;
    int y = (int)sy;
    int half_w = h / 4;
    if (x < -h || x > SCREEN_WIDTH + h || y < -h || y > SCREEN_HEIGHT + h) return;

    // Cast blob shadow opposite the sun (city daytime).
    if (g_shadow_a) {
        float sox, soz;
        sunBlobOffset(1.3f * scale, sox, soz);
        float shx, shy, shz;
        projectPoint(pos_x + sox, pos_y + 0.01f, pos_z + soz, shx, shy, shz);
        if (shz > 1.0f) {
            int trx = h / 3;
            int try_ = h / 9;
            if (try_ < 1) try_ = 1;
            shadowEllipse((int)shx, (int)shy, trx, try_, 6);
        }
    }

    int trunk_h = h / 3;
    int trunk_w = h / 10;
    if (trunk_w < 1) trunk_w = 1;
    int leaf_base_y = y - trunk_h;
    int top_y = y - h;

    uint16_t trunk_color = cityGrade(0x3920);
    uint16_t leaf_dark = cityGrade(0x0340);
    uint16_t leaf_mid = cityGrade(0x04C0);
    if (fog_a) {
        trunk_color = blend565(trunk_color, g_horizon565, fog_a);
        leaf_dark = blend565(leaf_dark, g_horizon565, fog_a);
        leaf_mid = blend565(leaf_mid, g_horizon565, fog_a);
    }

    sprite.fillRect(x - trunk_w / 2, leaf_base_y, trunk_w, trunk_h, trunk_color);
    sprite.fillTriangle(x - half_w, leaf_base_y,
                        x + half_w, leaf_base_y,
                        x, top_y,
                        leaf_dark);
    sprite.fillTriangle(x - half_w * 3 / 4, leaf_base_y - h / 4,
                        x + half_w * 3 / 4, leaf_base_y - h / 4,
                        x, top_y + h / 6,
                        leaf_mid);
}

// Billboard drawing
void drawBillboard(float pos_x, float pos_y, float pos_z, float rot_y, float scale, uint16_t color, uint8_t fog_a) {
    // 1. Draw the board face
    draw3DModel(billboard_vertices, 8, billboard_faces, billboard_normals, BILLBOARD_NUM_FACES,
                pos_x, pos_y, pos_z,
                0.0f, rot_y, 0.0f,
                scale, color, fog_a);

    // 2. Draw support posts as lines (simpler, faster)
    float c_y = cosf(rot_y), s_y = sinf(rot_y);
    float lp0_x = -1.5f * scale * c_y + pos_x;
    float lp0_y = pos_y;
    float lp0_z = 1.5f * scale * s_y + pos_z;
    float lp1_x = -1.5f * scale * c_y + pos_x;
    float lp1_y = 1.4f * scale + pos_y;
    float lp1_z = 1.5f * scale * s_y + pos_z;
    
    float rp0_x = 1.5f * scale * c_y + pos_x;
    float rp0_y = pos_y;
    float rp0_z = -1.5f * scale * s_y + pos_z;
    float rp1_x = 1.5f * scale * c_y + pos_x;
    float rp1_y = 1.4f * scale + pos_y;
    float rp1_z = -1.5f * scale * s_y + pos_z;
    
    float sx0, sy0, sz0, sx1, sy1, sz1;
    projectPoint(lp0_x, lp0_y, lp0_z, sx0, sy0, sz0);
    projectPoint(lp1_x, lp1_y, lp1_z, sx1, sy1, sz1);
    if (sz0 > 0.4f && sz1 > 0.4f) {
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x4A69);
    }
    projectPoint(rp0_x, rp0_y, rp0_z, sx0, sy0, sz0);
    projectPoint(rp1_x, rp1_y, rp1_z, sx1, sy1, sz1);
    if (sz0 > 0.4f && sz1 > 0.4f) {
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x4A69);
    }
    
    // 3. Draw Billboard Text (Removed to keep scene clean)
}

// 3D Point Projection Helper
bool projectPoint(float wx, float wy, float wz, float& sx, float& sy, float& sz) {
    float cx = wx - cam_x;
    float cy = wy - cam_y;
    float cz = wz - cam_z;

    // Yaw
    float rx1 = cx * cam_cos_yaw - cz * cam_sin_yaw;
    float rz1 = cx * cam_sin_yaw + cz * cam_cos_yaw;
    float ry1 = cy;
    
    // Pitch
    float rx = rx1;
    float ry = ry1 * cam_cos_pitch - rz1 * cam_sin_pitch;
    float rz = ry1 * cam_sin_pitch + rz1 * cam_cos_pitch;
    
    sz = rz;
    
    // Near-Plane Z-Clamping Hack
    // If the point is behind the camera, clamp it to the glass.
    // The perspective divide will stretch it way off-screen, perfectly filling the bottom gap!
    if (rz <= 0.1f) {
        rz = 0.1f;
    }
    
    float inv = fov / rz;
    sx = center_x + rx * inv;
    sy = center_y - ry * inv;

    // Radial clamp for far-stretched projections. The near-plane stretch
    // trick above can fling coordinates to tens of thousands of pixels,
    // which would overflow the rasterizer's int16 coordinates (wrapping into
    // garbage triangles). Scaling the offset vector UNIFORMLY preserves the
    // point's direction from the screen center exactly -- clamping x and y
    // independently would bend edge slopes (visible as road edges kinking
    // inward at the screen border).
    float ox = sx - center_x;
    float oy = sy - center_y;
    float ax = fabsf(ox);
    float ay = fabsf(oy);
    float m = (ax > ay) ? ax : ay;
    if (m > 2500.0f) {
        float k = 2500.0f / m;
        sx = center_x + ox * k;
        sy = center_y + oy * k;
    }
    return true;
}

void updateCameraTrig() {
    cam_cos_yaw = cosf(cam_yaw);
    cam_sin_yaw = sinf(cam_yaw);
    cam_cos_pitch = cosf(cam_pitch);
    cam_sin_pitch = sinf(cam_pitch);
}

float approachFloat(float current, float target, float response, float dt) {
    float blend = response * dt;
    if (blend > 1.0f) blend = 1.0f;
    if (blend < 0.0f) blend = 0.0f;
    return current + (target - current) * blend;
}

// Per-scanline dithered vertical gradient written straight into the
// framebuffer (sprite pixels are stored byte-swapped, hence the bswap).
// Rows [y_start, y_end) take their blend position from (y - span_start) / span_len.
void fillGradientRows(int y_start, int y_end, int span_start, int span_len,
                      int r0, int g0, int b0, int r1, int g1, int b1) {
    if (y_start < 0) y_start = 0;
    if (y_end > SCREEN_HEIGHT) y_end = SCREEN_HEIGHT;
    if (y_start >= y_end) return;
    if (span_len < 1) span_len = 1;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    float inv = 1.0f / (float)span_len;

    for (int y = y_start; y < y_end; y++) {
        float t = (float)(y - span_start) * inv;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        int r = r0 + (int)((r1 - r0) * t);
        int g = g0 + (int)((g1 - g0) * t);
        int b = b0 + (int)((b1 - b0) * t);
        // Two colors half a quantization step apart, alternated per pixel in a
        // checker pattern: cheap ordered dithering that kills banding.
        uint16_t ca = pack565(r, g, b);
        int r2 = r + 4; if (r2 > 255) r2 = 255;
        int g2 = g + 2; if (g2 > 255) g2 = 255;
        int b2 = b + 4; if (b2 > 255) b2 = 255;
        uint16_t cb = pack565(r2, g2, b2);
        uint16_t sa = __builtin_bswap16(ca);
        uint16_t sb = __builtin_bswap16(cb);
        uint32_t pat = (y & 1) ? ((uint32_t)sa << 16) | sb : ((uint32_t)sb << 16) | sa;
        uint32_t* row = (uint32_t*)(buf + y * SCREEN_WIDTH);
        for (int x = 0; x < SCREEN_WIDTH / 2; x++) row[x] = pat;
    }
}

// 2D Quad Drawer helper
void drawQuad(float sx0, float sy0, float sx1, float sy1, float sx2, float sy2, float sx3, float sy3, uint16_t color) {
    sprite.fillTriangle((int16_t)sx0, (int16_t)sy0,
                        (int16_t)sx1, (int16_t)sy1,
                        (int16_t)sx2, (int16_t)sy2, color);
    sprite.fillTriangle((int16_t)sx0, (int16_t)sy0,
                        (int16_t)sx2, (int16_t)sy2,
                        (int16_t)sx3, (int16_t)sy3, color);
}

// Translucent ground shadow: darkens the framebuffer pixels under an
// ellipse instead of painting a solid blob, so road markings stay visible.
void shadowEllipse(int cx, int cy, int rx, int ry, uint8_t darken) {
    if (rx <= 0 || ry <= 0) return;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    uint8_t keep = 32 - darken;
    for (int dy = -ry; dy <= ry; dy++) {
        int y = cy + dy;
        if (y < 0 || y >= SCREEN_HEIGHT) continue;
        float fy = (float)dy / (float)ry;
        int span = (int)(rx * sqrtf(1.0f - fy * fy));
        int x0 = cx - span; if (x0 < 0) x0 = 0;
        int x1 = cx + span; if (x1 >= SCREEN_WIDTH) x1 = SCREEN_WIDTH - 1;
        uint16_t* p = buf + y * SCREEN_WIDTH + x0;
        for (int x = x0; x <= x1; x++, p++) {
            uint16_t c = __builtin_bswap16(*p);
            *p = __builtin_bswap16(shade565(c, keep));
        }
    }
}

// Screen-space particles (dirt, sparks)
void spawnParticle(float x, float y, float vx, float vy, int life, uint16_t color) {
    particles[particle_cursor] = { x, y, vx, vy, (int16_t)life, color };
    particle_cursor = (particle_cursor + 1) % MAX_PARTICLES;
}

void spawnImpactSparks(int cx, int cy, float dir) {
    for (int i = 0; i < 8; i++) {
        uint16_t c = (i & 1) ? 0xFFE0 : ((i & 2) ? 0xFD20 : 0xFFFF);
        spawnParticle(cx + random(-6, 7), cy + random(-8, 9),
                      dir * (0.8f + random(0, 21) * 0.1f), -random(0, 25) * 0.1f,
                      8 + random(0, 9), c);
    }
}

void updateAndDrawParticles() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle& p = particles[i];
        if (p.life <= 0) continue;
        p.life--;
        p.x += p.vx;
        p.y += p.vy;
        p.vy += 0.18f; // gravity
        int size = (p.life > 8) ? 2 : 1;
        sprite.fillRect((int)p.x, (int)p.y, size, size, p.color);
    }
}

// Project the whole road network into the corner box once at startup.
void buildCityMinimap() {
    float minx = 1e9f, maxx = -1e9f, minz = 1e9f, maxz = -1e9f;
    for (int i = 0; i < CITY_ROAD_COUNT; i++) {
        const CityRoad& r = city_roads[i];
        if (r.x0 < minx) minx = r.x0;
        if (r.x1 < minx) minx = r.x1;
        if (r.x0 > maxx) maxx = r.x0;
        if (r.x1 > maxx) maxx = r.x1;
        if (r.z0 < minz) minz = r.z0;
        if (r.z1 < minz) minz = r.z1;
        if (r.z0 > maxz) maxz = r.z0;
        if (r.z1 > maxz) maxz = r.z1;
    }
    cmm_wx = (minx + maxx) * 0.5f;
    cmm_wz = (minz + maxz) * 0.5f;
    // Fit the road-net bounding box inside the disc (2R across, less a small
    // rim margin). Corners of the box clip at the rim, GTA-radar style.
    float fit = 2.0f * CMM_R - 5.0f;
    float sx = fit / ((maxx - minx) > 1.0f ? (maxx - minx) : 1.0f);
    float sz = fit / ((maxz - minz) > 1.0f ? (maxz - minz) : 1.0f);
    cmm_scale = (sx < sz) ? sx : sz;
    for (int i = 0; i < CITY_ROAD_COUNT; i++) {
        cmm_x0[i] = (int16_t)(CMM_CX + (city_roads[i].x0 - cmm_wx) * cmm_scale);
        cmm_y0[i] = (int16_t)(CMM_CY - (city_roads[i].z0 - cmm_wz) * cmm_scale);
        cmm_x1[i] = (int16_t)(CMM_CX + (city_roads[i].x1 - cmm_wx) * cmm_scale);
        cmm_y1[i] = (int16_t)(CMM_CY - (city_roads[i].z1 - cmm_wz) * cmm_scale);
    }
}

// Bresenham line plotted only where it falls inside the radar disc (per-pixel
// clip; the map is tiny so the pixel count is trivial).
static void cmmClippedLine(int x0, int y0, int x1, int y1, int r2, uint16_t color) {
    int dx = x1 - x0; if (dx < 0) dx = -dx;
    int dy = y1 - y0; if (dy < 0) dy = -dy;
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        int ex = x0 - CMM_CX, ey = y0 - CMM_CY;
        if (ex * ex + ey * ey <= r2) sprite.drawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// Pull a marker inward so it sits within `rmax` of the radar centre.
static void cmmClampToDisc(int& x, int& y, int rmax) {
    int ex = x - CMM_CX, ey = y - CMM_CY;
    int d2 = ex * ex + ey * ey;
    if (d2 > rmax * rmax && d2 > 0) {
        float inv = rmax / sqrtf((float)d2);
        x = CMM_CX + (int)(ex * inv);
        y = CMM_CY + (int)(ey * inv);
    }
}

// GTA-style corner radar: translucent round backing (framebuffer darken), the
// road network clipped to the disc, live traffic blips, and a heading arrow.
void drawCityMinimap() {
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    const int r2 = CMM_R * CMM_R;
    const int r2_in = (CMM_R - 1) * (CMM_R - 1);

    // Translucent circular backing: darken the framebuffer under the disc.
    for (int y = CMM_CY - CMM_R; y <= CMM_CY + CMM_R; y++) {
        if (y < 0 || y >= SCREEN_HEIGHT) continue;
        int ey = y - CMM_CY;
        uint16_t* prow = buf + y * SCREEN_WIDTH;
        for (int x = CMM_CX - CMM_R; x <= CMM_CX + CMM_R; x++) {
            if (x < 0 || x >= SCREEN_WIDTH) continue;
            int ex = x - CMM_CX;
            if (ex * ex + ey * ey > r2) continue;
            uint16_t c = __builtin_bswap16(prow[x]);
            prow[x] = __builtin_bswap16(shade565(c, 12));
        }
    }
    sprite.drawCircle(CMM_CX, CMM_CY, CMM_R, 0x4208);

    for (int i = 0; i < CITY_ROAD_COUNT; i++) {
        cmmClippedLine(cmm_x0[i], cmm_y0[i], cmm_x1[i], cmm_y1[i], r2_in, 0x8C71);
    }

    // Traffic blips (only those inside the disc).
    for (int i = 0; i < CITY_NPC_COUNT; i++) {
        int px = CMM_CX + (int)((city_npcs[i].x - cmm_wx) * cmm_scale);
        int py = CMM_CY - (int)((city_npcs[i].z - cmm_wz) * cmm_scale);
        int ex = px - CMM_CX, ey = py - CMM_CY;
        if (ex * ex + ey * ey <= r2_in) sprite.drawPixel(px, py, 0xFD20);
    }

    // Courier target: a blinking marker (gold pickup / green drop) pinned to the
    // rim so it always points you somewhere.
    if (current_state == PLAYING && ((millis() / 350u) & 1)) {
        int tx = CMM_CX + (int)((courier_tx - cmm_wx) * cmm_scale);
        int ty = CMM_CY - (int)((courier_tz - cmm_wz) * cmm_scale);
        cmmClampToDisc(tx, ty, CMM_R - 3);
        uint16_t tcol = courier_has_fare ? 0x07E0 : 0xFE60;
        sprite.drawLine(tx - 2, ty, tx + 2, ty, tcol);
        sprite.drawLine(tx, ty - 2, tx, ty + 2, tcol);
    }

    // Player marker + heading arrow, kept inside the disc (rim margin leaves
    // room for the 4 px arrow so it never poke past the ring).
    int px = CMM_CX + (int)((player_x - cmm_wx) * cmm_scale);
    int py = CMM_CY - (int)((player_z - cmm_wz) * cmm_scale);
    cmmClampToDisc(px, py, CMM_R - 5);
    int hx = px + (int)(sinf(player_heading) * 4.0f);
    int hy = py - (int)(cosf(player_heading) * 4.0f);
    sprite.drawLine(px, py, hx, hy, 0xFFFF);
    sprite.fillRect(px - 1, py - 1, 3, 3, 0xF800);
}

// UI Head-Up Display Overlay
void drawHUD() {
    sprite.setTextDatum(TL_DATUM);
    sprite.setTextSize(1);

    if (!courier_started) {
        // Free-roam: no clock yet, just point the player at the first fare.
        sprite.setTextColor(0xFE60);
        sprite.drawString("GO PICK UP A FARE", 10, 8);
    } else {
        // Objective banner: where to go next.
        if (courier_has_fare) {
            sprite.setTextColor(0x07E0);
            sprite.drawString("DROP OFF FARE", 10, 8);
        } else {
            sprite.setTextColor(0xFE60);
            sprite.drawString("PICK UP FARE", 10, 8);
        }

        // Money + deliveries + streak multiplier.
        char info_str[28];
        int mult = 1 + courier_streak / 3;
        if (mult > 5) mult = 5;
        snprintf(info_str, sizeof(info_str), "$%ld  x%d  D:%d",
                 courier_money, mult, courier_deliveries);
        sprite.setTextColor(0xFFFF);
        sprite.drawString(info_str, 10, 20);

        // Time bank: turns amber then red and blinks as it runs low.
        int secs = (int)(courier_time_left + 0.5f);
        char tstr[12];
        snprintf(tstr, sizeof(tstr), "%d:%02d", secs / 60, secs % 60);
        uint16_t tcol = 0x07FF;
        if (courier_time_left < 10.0f) tcol = ((millis() / 200u) & 1) ? 0xF800 : 0xFFFF;
        else if (courier_time_left < 20.0f) tcol = 0xFD20;
        sprite.setTextColor(tcol);
        sprite.setTextSize(2);
        sprite.drawString(tstr, 10, 32);
        sprite.setTextSize(1);
    }

    // Brief reward flash near the screen center on board/deliver.
    if (courier_popup_ms > 0) {
        sprite.setTextDatum(MC_DATUM);
        sprite.setTextSize(2);
        if (courier_popup_amount > 0) {
            char pop[16];
            snprintf(pop, sizeof(pop), "+$%ld", courier_popup_amount);
            sprite.setTextColor(0x07E0);
            sprite.drawString(pop, 160, 54);
        } else {
            sprite.setTextColor(0xFE60);
            sprite.drawString("FARE ABOARD", 160, 54);
        }
        sprite.setTextSize(1);
        sprite.setTextDatum(TL_DATUM);
    }

    sprite.setTextDatum(BR_DATUM);
    sprite.setTextSize(2);
    sprite.setTextColor(0xFFFF);
    char speed_str[6];
    snprintf(speed_str, sizeof(speed_str), "%d", (int)player_speed);
    sprite.drawString(speed_str, 310, 150);
    sprite.setTextSize(1);
    sprite.setTextColor(0x7BEF);
    sprite.drawString("km/h", 310, 166);

    if (SHOW_FPS || SHOW_FRAME_TIMING) {
        sprite.setTextDatum(BL_DATUM);
        sprite.setTextSize(1);
        sprite.setTextColor(0x07E0);
        char perf_str[32];
        int len = 0;
        if (SHOW_FPS) {
            int fps10 = (int)(measured_fps * 10.0f + 0.5f);
            len += snprintf(perf_str + len, sizeof(perf_str) - len,
                            "FPS:%d.%d", fps10 / 10, fps10 % 10);
        }
        if (SHOW_FRAME_TIMING) {
            int r10 = (int)(perf_render_ms * 10.0f + 0.5f);
            int w10 = (int)(perf_worst_ms * 10.0f + 0.5f);
            len += snprintf(perf_str + len, sizeof(perf_str) - len,
                            "%s%d.%d/%d.%dms", len ? " " : "",
                            r10 / 10, r10 % 10, w10 / 10, w10 % 10); // avg/worst
        }
        if (!use_dma) {
            snprintf(perf_str + len, sizeof(perf_str) - len, " SB");
        }
        sprite.drawString(perf_str, 10, 166);
    }
}

// Race Finished UI overlay
// City Courier shift-over card: final earnings, deliveries, best, restart.
void drawCourierGameOver() {
    sprite.fillRect(34, 24, 252, 122, 0x10A2);
    sprite.drawRect(34, 24, 252, 122, 0xFE60);

    sprite.setTextDatum(MC_DATUM);
    sprite.setTextSize(3);
    sprite.setTextColor(0xFE60);
    sprite.drawString("SHIFT OVER", 160, 46);

    char line[28];
    sprite.setTextSize(2);
    sprite.setTextColor(0xFFFF);
    snprintf(line, sizeof(line), "$%ld", courier_money);
    sprite.drawString(line, 160, 72);

    sprite.setTextSize(1);
    sprite.setTextColor(0x07FF);
    snprintf(line, sizeof(line), "%d DELIVERIES", courier_deliveries);
    sprite.drawString(line, 160, 92);

    sprite.setTextColor(0xFCE0);
    if (courier_money >= (long)courier_best && courier_money > 0) {
        sprite.drawString("NEW BEST!", 160, 106);
    } else {
        snprintf(line, sizeof(line), "BEST $%lu", courier_best);
        sprite.drawString(line, 160, 106);
    }

    sprite.setTextColor(((millis() / 400u) & 1) ? 0xFFFF : 0x7BEF);
    sprite.drawString("PRESS A BUTTON TO DRIVE AGAIN", 160, 130);

    // Debounced restart (wait for release first, then a fresh press).
    static bool released = false;
    bool pressed = btn_left_down || btn_right_down;
    if (!pressed) released = true;
    if (released && pressed) {
        released = false;
        resetRaceState();
        current_state = PLAYING;
    }
}

void drawFinished() {
    drawCourierGameOver();
}

// 3D Start Screen (dark garage, rotating car demo)
void drawStartScreen() {
    float t = millis() * 0.001f;
    g_city_dim = 0; // the garage is indoors; never grade the menu by city time

    center_x = 160.0f;
    center_y = 90.0f;
    cam_x = 0.0f;
    cam_y = 1.5f;
    cam_z = 0.0f;
    cam_yaw = 0.0f;
    cam_pitch = -0.15f;
    updateCameraTrig();
    
    drawMenuGarage(t);
    drawMenuGarageProps(t);

    // Hero car: parked on the apron at a fixed 3/4 FRONT angle (front + right
    // flank toward the camera), perfectly stationary.
    const float rot_y = -0.62f + 3.14159f;
    const float car_scale = 1.22f;
    float car_x_val = 0.0f;
    float car_y_val = 0.16f;
    float car_z_val = 6.2f;
    float car_roll = 0.0f;

    // Ground shadow projected from the car base so it tracks the car distance.
    {
        float shx, shy, shz;
        projectPoint(car_x_val, 0.0f, car_z_val, shx, shy, shz);
        if (shz > 0.4f) {
            int sw = (int)(480.0f / shz); if (sw < 16) sw = 16;
            int sh = sw / 6; if (sh < 3) sh = 3;
            drawMenuShadow((int)shx, (int)shy, sw, sh);
        }
    }

    // Fixed golden-hour key light (static -- the scene no longer animates).
    g_light_x = 0.30f;
    g_light_y = 0.92f;
    g_light_z = -0.34f;
    { float l = sqrtf(g_light_x * g_light_x + g_light_y * g_light_y + g_light_z * g_light_z);
      g_light_x /= l; g_light_y /= l; g_light_z /= l; }

    // Wheels before body (body occludes the wheel tops -- see drawCarWheels).
    drawCarWheels(car_x_val, car_y_val, car_z_val, rot_y, car_roll, car_scale, 0.0f);
    draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, car_normals, CAR_NUM_FACES,
                car_x_val, car_y_val, car_z_val,
                0.0f, rot_y, car_roll, car_scale, 0x021F, 0); // Subaru blue
    drawCarRearGlass(car_x_val, car_y_val, car_z_val,
                     0.0f, rot_y, car_roll, car_scale, false);

    float cyr = cosf(rot_y), syr = sinf(rot_y);

    // Headlights flash a quick double-blink every ~5 s (otherwise the lenses
    // sit dark). The car model's front is at local +z, so the lenses live there.
    float fp = fmodf(t, 5.0f);
    bool head_on = (fp < 0.16f) || (fp > 0.30f && fp < 0.46f);
    for (int side = -1; side <= 1; side += 2) {
        float lx = 0.45f * side * car_scale, ly = 0.42f * car_scale, lz = 1.55f * car_scale;
        float wx = car_x_val + lx * cyr + lz * syr;
        float wz = car_z_val - lx * syr + lz * cyr;
        float sx, sy, sz;
        projectPoint(wx, car_y_val + ly, wz, sx, sy, sz);
        if (sz > 0.4f) {
            // Rectangular R32 headlights.
            int hw = (int)(0.26f * fov / sz); if (hw < 4) hw = 4; if (hw > 11) hw = 11;
            int hh = (int)(0.12f * fov / sz); if (hh < 2) hh = 2; if (hh > 5) hh = 5;
            int rx = (int)sx - hw / 2, ry = (int)sy - hh / 2;
            if (head_on) {
                sprite.fillRect(rx, ry, hw, hh, 0xFFF6);
                sprite.drawRect(rx, ry, hw, hh, 0xFFFF);
            } else {
                sprite.fillRect(rx, ry, hw, hh, 0xAD55); // dim glass lens
                sprite.drawRect(rx, ry, hw, hh, 0x632C); // housing
            }
        }
        if (!head_on) continue;
        float fwx = car_x_val + lx * cyr + (lz + 1.6f * car_scale) * syr;
        float fwz = car_z_val - lx * syr + (lz + 1.6f * car_scale) * cyr;
        float gx, gy, gz;
        projectPoint(fwx, 0.03f, fwz, gx, gy, gz);
        if (gz > 0.4f) brightenEllipse((int)gx, (int)gy, 10, 3, 0xFFB5, 4);
    }

    // UI titles -- white, pushed up to hug the top edge with a tiny gap.
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextSize(3);
    sprite.setTextColor(0x0000);
    sprite.drawString("Courier32", 162, 16);
    sprite.setTextColor(0xFFFF);
    sprite.drawString("Courier32", 160, 14);
    
    // Only the title and a blinking prompt remain on the menu.
    if ((millis() / 500) % 2 == 0) {
        sprite.setTextSize(1);
        sprite.setTextColor(0x0000);
        sprite.drawString("PRESS ANY BUTTON", 161, 154);
        sprite.setTextColor(0xFFFF);
        sprite.drawString("PRESS ANY BUTTON", 160, 153);
    }
}

// A small articulated walker for the title scene (a passer-by on the
// sidewalk). Self-contained -- no city grading or fog.
static void drawMenuWalker(float wx, float wz, float phase, uint16_t shirt) {
    float sx, sy, sz;
    if (!projectPoint(wx, 0.0f, wz, sx, sy, sz)) return;
    if (sz < 1.0f) return;
    int x = (int)sx, y = (int)sy;
    int ph = (int)(1.78f * fov / sz);
    if (ph < 6) ph = 6;
    if (ph > 90) ph = 90;
    drawMenuEllipse(x, y, ph / 5 + 2, ph / 16 + 1, 0x2104);
    int head_r = ph / 9 + 1;
    int head_cy = y - ph + head_r;
    int hip = y - (ph * 9) / 20;
    int torso_top = head_cy + head_r;
    int tw = ph / 5 + 1;
    int spread = (int)(sinf(phase) * ph * 0.16f);
    uint16_t pants = 0x29A6, skin = 0xE54F;
    sprite.drawLine(x, hip, x - spread, y, pants);
    sprite.drawLine(x, hip, x + spread, y, pants);
    sprite.fillRect(x - tw / 2, torso_top, tw, hip - torso_top, shirt);
    sprite.drawLine(x - tw / 2, torso_top + 1, x - tw / 2 - spread / 2, hip - 1, shirt);
    sprite.drawLine(x + tw / 2, torso_top + 1, x + tw / 2 + spread / 2, hip - 1, shirt);
    sprite.fillCircle(x, head_cy, head_r, skin);
}

// A simple backdrop pine for the menu forest: short trunk + three stacked
// foliage tiers. Self-contained; sizes scale with projected depth.
static void drawMenuTree(float wx, float wz, float h, uint16_t leaf) {
    float bx, by, bz, tx, ty, tz;
    projectPoint(wx, 0.0f, wz, bx, by, bz);
    projectPoint(wx, h, wz, tx, ty, tz);
    if (bz < 0.5f) return;
    int cx = (int)tx, top = (int)ty, bot = (int)by;
    int span = bot - top; if (span < 6) span = 6;
    int tw = (int)(40.0f / bz); if (tw < 1) tw = 1;
    sprite.fillRect(cx - tw / 2, bot - span / 4, tw, span / 4 + 1, 0x4221); // trunk
    int cw = (int)(150.0f / bz); if (cw < 4) cw = 4;
    uint16_t leaf_dk = shade565(leaf, 16);
    for (int k = 0; k < 3; k++) {
        int apexY = top + k * span / 5;
        int baseY = top + (k + 2) * span / 5 + span / 6;
        int w = cw * (k + 1) / 3;
        sprite.fillTriangle(cx, apexY, cx - w, baseY, cx + w, baseY, (k == 1) ? leaf_dk : leaf);
    }
}

// Title scene: an outdoor forecourt in front of a city garage at golden hour.
// Sky + distant treeline, the street, a flanking forest, and the garage with
// an open door and signage. (Animated props/pedestrian live in the Props pass.)
void drawMenuGarage(float t) {
    int horizon_y = (int)(center_y + cam_pitch * fov);
    if (horizon_y < 1) horizon_y = 1;
    if (horizon_y > SCREEN_HEIGHT) horizon_y = SCREEN_HEIGHT;

    // Sky: late-afternoon gradient, deep blue overhead to warm haze at the rim.
    fillGradientRows(0, horizon_y, 0, horizon_y,
                     36, 92, 180, 250, 198, 150);
    // Low sun with a soft halo.
    int sun_x = 246, sun_y = horizon_y - 15;
    sprite.fillCircle(sun_x, sun_y, 15, 0xFE6B);
    sprite.fillCircle(sun_x, sun_y, 9, 0xFF34);
    sprite.fillCircle(sun_x, sun_y, 5, 0xFFFF);

    // A few soft clouds drifting slowly across the upper sky (behind trees).
    {
        static const int   cl_y[3]   = { 16, 28, 10 };
        static const float cl_spd[3] = { 6.0f, 4.0f, 8.0f };
        static const int   cl_s[3]   = { 1, 2, 1 };
        for (int c = 0; c < 3; c++) {
            int x = (int)(fmodf(t * cl_spd[c] + c * 150.0f, (float)(SCREEN_WIDTH + 80)) - 40.0f);
            int y = cl_y[c], s = cl_s[c];
            uint16_t cloud = 0xEF7D, edge = 0xC618;
            sprite.fillCircle(x + 9 * s, y + 1, 5 * s + 1, edge);     // soft underside
            sprite.fillCircle(x,          y, 5 * s, cloud);
            sprite.fillCircle(x + 8 * s,  y - 2, 6 * s, cloud);
            sprite.fillCircle(x + 17 * s, y, 5 * s, cloud);
        }
    }

    // Distant forested ridge along the horizon -- the woodland backdrop.
    uint16_t ridge = blend565(0x2C44, 0xBDF7, 6);    // hazy dark green
    uint16_t ridge2 = blend565(0x33C5, 0xBDF7, 7);   // slightly lighter humps
    sprite.fillRect(0, horizon_y - 6, SCREEN_WIDTH, 7, ridge);
    for (int bx = -6, i = 0; bx < SCREEN_WIDTH + 6; bx += 11, i++) {
        uint32_t hsh = cityHash((uint32_t)(bx + 400), 0x7E15u);
        int th = 10 + (int)(hsh % 14);                // hump height
        int top = horizon_y - th; if (top < 0) top = 0;
        sprite.fillTriangle(bx, top, bx - 9, horizon_y, bx + 9, horizon_y,
                            (i & 1) ? ridge2 : ridge);
    }

    // --- Ground: a grass base, a foreground asphalt road, curb, and the
    // concrete forecourt apron the car is parked on. ---
    // Near edges are kept narrow (x=+/-14) on purpose: at z=1.0 that still
    // projects far wider than the 320 px screen, but stays under projectPoint's
    // 2500 px radial clamp. At the old +/-45 the near corners blew past the
    // clamp, which scales x and y together and so dragged the bottom edge UP
    // off the screen floor -- the black band at the very bottom of the menu.
    drawMenuQuad3D(-14, 0, 1.0f, 14, 0, 1.0f, 45, 0, 40, -45, 0, 40, 0x3CC6);              // grass base
    drawMenuQuad3D(-14, 0.01f, 1.0f, 14, 0.01f, 1.0f, 45, 0.01f, 5.0f, -45, 0.01f, 5.0f, 0x39C7);   // asphalt road
    drawMenuQuad3D(-45, 0.02f, 5.0f, 45, 0.02f, 5.0f, 45, 0.02f, 5.4f, -45, 0.02f, 5.4f, 0xBDF7);   // curb line
    drawMenuQuad3D(-8.5f, 0.02f, 5.4f, 11.0f, 0.02f, 5.4f, 11.0f, 0.02f, 14.2f, -8.5f, 0.02f, 14.2f, 0x8410); // forecourt apron

    // Forest rising behind and beside the garage (frames the hero). Drawn
    // before the garage so the facade occludes the trunks directly behind it.
    static const float tree_x[]   = { -16, -12, -8.5f,  8.5f,  12,   16,   -4,    4,     0 };
    static const float tree_z[]   = {  19,  17.5f, 18,  18,   17.5f, 19,   22,    22,    24 };
    static const float tree_h[]   = { 6.5f, 7,    6,    6,     7,    6.5f, 7.5f,  7.5f,  8 };
    static const uint16_t tree_c[] = { 0x3D45, 0x2C04, 0x4486, 0x4486, 0x2C04, 0x3D45, 0x33C5, 0x33C5, 0x2B23 };
    for (int i = 0; i < 9; i++)
        drawMenuTree(tree_x[i], tree_z[i], tree_h[i], tree_c[i]);

    // --- Garage (the hero structure): tall, wide, plain-walled (no windows) ---
    const float gz = 14.0f, gh = 5.0f, gw = 8.0f;
    uint16_t wall = 0xCD4F, wall_dk = shade565(wall, 21);
    drawMenuQuad3D(gw, 0, gz, gw, 0, gz + 3.5f, gw, gh, gz + 3.5f, gw, gh, gz, wall_dk);   // right return
    drawMenuQuad3D(-gw, 0, gz, gw, 0, gz, gw, gh, gz, -gw, gh, gz, wall);                   // facade
    drawMenuQuad3D(-gw - 0.3f, gh, gz, gw + 0.3f, gh, gz, gw + 0.3f, gh + 0.45f, gz, -gw - 0.3f, gh + 0.45f, gz, 0x8C30); // parapet

    // Open roll-up door: dark opening, lighter floor inside, frame.
    drawMenuQuad3D(-2.8f, 0, gz - 0.05f, 2.8f, 0, gz - 0.05f, 2.8f, 4.0f, gz - 0.05f, -2.8f, 4.0f, gz - 0.05f, 0x1082);
    drawMenuQuad3D(-2.6f, 0, gz + 0.7f, 2.6f, 0, gz + 0.7f, 2.6f, 0.12f, gz + 0.7f, -2.6f, 0.12f, gz + 0.7f, 0x39C7);
    drawMenuQuad3D(-3.1f, 0, gz - 0.02f, -2.8f, 0, gz - 0.02f, -2.8f, 4.2f, gz - 0.02f, -3.1f, 4.2f, gz - 0.02f, 0x6B4D);
    drawMenuQuad3D( 2.8f, 0, gz - 0.02f,  3.1f, 0, gz - 0.02f,  3.1f, 4.2f, gz - 0.02f,  2.8f, 4.2f, gz - 0.02f, 0x6B4D);
    drawMenuQuad3D(-3.1f, 4.0f, gz - 0.02f, 3.1f, 4.0f, gz - 0.02f, 3.1f, 4.2f, gz - 0.02f, -3.1f, 4.2f, gz - 0.02f, 0x6B4D);

    // Small personnel door to the right of the roll-up door.
    drawMenuQuad3D(4.55f, 0, gz - 0.04f, 5.75f, 0, gz - 0.04f, 5.75f, 2.35f, gz - 0.04f, 4.55f, 2.35f, gz - 0.04f, 0x6B4D); // frame
    drawMenuQuad3D(4.7f, 0, gz - 0.05f, 5.6f, 0, gz - 0.05f, 5.6f, 2.2f, gz - 0.05f, 4.7f, 2.2f, gz - 0.05f, 0x5326);       // door panel
    {
        float kx, ky, kz;
        projectPoint(5.45f, 1.1f, gz - 0.06f, kx, ky, kz);
        if (kz > 0.5f) sprite.fillCircle((int)kx, (int)ky, 1, 0xFD60);       // brass knob
    }

    // Wall lamp above the personnel door.
    float lsx, lsy, lsz;
    projectPoint(5.15f, 2.85f, gz - 0.06f, lsx, lsy, lsz);
    if (lsz > 0.5f) {
        sprite.fillCircle((int)lsx, (int)lsy, 3, 0xFE60);
        sprite.fillCircle((int)lsx, (int)lsy, 1, 0xFFFF);
    }

    // Two-row shop sign on the left of the facade ("Lily's Go" over "Garage"),
    // kept off-centre and below the title at the top of the frame.
    float gsx, gsy, gsz;
    projectPoint(-5.4f, 2.7f, gz - 0.07f, gsx, gsy, gsz);
    if (gsz > 0.5f) {
        int sx = (int)gsx, sy = (int)gsy;
        // Mounting brackets up to the wall.
        sprite.drawLine(sx - 20, sy - 13, sx - 20, sy - 19, 0x6B4D);
        sprite.drawLine(sx + 20, sy - 13, sx + 20, sy - 19, 0x6B4D);
        sprite.fillRect(sx - 33, sy - 13, 66, 26, 0x2945);   // dark plate
        sprite.drawRect(sx - 33, sy - 13, 66, 26, 0xFD60);   // amber frame
        sprite.drawRect(sx - 32, sy - 12, 64, 24, 0xFD60);
        sprite.setTextDatum(MC_DATUM);
        sprite.setTextSize(1);
        sprite.setTextColor(0xFFE0);
        sprite.drawString("Lily's Go", sx, sy - 5);
        sprite.setTextColor(0xFD60);
        sprite.drawString("GARAGE", sx, sy + 6);
        sprite.setTextDatum(TL_DATUM);
    }

    // Raised left sidewalk with a curb (the pedestrian strolls here).
    drawMenuQuad3D(-13.0f, 0.16f, 2.0f, -8.1f, 0.16f, 2.0f, -8.1f, 0.16f, 14.0f, -13.0f, 0.16f, 14.0f, 0x9CD3);
    drawMenuQuad3D(-8.1f, 0.0f, 2.0f, -8.1f, 0.0f, 14.0f, -8.1f, 0.16f, 14.0f, -8.1f, 0.16f, 2.0f, 0x6B4D);
}

void drawMenuGarageProps(float t) {
    // Centre lane dashes on the foreground asphalt road (z 1..5).
    for (int i = 0; i < 3; i++) {
        float z0 = 1.4f + i * 1.3f, z1 = z0 + 0.7f;
        drawMenuQuad3D(-0.22f, 0.03f, z0, 0.22f, 0.03f, z0, 0.22f, 0.03f, z1, -0.22f, 0.03f, z1, 0xE6B5);
    }

    // Street lamp on the left sidewalk, arm reaching over the walk.
    {
        float px = -9.5f, pz = 4.5f;
        float bx, by, bz, tx, ty, tz, hx, hy, hz;
        projectPoint(px, 0.0f, pz, bx, by, bz);
        projectPoint(px, 4.2f, pz, tx, ty, tz);
        projectPoint(px + 1.0f, 4.0f, pz, hx, hy, hz);
        if (bz > 0.5f && tz > 0.5f) {
            sprite.drawLine((int)bx, (int)by, (int)tx, (int)ty, 0x5AEB);
            sprite.drawLine((int)tx, (int)ty, (int)hx, (int)hy, 0x5AEB);
            sprite.fillCircle((int)hx, (int)hy, 3, 0xFE60);
            sprite.fillCircle((int)hx, (int)hy, 1, 0xFFFF);
        }
    }

    // A parked car on the right of the forecourt, for life.
    drawParkedCar(8.6f, 7.8f, -1.45f, 0xF800, 0);

    // Pedestrian strolling the left sidewalk, looping far<->near.
    float wc = fmodf(t * 0.05f, 2.0f);
    float pz = (wc < 1.0f) ? (13.0f - wc * 10.0f) : (3.0f + (wc - 1.0f) * 10.0f);
    drawMenuWalker(-10.5f, pz, t * 6.0f, 0x07FF);
}

void drawMenuQuad3D(float x0, float y0, float z0, float x1, float y1, float z1,
                    float x2, float y2, float z2, float x3, float y3, float z3,
                    uint16_t color) {
    float sx0, sy0, sz0, sx1, sy1, sz1, sx2, sy2, sz2, sx3, sy3, sz3;
    projectPoint(x0, y0, z0, sx0, sy0, sz0);
    projectPoint(x1, y1, z1, sx1, sy1, sz1);
    projectPoint(x2, y2, z2, sx2, sy2, sz2);
    projectPoint(x3, y3, z3, sx3, sy3, sz3);
    drawQuad(sx0, sy0, sx1, sy1, sx2, sy2, sx3, sy3, color);
}

void drawMenuEllipse(int cx, int cy, int w, int h, uint16_t color) {
    int half_h = h / 2;
    for (int y = -half_h; y <= half_h; y++) {
        int span = w - (abs(y) * w) / (half_h + 1);
        if (span < 2) continue;
        sprite.drawFastHLine(cx - span / 2, cy + y, span, color);
    }
}

void drawMenuShadow(int cx, int cy, int w, int h) {
    int half_h = h / 2;
    for (int y = -half_h; y <= half_h; y++) {
        int span = w - (abs(y) * w) / (half_h + 1);
        if (span < 2) continue;
        uint16_t color = (abs(y) < half_h / 2) ? 0x1082 : 0x2104;
        sprite.drawFastHLine(cx - span / 2, cy + y, span, color);
    }
}

// ===== GENERATED MODEL DATA (tools/gen_models.py) -- do not hand-edit =====
const Point3D car_vertices[144] = {
    {    0.5302f,    0.0904f,    1.5635f },
    {    0.6218f,    0.0896f,    1.2778f },
    {    0.6599f,    0.1201f,    0.6960f },
    {    0.6599f,    0.1270f,   -0.7090f },
    {    0.6310f,    0.1603f,   -1.2883f },
    {    0.6046f,    0.1988f,   -1.6153f },
    {    0.5546f,    0.6225f,    1.1349f },
    {    0.5489f,    0.6982f,    0.5969f },
    {    0.4124f,    0.9995f,    0.1490f },
    {    0.4132f,    1.0095f,   -0.7359f },
    {    0.5695f,    0.4405f,    1.5629f },
    {    0.5405f,    0.5355f,    1.4935f },
    {    0.6158f,    0.5510f,    1.2535f },
    {    0.5642f,    0.1647f,    1.4863f },
    {    0.6409f,    0.1703f,    1.2711f },
    {    0.6566f,    0.3164f,    1.2669f },
    {    0.6603f,    0.4426f,    1.1879f },
    {    0.6576f,    0.5020f,    1.0852f },
    {    0.6634f,    0.5083f,    0.9125f },
    {    0.6790f,    0.3165f,    0.7111f },
    {    0.6785f,    0.5057f,   -0.9178f },
    {    0.6699f,    0.4459f,   -0.7978f },
    {    0.6693f,    0.4426f,   -1.1955f },
    {    0.6717f,    0.3319f,   -1.2694f },
    {    0.5974f,    0.7065f,   -1.2637f },
    {    0.5475f,    0.7147f,   -1.6205f },
    {    0.6693f,    0.4426f,    0.7963f },
    {    0.6659f,    0.5861f,   -1.2645f },
    {    0.6220f,    0.5819f,   -1.6472f },
    {    0.6319f,    0.4662f,   -1.6596f },
    {    0.6386f,    0.3843f,   -1.6637f },
    {    0.5700f,    0.2900f,    1.5614f },
    {    0.5272f,    0.7151f,   -1.6447f },
    {    0.5763f,    0.4700f,   -1.6924f },
    {    0.5766f,    0.3857f,   -1.7184f },
    {    0.5379f,    0.2043f,   -1.6700f },
    {    0.5732f,    0.5905f,   -1.6831f },
    {    0.6783f,    0.5020f,   -1.2609f },
    {    0.5764f,    0.4517f,   -1.7250f },
    {    0.6779f,    0.3053f,   -0.7165f },
    {    0.6640f,    0.4990f,   -1.0858f },
    {    0.2974f,    0.5466f,    1.5657f },
    {    0.2909f,    0.4356f,    1.6813f },
    {    0.2874f,    0.2927f,    1.6801f },
    {    0.2868f,    0.1721f,    1.5719f },
    {    0.0000f,    1.0065f,   -0.7287f },
    {    0.0000f,    0.6396f,    1.1349f },
    {    0.0000f,    0.4308f,    1.7250f },
    {    0.0000f,    0.1801f,    1.6647f },
    {   -0.5695f,    0.4405f,    1.5629f },
    {   -0.5700f,    0.2900f,    1.5614f },
    {   -0.6566f,    0.3164f,    1.2669f },
    {   -0.6603f,    0.4426f,    1.1879f },
    {   -0.6576f,    0.5020f,    1.0852f },
    {   -0.6158f,    0.5510f,    1.2535f },
    {   -0.5405f,    0.5355f,    1.4935f },
    {   -0.5642f,    0.1647f,    1.4863f },
    {   -0.5302f,    0.0904f,    1.5635f },
    {   -0.6218f,    0.0896f,    1.2778f },
    {   -0.6409f,    0.1703f,    1.2711f },
    {   -0.5974f,    0.7065f,   -1.2637f },
    {   -0.4132f,    1.0095f,   -0.7359f },
    {   -0.4124f,    0.9995f,    0.1490f },
    {   -0.5489f,    0.6982f,    0.5969f },
    {   -0.6659f,    0.5861f,   -1.2645f },
    {   -0.6220f,    0.5819f,   -1.6472f },
    {   -0.5475f,    0.7147f,   -1.6205f },
    {   -0.5546f,    0.6225f,    1.1349f },
    {    0.0000f,    1.0005f,    0.1560f },
    {    0.0000f,    0.6982f,    0.6529f },
    {    0.0000f,    0.0905f,    1.7106f },
    {   -0.6599f,    0.1270f,   -0.7090f },
    {   -0.6599f,    0.1201f,    0.6960f },
    {   -0.6640f,    0.4990f,   -1.0858f },
    {   -0.6783f,    0.5020f,   -1.2609f },
    {   -0.6634f,    0.5083f,    0.9125f },
    {   -0.6785f,    0.5057f,   -0.9178f },
    {   -0.5272f,    0.7151f,   -1.6447f },
    {   -0.5732f,    0.5905f,   -1.6831f },
    {   -0.5763f,    0.4700f,   -1.6924f },
    {   -0.6319f,    0.4662f,   -1.6596f },
    {   -0.6386f,    0.3862f,   -1.6637f },
    {   -0.5766f,    0.3857f,   -1.7184f },
    {   -0.5764f,    0.4517f,   -1.7250f },
    {   -0.6046f,    0.1988f,   -1.6153f },
    {   -0.5379f,    0.2043f,   -1.6700f },
    {   -0.6693f,    0.4426f,    0.7963f },
    {   -0.6698f,    0.4464f,   -0.7978f },
    {   -0.6790f,    0.3165f,    0.7111f },
    {   -0.6779f,    0.3053f,   -0.7165f },
    {   -0.6717f,    0.3319f,   -1.2694f },
    {   -0.6310f,    0.1603f,   -1.2883f },
    {   -0.6693f,    0.4426f,   -1.1955f },
    {   -0.2868f,    0.1721f,    1.5719f },
    {   -0.2909f,    0.4356f,    1.6813f },
    {   -0.2974f,    0.5466f,    1.5657f },
    {   -0.2874f,    0.2927f,    1.6801f },
    {    0.0000f,    0.5504f,    1.6002f },
    {    0.0000f,    0.2953f,    1.7246f },
    {    0.0000f,    0.7065f,   -1.3418f },
    {   -0.3921f,    1.0309f,   -0.6173f },
    {    0.3921f,    1.0297f,   -0.6173f },
    {   -0.5759f,    0.7100f,   -1.3709f },
    {    0.5729f,    0.7105f,   -1.3709f },
    {    0.4330f,    0.8342f,   -1.6421f },
    {   -0.4332f,    0.8342f,   -1.6421f },
    {   -0.4733f,    0.8290f,   -1.5258f },
    {    0.4706f,    0.8295f,   -1.5258f },
    {    0.4628f,    0.7151f,   -1.6447f },
    {    0.3892f,    0.8030f,   -1.6428f },
    {   -0.4077f,    0.8035f,   -1.6428f },
    {   -0.4813f,    0.7151f,   -1.6447f },
    {   -0.4807f,    0.7149f,   -1.3709f },
    {   -0.4089f,    0.8035f,   -1.4759f },
    {    0.3888f,    0.8034f,   -1.4763f },
    {    0.4628f,    0.7153f,   -1.3709f },
    {    0.6677f,    0.7457f,    0.5426f },
    {    0.6692f,    0.7933f,    0.4703f },
    {    0.6662f,    0.6981f,    0.4703f },
    {    0.5261f,    0.7542f,    0.4919f },
    {    0.5500f,    0.7152f,    0.4912f },
    {    0.5379f,    0.7347f,    0.5214f },
    {   -0.6678f,    0.7457f,    0.5426f },
    {   -0.6663f,    0.6981f,    0.4704f },
    {   -0.6693f,    0.7933f,    0.4704f },
    {   -0.5383f,    0.7347f,    0.5214f },
    {   -0.5501f,    0.7152f,    0.4912f },
    {   -0.5278f,    0.7542f,    0.4919f },
    {    0.4146f,    0.1270f,   -0.7090f },
    {    0.4159f,    0.1603f,   -1.2883f },
    {   -0.4261f,    0.1270f,   -0.7090f },
    {   -0.4285f,    0.1603f,   -1.2883f },
    {    0.4260f,    0.0896f,    1.2778f },
    {    0.4256f,    0.1201f,    0.6960f },
    {   -0.4389f,    0.0896f,    1.2778f },
    {   -0.4387f,    0.1201f,    0.6960f },
    {    0.4520f,    0.9596f,    0.1084f },
    {    0.5885f,    0.6583f,    0.5563f },
    {   -0.4521f,    0.9594f,    0.1083f },
    {   -0.5886f,    0.6581f,    0.5562f },
    {    0.4581f,    0.9680f,   -0.6964f },
    {    0.6423f,    0.6650f,   -1.2242f },
    {   -0.4581f,    0.9680f,   -0.6964f },
    {   -0.6423f,    0.6650f,   -1.2242f },
};

const Face car_faces[232] = {
    { { 10, 15, 31,  0}, 3, 0, 0xFFFF },
    { { 15, 10, 16,  0}, 3, 0, 0xFFFF },
    { { 16, 12, 17,  0}, 3, 0, 0xFFFF },
    { {  0, 14,  1,  0}, 3, 0, 0xFFFF },
    { { 14,  0, 13,  0}, 3, 0, 0xFFFF },
    { {101,  7,  8,  0}, 3, 0, 0x0841 },
    { {  9,  7,101,  0}, 3, 0, 0x0841 },
    { { 24,  7,  9,  0}, 3, 0, 0x0841 },
    { {  6, 12, 11,  0}, 3, 0, 0xFFFF },
    { {  8, 69, 68,  0}, 3, 0, 0x0841 },
    { { 69,  8,  7,  0}, 3, 0, 0x0841 },
    { {  6, 69,  7,  0}, 3, 0, 0xFFFF },
    { { 69,  6, 46,  0}, 3, 0, 0xFFFF },
    { { 40, 27, 37,  0}, 3, 0, 0xFFFF },
    { { 27, 17, 12,  0}, 3, 0, 0xFFFF },
    { { 27, 18, 17,  0}, 3, 0, 0xFFFF },
    { { 27, 20, 18,  0}, 3, 0, 0xFFFF },
    { { 40, 20, 27,  0}, 3, 0, 0xFFFF },
    { { 36, 25, 32,  0}, 3, 0, 0xFFFF },
    { { 25, 36, 28,  0}, 3, 0, 0xFFFF },
    { { 29, 34, 30,  0}, 3, 0, 0xFFFF },
    { { 29, 38, 34,  0}, 3, 0, 0xFFFF },
    { { 33, 38, 29,  0}, 3, 0, 0xFFFF },
    { { 34,  5, 30,  0}, 3, 0, 0xFFFF },
    { {  5, 34, 35,  0}, 3, 0, 0xFFFF },
    { { 29, 36, 33,  0}, 3, 0, 0xFFFF },
    { { 36, 29, 28,  0}, 3, 0, 0xFFFF },
    { { 28, 37, 27,  0}, 3, 0, 0xFFFF },
    { { 37, 28, 29,  0}, 3, 0, 0xFFFF },
    { { 34, 83, 82,  0}, 3, 0, 0xFFFF },
    { { 83, 34, 38,  0}, 3, 0, 0xFFFF },
    { { 31, 14, 13,  0}, 3, 0, 0xFFFF },
    { { 14, 31, 15,  0}, 3, 0, 0xFFFF },
    { { 26, 20, 21,  0}, 3, 0, 0xFFFF },
    { { 20, 26, 18,  0}, 3, 0, 0xFFFF },
    { { 26, 39, 19,  0}, 3, 0, 0xFFFF },
    { { 39, 26, 21,  0}, 3, 0, 0xFFFF },
    { { 19,  3,  2,  0}, 3, 0, 0xFFFF },
    { {  3, 19, 39,  0}, 3, 0, 0xFFFF },
    { {  4, 30,  5,  0}, 3, 0, 0xFFFF },
    { { 30,  4, 23,  0}, 3, 0, 0xFFFF },
    { { 30, 37, 29,  0}, 3, 0, 0xFFFF },
    { { 23, 37, 30,  0}, 3, 0, 0xFFFF },
    { { 22, 37, 23,  0}, 3, 0, 0xFFFF },
    { { 40, 37, 22,  0}, 3, 0, 0xFFFF },
    { { 33, 83, 38,  0}, 3, 0, 0xFFFF },
    { { 83, 33, 79,  0}, 3, 0, 0xFFFF },
    { {  0, 44, 13,  0}, 3, 0, 0xFFFF },
    { { 44, 70, 48,  0}, 3, 0, 0xFFFF },
    { {  0, 70, 44,  0}, 3, 0, 0xFFFF },
    { { 42, 11, 10,  0}, 3, 0, 0xFFFF },
    { { 11, 42, 41,  0}, 3, 0, 0xFFFF },
    { { 31, 42, 10,  0}, 3, 0, 0xFFFF },
    { { 42, 31, 43,  0}, 3, 0, 0xFFFF },
    { { 44, 31, 13,  0}, 3, 0, 0xFFFF },
    { { 31, 44, 43,  0}, 3, 0, 0xFFFF },
    { { 97, 42, 47,  0}, 3, 0, 0xFFFF },
    { { 42, 97, 41,  0}, 3, 0, 0xFFFF },
    { { 47, 43, 98,  0}, 3, 0, 0xFFFF },
    { { 43, 47, 42,  0}, 3, 0, 0xFFFF },
    { { 43, 48, 98,  0}, 3, 0, 0xFFFF },
    { { 48, 43, 44,  0}, 3, 0, 0xFFFF },
    { {101, 45,  9,  0}, 3, 0, 0xFFFF },
    { {101, 61, 45,  0}, 3, 0, 0xFFFF },
    { {101,100, 61,  0}, 3, 0, 0xFFFF },
    { {  9, 99, 24,  0}, 3, 0, 0x0841 },
    { { 99,  9, 45,  0}, 3, 0, 0x0841 },
    { { 41,  6, 11,  0}, 3, 0, 0xFFFF },
    { { 41, 46,  6,  0}, 3, 0, 0xFFFF },
    { { 97, 46, 41,  0}, 3, 0, 0xFFFF },
    { { 35, 82, 85,  0}, 3, 0, 0xFFFF },
    { { 82, 35, 34,  0}, 3, 0, 0xFFFF },
    { { 49, 51, 52,  0}, 3, 0, 0xFFFF },
    { { 51, 49, 50,  0}, 3, 0, 0xFFFF },
    { { 54, 52, 53,  0}, 3, 0, 0xFFFF },
    { { 57, 59, 56,  0}, 3, 0, 0xFFFF },
    { { 59, 57, 58,  0}, 3, 0, 0xFFFF },
    { { 63,100, 62,  0}, 3, 0, 0x0841 },
    { { 63, 61,100,  0}, 3, 0, 0x0841 },
    { { 60, 61, 63,  0}, 3, 0, 0x0841 },
    { { 54, 63, 67,  0}, 3, 0, 0xFFFF },
    { { 64, 63, 54,  0}, 3, 0, 0xFFFF },
    { { 64, 60, 63,  0}, 3, 0, 0xFFFF },
    { { 64,102, 60,  0}, 3, 0, 0xFFFF },
    { { 64, 66,102,  0}, 3, 0, 0xFFFF },
    { { 64, 65, 66,  0}, 3, 0, 0xFFFF },
    { { 62, 69, 63,  0}, 3, 0, 0x0841 },
    { { 69, 62, 68,  0}, 3, 0, 0x0841 },
    { {  1, 70,  0,  0}, 3, 0, 0xFFFF },
    { {  1, 57, 70,  0}, 3, 0, 0xFFFF },
    { {132, 57,  1,  0}, 3, 0, 0xFFFF },
    { {134, 57,132,  0}, 3, 0, 0xFFFF },
    { { 58, 57,134,  0}, 3, 0, 0xFFFF },
    { {  3,133,  2,  0}, 3, 0, 0xFFFF },
    { {  3,135,133,  0}, 3, 0, 0xFFFF },
    { {  3, 72,135,  0}, 3, 0, 0xFFFF },
    { {128, 72,  3,  0}, 3, 0, 0xFFFF },
    { {130, 72,128,  0}, 3, 0, 0xFFFF },
    { { 71, 72,130,  0}, 3, 0, 0xFFFF },
    { { 67, 69, 46,  0}, 3, 0, 0xFFFF },
    { { 69, 67, 63,  0}, 3, 0, 0xFFFF },
    { { 53, 64, 54,  0}, 3, 0, 0xFFFF },
    { { 75, 64, 53,  0}, 3, 0, 0xFFFF },
    { { 76, 64, 75,  0}, 3, 0, 0xFFFF },
    { { 73, 64, 76,  0}, 3, 0, 0xFFFF },
    { { 73, 74, 64,  0}, 3, 0, 0xFFFF },
    { { 78, 66, 65,  0}, 3, 0, 0xFFFF },
    { { 66, 78, 77,  0}, 3, 0, 0xFFFF },
    { { 82, 80, 81,  0}, 3, 0, 0xFFFF },
    { { 83, 80, 82,  0}, 3, 0, 0xFFFF },
    { { 79, 80, 83,  0}, 3, 0, 0xFFFF },
    { { 82, 84, 85,  0}, 3, 0, 0xFFFF },
    { { 84, 82, 81,  0}, 3, 0, 0xFFFF },
    { { 80, 78, 65,  0}, 3, 0, 0xFFFF },
    { { 78, 80, 79,  0}, 3, 0, 0xFFFF },
    { { 65, 74, 80,  0}, 3, 0, 0xFFFF },
    { { 74, 65, 64,  0}, 3, 0, 0xFFFF },
    { {108, 36, 32,  0}, 3, 0, 0xFFFF },
    { {111, 36,108,  0}, 3, 0, 0xFFFF },
    { { 77, 36,111,  0}, 3, 0, 0xFFFF },
    { { 77, 78, 36,  0}, 3, 0, 0xFFFF },
    { { 78, 33, 36,  0}, 3, 0, 0xFFFF },
    { { 33, 78, 79,  0}, 3, 0, 0xFFFF },
    { { 50, 59, 51,  0}, 3, 0, 0xFFFF },
    { { 59, 50, 56,  0}, 3, 0, 0xFFFF },
    { { 86, 76, 75,  0}, 3, 0, 0xFFFF },
    { { 76, 86, 87,  0}, 3, 0, 0xFFFF },
    { { 87, 88, 89,  0}, 3, 0, 0xFFFF },
    { { 88, 87, 86,  0}, 3, 0, 0xFFFF },
    { { 88, 71, 89,  0}, 3, 0, 0xFFFF },
    { { 71, 88, 72,  0}, 3, 0, 0xFFFF },
    { { 91, 81, 90,  0}, 3, 0, 0xFFFF },
    { { 81, 91, 84,  0}, 3, 0, 0xFFFF },
    { { 74, 81, 80,  0}, 3, 0, 0xFFFF },
    { { 74, 90, 81,  0}, 3, 0, 0xFFFF },
    { { 74, 92, 90,  0}, 3, 0, 0xFFFF },
    { { 73, 92, 74,  0}, 3, 0, 0xFFFF },
    { { 70, 93, 48,  0}, 3, 0, 0xFFFF },
    { { 57, 93, 70,  0}, 3, 0, 0xFFFF },
    { { 57, 56, 93,  0}, 3, 0, 0xFFFF },
    { { 94, 55, 95,  0}, 3, 0, 0xFFFF },
    { { 55, 94, 49,  0}, 3, 0, 0xFFFF },
    { { 50, 94, 96,  0}, 3, 0, 0xFFFF },
    { { 94, 50, 49,  0}, 3, 0, 0xFFFF },
    { { 93, 50, 96,  0}, 3, 0, 0xFFFF },
    { { 50, 93, 56,  0}, 3, 0, 0xFFFF },
    { { 97, 94, 95,  0}, 3, 0, 0xFFFF },
    { { 94, 97, 47,  0}, 3, 0, 0xFFFF },
    { { 47, 96, 94,  0}, 3, 0, 0xFFFF },
    { { 96, 47, 98,  0}, 3, 0, 0xFFFF },
    { { 96, 48, 93,  0}, 3, 0, 0xFFFF },
    { { 48, 96, 98,  0}, 3, 0, 0xFFFF },
    { { 61, 99, 45,  0}, 3, 0, 0x0841 },
    { { 99, 61, 60,  0}, 3, 0, 0x0841 },
    { {107,105,104,  0}, 3, 0, 0xFFFF },
    { {105,107,106,  0}, 3, 0, 0xFFFF },
    { { 67, 95, 55,  0}, 3, 0, 0xFFFF },
    { { 46, 95, 67,  0}, 3, 0, 0xFFFF },
    { { 97, 95, 46,  0}, 3, 0, 0xFFFF },
    { { 35,  4,  5,  0}, 3, 0, 0xFFFF },
    { { 35,129,  4,  0}, 3, 0, 0xFFFF },
    { { 35,131,129,  0}, 3, 0, 0xFFFF },
    { { 35, 91,131,  0}, 3, 0, 0xFFFF },
    { { 91, 85, 84,  0}, 3, 0, 0xFFFF },
    { { 35, 85, 91,  0}, 3, 0, 0xFFFF },
    { { 68,101,  8,  0}, 3, 0, 0xFFFF },
    { { 62,101, 68,  0}, 3, 0, 0xFFFF },
    { {100,101, 62,  0}, 3, 0, 0xFFFF },
    { {102, 99, 60,  0}, 3, 0, 0xFFFF },
    { { 99,103, 24,  0}, 3, 0, 0xFFFF },
    { { 99,115,103,  0}, 3, 0, 0xFFFF },
    { { 99,112,115,  0}, 3, 0, 0xFFFF },
    { {102,112, 99,  0}, 3, 0, 0xFFFF },
    { { 66,106,102,  0}, 3, 0, 0xFFFF },
    { { 66,105,106,  0}, 3, 0, 0xFFFF },
    { { 77,105, 66,  0}, 3, 0, 0xFFFF },
    { { 25,104, 32,  0}, 3, 0, 0xFFFF },
    { { 25,107,104,  0}, 3, 0, 0xFFFF },
    { {103,107, 25,  0}, 3, 0, 0xFFFF },
    { {104,108, 32,  0}, 3, 0, 0xFFFF },
    { {104,109,108,  0}, 3, 0, 0xFFFF },
    { {105,109,104,  0}, 3, 0, 0xFFFF },
    { {105,110,109,  0}, 3, 0, 0xFFFF },
    { { 77,110,105,  0}, 3, 0, 0xFFFF },
    { {111,110, 77,  0}, 3, 0, 0xFFFF },
    { {106,112,102,  0}, 3, 0, 0xFFFF },
    { {106,113,112,  0}, 3, 0, 0xFFFF },
    { {107,113,106,  0}, 3, 0, 0xFFFF },
    { {107,114,113,  0}, 3, 0, 0xFFFF },
    { {103,114,107,  0}, 3, 0, 0xFFFF },
    { {115,114,103,  0}, 3, 0, 0xFFFF },
    { {115,111,108,  0}, 3, 0, 0xFFFF },
    { {111,115,112,  0}, 3, 0, 0xFFFF },
    { {113,109,110,  0}, 3, 0, 0xFFFF },
    { {109,113,114,  0}, 3, 0, 0xFFFF },
    { {114,108,109,  0}, 3, 0, 0xFFFF },
    { {108,114,115,  0}, 3, 0, 0xFFFF },
    { {113,111,112,  0}, 3, 0, 0xFFFF },
    { {111,113,110,  0}, 3, 0, 0xFFFF },
    { { 10, 12, 16,  0}, 3, 0, 0xFFFF },
    { { 12, 10, 11,  0}, 3, 0, 0xFFFF },
    { { 27, 25, 28,  0}, 3, 0, 0xFFFF },
    { { 27,103, 25,  0}, 3, 0, 0xFFFF },
    { { 27, 24,103,  0}, 3, 0, 0xFFFF },
    { { 27,  7, 24,  0}, 3, 0, 0xFFFF },
    { { 12,  7, 27,  0}, 3, 0, 0xFFFF },
    { { 12,  6,  7,  0}, 3, 0, 0xFFFF },
    { { 49, 54, 55,  0}, 3, 0, 0xFFFF },
    { { 54, 49, 52,  0}, 3, 0, 0xFFFF },
    { { 54, 67, 55,  0}, 3, 0, 0xFFFF },
    { {116,117,118,  0}, 3, 0, 0xFFFF },
    { {117,121,119,  0}, 3, 0, 0xFFFF },
    { {121,117,116,  0}, 3, 0, 0xFFFF },
    { {118,119,120,  0}, 3, 0, 0xFFFF },
    { {119,118,117,  0}, 3, 0, 0xFFFF },
    { {116,120,121,  0}, 3, 0, 0xFFFF },
    { {120,116,118,  0}, 3, 0, 0xFFFF },
    { {124,122,123,  0}, 3, 0, 0xFFFF },
    { {122,127,125,  0}, 3, 0, 0xFFFF },
    { {127,122,124,  0}, 3, 0, 0xFFFF },
    { {124,126,127,  0}, 3, 0, 0xFFFF },
    { {126,124,123,  0}, 3, 0, 0xFFFF },
    { {123,125,126,  0}, 3, 0, 0xFFFF },
    { {125,123,122,  0}, 3, 0, 0xFFFF },
    { {130,129,131,  0}, 3, 0, 0xFFFF },
    { {129,130,128,  0}, 3, 0, 0xFFFF },
    { {132,135,134,  0}, 3, 0, 0xFFFF },
    { {135,132,133,  0}, 3, 0, 0xFFFF },
    { {  8,  7,137,136}, 4, 0, 0xFFFF },
    { { 62, 63,139,138}, 4, 0, 0xFFFF },
    { {  9, 24,141,140}, 4, 0, 0xFFFF },
    { { 61, 60,143,142}, 4, 0, 0xFFFF },
};

const Point3D car_normals[232] = {
    {    0.9594f,    0.0004f,    0.2822f },
    {    0.9649f,    0.1186f,    0.2343f },
    {    0.9431f,    0.2978f,    0.1477f },
    {    0.9337f,   -0.1956f,    0.2999f },
    {    0.9393f,   -0.0828f,    0.3328f },
    {    0.9154f,    0.4024f,   -0.0084f },
    {    0.8208f,    0.5691f,    0.0493f },
    {    0.8729f,    0.4872f,    0.0249f },
    {    0.5311f,    0.8184f,    0.2195f },
    {    0.0108f,    0.8543f,    0.5197f },
    {    0.0556f,    0.8362f,    0.5456f },
    {    0.0142f,    0.9902f,    0.1391f },
    {    0.0306f,    0.9922f,    0.1206f },
    {    0.9854f,    0.1486f,    0.0828f },
    {    0.8001f,    0.5994f,    0.0243f },
    {   -0.0299f,    0.9989f,    0.0357f },
    {    0.9821f,    0.1882f,    0.0079f },
    {    0.9806f,   -0.1799f,   -0.0774f },
    {    0.6938f,    0.4334f,   -0.5752f },
    {    0.5778f,    0.4596f,   -0.6745f },
    {    0.6599f,    0.0911f,   -0.7458f },
    {    0.7676f,   -0.0617f,   -0.6379f },
    {    0.3294f,    0.8235f,   -0.4619f },
    {    0.6267f,   -0.3021f,   -0.7183f },
    {    0.5840f,   -0.3230f,   -0.7447f },
    {    0.5108f,    0.0795f,   -0.8560f },
    {    0.6029f,    0.1361f,   -0.7861f },
    {    0.9835f,    0.1399f,   -0.1145f },
    {    0.9875f,    0.0977f,   -0.1238f },
    {    0.0000f,   -0.0996f,   -0.9950f },
    {    0.0000f,   -0.0996f,   -0.9950f },
    {    0.9174f,   -0.2350f,    0.3211f },
    {    0.9573f,   -0.0950f,    0.2731f },
    {    0.9896f,   -0.1436f,    0.0000f },
    {    0.9971f,    0.0752f,    0.0081f },
    {    0.9970f,    0.0778f,   -0.0014f },
    {    0.9983f,    0.0575f,    0.0005f },
    {    0.9953f,   -0.0967f,   -0.0005f },
    {    0.9950f,   -0.1002f,    0.0000f },
    {    0.9733f,   -0.2050f,   -0.1028f },
    {    0.9698f,   -0.2177f,   -0.1104f },
    {    0.9886f,    0.0872f,   -0.1229f },
    {    0.9955f,   -0.0341f,   -0.0882f },
    {    0.9943f,   -0.0433f,    0.0973f },
    {    0.9948f,   -0.0625f,    0.0799f },
    {    0.0000f,    0.8718f,   -0.4898f },
    {    0.0000f,    0.8718f,   -0.4898f },
    {    0.2412f,    0.6443f,    0.7258f },
    {    0.2880f,    0.4365f,    0.8524f },
    {    0.2193f,    0.5723f,    0.7902f },
    {    0.2973f,    0.6207f,    0.7255f },
    {    0.2339f,    0.6945f,    0.6804f },
    {    0.3911f,   -0.0083f,    0.9203f },
    {    0.3871f,   -0.0173f,    0.9219f },
    {    0.2420f,   -0.5069f,    0.8274f },
    {    0.2928f,   -0.6391f,    0.7113f },
    {    0.0918f,    0.7190f,    0.6889f },
    {    0.0895f,    0.7157f,    0.6926f },
    {    0.1531f,   -0.0030f,    0.9882f },
    {    0.1488f,   -0.0121f,    0.9888f },
    {    0.1322f,   -0.4570f,    0.8796f },
    {    0.2169f,   -0.6522f,    0.7263f },
    {   -0.0101f,    0.9854f,   -0.1697f },
    {    0.0112f,    0.9703f,   -0.2416f },
    {    0.0015f,    0.9841f,   -0.1777f },
    {    0.0628f,    0.8749f,   -0.4803f },
    {   -0.0141f,    0.8981f,   -0.4395f },
    {    0.1151f,    0.9643f,    0.2386f },
    {    0.0302f,    0.9811f,    0.1910f },
    {    0.0344f,    0.9815f,    0.1882f },
    {    0.0000f,   -0.2577f,   -0.9662f },
    {    0.0000f,   -0.2577f,   -0.9662f },
    {   -0.9649f,    0.1186f,    0.2343f },
    {   -0.9594f,    0.0004f,    0.2822f },
    {   -0.9431f,    0.2978f,    0.1477f },
    {   -0.9393f,   -0.0828f,    0.3328f },
    {   -0.9337f,   -0.1956f,    0.2999f },
    {   -0.9150f,    0.4033f,   -0.0077f },
    {   -0.8277f,    0.5592f,    0.0463f },
    {   -0.8729f,    0.4872f,    0.0249f },
    {   -0.6719f,    0.7344f,    0.0963f },
    {   -0.8652f,    0.5008f,    0.0242f },
    {   -0.8689f,    0.4944f,    0.0248f },
    {   -0.8580f,    0.4894f,   -0.1557f },
    {   -0.8396f,    0.5364f,   -0.0855f },
    {   -0.8585f,    0.5022f,   -0.1040f },
    {   -0.0556f,    0.8362f,    0.5456f },
    {   -0.0108f,    0.8543f,    0.5197f },
    {    0.0007f,   -1.0000f,    0.0032f },
    {   -0.0003f,   -1.0000f,    0.0017f },
    {    0.0000f,   -1.0000f,    0.0030f },
    {    0.0000f,   -1.0000f,    0.0030f },
    {    0.0000f,   -1.0000f,    0.0030f },
    {    0.0000f,   -1.0000f,   -0.0049f },
    {    0.0000f,   -1.0000f,   -0.0049f },
    {    0.0000f,   -1.0000f,   -0.0049f },
    {    0.0000f,   -1.0000f,   -0.0049f },
    {    0.0000f,   -1.0000f,   -0.0049f },
    {    0.0000f,   -1.0000f,   -0.0049f },
    {   -0.0306f,    0.9922f,    0.1206f },
    {   -0.0142f,    0.9902f,    0.1391f },
    {   -0.8000f,    0.5995f,    0.0243f },
    {    0.0299f,    0.9989f,    0.0357f },
    {   -0.9821f,    0.1882f,    0.0079f },
    {   -0.9806f,   -0.1799f,   -0.0774f },
    {   -0.9854f,    0.1486f,    0.0828f },
    {   -0.5778f,    0.4596f,   -0.6745f },
    {   -0.6938f,    0.4334f,   -0.5752f },
    {   -0.6581f,    0.0932f,   -0.7471f },
    {   -0.7676f,   -0.0617f,   -0.6379f },
    {   -0.3295f,    0.8234f,   -0.4619f },
    {   -0.5840f,   -0.3230f,   -0.7447f },
    {   -0.6326f,   -0.2990f,   -0.7144f },
    {   -0.6029f,    0.1361f,   -0.7861f },
    {   -0.5108f,    0.0795f,   -0.8560f },
    {   -0.9875f,    0.0977f,   -0.1238f },
    {   -0.9835f,    0.1399f,   -0.1144f },
    {    0.0000f,    0.2945f,   -0.9556f },
    {    0.0000f,    0.2945f,   -0.9556f },
    {    0.0000f,    0.2945f,   -0.9556f },
    {    0.0000f,    0.2945f,   -0.9556f },
    {    0.0000f,    0.0772f,   -0.9970f },
    {    0.0000f,    0.0772f,   -0.9970f },
    {   -0.9573f,   -0.0950f,    0.2731f },
    {   -0.9174f,   -0.2350f,    0.3211f },
    {   -0.9971f,    0.0752f,    0.0081f },
    {   -0.9895f,   -0.1448f,    0.0000f },
    {   -0.9984f,    0.0563f,   -0.0012f },
    {   -0.9971f,    0.0765f,    0.0005f },
    {   -0.9950f,   -0.1002f,    0.0000f },
    {   -0.9953f,   -0.0967f,   -0.0005f },
    {   -0.9697f,   -0.2175f,   -0.1115f },
    {   -0.9738f,   -0.2029f,   -0.1026f },
    {   -0.9884f,    0.0893f,   -0.1231f },
    {   -0.9955f,   -0.0341f,   -0.0884f },
    {   -0.9943f,   -0.0433f,    0.0973f },
    {   -0.9948f,   -0.0625f,    0.0799f },
    {   -0.2880f,    0.4365f,    0.8524f },
    {   -0.2193f,    0.5723f,    0.7902f },
    {   -0.2412f,    0.6443f,    0.7258f },
    {   -0.2339f,    0.6945f,    0.6804f },
    {   -0.2973f,    0.6207f,    0.7255f },
    {   -0.3871f,   -0.0173f,    0.9219f },
    {   -0.3911f,   -0.0083f,    0.9203f },
    {   -0.2928f,   -0.6391f,    0.7113f },
    {   -0.2420f,   -0.5069f,    0.8274f },
    {   -0.0895f,    0.7157f,    0.6926f },
    {   -0.0918f,    0.7190f,    0.6889f },
    {   -0.1488f,   -0.0121f,    0.9888f },
    {   -0.1531f,   -0.0030f,    0.9882f },
    {   -0.2169f,   -0.6522f,    0.7263f },
    {   -0.1322f,   -0.4570f,    0.8796f },
    {    0.0141f,    0.8981f,   -0.4395f },
    {   -0.0628f,    0.8749f,   -0.4803f },
    {    0.0000f,    0.9992f,    0.0399f },
    {   -0.0005f,    0.9990f,    0.0439f },
    {   -0.1151f,    0.9643f,    0.2386f },
    {   -0.0302f,    0.9811f,    0.1910f },
    {   -0.0344f,    0.9815f,    0.1882f },
    {    0.0148f,   -0.9929f,   -0.1183f },
    {    0.0000f,   -0.9934f,   -0.1147f },
    {    0.0000f,   -0.9934f,   -0.1147f },
    {    0.0000f,   -0.9934f,   -0.1147f },
    {   -0.0148f,   -0.9929f,   -0.1183f },
    {    0.0000f,   -0.9934f,   -0.1147f },
    {    0.0030f,    0.9992f,    0.0393f },
    {   -0.0030f,    0.9993f,    0.0362f },
    {    0.0015f,    0.9992f,    0.0409f },
    {    0.0044f,    0.9994f,    0.0337f },
    {   -0.0050f,    0.9992f,    0.0386f },
    {    0.0310f,    0.7079f,    0.7056f },
    {   -0.0004f,    0.9590f,    0.2833f },
    {   -0.0339f,    0.6622f,    0.7486f },
    {   -0.8043f,    0.5888f,   -0.0805f },
    {   -0.7245f,    0.6530f,   -0.2205f },
    {   -0.6548f,    0.5284f,   -0.5405f },
    {    0.6544f,    0.5292f,   -0.5401f },
    {    0.7248f,    0.6569f,   -0.2079f },
    {    0.8005f,    0.5951f,   -0.0715f },
    {    0.0000f,    0.0215f,   -0.9998f },
    {    0.0004f,    0.0216f,   -0.9998f },
    {    0.0000f,    0.0221f,   -0.9998f },
    {    0.0000f,    0.0216f,   -0.9998f },
    {   -0.0001f,    0.0216f,   -0.9998f },
    {    0.0000f,    0.0215f,   -0.9998f },
    {   -0.0412f,    0.8051f,    0.5917f },
    {   -0.1340f,    0.8006f,    0.5840f },
    {   -0.0005f,    0.8895f,    0.4570f },
    {    0.0003f,    0.8839f,    0.4677f },
    {    0.0846f,    0.8162f,    0.5716f },
    {    0.0341f,    0.7785f,    0.6267f },
    {    0.0000f,    1.0000f,   -0.0006f },
    {   -0.0005f,    1.0000f,    0.0010f },
    {   -0.0006f,   -1.0000f,   -0.0003f },
    {    0.0000f,   -1.0000f,    0.0025f },
    {   -0.7666f,   -0.6421f,   -0.0003f },
    {   -0.7660f,   -0.6429f,    0.0004f },
    {    0.7754f,   -0.6315f,   -0.0024f },
    {    0.7684f,   -0.6400f,    0.0051f },
    {    0.9413f,    0.2476f,    0.2293f },
    {    0.8342f,    0.4678f,    0.2919f },
    {    0.8585f,    0.5022f,   -0.1040f },
    {    0.8284f,    0.5551f,   -0.0750f },
    {    0.8549f,    0.4878f,   -0.1768f },
    {    0.8689f,    0.4944f,    0.0248f },
    {    0.8652f,    0.5008f,    0.0242f },
    {    0.6719f,    0.7344f,    0.0963f },
    {   -0.8342f,    0.4678f,    0.2919f },
    {   -0.9413f,    0.2476f,    0.2293f },
    {   -0.5311f,    0.8184f,    0.2195f },
    {    0.9995f,   -0.0310f,    0.0000f },
    {   -0.1317f,    0.8019f,    0.5828f },
    {   -0.1587f,    0.8259f,    0.5410f },
    {   -0.1903f,   -0.1005f,   -0.9766f },
    {   -0.1501f,    0.0047f,   -0.9887f },
    {   -0.0163f,   -0.8427f,    0.5382f },
    {   -0.0238f,   -0.8345f,    0.5504f },
    {   -0.9995f,   -0.0310f,    0.0000f },
    {    0.1636f,    0.7951f,    0.5840f },
    {    0.1465f,    0.8274f,    0.5422f },
    {    0.1340f,   -0.0601f,   -0.9892f },
    {    0.1761f,    0.0055f,   -0.9844f },
    {    0.0280f,   -0.8444f,    0.5349f },
    {    0.0190f,   -0.8347f,    0.5504f },
    {    0.0000f,   -0.9984f,   -0.0573f },
    {    0.0000f,   -0.9984f,   -0.0573f },
    {    0.0000f,   -0.9986f,   -0.0525f },
    {    0.0000f,   -0.9986f,   -0.0525f },
    {    0.8208f,    0.5691f,    0.0493f },
    {   -0.8277f,    0.5592f,    0.0463f },
    {    0.8729f,    0.4872f,    0.0249f },
    {   -0.8729f,    0.4872f,    0.0249f },
};

const Point3D wheel_vertices[31] = {
    {    0.0850f,    0.0000f,    0.0000f },
    {    0.0850f,    0.1650f,    0.0000f },
    {    0.0850f,    0.2250f,    0.0000f },
    {    0.0850f,    0.1335f,    0.0970f },
    {    0.0850f,    0.1820f,    0.1323f },
    {    0.0850f,    0.0510f,    0.1569f },
    {    0.0850f,    0.0695f,    0.2140f },
    {    0.0850f,   -0.0510f,    0.1569f },
    {    0.0850f,   -0.0695f,    0.2140f },
    {    0.0850f,   -0.1335f,    0.0970f },
    {    0.0850f,   -0.1820f,    0.1323f },
    {    0.0850f,   -0.1650f,    0.0000f },
    {    0.0850f,   -0.2250f,    0.0000f },
    {    0.0850f,   -0.1335f,   -0.0970f },
    {    0.0850f,   -0.1820f,   -0.1323f },
    {    0.0850f,   -0.0510f,   -0.1569f },
    {    0.0850f,   -0.0695f,   -0.2140f },
    {    0.0850f,    0.0510f,   -0.1569f },
    {    0.0850f,    0.0695f,   -0.2140f },
    {    0.0850f,    0.1335f,   -0.0970f },
    {    0.0850f,    0.1820f,   -0.1323f },
    {   -0.0850f,    0.2250f,    0.0000f },
    {   -0.0850f,    0.1820f,    0.1323f },
    {   -0.0850f,    0.0695f,    0.2140f },
    {   -0.0850f,   -0.0695f,    0.2140f },
    {   -0.0850f,   -0.1820f,    0.1323f },
    {   -0.0850f,   -0.2250f,    0.0000f },
    {   -0.0850f,   -0.1820f,   -0.1323f },
    {   -0.0850f,   -0.0695f,   -0.2140f },
    {   -0.0850f,    0.0695f,   -0.2140f },
    {   -0.0850f,    0.1820f,   -0.1323f },
};
const Face wheel_faces[30] = {
    { {  0,  1,  3,  0}, 3, 0, 0xC618 },
    { {  0,  3,  5,  0}, 3, 0, 0x2965 },
    { {  0,  5,  7,  0}, 3, 0, 0xC618 },
    { {  0,  7,  9,  0}, 3, 0, 0x2965 },
    { {  0,  9, 11,  0}, 3, 0, 0xC618 },
    { {  0, 11, 13,  0}, 3, 0, 0x2965 },
    { {  0, 13, 15,  0}, 3, 0, 0xC618 },
    { {  0, 15, 17,  0}, 3, 0, 0x2965 },
    { {  0, 17, 19,  0}, 3, 0, 0xC618 },
    { {  0, 19,  1,  0}, 3, 0, 0x2965 },
    { {  1,  2,  4,  3}, 4, 0, 0x1082 },
    { {  3,  4,  6,  5}, 4, 0, 0x1082 },
    { {  5,  6,  8,  7}, 4, 0, 0x1082 },
    { {  7,  8, 10,  9}, 4, 0, 0x1082 },
    { {  9, 10, 12, 11}, 4, 0, 0x1082 },
    { { 11, 12, 14, 13}, 4, 0, 0x1082 },
    { { 13, 14, 16, 15}, 4, 0, 0x1082 },
    { { 15, 16, 18, 17}, 4, 0, 0x1082 },
    { { 17, 18, 20, 19}, 4, 0, 0x1082 },
    { { 19, 20,  2,  1}, 4, 0, 0x1082 },
    { {  2, 21, 22,  4}, 4, 0, 0x1082 },
    { {  4, 22, 23,  6}, 4, 0, 0x1082 },
    { {  6, 23, 24,  8}, 4, 0, 0x1082 },
    { {  8, 24, 25, 10}, 4, 0, 0x1082 },
    { { 10, 25, 26, 12}, 4, 0, 0x1082 },
    { { 12, 26, 27, 14}, 4, 0, 0x1082 },
    { { 14, 27, 28, 16}, 4, 0, 0x1082 },
    { { 16, 28, 29, 18}, 4, 0, 0x1082 },
    { { 18, 29, 30, 20}, 4, 0, 0x1082 },
    { { 20, 30, 21,  2}, 4, 0, 0x1082 },
};
const Point3D wheel_normals[30] = {
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    1.0000f,    0.0000f,    0.0000f },
    {    0.0000f,    0.9511f,    0.3090f },
    {    0.0000f,    0.5878f,    0.8090f },
    {    0.0000f,    0.0000f,    1.0000f },
    {    0.0000f,   -0.5878f,    0.8090f },
    {    0.0000f,   -0.9511f,    0.3090f },
    {    0.0000f,   -0.9511f,   -0.3090f },
    {    0.0000f,   -0.5878f,   -0.8090f },
    {    0.0000f,   -0.0000f,   -1.0000f },
    {    0.0000f,    0.5878f,   -0.8090f },
    {    0.0000f,    0.9511f,   -0.3090f },
};

// STATUE_NUM_VERTICES 80  STATUE_NUM_FACES 51
const Point3D statue_vertices[80] = {
    {  -0.5250f,   0.0000f,  -0.5250f },
    {   0.5250f,   0.0000f,  -0.5250f },
    {   0.5250f,   0.0000f,   0.5250f },
    {  -0.5250f,   0.0000f,   0.5250f },
    {  -0.5250f,   0.7000f,  -0.5250f },
    {   0.5250f,   0.7000f,  -0.5250f },
    {   0.5250f,   0.7000f,   0.5250f },
    {  -0.5250f,   0.7000f,   0.5250f },
    {  -0.4000f,   0.7000f,  -0.4000f },
    {   0.4000f,   0.7000f,  -0.4000f },
    {   0.4000f,   0.7000f,   0.4000f },
    {  -0.4000f,   0.7000f,   0.4000f },
    {  -0.4000f,   0.8600f,  -0.4000f },
    {   0.4000f,   0.8600f,  -0.4000f },
    {   0.4000f,   0.8600f,   0.4000f },
    {  -0.4000f,   0.8600f,   0.4000f },
    {  -0.2600f,   0.8600f,  -0.2200f },
    {   0.2600f,   0.8600f,  -0.2200f },
    {   0.2600f,   0.8600f,   0.2200f },
    {  -0.2600f,   0.8600f,   0.2200f },
    {  -0.2600f,   1.7600f,  -0.2200f },
    {   0.2600f,   1.7600f,  -0.2200f },
    {   0.2600f,   1.7600f,   0.2200f },
    {  -0.2600f,   1.7600f,   0.2200f },
    {  -0.2300f,   1.7400f,  -0.1800f },
    {   0.2300f,   1.7400f,  -0.1800f },
    {   0.2300f,   1.7400f,   0.1800f },
    {  -0.2300f,   1.7400f,   0.1800f },
    {  -0.2300f,   2.0800f,  -0.1800f },
    {   0.2300f,   2.0800f,  -0.1800f },
    {   0.2300f,   2.0800f,   0.1800f },
    {  -0.2300f,   2.0800f,   0.1800f },
    {  -0.2500f,   2.0600f,  -0.1600f },
    {   0.2500f,   2.0600f,  -0.1600f },
    {   0.2500f,   2.0600f,   0.1800f },
    {  -0.2500f,   2.0600f,   0.1800f },
    {  -0.2500f,   2.5600f,  -0.1600f },
    {   0.2500f,   2.5600f,  -0.1600f },
    {   0.2500f,   2.5600f,   0.1800f },
    {  -0.2500f,   2.5600f,   0.1800f },
    {  -0.0800f,   2.5500f,  -0.0600f },
    {   0.0800f,   2.5500f,  -0.0600f },
    {   0.0800f,   2.5500f,   0.1000f },
    {  -0.0800f,   2.5500f,   0.1000f },
    {  -0.0800f,   2.7300f,  -0.0600f },
    {   0.0800f,   2.7300f,  -0.0600f },
    {   0.0800f,   2.7300f,   0.1000f },
    {  -0.0800f,   2.7300f,   0.1000f },
    {  -0.1200f,   2.7100f,  -0.0900f },
    {   0.1200f,   2.7100f,  -0.0900f },
    {   0.1200f,   2.7100f,   0.1500f },
    {  -0.1200f,   2.7100f,   0.1500f },
    {  -0.1200f,   2.9700f,  -0.0900f },
    {   0.1200f,   2.9700f,  -0.0900f },
    {   0.1200f,   2.9700f,   0.1500f },
    {  -0.1200f,   2.9700f,   0.1500f },
    {  -0.3950f,   1.8500f,  -0.0550f },
    {  -0.2650f,   1.8500f,  -0.0550f },
    {  -0.2650f,   1.8500f,   0.0950f },
    {  -0.3950f,   1.8500f,   0.0950f },
    {  -0.3950f,   2.4700f,  -0.0550f },
    {  -0.2650f,   2.4700f,  -0.0550f },
    {  -0.2650f,   2.4700f,   0.0950f },
    {  -0.3950f,   2.4700f,   0.0950f },
    {   0.1800f,   2.3450f,  -0.0200f },
    {   0.4800f,   2.3450f,  -0.0200f },
    {   0.4800f,   2.3450f,   0.1200f },
    {   0.1800f,   2.3450f,   0.1200f },
    {   0.1800f,   2.4750f,  -0.0200f },
    {   0.4800f,   2.4750f,  -0.0200f },
    {   0.4800f,   2.4750f,   0.1200f },
    {   0.1800f,   2.4750f,   0.1200f },
    {   0.4150f,   2.5100f,   0.0000f },
    {   0.5450f,   2.5100f,   0.0000f },
    {   0.5450f,   2.5100f,   0.1400f },
    {   0.4150f,   2.5100f,   0.1400f },
    {   0.4150f,   2.9300f,   0.0000f },
    {   0.5450f,   2.9300f,   0.0000f },
    {   0.5450f,   2.9300f,   0.1400f },
    {   0.4150f,   2.9300f,   0.1400f },
};
const Face statue_faces[51] = {
    { {  4,  5,  6,  7}, 4, 0, 0xAD75 },
    { {  3,  7,  6,  2}, 4, 0, 0xAD75 },
    { {  1,  5,  4,  0}, 4, 0, 0xAD75 },
    { {  2,  6,  5,  1}, 4, 0, 0xAD75 },
    { {  0,  4,  7,  3}, 4, 0, 0xAD75 },
    { { 12, 13, 14, 15}, 4, 0, 0xE71C },
    { {  8, 11, 10,  9}, 4, 0, 0xE71C },
    { { 11, 15, 14, 10}, 4, 0, 0xE71C },
    { {  9, 13, 12,  8}, 4, 0, 0xE71C },
    { { 10, 14, 13,  9}, 4, 0, 0xE71C },
    { {  8, 12, 15, 11}, 4, 0, 0xE71C },
    { { 20, 21, 22, 23}, 4, 0, 0xCE79 },
    { { 19, 23, 22, 18}, 4, 0, 0xCE79 },
    { { 17, 21, 20, 16}, 4, 0, 0xCE79 },
    { { 18, 22, 21, 17}, 4, 0, 0xCE79 },
    { { 16, 20, 23, 19}, 4, 0, 0xCE79 },
    { { 28, 29, 30, 31}, 4, 0, 0xCE79 },
    { { 27, 31, 30, 26}, 4, 0, 0xCE79 },
    { { 25, 29, 28, 24}, 4, 0, 0xCE79 },
    { { 26, 30, 29, 25}, 4, 0, 0xCE79 },
    { { 24, 28, 31, 27}, 4, 0, 0xCE79 },
    { { 36, 37, 38, 39}, 4, 0, 0xE71C },
    { { 35, 39, 38, 34}, 4, 0, 0xE71C },
    { { 33, 37, 36, 32}, 4, 0, 0xE71C },
    { { 34, 38, 37, 33}, 4, 0, 0xE71C },
    { { 32, 36, 39, 35}, 4, 0, 0xE71C },
    { { 44, 45, 46, 47}, 4, 0, 0xE71C },
    { { 43, 47, 46, 42}, 4, 0, 0xE71C },
    { { 41, 45, 44, 40}, 4, 0, 0xE71C },
    { { 42, 46, 45, 41}, 4, 0, 0xE71C },
    { { 40, 44, 47, 43}, 4, 0, 0xE71C },
    { { 52, 53, 54, 55}, 4, 0, 0xE71C },
    { { 51, 55, 54, 50}, 4, 0, 0xE71C },
    { { 49, 53, 52, 48}, 4, 0, 0xE71C },
    { { 50, 54, 53, 49}, 4, 0, 0xE71C },
    { { 48, 52, 55, 51}, 4, 0, 0xE71C },
    { { 60, 61, 62, 63}, 4, 0, 0xE71C },
    { { 59, 63, 62, 58}, 4, 0, 0xE71C },
    { { 57, 61, 60, 56}, 4, 0, 0xE71C },
    { { 58, 62, 61, 57}, 4, 0, 0xE71C },
    { { 56, 60, 63, 59}, 4, 0, 0xE71C },
    { { 68, 69, 70, 71}, 4, 0, 0xE71C },
    { { 67, 71, 70, 66}, 4, 0, 0xE71C },
    { { 65, 69, 68, 64}, 4, 0, 0xE71C },
    { { 66, 70, 69, 65}, 4, 0, 0xE71C },
    { { 64, 68, 71, 67}, 4, 0, 0xE71C },
    { { 76, 77, 78, 79}, 4, 0, 0xE71C },
    { { 75, 79, 78, 74}, 4, 0, 0xE71C },
    { { 73, 77, 76, 72}, 4, 0, 0xE71C },
    { { 74, 78, 77, 73}, 4, 0, 0xE71C },
    { { 72, 76, 79, 75}, 4, 0, 0xE71C },
};
const Point3D statue_normals[51] = {
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,  -1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
};
// RAILING_NUM_VERTICES 56  RAILING_NUM_FACES 35
const Point3D railing_vertices[56] = {
    {  -6.0000f,   0.2700f,  -0.1400f },
    {   6.0000f,   0.2700f,  -0.1400f },
    {   6.0000f,   0.2700f,   0.1400f },
    {  -6.0000f,   0.2700f,   0.1400f },
    {  -6.0000f,   0.5700f,  -0.1400f },
    {   6.0000f,   0.5700f,  -0.1400f },
    {   6.0000f,   0.5700f,   0.1400f },
    {  -6.0000f,   0.5700f,   0.1400f },
    {  -6.0000f,   0.7200f,  -0.2000f },
    {   6.0000f,   0.7200f,  -0.2000f },
    {   6.0000f,   0.7200f,   0.2000f },
    {  -6.0000f,   0.7200f,   0.2000f },
    {  -6.0000f,   0.8400f,  -0.2000f },
    {   6.0000f,   0.8400f,  -0.2000f },
    {   6.0000f,   0.8400f,   0.2000f },
    {  -6.0000f,   0.8400f,   0.2000f },
    {  -4.8900f,   0.4800f,  -0.1000f },
    {  -4.7100f,   0.4800f,  -0.1000f },
    {  -4.7100f,   0.4800f,   0.1000f },
    {  -4.8900f,   0.4800f,   0.1000f },
    {  -4.8900f,   0.7200f,  -0.1000f },
    {  -4.7100f,   0.7200f,  -0.1000f },
    {  -4.7100f,   0.7200f,   0.1000f },
    {  -4.8900f,   0.7200f,   0.1000f },
    {  -2.4900f,   0.4800f,  -0.1000f },
    {  -2.3100f,   0.4800f,  -0.1000f },
    {  -2.3100f,   0.4800f,   0.1000f },
    {  -2.4900f,   0.4800f,   0.1000f },
    {  -2.4900f,   0.7200f,  -0.1000f },
    {  -2.3100f,   0.7200f,  -0.1000f },
    {  -2.3100f,   0.7200f,   0.1000f },
    {  -2.4900f,   0.7200f,   0.1000f },
    {  -0.0900f,   0.4800f,  -0.1000f },
    {   0.0900f,   0.4800f,  -0.1000f },
    {   0.0900f,   0.4800f,   0.1000f },
    {  -0.0900f,   0.4800f,   0.1000f },
    {  -0.0900f,   0.7200f,  -0.1000f },
    {   0.0900f,   0.7200f,  -0.1000f },
    {   0.0900f,   0.7200f,   0.1000f },
    {  -0.0900f,   0.7200f,   0.1000f },
    {   2.3100f,   0.4800f,  -0.1000f },
    {   2.4900f,   0.4800f,  -0.1000f },
    {   2.4900f,   0.4800f,   0.1000f },
    {   2.3100f,   0.4800f,   0.1000f },
    {   2.3100f,   0.7200f,  -0.1000f },
    {   2.4900f,   0.7200f,  -0.1000f },
    {   2.4900f,   0.7200f,   0.1000f },
    {   2.3100f,   0.7200f,   0.1000f },
    {   4.7100f,   0.4800f,  -0.1000f },
    {   4.8900f,   0.4800f,  -0.1000f },
    {   4.8900f,   0.4800f,   0.1000f },
    {   4.7100f,   0.4800f,   0.1000f },
    {   4.7100f,   0.7200f,  -0.1000f },
    {   4.8900f,   0.7200f,  -0.1000f },
    {   4.8900f,   0.7200f,   0.1000f },
    {   4.7100f,   0.7200f,   0.1000f },
};
const Face railing_faces[35] = {
    { {  4,  5,  6,  7}, 4, 0, 0x9CD3 },
    { {  3,  7,  6,  2}, 4, 0, 0x9CD3 },
    { {  1,  5,  4,  0}, 4, 0, 0x9CD3 },
    { {  2,  6,  5,  1}, 4, 0, 0x9CD3 },
    { {  0,  4,  7,  3}, 4, 0, 0x9CD3 },
    { { 12, 13, 14, 15}, 4, 0, 0xDEDB },
    { { 11, 15, 14, 10}, 4, 0, 0xDEDB },
    { {  9, 13, 12,  8}, 4, 0, 0xDEDB },
    { { 10, 14, 13,  9}, 4, 0, 0xDEDB },
    { {  8, 12, 15, 11}, 4, 0, 0xDEDB },
    { { 20, 21, 22, 23}, 4, 0, 0xDEDB },
    { { 19, 23, 22, 18}, 4, 0, 0xDEDB },
    { { 17, 21, 20, 16}, 4, 0, 0xDEDB },
    { { 18, 22, 21, 17}, 4, 0, 0xDEDB },
    { { 16, 20, 23, 19}, 4, 0, 0xDEDB },
    { { 28, 29, 30, 31}, 4, 0, 0xDEDB },
    { { 27, 31, 30, 26}, 4, 0, 0xDEDB },
    { { 25, 29, 28, 24}, 4, 0, 0xDEDB },
    { { 26, 30, 29, 25}, 4, 0, 0xDEDB },
    { { 24, 28, 31, 27}, 4, 0, 0xDEDB },
    { { 36, 37, 38, 39}, 4, 0, 0xDEDB },
    { { 35, 39, 38, 34}, 4, 0, 0xDEDB },
    { { 33, 37, 36, 32}, 4, 0, 0xDEDB },
    { { 34, 38, 37, 33}, 4, 0, 0xDEDB },
    { { 32, 36, 39, 35}, 4, 0, 0xDEDB },
    { { 44, 45, 46, 47}, 4, 0, 0xDEDB },
    { { 43, 47, 46, 42}, 4, 0, 0xDEDB },
    { { 41, 45, 44, 40}, 4, 0, 0xDEDB },
    { { 42, 46, 45, 41}, 4, 0, 0xDEDB },
    { { 40, 44, 47, 43}, 4, 0, 0xDEDB },
    { { 52, 53, 54, 55}, 4, 0, 0xDEDB },
    { { 51, 55, 54, 50}, 4, 0, 0xDEDB },
    { { 49, 53, 52, 48}, 4, 0, 0xDEDB },
    { { 50, 54, 53, 49}, 4, 0, 0xDEDB },
    { { 48, 52, 55, 51}, 4, 0, 0xDEDB },
};
const Point3D railing_normals[35] = {
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
    {   0.0000f,   1.0000f,   0.0000f },
    {   0.0000f,   0.0000f,   1.0000f },
    {   0.0000f,   0.0000f,  -1.0000f },
    {   1.0000f,   0.0000f,   0.0000f },
    {  -1.0000f,   0.0000f,   0.0000f },
};

const Point3D lod_car_vertices[50] = {
    { -0.4600f,  0.0600f,  1.4000f },
    { -0.5000f,  0.3400f,  1.3200f },
    { -0.6000f,  0.4000f,  0.5500f },
    { -0.5200f,  0.7400f,  0.0500f },
    { -0.5400f,  0.7400f, -0.4500f },
    { -0.6400f,  0.5000f, -0.8800f },
    { -0.6000f,  0.4600f, -1.3000f },
    { -0.5600f,  0.0600f, -1.3800f },
    {  0.4600f,  0.0600f,  1.4000f },
    {  0.5000f,  0.3400f,  1.3200f },
    {  0.6000f,  0.4000f,  0.5500f },
    {  0.5200f,  0.7400f,  0.0500f },
    {  0.5400f,  0.7400f, -0.4500f },
    {  0.6400f,  0.5000f, -0.8800f },
    {  0.6000f,  0.4600f, -1.3000f },
    {  0.5600f,  0.0600f, -1.3800f },
    { -0.5000f,  0.6000f, -1.1600f },
    {  0.5000f,  0.6000f, -1.1600f },
    {  0.5000f,  0.6600f, -1.4000f },
    { -0.5000f,  0.6600f, -1.4000f },
    { -0.4400f,  0.4600f, -1.2400f },
    { -0.4700f,  0.6100f, -1.2000f },
    { -0.4700f,  0.6500f, -1.3800f },
    {  0.4400f,  0.4600f, -1.2400f },
    {  0.4700f,  0.6100f, -1.2000f },
    {  0.4700f,  0.6500f, -1.3800f },
    { -0.6600f,  0.3700f,  1.0752f },
    { -0.6600f,  0.5000f,  0.8500f },
    { -0.6600f,  0.3700f,  0.6248f },
    { -0.6600f,  0.1100f,  0.6248f },
    { -0.6600f, -0.0200f,  0.8500f },
    { -0.6600f,  0.1100f,  1.0752f },
    { -0.6600f,  0.3700f, -0.6248f },
    { -0.6600f,  0.5000f, -0.8500f },
    { -0.6600f,  0.3700f, -1.0752f },
    { -0.6600f,  0.1100f, -1.0752f },
    { -0.6600f, -0.0200f, -0.8500f },
    { -0.6600f,  0.1100f, -0.6248f },
    {  0.6600f,  0.3700f,  1.0752f },
    {  0.6600f,  0.5000f,  0.8500f },
    {  0.6600f,  0.3700f,  0.6248f },
    {  0.6600f,  0.1100f,  0.6248f },
    {  0.6600f, -0.0200f,  0.8500f },
    {  0.6600f,  0.1100f,  1.0752f },
    {  0.6600f,  0.3700f, -0.6248f },
    {  0.6600f,  0.5000f, -0.8500f },
    {  0.6600f,  0.3700f, -1.0752f },
    {  0.6600f,  0.1100f, -1.0752f },
    {  0.6600f, -0.0200f, -0.8500f },
    {  0.6600f,  0.1100f, -0.6248f }
};

const Face lod_car_faces[30] = {
    { {  0,  8,  9,  1}, 4, 0, 0xFFFF },
    { {  1,  9, 10,  2}, 4, 0, 0xFFFF },
    { {  2, 10, 11,  3}, 4, 0, 0x0008 },
    { {  3, 11, 12,  4}, 4, 0, 0xFFFF },
    { {  4, 12, 13,  5}, 4, 0, 0x0008 },
    { {  5, 13, 14,  6}, 4, 0, 0xFFFF },
    { {  6, 14, 15,  7}, 4, 0, 0x3186 },
    { {  7,  0,  1,  0}, 3, 0, 0xFFFF },
    { { 15,  9,  8,  0}, 3, 0, 0xFFFF },
    { {  7,  1,  2,  0}, 3, 0, 0xFFFF },
    { { 15, 10,  9,  0}, 3, 0, 0xFFFF },
    { {  7,  2,  3,  0}, 3, 0, 0xFFFF },
    { { 15, 11, 10,  0}, 3, 0, 0xFFFF },
    { {  7,  3,  4,  0}, 3, 0, 0xFFFF },
    { { 15, 12, 11,  0}, 3, 0, 0xFFFF },
    { {  7,  4,  5,  0}, 3, 0, 0xFFFF },
    { { 15, 13, 12,  0}, 3, 0, 0xFFFF },
    { {  7,  5,  6,  0}, 3, 0, 0xFFFF },
    { { 15, 14, 13,  0}, 3, 0, 0xFFFF },
    { { 16, 19, 18, 17}, 4, 1, 0x18C3 },
    { { 20, 21, 22,  0}, 3, 1, 0x18C3 },
    { { 23, 24, 25,  0}, 3, 1, 0x18C3 },
    { { 26, 27, 28, 29}, 4, 1, 0x0000 },
    { { 26, 29, 30, 31}, 4, 1, 0x0000 },
    { { 32, 33, 34, 35}, 4, 1, 0x0000 },
    { { 32, 35, 36, 37}, 4, 1, 0x0000 },
    { { 38, 39, 40, 41}, 4, 1, 0x0000 },
    { { 38, 41, 42, 43}, 4, 1, 0x0000 },
    { { 44, 45, 46, 47}, 4, 1, 0x0000 },
    { { 44, 47, 48, 49}, 4, 1, 0x0000 }
};

const Point3D lod_car_normals[30] = {
    {  0.0000f,  0.2747f,  0.9615f },
    {  0.0000f,  0.9970f,  0.0777f },
    {  0.0000f,  0.8269f,  0.5623f },
    {  0.0000f,  1.0000f,  0.0000f },
    {  0.0000f,  0.8732f, -0.4874f },
    {  0.0000f,  0.9955f, -0.0948f },
    {  0.0000f,  0.1961f, -0.9806f },
    { -0.9907f, -0.1313f,  0.0356f },
    {  0.9907f, -0.1313f,  0.0356f },
    { -0.8580f, -0.5085f,  0.0718f },
    {  0.8580f, -0.5085f,  0.0718f },
    { -0.9859f,  0.1604f, -0.0487f },
    {  0.9859f,  0.1604f, -0.0487f },
    { -0.9989f, -0.0253f,  0.0400f },
    {  0.9989f, -0.0253f,  0.0400f },
    { -0.5487f, -0.6692f,  0.5011f },
    {  0.5487f, -0.6692f,  0.5011f },
    { -0.9929f, -0.0819f, -0.0868f },
    {  0.9929f, -0.0819f, -0.0868f },
    {  0.0000f, -0.9701f, -0.2425f },
    { -0.9818f, -0.1854f, -0.0412f },
    { -0.9818f,  0.1854f,  0.0412f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f }
};

const Point3D billboard_vertices[8] = {
    { -1.5000f,  0.0000f,  0.0000f },
    { -1.5000f,  1.4000f,  0.0000f },
    {  1.5000f,  0.0000f,  0.0000f },
    {  1.5000f,  1.4000f,  0.0000f },
    { -1.8000f,  1.3000f,  0.0000f },
    {  1.8000f,  1.3000f,  0.0000f },
    {  1.8000f,  2.4000f,  0.0000f },
    { -1.8000f,  2.4000f,  0.0000f }
};

const Face billboard_faces[1] = {
    { {  4,  5,  6,  7}, 4, 1, 0xFFFF }
};

const Point3D billboard_normals[1] = {
    {  0.0000f,  0.0000f,  1.0000f }
};

const Point3D bridge_vertices[24] = {
    { -3.4000f,  0.0000f, -0.4000f },
    { -2.9000f,  0.0000f, -0.4000f },
    { -2.9000f,  3.5000f, -0.4000f },
    { -3.4000f,  3.5000f, -0.4000f },
    { -3.4000f,  0.0000f,  0.4000f },
    { -2.9000f,  0.0000f,  0.4000f },
    { -2.9000f,  3.5000f,  0.4000f },
    { -3.4000f,  3.5000f,  0.4000f },
    {  2.9000f,  0.0000f, -0.4000f },
    {  3.4000f,  0.0000f, -0.4000f },
    {  3.4000f,  3.5000f, -0.4000f },
    {  2.9000f,  3.5000f, -0.4000f },
    {  2.9000f,  0.0000f,  0.4000f },
    {  3.4000f,  0.0000f,  0.4000f },
    {  3.4000f,  3.5000f,  0.4000f },
    {  2.9000f,  3.5000f,  0.4000f },
    { -3.4000f,  3.5000f, -0.4000f },
    {  3.4000f,  3.5000f, -0.4000f },
    {  3.4000f,  4.3000f, -0.4000f },
    { -3.4000f,  4.3000f, -0.4000f },
    { -3.4000f,  3.5000f,  0.4000f },
    {  3.4000f,  3.5000f,  0.4000f },
    {  3.4000f,  4.3000f,  0.4000f },
    { -3.4000f,  4.3000f,  0.4000f }
};

const Face bridge_faces[12] = {
    { {  0,  3,  2,  1}, 4, 1, 0x5AEB },
    { {  4,  5,  6,  7}, 4, 1, 0x5AEB },
    { {  0,  4,  7,  3}, 4, 1, 0x3186 },
    { {  1,  2,  6,  5}, 4, 1, 0x3186 },
    { {  8, 11, 10,  9}, 4, 1, 0x5AEB },
    { { 12, 13, 14, 15}, 4, 1, 0x5AEB },
    { {  8, 12, 15, 11}, 4, 1, 0x3186 },
    { {  9, 10, 14, 13}, 4, 1, 0x3186 },
    { { 16, 19, 18, 17}, 4, 1, 0xF800 },
    { { 20, 21, 22, 23}, 4, 1, 0xF800 },
    { { 19, 23, 22, 18}, 4, 1, 0xFBE0 },
    { { 16, 17, 21, 20}, 4, 1, 0x5AEB }
};

const Point3D bridge_normals[12] = {
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    {  0.0000f,  1.0000f,  0.0000f },
    {  0.0000f, -1.0000f,  0.0000f }
};

// ===== END GENERATED MODEL DATA =====
