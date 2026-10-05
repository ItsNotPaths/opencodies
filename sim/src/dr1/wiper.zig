// the windscreen wiper (car+0x5b4): visual, but its noise draws from the car's generator, which the engine shares
const math = @import("math.zig");
const engine = @import("engine.zig");

// class Wiper
pub const Wiper = extern struct {
    visual_offset: f32, // m_sweepVisualOffset: the phase of this wiper
    pos: f32, // m_sweepPos 0..2: out and back
    rate: f32, // m_sweepRate, 0 while holding
    time_at_rate: f32, // m_sweepTimeAtThisRate
    hold_time: f32, // > 0: stop at each end for this long
    sweep_time: f32, // 0: never sweeps
    noise: f32, // m_noisePosition
    noise_target: f32,
    noise_time: f32, // m_noiseTimeTillTargetChange
    forward: bool, // m_sweepForward
    only_when_moving: bool, // m_onlySweepsWhenCarMoving: from rest only above 4 m/s
};

// 0xf6f210 Wiper::update
pub fn update(w: *Wiper, random: *u32, dt: f32, speed: f32) void {
    w.noise_time = w.noise_time - @abs(dt);
    if (0 > w.noise_time) {
        const s = speed / 40;
        const k = if (s > 1) 1 else math.maxss(s, 0);
        w.noise_target = (@as(f32, @floatFromInt(engine.random(random) & 0xff)) - 127.5) * k * 3.9215687e-05;
        w.noise_time = @as(f32, @floatFromInt(engine.random(random) & 0xff)) * 0.09 / 255 + 0.01;
    }
    const q = @abs(dt) * 15;
    const k: f32 = if (q > 1) 1 else if (0 < q) q else 0;
    w.noise = k * (w.noise_target - w.noise) + w.noise;
    var phase = w.visual_offset + w.pos;
    if (phase > 2) phase = phase - 2;
    if (phase > 1) phase = 2 - phase;
    var x = w.noise + phase;
    var pace: f32 = undefined; // of the sweep rate: slow at the ends
    if (x > 1) {
        x = 0;
        pace = held(w.*, x);
    } else if (x > 0) {
        if (x > 0.5) x = 1 - x;
        pace = held(w.*, x);
    } else {
        pace = if (w.hold_time > 0) held(w.*, 0) else 0.1;
    }
    w.forward = 1 >= w.pos;
    const pos = pace * w.rate * dt + w.pos;
    w.pos = pos;
    const t = w.time_at_rate + dt;
    w.time_at_rate = if (t < 0) 0 else t;
    if (w.rate == 0) {
        if (w.time_at_rate > w.hold_time and w.sweep_time != 0 and (!w.only_when_moving or speed > 4)) {
            w.time_at_rate = 0;
            w.rate = 1 / w.sweep_time;
        }
        return;
    }
    if (!(pos >= 2) and !(0 > pos)) return;
    w.time_at_rate = 0;
    if (w.hold_time > 0 or w.sweep_time == 0) {
        w.pos = 0;
        w.rate = 0;
        return;
    }
    w.rate = 1 / w.sweep_time;
    w.pos = if (pos >= 2) pos - 2 else pos + 2;
}

// the pace near an end: x from the nearest end (with a hold, also limited by the position)
fn held(w: Wiper, x: f32) f32 {
    var y = x;
    if (w.hold_time > 0) y = math.minss(2 - w.pos, math.minss(w.pos, x));
    const s = @sqrt(y * 4) * 0.9 + 0.1;
    return if (s > 1) 1 else if (0 < s) s else 0;
}
