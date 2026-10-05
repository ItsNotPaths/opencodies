const std = @import("std");
const ByTime = @FieldType(@import("../material.zig").Bumps, "by_time");

pub const Gain = struct { surface_factor: f32, amplification: f32 }; // car+0x16e8, car+0x16ec

// 0x7f4740: road noise for wheel i, time based on every material; the "ls" layer is 0 (no ls_amp anywhere)
pub fn height(gain: Gain, m: ByTime, wheel: usize, time: f32, speed: f32) f32 {
    const n: f32 = @floatFromInt(noise(time / m.wavelength, @floatFromInt(wheel)));
    const fade = @min(speed * 0.1, 1);
    const share = gain.surface_factor * m.car_share + (1 - m.car_share);
    return n * (m.magnitude * fade) * (1.0 / 128.0) * share * gain.amplification;
}

// 2D value noise on a wrapping 256 x 256 lattice, bilinear with 8-bit weights: -128..127
fn noise(u: f32, v: f32) i32 {
    const ui = truncate(u * 256);
    const vi = truncate(v * 256);
    const row = (ui & 0xffff) >> 8;
    const col = (vi & 0xffff) >> 8;
    const fu = ui & 0xff;
    const fv = vi & 0xff;
    const r0 = row * 256;
    const r1 = ((row + 1) & 0xff) * 256;
    const c1 = (col + 1) & 0xff;
    const a = (lattice(col + r1) * fu + lattice(col + r0) * (256 - fu)) >> 8;
    const b = (lattice(c1 + r1) * fu + lattice(c1 + r0) * (256 - fu)) >> 8;
    return ((a * (256 - fv) + b * fv) >> 8) - 128;
}

// cvttss2si: out of range or NaN gives the "integer indefinite" 0x80000000 (a material without a wavelength)
fn truncate(x: f32) i32 {
    return if (x > -0x1p31 and x < 0x1p31) @intFromFloat(x) else std.math.minInt(i32);
}

fn lattice(n: i32) i32 {
    const x = @sqrt(@as(f32, @floatFromInt(n)) + 2.158833) * 987345.625;
    return @as(i32, @intFromFloat(x)) & 0xff;
}

test "noise stays in its range and repeats every 256 steps" {
    for (0..200) |k| {
        const u = @as(f32, @floatFromInt(k)) * 0.375;
        const n = noise(u, 3);
        try std.testing.expect(n >= -128 and n <= 127);
        try std.testing.expectEqual(n, noise(u + 256, 3));
    }
}
