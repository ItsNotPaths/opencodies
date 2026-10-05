// glibc's float libm as the emulator runs it: in f64, rounded once to f32
const std = @import("std");

// SSE maxss: a unless b is larger; NaN in either gives b
pub fn maxss(a: f32, b: f32) f32 {
    return if (a > b) a else b;
}

// 1, -1 or 0 (also for NaN), as the game's mask code builds it
pub fn sign(x: f32) f32 {
    return if (x > 0) 1 else if (x < 0) -1 else 0;
}

pub fn pow(x: f32, y: f32) f32 {
    return @floatCast(std.math.pow(f64, x, y));
}

pub fn log(x: f32) f32 {
    return @floatCast(@log(@as(f64, x)));
}

pub fn atan(x: f32) f32 {
    return @floatCast(std.math.atan(@as(f64, x)));
}

pub fn atan2(y: f32, x: f32) f32 {
    return @floatCast(std.math.atan2(@as(f64, y), @as(f64, x)));
}

pub fn cos(x: f32) f32 {
    return @floatCast(@cos(@as(f64, x)));
}

pub fn sin(x: f32) f32 {
    return @floatCast(@sin(@as(f64, x)));
}

pub fn tan(x: f32) f32 {
    return @floatCast(@tan(@as(f64, x)));
}

pub fn asin(x: f32) f32 {
    return @floatCast(std.math.asin(@as(f64, x)));
}

pub fn acos(x: f32) f32 {
    return @floatCast(std.math.acos(@as(f64, x)));
}

// SSE minss: a unless b is smaller; NaN in either gives b
pub fn minss(a: f32, b: f32) f32 {
    return if (a < b) a else b;
}

pub fn exp(x: f32) f32 {
    return @floatCast(@exp(@as(f64, x)));
}

// cvttss2si: NaN and out of range give 0x80000000
pub fn truncate(x: f32) i32 {
    return if (x > -0x1p31 and x < 0x1p31) @intFromFloat(x) else std.math.minInt(i32);
}
