const d3math = @import("d3math.zig");

pub const V3 = @Vector(3, f32);

// 0x4818b0: rows of the right-handed rotation about a unit axis, applied as toWorld(m, v)
pub fn rotation(axis: V3, angle: f32) [3]V3 {
    const s, const c = d3math.sinCos(angle);
    const sa = axis * splat(s);
    const skew = [3]V3{ .{ c, sa[2], -sa[1] }, .{ -sa[2], c, sa[0] }, .{ sa[1], -sa[0], c } };
    var m: [3]V3 = undefined;
    inline for (&m, skew, 0..) |*row, k, i| row.* = k + splat(axis[i]) * axis * splat(1 - c);
    return m;
}

pub fn toBody(axes: [3]V3, v: V3) V3 {
    return .{ dot(axes[0], v), dot(axes[1], v), dot(axes[2], v) };
}

pub fn toWorld(axes: [3]V3, v: V3) V3 {
    return axes[1] * splat(v[1]) + (axes[2] * splat(v[2]) + axes[0] * splat(v[0])); // sum order of 0x7e9e00
}

pub fn splat(x: f32) V3 {
    return @splat(x);
}

pub fn dot(a: V3, b: V3) f32 {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; // SSE sum order of D3
}

// D3's SSE normalize: a horizontal sum in every lane, so y sums in another order; then rsqrtps and one
// Newton step (the emulator's rsqrtps is exact, real hardware has 12 bits)
pub fn normalize(v: V3) V3 {
    const sq = v * v;
    const xz = invSqrt((sq[0] + sq[1]) + sq[2]);
    return .{ xz * v[0], invSqrt(sq[0] + (sq[1] + sq[2])) * v[1], xz * v[2] };
}

// D3's SSE length: s = rsqrt(x)·x refined by one Newton step; 0 stays 0
pub fn length(v: V3) f32 {
    const x = dot(v, v);
    if (x == 0) return 0;
    const r = 1 / @sqrt(x);
    const s = r * x;
    return (1 - s * r) * (s * 0.5) + s;
}

fn invSqrt(x: f32) f32 {
    const r = 1 / @sqrt(x);
    return (1 - r * x * r) * (r * 0.5) + r;
}

pub fn cross(a: V3, b: V3) V3 {
    return .{ a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
}
