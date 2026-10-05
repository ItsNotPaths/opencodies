const std = @import("std");

// MSVC CRT sinf 0xd41afe, cosf 0xd419a7: a 64-step table of sin and cos plus a short polynomial, in f64
pub fn sin(x: f32) f32 {
    return tableTrig(x, 0);
}

pub fn cos(x: f32) f32 {
    return tableTrig(x, 16);
}

const sixteenths = [17]f64{ // sin(k * 2pi/64), k = 0..16
    0,                    0x1.917a6bc29b42cp-4, 0x1.8f8b83c69a60bp-3, 0x1.294062ed59f06p-2,
    0x1.87de2a6aea963p-2, 0x1.e2b5d3806f63bp-2, 0x1.1c73b39ae68c8p-1, 0x1.44cf325091dd6p-1,
    0x1.6a09e667f3bcdp-1, 0x1.8bc806b151741p-1, 0x1.a9b66290ea1a3p-1, 0x1.c38b2f180bdb1p-1,
    0x1.d906bcf328d46p-1, 0x1.e9f4156c62ddap-1, 0x1.f6297cff75cb0p-1, 0x1.fd88da3d12526p-1,
    0x1.0000000000000p+0,
};

const sine_table: [64]f64 = blk: { // the table holds +0, never -0
    var t: [64]f64 = undefined;
    for (&t, 0..) |*v, i| {
        const k = i % 16;
        v.* = switch (i / 16) {
            0 => sixteenths[k],
            1 => sixteenths[16 - k],
            2 => 0 - sixteenths[k],
            else => 0 - sixteenths[16 - k],
        };
    }
    break :blk t;
};

fn tableTrig(x: f32, quarter: u32) f32 {
    const hi = (@as(u32, @bitCast(x)) >> 16) & 0x7fff;
    if (hi < 0x80) return if (quarter == 0) @floatCast(@as(f64, x) * 0x1.fffffff8p-1) else @floatCast(1 - @as(f64, x) * x);
    if (hi > 0x45ff) return if (std.math.isFinite(x)) @floatCast(if (quarter == 0) @sin(@as(f64, x)) else @cos(@as(f64, x))) else x - x;
    const steps = x * 0x1.45f306p3; // 64/2pi
    const n = (steps + 0x1.8p23) - 0x1.8p23; // nearest, ties to even
    const i = (@as(u32, @bitCast(@as(i32, @intFromFloat(n)))) +% quarter) & 63;
    const head = @as(f64, x) - @as(f64, n) * 0x1.921fb54444000p-4; // 2pi/64 in two parts
    const r = @as(f64, n) * 0x1.2e7b9676733afp-44 + head;
    const r2 = head * head;
    const r4 = r2 * r2;
    const c = (r2 * -0.5 + 1) + r4 * 0x1.5555555555555p-5;
    const s = (r2 * -0x1.5555555555555p-3 + 1) + r4 * 0x1.1111111111111p-7;
    return @floatCast(sine_table[i] * c + (r * sine_table[(i + 16) & 63]) * s);
}

// MSVC CRT powf 0xd454e5: log2 from a 65-step table, exp2 from a 32-step table, in f64
pub fn pow(x: f32, y: f32) f32 {
    const bits: u32 = @bitCast(x);
    const hi = bits >> 16;
    if (hi < 0x80 or hi > 0x7f7f or !std.math.isFinite(y)) return std.math.pow(f32, x, y); // x zero, denormal, negative or huge: not ported
    const k = ((hi & 0x7f) + 1) >> 1;
    const m: f64 = @bitCast(@as(u64, bits & 0x7fffff) << 29 | 0x3ff0000000000000); // mantissa in [1, 2)
    const exponent: f64 = @floatFromInt((@as(i32, @intCast(hi)) - 0x3f3f) >> 7);
    const inv = log_table[k][0];
    const inv_hi: f64 = @bitCast(@as(u64, @bitCast(inv)) >> 26 << 26);
    const r = m * (inv - inv_hi) + (inv_hi * m - log_table[0][0]);
    const r2 = r * r;
    const r4 = r2 * r2;
    const scaled_y = @as(f64, y) * 32;
    const v = (((log_table[k][1] + r) + exponent) + -0x1.62e42fefa39efp-2 * r2) * scaled_y; // 32 log2(x^y)
    const v_exponent = (@as(u64, @bitCast(v)) >> 52) & 0x7ff;
    if (v_exponent < 0x3e6) return 1;
    const nearest = (v + 0x1.8p52) - 0x1.8p52;
    if (v_exponent > 0x41d or @abs(nearest) >= 0x2000) return if (v > 0) std.math.inf(f32) else 0;
    const n: i32 = @intFromFloat(nearest);
    const tail = (0x1.7a3341fac5e29p-5 * r + -0x1.55046a1437890p-4) * r4 + (0x1.47fd3ffac83b4p-3 * r) * r2;
    const f = (v - nearest) + scaled_y * tail;
    const e = exp_table[@intCast(n & 31)];
    const poly = (0x1.62e42fefa39efp-6 + 0x1.ebfbdff82c58ep-13 * f) + (f * f) * (0x1.c6b08d704a0bfp-20 + 0x1.3b2ab6fba4e77p-27 * f);
    return @floatCast(std.math.ldexp(e + (f * e) * poly, n >> 5)); // the game scales in one or two exact steps
}

const log_table = [65][2]f64{ // 1/(c ln 2) and log2(c) for c = 1 + k/64, halved from k = 32
    .{ 0x1.71547652b82fep+0, 0 },
    .{ 0x1.6ba5ded75ac4dp+0, 0x1.6e79685c2c000p-6 },
    .{ 0x1.66235b77002e7p+0, 0x1.6bad3758f0000p-5 },
    .{ 0x1.60caf2ef7e44bp+0, 0x1.0eb389fa2a000p-4 },
    .{ 0x1.5b9ac9b743f0dp+0, 0x1.663f6fac91000p-4 },
    .{ 0x1.56911fd6002c7p+0, 0x1.bc84240adb000p-4 },
    .{ 0x1.51ac4eec8b247p+0, 0x1.08c588cda7800p-3 },
    .{ 0x1.4ceac86768bb6p+0, 0x1.32ae9e278b000p-3 },
    .{ 0x1.484b13d7c02a9p+0, 0x1.5c01a39fbd800p-3 },
    .{ 0x1.43cbcd6f18b64p+0, 0x1.84c2bd02f0000p-3 },
    .{ 0x1.3f6ba49a91758p+0, 0x1.acf5e2db4f000p-3 },
    .{ 0x1.3b295abaa3ffep+0, 0x1.d49ee4c325800p-3 },
    .{ 0x1.3703c1f4d0ffep+0, 0x1.fbc16b9026800p-3 },
    .{ 0x1.32f9bc1cdb958p+0, 0x1.11307dad30c00p-2 },
    .{ 0x1.2f0a39b3764ebp+0, 0x1.24407ab0e0800p-2 },
    .{ 0x1.2b3438f87b4a7p+0, 0x1.37124cea4cc00p-2 },
    .{ 0x1.2776c50ef9bfep+0, 0x1.49a784bcd1c00p-2 },
    .{ 0x1.23d0f5318e5ecp+0, 0x1.5c01a39fbd800p-2 },
    .{ 0x1.2041ebf5a27cdp+0, 0x1.6e221cd9d0c00p-2 },
    .{ 0x1.1cc8d69c50564p+0, 0x1.800a563161c00p-2 },
    .{ 0x1.1964ec6fc9491p+0, 0x1.91bba891f1800p-2 },
    .{ 0x1.16156e2c365a4p+0, 0x1.a33760a7f6000p-2 },
    .{ 0x1.12d9a57323dc3p+0, 0x1.b47ebf7388400p-2 },
    .{ 0x1.0fb0e4489f08cp+0, 0x1.c592fad295c00p-2 },
    .{ 0x1.0c9a84994022dp+0, 0x1.d6753e032ec00p-2 },
    .{ 0x1.0995e7c86d702p+0, 0x1.e726aa1e75400p-2 },
    .{ 0x1.06a2764633554p+0, 0x1.f7a8568cb0800p-2 },
    .{ 0x1.03bf9f2c1c437p+0, 0x1.03fda8b979a00p-1 },
    .{ 0x1.00ecd7e080215p+0, 0x1.0c10500d63a00p-1 },
    .{ 0x1.fc53377f9d292p-1, 0x1.140c9faa1e600p-1 },
    .{ 0x1.f6ead796c4570p-1, 0x1.1bf311e95d000p-1 },
    .{ 0x1.f19f9cbae7ffdp-1, 0x1.23c41d4272800p-1 },
    .{ 0x1.ec709dc3a03fdp-1, -0x1.a8ff971810c00p-2 },
    .{ 0x1.e75cfb25e5daep-1, -0x1.99b072a96c800p-2 },
    .{ 0x1.e263de767da1dp-1, -0x1.8a8980abfbc00p-2 },
    .{ 0x1.dd8479f4003dfp-1, -0x1.7b89f02cf2c00p-2 },
    .{ 0x1.d8be0817f5ffep-1, -0x1.6cb0f6865c800p-2 },
    .{ 0x1.d40fcb2e891bcp-1, -0x1.5dfdcf1eeb000p-2 },
    .{ 0x1.cf790cf45a967p-1, -0x1.4f6fbb2cec400p-2 },
    .{ 0x1.caf91e3a0f252p-1, -0x1.4106017c3ec00p-2 },
    .{ 0x1.c68f568d31760p-1, -0x1.32bfee370f000p-2 },
    .{ 0x1.c23b13e60edb5p-1, -0x1.249cd2b13cc00p-2 },
    .{ 0x1.bdfbba5a3a303p-1, -0x1.169c05363f000p-2 },
    .{ 0x1.b9d0b3d3671a3p-1, -0x1.08bce0d95fc00p-2 },
    .{ 0x1.b5b96fca558e1p-1, -0x1.f5fd8a9064000p-3 },
    .{ 0x1.b1b563058ac9dp-1, -0x1.dac22d3e44000p-3 },
    .{ 0x1.adc4075b99d15p-1, -0x1.bfc67a7fff800p-3 },
    .{ 0x1.a9e4db78c1f20p-1, -0x1.a5094b54d2800p-3 },
    .{ 0x1.a61762a7aded9p-1, -0x1.8a8980abfc000p-3 },
    .{ 0x1.a25b249d2231bp-1, -0x1.7046031c7a000p-3 },
    .{ 0x1.9eafad466bffep-1, -0x1.563dc29ffb000p-3 },
    .{ 0x1.9b148c9a669bbp-1, -0x1.3c6fb650ce000p-3 },
    .{ 0x1.9789566cee8d2p-1, -0x1.22dadc2ab3800p-3 },
    .{ 0x1.940da2449dbe4p-1, -0x1.097e38ce60800p-3 },
    .{ 0x1.90a10b32adc32p-1, -0x1.e0b1ae8f30000p-4 },
    .{ 0x1.8d432facdfeebp-1, -0x1.aed391ab66000p-4 },
    .{ 0x1.89f3b1694cffep-1, -0x1.7d60496cfc000p-4 },
    .{ 0x1.86b2353c0032ap-1, -0x1.4c560fe68b000p-4 },
    .{ 0x1.837e62f643580p-1, -0x1.1bb32a6005000p-4 },
    .{ 0x1.8057e54783511p-1, -0x1.d6ebd1f1fe000p-5 },
    .{ 0x1.7d3e699fb5deep-1, -0x1.77394c9d96000p-5 },
    .{ 0x1.7a31a0132b331p-1, -0x1.184b8e4c56000p-5 },
    .{ 0x1.77313b3fb70c1p-1, -0x1.743ee861f4000p-6 },
    .{ 0x1.743cf0331e6ccp-1, -0x1.72c7ba20f8000p-7 },
    .{ 0x1.71547652b82fep-1, 0 },
};

const exp_table = [32]f64{ // 2^(k/32)
    0x1.0000000000000p+0, 0x1.059b0d3158574p+0, 0x1.0b5586cf9890fp+0, 0x1.11301d0125b51p+0,
    0x1.172b83c7d517bp+0, 0x1.1d4873168b9aap+0, 0x1.2387a6e756238p+0, 0x1.29e9df51fdee1p+0,
    0x1.306fe0a31b715p+0, 0x1.371a7373aa9cbp+0, 0x1.3dea64c123422p+0, 0x1.44e086061892dp+0,
    0x1.4bfdad5362a27p+0, 0x1.5342b569d4f82p+0, 0x1.5ab07dd485429p+0, 0x1.6247eb03a5585p+0,
    0x1.6a09e667f3bcdp+0, 0x1.71f75e8ec5f74p+0, 0x1.7a11473eb0187p+0, 0x1.82589994cce13p+0,
    0x1.8ace5422aa0dbp+0, 0x1.93737b0cdc5e5p+0, 0x1.9c49182a3f090p+0, 0x1.a5503b23e255dp+0,
    0x1.ae89f995ad3adp+0, 0x1.b7f76f2fb5e47p+0, 0x1.c199bdd85529cp+0, 0x1.cb720dcef9069p+0,
    0x1.d5818dcfba487p+0, 0x1.dfc97337b9b5fp+0, 0x1.ea4afa2a490dap+0, 0x1.f50765b6e4540p+0,
};

// the inline SSE sincos of 0x4818b0 and 0x7ffea0, all f32: quadrant by round(2x/pi), then a polynomial
pub fn sinCos(x: f32) [2]f32 {
    const t = x * 0x1.45f306p-1; // 2/pi
    const q: i32 = @intFromFloat(t + std.math.copysign(@as(f32, 0.5), t));
    const qf: f32 = @floatFromInt(q);
    const r = (x - qf * 0x1.921fb4p0) - qf * 0x1.4442d2p-24; // pi/2 in two parts
    const r2 = r * r;
    const c = ((-0x1.649326p-10 * r2 + 0x1.55406cp-5) * r2 + -0x1.ffffbep-2) * r2 + 1;
    const s = ((-0x1.9918dcp-13 * r2 + 0x1.110684p-7) * r2 + -0x1.555542p-3) * (r * r2) + r;
    const sin_ = if (q & 1 == 0) s else c;
    const cos_ = if (q & 1 == 0) c else s;
    return .{ if (q & 2 == 0) sin_ else -sin_, if ((q & 3) + 1 & 2 == 0) cos_ else -cos_ };
}

test "game trig and pow near the libm values" {
    for ([_]f32{ -3, -0.5, 0, 1e-3, 0.3, 1.2, 7, 100 }) |x| {
        try std.testing.expectApproxEqAbs(@sin(x), sin(x), 1e-6);
        try std.testing.expectApproxEqAbs(@cos(x), cos(x), 1e-6);
        const s, const c = sinCos(x);
        try std.testing.expectApproxEqAbs(@sin(x), s, 2e-6);
        try std.testing.expectApproxEqAbs(@cos(x), c, 2e-6);
    }
    try std.testing.expectEqual(@as(f32, 1), pow(1, 1.3));
    try std.testing.expectApproxEqRel(std.math.pow(f32, 0.37, 1.3), pow(0.37, 1.3), 1e-6);
}
