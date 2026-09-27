shader_type canvas_item;
render_mode unshaded;

// Sensor and lens effects applied to the pinhole render, in the order light meets them:
// rolling shutter (rows are exposed at different times), lens distortion, motion blur over the
// shutter interval, then exposure gain and sensor noise. Every effect is off at its neutral
// value, so a camera with default settings publishes the render untouched.

uniform float half_hfov_rad = 0.9599;       // half of the horizontal field of view
uniform vec4 distortion_k = vec4(0.0);      // radial coefficients, all zero is a plain pinhole
uniform vec2 rolling_shutter_shift = vec2(0.0);  // uv shift between the first and last row
uniform vec2 motion_blur_shift = vec2(0.0);      // uv travel during the shutter interval
uniform int motion_blur_taps : hint_range(1, 16) = 1;
uniform float exposure_gain = 1.0;
uniform float noise_sigma = 0.0;
uniform float noise_seed = 0.0;
uniform float aspect = 1.777778;            // width / height, to keep the distortion circular

// Radial lens distortion by incident angle, so all-zero coefficients leave the image alone
vec2 distort(vec2 uv) {
    if (distortion_k == vec4(0.0)) {
        return uv;
    }
    vec2 centred = (uv - vec2(0.5)) * vec2(aspect, 1.0);
    float radius = length(centred);
    if (radius < 1e-6) {
        return uv;
    }
    float theta = atan(radius * tan(half_hfov_rad));
    float t2 = theta * theta;
    float scale = 1.0 + t2 * (distortion_k.x + t2 * (distortion_k.y
                + t2 * (distortion_k.z + t2 * distortion_k.w)));
    return vec2(0.5) + centred * scale / vec2(aspect, 1.0);
}

// Cheap hash noise; the seed changes per frame so grain does not sit still
float hash(vec2 p, float seed) {
    return fract(sin(dot(p, vec2(127.1, 311.7)) + seed) * 43758.5453);
}

void fragment() {
    // rows further down the sensor are read later, so they see a slightly later pose
    vec2 uv = UV + rolling_shutter_shift * (UV.y - 0.5);
    uv = distort(uv);

    vec3 colour = vec3(0.0);
    if (motion_blur_taps > 1) {
        for (int i = 0; i < motion_blur_taps; i++) {
            float t = float(i) / float(motion_blur_taps - 1) - 0.5;
            colour += texture(TEXTURE, uv + motion_blur_shift * t).rgb;
        }
        colour /= float(motion_blur_taps);
    } else {
        colour = texture(TEXTURE, uv).rgb;
    }

    colour *= exposure_gain;
    if (noise_sigma > 0.0) {
        // two uniforms shaped into a roughly normal sample, one per channel
        vec3 n = vec3(hash(FRAGCOORD.xy, noise_seed), hash(FRAGCOORD.xy, noise_seed + 17.0),
                      hash(FRAGCOORD.xy, noise_seed + 31.0)) - vec3(0.5);
        colour += n * noise_sigma * 3.4641;  // uniform(-0.5, 0.5) has sd 1/sqrt(12)
    }
    COLOR = vec4(clamp(colour, vec3(0.0), vec3(1.0)), 1.0);
}
