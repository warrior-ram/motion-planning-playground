//! Planar serial arm: forward kinematics, damped-least-squares IK, collision validity.

use crate::geom::{capsule_hits, wrap_angle, Shape, V2};

#[derive(Clone, Debug)]
pub struct Arm {
    pub base: V2,
    pub links: Vec<f64>,
}

impl Arm {
    pub fn new(base: V2, links: Vec<f64>) -> Self {
        Arm { base, links }
    }

    /// Total reach (sum of link lengths).
    pub fn reach(&self) -> f64 {
        self.links.iter().sum()
    }

    /// Joint positions: base, each intermediate joint, end effector (len = links + 1).
    /// Angles are relative to the previous link; x += L cos(phi), y += L sin(phi).
    pub fn fk(&self, angles: &[f64]) -> Vec<V2> {
        let mut out = Vec::with_capacity(self.links.len() + 1);
        let mut p = self.base;
        let mut phi = 0.0;
        out.push(p);
        for (l, a) in self.links.iter().zip(angles.iter()) {
            phi += a;
            p = V2::new(p.x + l * phi.cos(), p.y + l * phi.sin());
            out.push(p);
        }
        out
    }

    pub fn end_effector(&self, angles: &[f64]) -> V2 {
        *self.fk(angles).last().unwrap_or(&self.base)
    }

    /// True if no link capsule (radius = thickness / 2) touches any obstacle.
    pub fn config_valid(&self, angles: &[f64], obstacles: &[Shape], radius: f64) -> bool {
        let joints = self.fk(angles);
        for w in joints.windows(2) {
            for o in obstacles {
                if capsule_hits(w[0], w[1], radius, o) {
                    return false;
                }
            }
        }
        true
    }
}

#[derive(Clone, Debug)]
pub struct IkOptions {
    /// Optional per-joint (min, max). Joints with limits are clamped, not wrapped.
    pub limits: Option<Vec<(f64, f64)>>,
    pub max_iterations: usize,
    /// World-unit distance at which the target counts as reached.
    pub tolerance: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions { limits: None, max_iterations: 100, tolerance: 0.5 }
    }
}

#[derive(Clone, Debug)]
pub struct IkResult {
    pub angles: Vec<f64>,
    pub joints: Vec<V2>,
    pub reached: bool,
    /// Final distance from end effector to target (world units).
    pub error: f64,
    pub iterations: usize,
}

const MAX_JOINT_STEP: f64 = 0.4; // rad per iteration, keeps drags smooth
const LAMBDA_INIT: f64 = 0.1;
const LAMBDA_MIN: f64 = 1e-3;
const LAMBDA_MAX: f64 = 10.0;

fn constrain(angles: &mut [f64], limits: &Option<Vec<(f64, f64)>>) {
    for (i, a) in angles.iter_mut().enumerate() {
        match limits.as_ref().and_then(|l| l.get(i)) {
            Some(&(lo, hi)) => *a = a.clamp(lo, hi),
            None => *a = wrap_angle(*a),
        }
    }
}

/// End effector (relative to base, in normalised units where reach = 1) and Jacobian columns.
fn fk_jac(links: &[f64], angles: &[f64], jac: &mut [V2]) -> V2 {
    let n = links.len();
    let mut pts = [V2::new(0.0, 0.0); 16];
    let mut p = V2::new(0.0, 0.0);
    let mut phi = 0.0;
    for i in 0..n {
        pts[i] = p;
        phi += angles[i];
        p = V2::new(p.x + links[i] * phi.cos(), p.y + links[i] * phi.sin());
    }
    for i in 0..n {
        let r = p.sub(pts[i]);
        jac[i] = V2::new(-r.y, r.x);
    }
    p
}

/// Damped-least-squares (Levenberg-Marquardt style adaptive damping) IK, seeded from `seed`.
/// Arms of up to 16 links are supported (the API layer restricts to small chains).
pub fn solve_ik(arm: &Arm, seed: &[f64], target: V2, opts: &IkOptions) -> IkResult {
    let n = arm.links.len().min(16);
    let reach = arm.reach().max(1e-9);
    let links: Vec<f64> = arm.links.iter().take(n).map(|l| l / reach).collect();
    let t = V2::new((target.x - arm.base.x) / reach, (target.y - arm.base.y) / reach);
    let tol = (opts.tolerance / reach).max(0.0);

    let mut q: Vec<f64> = seed.iter().take(n).copied().collect();
    q.resize(n, 0.0);
    constrain(&mut q, &opts.limits);

    let mut jac = [V2::new(0.0, 0.0); 16];
    let mut e = fk_jac(&links, &q, &mut jac);
    let mut err = t.sub(e).len();
    let mut lambda = LAMBDA_INIT;
    let mut iterations = 0;
    let mut nudges = 0;
    let target_dist = t.len();
    let reachable = target_dist < 1.0 - tol;
    let mut trial = vec![0.0; n];
    let mut tjac = [V2::new(0.0, 0.0); 16];

    while iterations < opts.max_iterations && err > tol {
        iterations += 1;
        let ev = t.sub(e);
        // A = J J^T + lambda^2 I (2x2), solve A w = ev, dq = J^T w
        let (mut a, mut b, mut d) = (lambda * lambda, 0.0, lambda * lambda);
        for c in jac.iter().take(n) {
            a += c.x * c.x;
            b += c.x * c.y;
            d += c.y * c.y;
        }
        let det = a * d - b * b;
        let mut max_step: f64 = 0.0;
        if det.abs() > 1e-300 {
            let wx = (d * ev.x - b * ev.y) / det;
            let wy = (a * ev.y - b * ev.x) / det;
            for i in 0..n {
                trial[i] = jac[i].x * wx + jac[i].y * wy;
                max_step = max_step.max(trial[i].abs());
            }
        } else {
            trial.iter_mut().for_each(|v| *v = 0.0);
        }
        let scale = if max_step > MAX_JOINT_STEP { MAX_JOINT_STEP / max_step } else { 1.0 };
        for i in 0..n {
            trial[i] = q[i] + trial[i] * scale;
        }
        constrain(&mut trial, &opts.limits);
        let te = fk_jac(&links, &trial, &mut tjac);
        let terr = t.sub(te).len();

        if terr < err {
            let improvement = err - terr;
            q.copy_from_slice(&trial);
            e = te;
            jac[..n].copy_from_slice(&tjac[..n]);
            err = terr;
            lambda = (lambda * 0.5).max(LAMBDA_MIN);
            if improvement > 1e-12 {
                continue;
            }
        } else {
            lambda = (lambda * 3.0).min(LAMBDA_MAX);
            if lambda < LAMBDA_MAX {
                continue;
            }
        }
        // Stalled (typically: arm straight, target inside reach on the arm's line).
        // Bend the inner joints in the direction they already lean to preserve elbow sense.
        if reachable && nudges < 3 && n > 1 {
            nudges += 1;
            for i in 1..n {
                let s = if q[i] < 0.0 { -1.0 } else { 1.0 };
                q[i] += s * 0.15;
            }
            constrain(&mut q, &opts.limits);
            e = fk_jac(&links, &q, &mut jac);
            err = t.sub(e).len();
            lambda = LAMBDA_INIT;
        } else {
            break;
        }
    }

    // Out of (or at the edge of) reach: compare against the analytic full stretch pointing
    // at the target, keeping a tiny bend in the seed's sense so later drags back inside
    // reach do not flip the elbow.
    if target_dist >= 1.0 - tol && err > tol {
        let mut s: Vec<f64> = (0..n)
            .map(|i| {
                if i == 0 {
                    0.0
                } else if seed.get(i).copied().unwrap_or(0.0) < 0.0 {
                    -1e-3
                } else {
                    1e-3
                }
            })
            .collect();
        let e0 = fk_jac(&links, &s, &mut tjac);
        s[0] = t.y.atan2(t.x) - e0.y.atan2(e0.x);
        constrain(&mut s, &opts.limits);
        let se = fk_jac(&links, &s, &mut tjac);
        let serr = t.sub(se).len();
        if serr < err {
            q = s;
            err = serr;
        }
    }

    let joints = arm.fk(&q);
    let err_world = err * reach;
    IkResult {
        angles: q,
        joints,
        reached: err_world <= opts.tolerance,
        error: err_world,
        iterations,
    }
}
