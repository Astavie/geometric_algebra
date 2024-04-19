pub mod polynomial;
pub mod simd;

mod generated;
pub use generated::*;

impl Automorphism for f32 {
    fn automorphism(self) -> f32 {
        self
    }
}

impl Reverse for f32 {
    fn reverse(self) -> f32 {
        self
    }
}

impl Conjugate for f32 {
    fn conjugate(self) -> f32 {
        self
    }
}

impl GeometricProduct<f32> for f32 {
    type Output = f32;

    fn geometric_product(self, other: f32) -> f32 {
        self * other
    }
}

impl OuterProduct<f32> for f32 {
    type Output = f32;

    fn meet(self, other: f32) -> f32 {
        self * other
    }
}

impl InnerProduct<f32> for f32 {
    type Output = f32;

    fn dot(self, other: f32) -> f32 {
        self * other
    }
}

impl LeftContraction<f32> for f32 {
    type Output = f32;

    fn left_contraction(self, other: f32) -> f32 {
        self * other
    }
}

impl RightContraction<f32> for f32 {
    type Output = f32;

    fn right_contraction(self, other: f32) -> f32 {
        self * other
    }
}

impl ScalarProduct<f32> for f32 {
    type Output = f32;

    fn scalar_product(self, other: f32) -> f32 {
        self * other
    }
}

impl Magnitude for f32 {
    fn length(self) -> f32 {
        self.abs()
    }
    fn length_squared(self) -> f32 {
        self * self
    }
}

impl Normalization for f32 {
    fn normalized(self) -> f32 {
        f32::signum(self)
    }
}

impl Inverse for f32 {
    fn inverse(self) -> f32 {
        1.0 / self
    }
}

impl GeometricQuotient<f32> for f32 {
    type Output = f32;

    fn geometric_quotient(self, other: f32) -> f32 {
        self.geometric_product(other.inverse())
    }
}

impl SandwichProduct<f32> for f32 {
    type Output = f32;

    fn sandwich(self, other: f32) -> f32 {
        self.geometric_product(other)
            .geometric_product(self.reverse())
    }
}

impl Magnitude for ppga2d::Direction {
    fn length_squared(self) -> f32 {
        self[0] * self[0] + self[1] * self[1]
    }
}

impl Normalization for ppga2d::Direction {
    fn normalized(self) -> Self {
        self / self.length()
    }
}

impl Magnitude for ppga3d::Direction {
    fn length_squared(self) -> f32 {
        self[0] * self[0] + self[1] * self[1] + self[2] * self[2]
    }
}

impl Normalization for ppga3d::Direction {
    fn normalized(self) -> Self {
        self / self.length()
    }
}

impl Magnitude for ppga3d::IdealLine {
    fn length_squared(self) -> f32 {
        self[0] * self[0] + self[1] * self[1] + self[2] * self[2]
    }
}

impl Normalization for ppga3d::IdealLine {
    fn normalized(self) -> Self {
        self / self.length()
    }
}

impl epga1d::ComplexNumber {
    pub const fn new(real: f32, im: f32) -> Self {
        Self::from_raw(real, im)
    }
    pub fn real(self) -> f32 {
        self[0]
    }
    pub fn imaginary(self) -> f32 {
        self[1]
    }
    pub fn from_polar(magnitude: f32, argument: f32) -> Self {
        Self::new(magnitude * argument.cos(), magnitude * argument.sin())
    }
    pub fn arg(self) -> f32 {
        self.imaginary().atan2(self.real())
    }
    pub fn exp(self) -> Self {
        Self::from_polar(self[0].exp(), self[1])
    }
    pub fn ln(self) -> Self {
        Self::new(self.length().ln(), self.arg())
    }
    pub fn powf(self, exponent: f32) -> Self {
        Self::from_polar(self.length().powf(exponent), self.arg() * exponent)
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl ppga2d::Point {
    pub const fn new(x: f32, y: f32) -> Self {
        Self::from_raw(1.0, y, x)
    }
    pub fn x(self) -> f32 {
        return self[2] / self[0];
    }
    pub fn y(self) -> f32 {
        return self[1] / self[0];
    }
}

impl ppga2d::Direction {
    pub const fn new(x: f32, y: f32) -> Self {
        Self::from_raw(y, x)
    }
}

impl ppga2d::Direction {
    pub fn exp(self) -> ppga2d::Translator {
        ppga2d::Translator::from_raw(1.0, self[0], self[1])
    }
}

impl ppga2d::Translator {
    pub fn ln(self) -> ppga2d::Direction {
        let result: ppga2d::Direction = self.into();
        result * (1.0 / self[0])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl ppga2d::Origin {
    pub fn exp(self) -> ppga2d::Rotor {
        let det = self[0] * self[0];
        let a = det.sqrt();
        let c = a.cos();
        let s = a.sin();
        ppga2d::Rotor::from_raw(c, s)
    }
}

impl ppga2d::Rotor {
    pub fn ln(self) -> ppga2d::Origin {
        let det = 1.0 - self[0] * self[0];
        let a = 1.0 / det;
        let b = self[0].acos() * a.sqrt();
        ppga2d::Origin::from_raw(b * self[1])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl ppga2d::Point {
    pub fn exp(self) -> ppga2d::Motor {
        let det = self[0] * self[0];
        if det <= 0.0 {
            return ppga2d::Motor::from_raw(1.0, 0.0, self[1], self[2]);
        }
        let a = det.sqrt();
        let c = a.cos();
        let s = a.sin() / a;
        let g0 = simd::Simd32x3::from(s) * self.group0();
        ppga2d::Motor::from_raw(c, g0[0], g0[1], g0[2])
    }
}

impl ppga2d::Motor {
    pub fn ln(self) -> ppga2d::Point {
        let det = 1.0 - self[0] * self[0];
        if det <= 0.0 {
            return ppga2d::Point::from_raw(0.0, self[2], self[3]);
        }
        let a = 1.0 / det;
        let b = self[0].acos() * a.sqrt();
        let g0 = simd::Simd32x4::from(b) * self.group0();
        ppga2d::Point::from_raw(g0[1], g0[2], g0[3])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl ppga3d::IdealLine {
    pub fn exp(self) -> ppga3d::Translator {
        ppga3d::Translator::from_raw(1.0, self[0], self[1], self[2])
    }
}

impl ppga3d::Translator {
    pub fn ln(self) -> ppga3d::IdealLine {
        let result: ppga3d::IdealLine = self.into();
        result * (1.0 / self[0])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl ppga3d::Branch {
    pub fn exp(self) -> ppga3d::Rotor {
        let det = self[0] * self[0] + self[1] * self[1] + self[2] * self[2];
        let a = det.sqrt();
        let c = a.cos();
        let s = a.sin() / a;
        let g0 = simd::Simd32x3::from(s) * self.group0();
        ppga3d::Rotor::from_raw(c, g0[0], g0[1], g0[2])
    }
}

impl ppga3d::Rotor {
    pub fn ln(self) -> ppga3d::Branch {
        let det = 1.0 - self[0] * self[0];
        let a = 1.0 / det;
        let b = self[0].acos() * a.sqrt();
        let g0 = simd::Simd32x4::from(b) * self.group0();
        ppga3d::Branch::from_raw(g0[1], g0[2], g0[3])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl ppga3d::Line {
    pub fn exp(self) -> ppga3d::Motor {
        let det = self[3] * self[3] + self[4] * self[4] + self[5] * self[5];
        if det <= 0.0 {
            return ppga3d::Motor::from_raw(1.0, 0.0, 0.0, 0.0, 0.0, self[0], self[1], self[2]);
        }
        let a = det.sqrt();
        let c = a.cos();
        let s = a.sin() / a;
        let m = self[0] * self[3] + self[1] * self[4] + self[2] * self[5];
        let t = m / det * (c - s);
        let g0 = simd::Simd32x3::from(s) * self.group1();
        let g1 = simd::Simd32x3::from(s) * self.group0() + simd::Simd32x3::from(t) * self.group1();
        ppga3d::Motor::from_raw(c, g0[0], g0[1], g0[2], s * m, g1[0], g1[1], g1[2])
    }
}

impl ppga3d::Motor {
    pub fn ln(self) -> ppga3d::Line {
        let det = 1.0 - self[0] * self[0];
        if det <= 0.0 {
            return ppga3d::Line::from_raw(self[5], self[6], self[7], 0.0, 0.0, 0.0);
        }
        let a = 1.0 / det;
        let b = self[0].acos() * a.sqrt();
        let c = a * self[4] * (1.0 - self[0] * b);
        let g0 = simd::Simd32x4::from(b) * self.group1() + simd::Simd32x4::from(c) * self.group0();
        let g1 = simd::Simd32x4::from(b) * self.group0();
        ppga3d::Line::from_raw(g0[1], g0[2], g0[3], g1[1], g1[2], g1[3])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
    pub fn sqrt(self) -> Self {
        self.powf(0.5)
    }
}

impl std::default::Default for ppga3d::Origin {
    fn default() -> Self {
        ppga3d::ORIGIN
    }
}

impl std::default::Default for ppga2d::Origin {
    fn default() -> Self {
        ppga2d::ORIGIN
    }
}

impl std::default::Default for ppga3d::PseudoScalar {
    fn default() -> Self {
        ppga3d::I
    }
}

impl std::default::Default for ppga2d::PseudoScalar {
    fn default() -> Self {
        ppga2d::I
    }
}

pub mod ppga3d {
    pub use super::generated::ppga3d::*;
    pub const ORIGIN: Origin = Origin::from_raw(1.0);
    pub const I: PseudoScalar = PseudoScalar::from_raw(1.0);
}

pub mod ppga2d {
    pub use super::generated::ppga2d::*;
    pub const ORIGIN: Origin = Origin::from_raw(1.0);
    pub const I: PseudoScalar = PseudoScalar::from_raw(1.0);

    pub const UP: Direction = Direction::new(0.0, 1.0);
    pub const DOWN: Direction = Direction::new(0.0, -1.0);
    pub const LEFT: Direction = Direction::new(-1.0, 0.0);
    pub const RIGHT: Direction = Direction::new(1.0, 0.0);
}

/// Element order reversed
pub trait Dual {
    type Output;
    fn dual(self) -> Self::Output;
}

/// Negates elements with `grade % 2 == 1`
///
/// Also called main involution
pub trait Automorphism {
    fn automorphism(self) -> Self;
}

/// Negates elements with `grade % 4 >= 2`
///
/// Also called transpose
pub trait Reverse {
    fn reverse(self) -> Self;
}

/// Negates elements with `(grade + 3) % 4 < 2`
pub trait Conjugate {
    fn conjugate(self) -> Self;
}

/// General multi vector multiplication
pub trait GeometricProduct<T> {
    type Output;
    fn geometric_product(self, other: T) -> Self::Output;
}

/// General multi vector division
pub trait GeometricQuotient<T> {
    type Output;
    fn geometric_quotient(self, other: T) -> Self::Output;
}

/// Dual of the geometric product grade filtered by `t == r + s`
///
/// Also called join
pub trait RegressiveProduct<T> {
    type Output;
    fn join(self, other: T) -> Self::Output;
}

/// Geometric product grade filtered by `t == r + s`
///
/// Also called meet or exterior product
pub trait OuterProduct<T> {
    type Output;
    fn meet(self, other: T) -> Self::Output;
}

/// Geometric product grade filtered by `t == (r - s).abs()`
///
/// Also called fat dot product
pub trait InnerProduct<T> {
    type Output;
    fn dot(self, other: T) -> Self::Output;
}

/// Geometric product grade filtered by `t == s - r`
pub trait LeftContraction<T> {
    type Output;
    fn left_contraction(self, other: T) -> Self::Output;
}

/// Geometric product grade filtered by `t == r - s`
pub trait RightContraction<T> {
    type Output;
    fn right_contraction(self, other: T) -> Self::Output;
}

/// Geometric product grade filtered by `t == 0`
pub trait ScalarProduct<T> {
    type Output;
    fn scalar_product(self, other: T) -> Self::Output;
}

/// `self * other * self.reverse()`
pub trait SandwichProduct<T> {
    type Output;
    fn sandwich(self, other: T) -> Self::Output;
}

/// Length as scalar
///
/// Also called amplitude, absolute value or norm
pub trait Magnitude: Sized {
    fn length_squared(self) -> f32;
    fn length(self) -> f32 {
        self.length_squared().sqrt()
    }
}

/// Direction without magnitude (set to scalar `-1.0` or `1.0`)
///
/// Also called sign or normalize
pub trait Normalization {
    fn normalized(self) -> Self;
}

/// Raises a number to the scalar power of `-1.0`
pub trait Inverse {
    fn inverse(self) -> Self;
}

#[cfg(test)]
mod tests {
    use super::*;
    use plotly::{common::Line, Plot, Scatter};

    #[test]
    fn it_works() {
        let mut plot = Plot::new();

        const POINTS: usize = 10;
        const ANGLE: f32 = 2.0 * std::f32::consts::PI / (POINTS as f32);

        let center = ppga2d::Point::new(2.0, 3.0);
        let points = (0..POINTS)
            .map(|p| point_on_circle(center, 1.5, ANGLE * ((p + 1) as f32)))
            .collect::<Vec<_>>();

        let trace = scatter(&points);
        plot.add_trace(trace);

        plot.write_html("out.html");
    }

    fn point_on_circle(center: ppga2d::Point, radius: f32, angle: f32) -> ppga2d::Point {
        let center = center.normalized();

        let up = ppga2d::UP * radius;
        let on_circle = center + up;

        let rotor = (angle / 2.0 * center).exp();
        println!("{:?}", rotor);
        assert!(false);
        rotor.sandwich(on_circle)
    }

    fn scatter(points: &[ppga2d::Point]) -> Box<Scatter<f32, f32>> {
        let xs = points
            .iter()
            .copied()
            .map(ppga2d::Point::x)
            .collect::<Vec<_>>();
        let ys = points
            .iter()
            .copied()
            .map(ppga2d::Point::y)
            .collect::<Vec<_>>();
        Scatter::new(xs, ys).line(Line::new().width(0.0))
    }
}
