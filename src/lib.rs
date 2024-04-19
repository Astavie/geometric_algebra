pub mod polynomial;
pub mod simd;

mod generated;
pub use generated::*;

impl Zero for f32 {
    fn zero() -> Self {
        0.0
    }
}

impl One for f32 {
    fn one() -> Self {
        1.0
    }
}

impl Automorphism for f32 {
    type Output = f32;

    fn automorphism(self) -> f32 {
        self
    }
}

impl Reverse for f32 {
    type Output = f32;

    fn reverse(self) -> f32 {
        self
    }
}

impl Conjugate for f32 {
    type Output = f32;

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

impl SquaredNorm for f32 {
    type Output = f32;

    fn length_squared(self) -> f32 {
        self.scalar_product(self.reverse())
    }
}

impl Norm for f32 {
    type Output = f32;

    fn length(self) -> f32 {
        self.abs()
    }
}

impl Normalization for f32 {
    type Output = f32;

    fn normalized(self) -> f32 {
        f32::signum(self)
    }
}

impl Inverse for f32 {
    type Output = f32;

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

impl epga1d::ComplexNumber {
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
}

impl epga1d::ComplexNumber {
    pub fn exp(self) -> Self {
        Self::from_polar(self[0].exp(), self[1])
    }
    pub fn ln(self) -> Self {
        Self::new(self.length().ln(), self.arg())
    }
    pub fn powf(self, exponent: f32) -> Self {
        Self::from_polar(self.length().powf(exponent), self.arg() * exponent)
    }
}

impl ppga2d::IdealPoint {
    pub fn exp(self) -> ppga2d::Translator {
        ppga2d::Translator::new(1.0, self[0], self[1])
    }
}

impl ppga2d::Translator {
    pub fn ln(self) -> ppga2d::IdealPoint {
        let result: ppga2d::IdealPoint = self.into();
        result * (1.0 / self[0])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
}

impl ppga2d::Point {
    pub fn exp(self) -> ppga2d::Motor {
        let det = self[0] * self[0];
        if det <= 0.0 {
            return ppga2d::Motor::new(1.0, 0.0, self[1], self[2]);
        }
        let a = det.sqrt();
        let c = a.cos();
        let s = a.sin() / a;
        let g0 = simd::Simd32x3::from(s) * self.group0();
        ppga2d::Motor::new(c, g0[0], g0[1], g0[2])
    }
}

impl ppga2d::Motor {
    pub fn ln(self) -> ppga2d::Point {
        let det = 1.0 - self[0] * self[0];
        if det <= 0.0 {
            return ppga2d::Point::new(0.0, self[2], self[3]);
        }
        let a = 1.0 / det;
        let b = self[0].acos() * a.sqrt();
        let g0 = simd::Simd32x4::from(b) * self.group0();
        ppga2d::Point::new(g0[1], g0[2], g0[3])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
}

impl ppga3d::IdealLine {
    pub fn exp(self) -> ppga3d::Translator {
        ppga3d::Translator::new(1.0, self[0], self[1], self[2])
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
}

impl ppga3d::Line {
    pub fn exp(self) -> ppga3d::Motor {
        let det = self[3] * self[3] + self[4] * self[4] + self[5] * self[5];
        if det <= 0.0 {
            return ppga3d::Motor::new(1.0, 0.0, 0.0, 0.0, 0.0, self[0], self[1], self[2]);
        }
        let a = det.sqrt();
        let c = a.cos();
        let s = a.sin() / a;
        let m = self[0] * self[3] + self[1] * self[4] + self[2] * self[5];
        let t = m / det * (c - s);
        let g0 = simd::Simd32x3::from(s) * self.group1();
        let g1 = simd::Simd32x3::from(s) * self.group0() + simd::Simd32x3::from(t) * self.group1();
        ppga3d::Motor::new(c, g0[0], g0[1], g0[2], s * m, g1[0], g1[1], g1[2])
    }
}

impl ppga3d::Motor {
    pub fn ln(self) -> ppga3d::Line {
        let det = 1.0 - self[0] * self[0];
        if det <= 0.0 {
            return ppga3d::Line::new(self[5], self[6], self[7], 0.0, 0.0, 0.0);
        }
        let a = 1.0 / det;
        let b = self[0].acos() * a.sqrt();
        let c = a * self[4] * (1.0 - self[0] * b);
        let g0 = simd::Simd32x4::from(b) * self.group1() + simd::Simd32x4::from(c) * self.group0();
        let g1 = simd::Simd32x4::from(b) * self.group0();
        ppga3d::Line::new(g0[1], g0[2], g0[3], g1[1], g1[2], g1[3])
    }
    pub fn powf(self, exponent: f32) -> Self {
        (self.ln() * exponent).exp()
    }
}

/// All elements set to `0.0`
pub trait Zero {
    fn zero() -> Self;
}

/// All elements set to `0.0`, except for the scalar, which is set to `1.0`
pub trait One {
    fn one() -> Self;
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
    type Output;
    fn automorphism(self) -> Self::Output;
}

/// Negates elements with `grade % 4 >= 2`
///
/// Also called transpose
pub trait Reverse {
    type Output;
    fn reverse(self) -> Self::Output;
}

/// Negates elements with `(grade + 3) % 4 < 2`
pub trait Conjugate {
    type Output;
    fn conjugate(self) -> Self::Output;
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

/// Square of the magnitude
pub trait SquaredNorm {
    type Output;
    fn length_squared(self) -> Self::Output;
}

/// Length as scalar
///
/// Also called amplitude, absolute value or norm
pub trait Norm {
    type Output;
    fn length(self) -> Self::Output;
}

/// Direction without magnitude (set to scalar `-1.0` or `1.0`)
///
/// Also called sign or normalize
pub trait Normalization {
    type Output;
    fn normalized(self) -> Self::Output;
}

/// Raises a number to the scalar power of `-1.0`
pub trait Inverse {
    type Output;
    fn inverse(self) -> Self::Output;
}
