//! Small fixed-size matrices for the tracking filters.

use std::ops::{Add, Mul, Sub};

/// An `R × C` matrix of `f64`, stored row-major.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Matrix<const R: usize, const C: usize>(pub [[f64; C]; R]);

impl<const R: usize, const C: usize> Matrix<R, C> {
    /// The zero matrix.
    pub(crate) fn zeros() -> Self {
        Matrix([[0.0; C]; R])
    }

    /// The transpose.
    pub(crate) fn transpose(&self) -> Matrix<C, R> {
        let mut t = Matrix::<C, R>::zeros();
        for (i, row) in self.0.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                t.0[j][i] = *v;
            }
        }
        t
    }

    /// Every element multiplied by `k`.
    pub(crate) fn scale(&self, k: f64) -> Self {
        let mut m = *self;
        m.0.iter_mut().flatten().for_each(|v| *v *= k);
        m
    }
}

impl<const N: usize> Matrix<N, N> {
    /// The identity matrix.
    pub(crate) fn identity() -> Self {
        let mut m = Self::zeros();
        for (i, row) in m.0.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        m
    }
}

impl<const R: usize, const K: usize, const C: usize> Mul<Matrix<K, C>> for Matrix<R, K> {
    type Output = Matrix<R, C>;

    fn mul(self, o: Matrix<K, C>) -> Matrix<R, C> {
        let mut m = Matrix::<R, C>::zeros();
        for i in 0..R {
            for j in 0..C {
                m.0[i][j] = (0..K).map(|k| self.0[i][k] * o.0[k][j]).sum();
            }
        }
        m
    }
}

impl<const R: usize, const C: usize> Add for Matrix<R, C> {
    type Output = Self;

    fn add(mut self, o: Self) -> Self {
        for (a, b) in self.0.iter_mut().flatten().zip(o.0.iter().flatten()) {
            *a += b;
        }
        self
    }
}

impl<const R: usize, const C: usize> Sub for Matrix<R, C> {
    type Output = Self;

    fn sub(self, o: Self) -> Self {
        self + o.scale(-1.0)
    }
}

/// Inverse of a 3×3 matrix by cofactors; `None` if the determinant is not
/// finite, `|det| ≤ 1e-12·|m₀₀·m₁₁·m₂₂|` (a scale-invariant singularity
/// test per axis; exactly 0 is rejected when a diagonal element is 0), or
/// the result is not finite.
pub(crate) fn inverse3(m: &Matrix<3, 3>) -> Option<Matrix<3, 3>> {
    let a = &m.0;
    let c =
        |r0: usize, r1: usize, c0: usize, c1: usize| a[r0][c0] * a[r1][c1] - a[r0][c1] * a[r1][c0];
    // Adjugate (transposed cofactors).
    let adj = Matrix([
        [c(1, 2, 1, 2), -c(0, 2, 1, 2), c(0, 1, 1, 2)],
        [-c(1, 2, 0, 2), c(0, 2, 0, 2), -c(0, 1, 0, 2)],
        [c(1, 2, 0, 1), -c(0, 2, 0, 1), c(0, 1, 0, 1)],
    ]);
    let det = a[0][0] * adj.0[0][0] + a[0][1] * adj.0[1][0] + a[0][2] * adj.0[2][0];
    let scale = (a[0][0] * a[1][1] * a[2][2]).abs();
    if !det.is_finite() || det.abs() <= 1e-12 * scale {
        return None;
    }
    let inv = adj.scale(1.0 / det);
    inv.0.iter().flatten().all(|v| v.is_finite()).then_some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse3_identity() {
        let i = Matrix::<3, 3>::identity();
        assert_eq!(inverse3(&i), Some(i));
        let singular = Matrix([[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]]);
        assert_eq!(inverse3(&singular), None);
        let nan = Matrix([[f64::NAN, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(inverse3(&nan), None);
        let m = Matrix([[4.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]]);
        let p = m * inverse3(&m).unwrap() - Matrix::identity();
        assert!(p.0.iter().flatten().all(|v| v.abs() < 1e-12));
        // The threshold is relative: a tiny but well-conditioned matrix inverts.
        let tiny = Matrix::<3, 3>::identity().scale(1e-7);
        assert!(inverse3(&tiny).is_some());
        // Disparate but independent axis scales are fine.
        let skew = Matrix([[1e7, 0.0, 0.0], [0.0, 1e7, 0.0], [0.0, 0.0, 2e-6]]);
        assert!(inverse3(&skew).is_some());
        assert_eq!(inverse3(&Matrix::zeros()), None);
    }
}
