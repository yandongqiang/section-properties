//! Matrix abstraction for linear solvers.
//!
//! Provides a common interface for different matrix formats
//! (Dense, CSR, CSC, Skyline) used by linear solvers.

use crate::fea::SparseMatrix;

/// Dense matrix in row-major format.
#[derive(Debug, Clone)]
pub struct DenseMatrix {
    pub nrows: usize,
    pub ncols: usize,
    data: Vec<f64>,
}

impl DenseMatrix {
    pub fn new(nrows: usize, ncols: usize) -> Self {
        Self {
            nrows,
            ncols,
            data: vec![0.0; nrows * ncols],
        }
    }

    pub fn from_vec(nrows: usize, ncols: usize, data: Vec<f64>) -> Self {
        assert_eq!(data.len(), nrows * ncols);
        Self { nrows, ncols, data }
    }

    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.ncols + col]
    }

    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row * self.ncols + col] = val;
    }

    pub fn add(&mut self, row: usize, col: usize, val: f64) {
        self.data[row * self.ncols + col] += val;
    }

    pub fn row(&self, row: usize) -> &[f64] {
        &self.data[row * self.ncols..(row + 1) * self.ncols]
    }

    pub fn row_mut(&mut self, row: usize) -> &mut [f64] {
        &mut self.data[row * self.ncols..(row + 1) * self.ncols]
    }

    pub fn col(&self, col: usize) -> Vec<f64> {
        (0..self.nrows).map(|i| self.get(i, col)).collect()
    }

    pub fn matvec(&self, x: &[f64], y: &mut [f64]) {
        assert_eq!(x.len(), self.ncols);
        assert_eq!(y.len(), self.nrows);
        for i in 0..self.nrows {
            let mut sum = 0.0;
            let row = self.row(i);
            for j in 0..self.ncols {
                sum += row[j] * x[j];
            }
            y[i] = sum;
        }
    }

    pub fn transpose(&self) -> Self {
        let mut data = vec![0.0; self.nrows * self.ncols];
        for i in 0..self.nrows {
            for j in 0..self.ncols {
                data[j * self.nrows + i] = self.get(i, j);
            }
        }
        DenseMatrix::from_vec(self.ncols, self.nrows, data)
    }
}

impl std::ops::Index<(usize, usize)> for DenseMatrix {
    type Output = f64;
    fn index(&self, (row, col): (usize, usize)) -> &f64 {
        &self.data[row * self.ncols + col]
    }
}

impl std::ops::IndexMut<(usize, usize)> for DenseMatrix {
    fn index_mut(&mut self, (row, col): (usize, usize)) -> &mut f64 {
        &mut self.data[row * self.ncols + col]
    }
}

/// Matrix format capabilities
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatrixFormat {
    Dense,
    Csr,
    Csc,
    Skyline,
}

/// Trait for matrix-like objects that can be used with linear solvers.
pub trait Matrix: Send + Sync {
    fn nrows(&self) -> usize;
    fn ncols(&self) -> usize;
    fn nnz(&self) -> usize;
    
    /// Matrix-vector product: y = A * x
    fn matvec(&self, x: &[f64], y: &mut [f64]) -> Result<(), SolverError>;
    
    /// Get matrix format
    fn format(&self) -> MatrixFormat;
    
    /// Convert to CSR format (for solvers that require CSR)
    fn to_csr(&self) -> &SparseMatrix;
    
    /// Get diagonal entries
    fn diagonal(&self) -> Vec<f64> {
        let n = self.nrows().min(self.ncols());
        let mut diag = vec![0.0; n];
        for i in 0..n {
            // Default implementation - overridden by specific types
            let mut row = vec![0.0; self.ncols()];
            let _ = self.matvec(&{
                let mut v = vec![0.0; self.ncols()];
                v[i] = 1.0;
                v
            }, &mut row);
            diag[i] = row[i];
        }
        diag
    }
    
    /// Frobenius norm
    fn frobenius_norm(&self) -> f64 {
        // Default implementation - can be overridden
        let mut norm = 0.0;
        for i in 0..self.nrows() {
            let mut row = vec![0.0; self.ncols()];
            let mut e_i = vec![0.0; self.ncols()];
            e_i[i] = 1.0;
            let _ = self.matvec(&e_i, &mut row);
            norm += row.iter().map(|v| v * v).sum::<f64>();
        }
        norm.sqrt()
    }
}

// Import SolverError for the trait
use crate::fea::solvers::SolverError;

// Implement Matrix for SparseMatrix
impl Matrix for SparseMatrix {
    fn nrows(&self) -> usize {
        self.n
    }
    
    fn ncols(&self) -> usize {
        self.n
    }
    
    fn nnz(&self) -> usize {
        if self.compressed {
            self.csr_vals.len()
        } else {
            self.vals.len()
        }
    }
    
    fn matvec(&self, x: &[f64], y: &mut [f64]) -> Result<(), SolverError> {
        if x.len() != self.ncols() {
            return Err(SolverError::InvalidInput(format!(
                "RHS length {} does not match matrix columns {}",
                x.len(), self.ncols()
            )));
        }
        if y.len() != self.nrows() {
            return Err(SolverError::InvalidInput(format!(
                "Output length {} does not match matrix rows {}",
                y.len(), self.nrows()
            )));
        }
        if !self.compressed {
            return Err(SolverError::InvalidInput("SparseMatrix must be compressed before matvec".to_string()));
        }
        SparseMatrix::matvec_into(self, x, y);
        Ok(())
    }
    
    fn format(&self) -> MatrixFormat {
        MatrixFormat::Csr
    }
    
    fn to_csr(&self) -> &SparseMatrix {
        self
    }
    
    fn diagonal(&self) -> Vec<f64> {
        if !self.compressed {
            return vec![0.0; self.n];
        }
        let mut diag = vec![0.0; self.n];
        for i in 0..self.n {
            let start = self.row_ptr[i];
            let end = self.row_ptr[i + 1];
            for k in start..end {
                if self.csr_cols[k] == i {
                    diag[i] = self.csr_vals[k];
                    break;
                }
            }
        }
        diag
    }
    
    fn frobenius_norm(&self) -> f64 {
        if !self.compressed {
            return 0.0;
        }
        self.csr_vals.iter().map(|v| v * v).sum::<f64>().sqrt()
    }
}

// Implement Matrix for CscMatrix
impl Matrix for crate::fea::CscMatrix {
    fn nrows(&self) -> usize {
        self.n_rows
    }
    
    fn ncols(&self) -> usize {
        self.n_cols
    }
    
    fn nnz(&self) -> usize {
        self.vals.len()
    }
    
    fn matvec(&self, x: &[f64], y: &mut [f64]) -> Result<(), SolverError> {
        if x.len() != self.ncols() {
            return Err(SolverError::InvalidInput(format!(
                "RHS length {} does not match matrix columns {}",
                x.len(), self.ncols()
            )));
        }
        if y.len() != self.nrows() {
            return Err(SolverError::InvalidInput(format!(
                "Output length {} does not match matrix rows {}",
                y.len(), self.nrows()
            )));
        }
        self.matvec(x, y);
        Ok(())
    }
    
    fn format(&self) -> MatrixFormat {
        MatrixFormat::Csc
    }
    
    fn to_csr(&self) -> &SparseMatrix {
        // CSC to CSR conversion would be expensive
        // For now, return a reference to a dummy - in practice this should be avoided
        unimplemented!("CscMatrix::to_csr not implemented - convert before calling")
    }
    
    fn diagonal(&self) -> Vec<f64> {
        let mut diag = vec![0.0; self.n_rows.min(self.n_cols)];
        for c in 0..self.n_cols {
            for k in self.col_ptr[c]..self.col_ptr[c + 1] {
                if self.rows[k] == c {
                    diag[c] = self.vals[k];
                    break;
                }
            }
        }
        diag
    }
}

// Implement Matrix for DenseMatrix
impl Matrix for DenseMatrix {
    fn nrows(&self) -> usize {
        self.nrows
    }
    
    fn ncols(&self) -> usize {
        self.ncols
    }
    
    fn nnz(&self) -> usize {
        self.nrows * self.ncols
    }
    
    fn matvec(&self, x: &[f64], y: &mut [f64]) -> Result<(), SolverError> {
        if x.len() != self.ncols() {
            return Err(SolverError::InvalidInput(format!(
                "RHS length {} does not match matrix columns {}",
                x.len(), self.ncols()
            )));
        }
        if y.len() != self.nrows() {
            return Err(SolverError::InvalidInput(format!(
                "Output length {} does not match matrix rows {}",
                y.len(), self.nrows()
            )));
        }
        DenseMatrix::matvec(self, x, y);
        Ok(())
    }
    
    fn format(&self) -> MatrixFormat {
        MatrixFormat::Dense
    }
    
    fn to_csr(&self) -> &SparseMatrix {
        unimplemented!("DenseMatrix::to_csr not implemented - convert before calling")
    }
    
    fn diagonal(&self) -> Vec<f64> {
        let n = self.nrows().min(self.ncols());
        let mut diag = Vec::with_capacity(n);
        for i in 0..n {
            diag.push(self.get(i, i));
        }
        diag
    }
    
    fn frobenius_norm(&self) -> f64 {
        self.data.iter().map(|v| v * v).sum::<f64>().sqrt()
    }
}