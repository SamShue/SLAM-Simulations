# Least Squares

This folder contains a series of documents that build up the **least squares** method from scratch: the tool used throughout this repository (and in GraphSLAM in particular) to combine many imperfect measurements into the single best estimate.

## Contents

| Path | What it is |
|---|---|
| [Documents/](Documents) | The documentation, with the figures it uses in [Documents/images/](Documents/images) |
| [leastSquaresFit.m](leastSquaresFit.m) | A reusable MATLAB function that solves `A*x = b` via the normal equations |
| [EXAMPLE_LinearFit.m](EXAMPLE_LinearFit.m) | Reproduces the document's worked line-fitting example |
| [EXAMPLE_PolynomialFit.m](EXAMPLE_PolynomialFit.m) | Fits a higher order polynomial, extending the document's "Fitting Curves and Surfaces" section |

## Documentation

| Document | What it covers |
|---|---|
| [Linear Least Squares](Documents/linear_least_squares.md) | Linear systems of equations written as $A\mathbf{x}=\mathbf{b}$, the column space of a matrix, reframing a solved system as a line fit through data points, and what it means when $\mathbf{b}$ doesn't lie in the column space |

More documents will be added to this series, extending this foundation to the least squares problem itself and to weighted least squares, the method GraphSLAM itself is built on. See the [GraphSLAM](../Graph_SLAM) folder for that application.

## MATLAB Scripts

[leastSquaresFit.m](leastSquaresFit.m) implements the normal equations $A^{\mathsf{T}}A\hat{\mathbf{x}}=A^{\mathsf{T}}\mathbf{b}$ described in the document, returning the least squares solution $\hat{\mathbf{x}}$, its projection $\mathbf{p}=A\hat{\mathbf{x}}$, and the residual $\mathbf{e}=\mathbf{b}-\mathbf{p}$. Run from the `Least_Squares/` folder:

```
matlab -batch "EXAMPLE_LinearFit"
matlab -batch "EXAMPLE_PolynomialFit"
```

[EXAMPLE_LinearFit.m](EXAMPLE_LinearFit.m) reproduces the document's line-fitting example exactly (the same $A$ and $\mathbf{b}=(1,2,6)$), confirming the same best fit line $y=2.5t+0.5$ and an orthogonal residual. [EXAMPLE_PolynomialFit.m](EXAMPLE_PolynomialFit.m) generalizes this to fitting a higher order polynomial by adding one column of $t^k$ to $A$ per degree, recovering noisy synthetic polynomial coefficients.

## Regenerating the Figures

The figures in the documents are produced by scripts in [Documents/images/](Documents/images), next to the images themselves. Each script also checks the numbers that its document quotes. Run it from the root of the repository:

```
python Least_Squares/Documents/images/generate_linear_least_squares_figures.py
```

| Script | Produces the figures for |
|---|---|
| [generate_linear_least_squares_figures.py](Documents/images/generate_linear_least_squares_figures.py) | Linear Least Squares |

The script needs NumPy and Matplotlib.
