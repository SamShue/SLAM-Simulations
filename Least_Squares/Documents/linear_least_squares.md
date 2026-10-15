# Linear Least Squares

Least squares is a method for fitting an equation to a set of data points. There are several ways to approach this problem; the one we'll explore here is the linear algebra approach.

## Preliminary Information

Before describing the least squares problem itself, we'll first recap some background material that it depends on.

### Linear Systems of Equations

Suppose we have the following system of equations relating two variables, $x_1$ and $x_2$:

```math
\begin{aligned}
x_1 &= 1,\\
x_1 + x_2 &= 2,\\
x_1 + 2x_2 &= 3.
\end{aligned}
```

Notice that not every equation includes both $x_1$ and $x_2$. We can rewrite them so that every term appears explicitly, using a coefficient of zero wherever a variable is missing:

```math
\begin{aligned}
x_1 + 0x_2 &= 1,\\
x_1 + x_2 &= 2,\\
x_1 + 2x_2 &= 3.
\end{aligned}
```

We could solve this system directly — by substitution — to find $x_1$ and $x_2$. But there's a more useful way to look at it, one that uses matrices and vectors instead.

We can collect the coefficients of $x_1$ and $x_2$ into a matrix, and the two variables themselves into a vector:

```math
\begin{bmatrix}
1 & 0\\
1 & 1\\
1 & 2
\end{bmatrix}
\begin{bmatrix}
x_1\\
x_2
\end{bmatrix}
=
\begin{bmatrix}
1\\
2\\
3
\end{bmatrix}
```

Multiplying the matrix by the vector of variables — taking the dot product of each row with the vector — reproduces exactly the system of equations we started with. Writing that out explicitly:

```math
\begin{bmatrix}
1 & 0\\
1 & 1\\
1 & 2
\end{bmatrix}
\begin{bmatrix}
x_1\\
x_2
\end{bmatrix}
=
\begin{bmatrix}
1x_1+0x_2\\
1x_1+1x_2\\
1x_1+2x_2
\end{bmatrix}
=
\begin{bmatrix}
1\\
2\\
3
\end{bmatrix}
```

Each row of the product is just the sum of that row's entries multiplied by the matching entries of $\mathbf{x}$ — and reading each row of the result as its own equation gives back exactly the three equations we started with.

This is called a **linear system of equations**, and it's usually written in the compact form

```math
\boxed{A\mathbf{x}=\mathbf{b}}
```

where $A$ is the **coefficient matrix**, $\mathbf{x}$ is the vector of unknowns, and $\mathbf{b}$ is the vector on the right-hand side.

Now let's set aside the right-hand side vector $\mathbf{b}$ entirely, and ask a different question: what vectors *can* the coefficient matrix $A$ produce? We're free to choose any values we like for $x_1$ and $x_2$, and $A$ will turn them into some vector.

For example, choosing $x_1=2$ and $x_2=3$ gives

```math
\begin{bmatrix}
1 & 0\\
1 & 1\\
1 & 2
\end{bmatrix}
\begin{bmatrix}
2\\
3
\end{bmatrix}
=
\begin{bmatrix}
2\\
5\\
8
\end{bmatrix}.
```

There are infinitely many combinations of $x_1$ and $x_2$ we could choose, each producing a different vector. But these vectors aren't arbitrary — every one of them is built from the same two columns of $A$, so they all share a common structure, as a few more examples show:

| $x_1$ | $x_2$ | $A\mathbf{x}$ |
|---|---|---|
| $2$ | $3$ | $(2,5,8)$ |
| $1$ | $1$ | $(1,2,3)$ |
| $0$ | $2$ | $(0,2,4)$ |
| $3$ | $0$ | $(3,3,3)$ |

Notice that choosing $x_1=1$ and $x_2=1$ reproduces $(1,2,3)$ — exactly the right-hand side from our very first example, since that's the solution we found there.

If we plot each of these resulting vectors as a point in three-dimensional space — one axis per equation — every single one lands on a flat plane that passes through the origin, no matter which $x_1$ and $x_2$ we picked:

![Four points, (2,5,8), (1,2,3), (0,2,4), and (3,3,3), each marked on a shared flat blue plane that passes through the origin. Two arrows, a1 and a2, point from the origin along the plane and are the columns of the matrix; every marked point can be reached by stretching and adding these two vectors together.](images/column_space_plane.png)

This plane — everywhere the matrix $A$ can reach — is called the **column space** of $A$, because it's made up of every possible linear combination of $A$'s two columns, $\mathbf{a}_1$ and $\mathbf{a}_2$:

```math
A=
\begin{bmatrix}
1 & 0\\
1 & 1\\
1 & 2
\end{bmatrix},
\qquad
\mathbf{a}_1=
\begin{bmatrix}
1\\
1\\
1
\end{bmatrix},
\qquad
\mathbf{a}_2=
\begin{bmatrix}
0\\
1\\
2
\end{bmatrix}
```

```math
\boxed{
\mathrm{Col}(A)=\{x_1\mathbf{a}_1+x_2\mathbf{a}_2 \mid x_1,x_2\in\mathbb{R}\}
}
```

## Reframing as a Data-Fitting Problem

The same linear system we've been using can be read in a completely different way: as fitting a line to a small set of data points.

Look again at the second column of $A$, which is $(0,1,2)$, and the vector $\mathbf{b}$, which is $(1,2,3)$. If we treat each row of the system as its own data point — using the entry from $A$'s second column as an input $t$, and the matching entry of $\mathbf{b}$ as an output $y$ — we get three points:

| Row | Input ($t$) | Output ($y$) |
|---|---|---|
| 1 | $0$ | $1$ |
| 2 | $1$ | $2$ |
| 3 | $2$ | $3$ |

![Three points plotted on a t-y graph: (0, 1), (1, 2), and (2, 3), each labeled with its coordinates.](images/data_points.png)

Written this way, each row of the system is really an equation for a line. The first column of $A$ is always $1$, so row $i$ says $x_1+x_2t_i=y_i$ — in other words, $y=x_2t+x_1$: a line with slope $x_2$ and intercept $x_1$. Those are the exact same $x_1$ and $x_2$ we solved for earlier. To avoid confusing this intercept with the vector $\mathbf{b}$, we'll call it $c$ instead, and write the line in the more familiar form

```math
\boxed{y=mt+c}
```

with $m=x_2$ and $c=x_1$.

We already solved this system and found $x_1=1$ and $x_2=1$, so $m=1$ and $c=1$. The solution to our linear system is really just the line $y=t+1$:

![The same three points, now with the line y equals t plus 1 drawn through them. The line passes exactly through all three points.](images/line_fit.png)

Every one of our three points lies exactly on this line, because the original system had an exact solution — there was an $x_1$ and $x_2$ that satisfied all three equations at once. Solving a linear system and fitting a line through data turn out to be the very same problem.


