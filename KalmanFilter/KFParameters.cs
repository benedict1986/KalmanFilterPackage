using System;
using MathNet.Numerics.LinearAlgebra;

namespace KalmanFilterPackage
{
    /// <summary>
    /// This is the parameters used by normal Kalman Filter
    /// The observation is the position
    /// The state is the position, velocity and acceleration
    /// </summary>
    public class KfParameters : Parameters
    {
        public new int T { get { return base.T; } set { base.T = value; } }
        public new int Dx { get { return base.Dx; } set { base.Dx = value; } }
        public new int Dy { get { return base.Dy; } set { base.Dy = value; } }
        public new double Dt { get { return base.Dt; } set { base.Dt = value; } }
        public new Matrix<double> X { get { return base.X; } private set { base.X = value; } }
        public Vector<double> Xt { get { return base.X.Column(this.T); } }
        public new Matrix<double> Y { get { return base.Y; } private set { base.Y = value; } }
        public Vector<double> Yt { get { return base.Y.Column(this.T); } }
        public new Matrix<double> A { get { return base.A; } private set { base.A = value; } }
        public new Matrix<double> B { get { return base.B; } private set { base.B = value; } }
        public new Matrix<double> C { get { return base.C; } private set { base.C = value; } }
        public new Matrix<double> P { get { return base.P; } private set { base.P = value; } }
        public new Matrix<double> Q { get { return base.Q; } private set { base.Q = value; } }
        public new Matrix<double> R { get { return base.R; } private set { base.R = value; } }
        public new Matrix<double> K { get { return base.K; } private set { base.K = value; } }

        public KfParameters(int dx, int dy, Vector<double> x, double dt, double p, double q, double r)
        {
            this.Dt = dt;
            this.T = 0;
            this.Dx = dx;
            this.Dy = dy;
            this.X = x.ToColumnMatrix();
            this.Y = Matrix<double>.Build.Dense(dy, 1, 0);
            this.GenerateA();
            this.GenerateC();
            this.GenerateP(p);
            this.GenerateQ(q);
            this.GenerateR(r);
        }

        /// <summary>
        /// Generate state transition matrix
        /// </summary>
        protected override sealed void GenerateA()
        {
            this.A = Matrix<double>.Build.DenseIdentity(this.Dx);
            this.A.SetSubMatrix(0, this.Dy, Matrix<double>.Build.DenseDiagonal(this.Dy, this.Dt));
            this.A.SetSubMatrix(this.Dy, 2 * this.Dy, Matrix<double>.Build.DenseDiagonal(this.Dy, this.Dt));
            this.A.SetSubMatrix(0, 2 * this.Dy, Matrix<double>.Build.DenseDiagonal(this.Dy, Math.Pow(this.Dt, 2) / 2));

        }
        /// <summary>
        /// Generate input matrix
        /// </summary>
        protected override sealed void GenerateB()
        {
            this.B = Matrix<double>.Build.DenseIdentity(this.Dy) * Math.Pow(this.Dt, 2) / 2;
            this.B = this.B.Append(Matrix<double>.Build.DenseIdentity(this.Dy) * this.Dt);
            this.B = this.B.Append(Matrix<double>.Build.DenseIdentity(this.Dy));
            this.B = this.B.Transpose();
        }
        /// <summary>
        /// Generate observation matrix
        /// </summary>
        protected override sealed void GenerateC()
        {
            this.C = Matrix<double>.Build.DenseIdentity(this.Dy);
            this.C = this.C.Append(Matrix<double>.Build.Dense(this.Dy, this.Dy, 0));
            this.C = this.C.Append(Matrix<double>.Build.Dense(this.Dy, this.Dy, 0));
        }
        /// <summary>
        /// Generate initial error covariance
        /// </summary>
        /// <param name="p">Coefficient of the components (the coefficient of different components can be different)</param>
        protected override sealed void GenerateP(Vector<double> p)
        {
            if (p.Count > this.Dx)
            {
                Console.WriteLine("[ GenerateP ] The length of p is longer than dimension of the state. The process will be stopped.");
                return;
            }
            if (p.Count < this.Dx)
                Console.WriteLine("[ GenerateP ] The length of p is shorter than dimension of the state. Zeros will be appended");

            this.P.SetSubMatrix(0, 0, Matrix<double>.Build.DenseOfColumnVectors(p));
        }
        /// <summary>
        /// Generate initial error covariance
        /// </summary>
        /// <param name="p">Coefficient of the components (the coefficient of different components are the same)</param>
        protected override sealed void GenerateP(double p)
        {
            this.P = Matrix<double>.Build.DenseIdentity(this.Dx) * p;
        }
        /// <summary>
        /// Generate system noise covariance
        /// </summary>
        /// <param name="q">Coefficient of the components (the coefficient of different components can be different)</param>
        protected override sealed void GenerateQ(Vector<double> q)
        {
            if (q.Count > this.Dx)
            {
                Console.WriteLine("[ GenerateQ ] The length of q is longer than dimension of the state. The process will be stopped.");
                return;
            }
            if (q.Count < this.Dx)
                Console.WriteLine("[ GenerateP ] The length of q is shorter than dimension of the state. Zeros will be appended");

            this.Q.SetSubMatrix(0, 0, Matrix<double>.Build.DenseOfColumnVectors(q));
        }
        /// <summary>
        /// Generate system noise covariance
        /// </summary>
        /// <param name="q">Coefficient of the components (the coefficient of different components are the same)</param>
        protected override sealed void GenerateQ(double q)
        {
            this.Q = Matrix<double>.Build.DenseIdentity(this.Dx) * q;
        }
        /// <summary>
        /// Generate measurement noise covariance
        /// </summary>
        /// <param name="r">Coefficient of the components (the coefficient of different components can be different)</param>
        protected override sealed void GenerateR(Vector<double> r)
        {
            if (r.Count > this.Dy)
            {
                Console.WriteLine("[ GenerateR ] The length of r is longer than dimension of the observation. The process will be stopped.");
                return;
            }
            if (r.Count < this.Dy)
                Console.WriteLine("[ GenerateP ] The length of r is shorter than dimension of the observation. Zeros will be appended");

            this.R.SetSubMatrix(0, 0, Matrix<double>.Build.DenseOfColumnVectors(r));
        }
        /// <summary>
        /// Generate system noise covariance
        /// </summary>
        /// <param name="r">Coefficient of the components (the coefficient of different components are the same)</param>
        protected override sealed void GenerateR(double r)
        {
            this.R = Matrix<double>.Build.DenseIdentity(this.Dy) * r;
        }
        /// <summary>
        /// Append observation vector into observation matrix
        /// </summary>
        /// <param name="y">Observation vector</param>
        public override void AppendY(Vector<double> y)
        {
            this.Y = this.Y.Append(y.ToColumnMatrix());
        }
        /// <summary>
        /// Append state vector into state matrix
        /// </summary>
        /// <param name="x">State vector</param>
        public override void AppendX(Vector<double> x)
        {
            this.X = this.X.Append(x.ToColumnMatrix());
        }
        /// <summary>
        /// Update state vector at time T
        /// </summary>
        /// <param name="x">State vector</param>
        public override void UpdateX(Vector<double> x)
        {
            this.X.SetColumn(this.T, x);
        }
        /// <summary>
        /// Update state vector at time t
        /// </summary>
        /// <param name="x">State vector</param>
        /// <param name="t">Time index</param>
        public override void UpdateX(Vector<double> x, int t)
        {
            this.X.SetColumn(t, x);
        }
        /// <summary>
        /// Update observation vector at time T
        /// </summary>
        /// <param name="y">State vector</param>
        public override void UpdateY(Vector<double> y)
        {
            this.Y.SetColumn(this.T, y);
        }
        /// <summary>
        /// Update state vector at time t
        /// </summary>
        /// <param name="y">State vector</param>
        /// <param name="t">Time index</param>
        public override void UpdateY(Vector<double> y, int t)
        {
            this.Y.SetColumn(t, y);
        }
        /// <summary>
        /// Update error covariance
        /// </summary>
        /// <param name="p">Error covariance</param>
        public override void UpdateP(Matrix<double> p)
        {
            this.P = p;
        }
        /// <summary>
        /// Update Kalman gain
        /// </summary>
        /// <param name="k">Kalman gain</param>
        public override void UpdateK(Matrix<double> k)
        {
            this.K = k;
        }
        /// <summary>
        /// Update current time T
        /// </summary>
        public override void UpdateT()
        {
            this.T += 1;
        }
    }
}