using System;
using MathNet.Numerics.LinearAlgebra;

namespace KalmanFilterPackage
{
    public class Parameters
    {

        ///// <summary>
        /////     Initialise parameters with P, Q and R initialised with values
        ///// </summary>
        ///// <param name="dy">Dimension of observation</param>
        ///// <param name="x">State vector at time 0</param>
        ///// <param name="a">State tansition matrix</param>
        ///// <param name="b">Input matrix</param>
        ///// <param name="c">Observation matrix</param>
        ///// <param name="p">Error covariance</param>
        ///// <param name="q">System noise covariance</param>
        ///// <param name="r">Measurement noise covariance</param>
        //protected Parameters(int dy,
        //    Vector<double> x,
        //    Matrix<double> a,
        //    Matrix<double> b,
        //    Matrix<double> c,
        //    double p,
        //    double q,
        //    double r)
        //{
        //    this.T = 0;
        //    this.Dy = dy;
        //    this.Dx = x.Count;
        //    this.Y = Matrix<double>.Build.Dense(dy, 1, 0);
        //    this.X = x.ToColumnMatrix();
        //    this.A = a;
        //    this.B = b;
        //    this.C = c;
        //    this.P = p * Matrix<double>.Build.DenseIdentity(this.Dy);
        //    this.Q = q * Matrix<double>.Build.DenseIdentity(this.Dx);
        //    this.R = r * Matrix<double>.Build.DenseIdentity(this.Dy);
        //}

        ///// <summary>
        /////     Initialise parameters with P, Q and R initialised with vectors
        ///// </summary>
        ///// <param name="dy">Dimension of observation</param>
        ///// <param name="x">State vector at time 0</param>
        ///// <param name="a">State tansition matrix</param>
        ///// <param name="b">Input matrix</param>
        ///// <param name="c">Observation matrix</param>
        ///// <param name="p">Error covariance</param>
        ///// <param name="q">System noise covariance</param>
        ///// <param name="r">Measurement noise covariance</param>
        //protected Parameters(int dy,
        //    Vector<double> x,
        //    Matrix<double> a,
        //    Matrix<double> b,
        //    Matrix<double> c,
        //    Vector<double> p,
        //    Vector<double> q,
        //    Vector<double> r)
        //{
        //    this.T = 0;
        //    this.Dy = dy;
        //    this.Dx = x.Count;
        //    this.Y = Matrix<double>.Build.Dense(dy, 1, 0);
        //    this.X = x.ToColumnMatrix();
        //    this.A = a;
        //    this.B = b;
        //    this.C = c;
        //    this.P = Matrix<double>.Build.DiagonalOfDiagonalVector(p) *
        //             Matrix<double>.Build.DenseIdentity(this.Dy);
        //    this.Q = Matrix<double>.Build.DiagonalOfDiagonalVector(q) *
        //             Matrix<double>.Build.DenseIdentity(this.Dx);
        //    this.R = Matrix<double>.Build.DiagonalOfDiagonalVector(r) *
        //             Matrix<double>.Build.DenseIdentity(this.Dy);
        //}

        ///// <summary>
        /////     Initialise parameters with P, Q and R initialised with matrices
        ///// </summary>
        ///// <param name="dy">Dimension of observation</param>
        ///// <param name="x">State vector at time 0</param>
        ///// <param name="a">State tansition matrix</param>
        ///// <param name="b">Input matrix</param>
        ///// <param name="c">Observation matrix</param>
        ///// <param name="p">Error covariance</param>
        ///// <param name="q">System noise covariance</param>
        ///// <param name="r">Measurement noise covariance</param>
        //protected Parameters(int dy,
        //    Vector<double> x,
        //    Matrix<double> a,
        //    Matrix<double> b,
        //    Matrix<double> c,
        //    Matrix<double> p,
        //    Matrix<double> q,
        //    Matrix<double> r)
        //{
        //    this.T = 0;
        //    this.Dy = dy;
        //    this.Dx = x.Count;
        //    this.Y = Matrix<double>.Build.Dense(dy, 1, 0);
        //    this.X = x.ToColumnMatrix();
        //    this.A = a;
        //    this.B = b;
        //    this.C = c;
        //    this.P = p;
        //    this.Q = q;
        //    this.R = r;
        //}

        protected int T { get; set; }
        protected int Dx { get; set; }
        protected int Dy { get; set; }
        protected double Dt { get; set; }
        protected Matrix<double> X { get; set; }
        protected Matrix<double> Y { get; set; }
        protected Matrix<double> A { get; set; }
        protected Matrix<double> B { get; set; }
        protected Matrix<double> C { get; set; }
        protected Matrix<double> P { get; set; }
        protected Matrix<double> Q { get; set; }
        protected Matrix<double> R { get; set; }
        protected Matrix<double> K { get; set; }

        protected virtual void GenerateA()
        {
        }
        protected virtual void GenerateB()
        {
        }
        protected virtual void GenerateC()
        {
        }
        protected virtual void GenerateP(Vector<double> p)
        {
        }
        protected virtual void GenerateP(double p)
        {
        }
        protected virtual void GenerateQ(Vector<double> q)
        {
        }
        protected virtual void GenerateQ(double q)
        {
        }
        protected virtual void GenerateR(Vector<double> r)
        {
        }
        protected virtual void GenerateR(double r)
        {

        }
        public virtual void AppendY(Vector<double> y)
        {
        }
        public virtual void AppendX(Vector<double> x)
        { }
        public virtual void UpdateX(Vector<double> x)
        { }
        public virtual void UpdateX(Vector<double> x, int t)
        { }
        public virtual void UpdateY(Vector<double> y)
        {
            
        }
        public virtual void UpdateY(Vector<double> x, int t)
        {
        }
        public virtual void UpdateP(Matrix<double> p)
        { }
        public virtual void UpdateK(Matrix<double> k)
        { }
        public virtual void UpdateT() 
        { }

        //#region Properties Operators

        ///// <summary>
        /////     Append the observation at time t to observation matrix
        ///// </summary>
        ///// <param name="yt">Observation at time t</param>
        //public void AppendY(Vector<double> yt)
        //{
        //    this.Y.Append(yt.ToColumnMatrix());
        //}

        ///// <summary>
        /////     Append the predicted state at time t to state matrix
        ///// </summary>
        ///// <param name="xt">Predicted state at time t</param>
        //public void AppendX(Vector<double> xt)
        //{
        //    this.X.Append(xt.ToColumnMatrix());
        //}

        ///// <summary>
        /////     Update state at time t, usually used by the update process
        ///// </summary>
        ///// <param name="x">State at time t</param>
        //public void UpdateX(Vector<double> x)
        //{
        //    this.X.SetColumn(this.T, x);
        //}

        ///// <summary>
        /////     Update error covariance
        ///// </summary>
        ///// <param name="p">Error covariance</param>
        //public void UpdateP(Matrix<double> p)
        //{
        //    this.P = p;
        //}

        ///// <summary>
        /////     Update Kalman gain
        ///// </summary>
        ///// <param name="k">Kalman gain</param>
        //public void UpdateK(Matrix<double> k)
        //{
        //    this.K = k;
        //}

        ///// <summary>
        /////     Update time T
        ///// </summary>
        //public void UpdateT()
        //{
        //    this.T += 1;
        //}

        //#endregion
    }
}