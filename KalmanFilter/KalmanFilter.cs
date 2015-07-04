using System;
using MathNet.Numerics.LinearAlgebra;

namespace KalmanFilterPackage
{
    public class KalmanFilter
    {
        public KfParameters _kfParameters { get; private set; }
        /// <summary>
        /// Initialise optimal Kalman Filter
        /// </summary>
        /// <param name="kfParameters">Parameter object for this Kalman Filter</param>
        public void InitialiseKalmanFilter(KfParameters kfParameters)
        {
            this._kfParameters = kfParameters;
        }

        /// <summary>
        /// Forward process of Kalman Filter. Use this function only for normal Kalman Filter
        /// </summary>
        /// <param name="yt">Observation vector at time t</param>
        /// <returns></returns>
        public void Forward(Vector<double> yt)
        {
            this._kfParameters.AppendY(yt);
            this.Predict();
            this.Update();
        }

        /// <summary>
        /// Predict process of the Kalman Filter
        /// </summary>
        private void Predict()
        {
            this._kfParameters.AppendX(this._kfParameters.A * this._kfParameters.Xt);
            this._kfParameters.UpdateP(this._kfParameters.A * this._kfParameters.P * this._kfParameters.A.Transpose() + this._kfParameters.Q);
        }

        /// <summary>
        /// Update process of the Kalman Filter
        /// </summary>
        private void Update()
        {
            this._kfParameters.UpdateT();
            this._kfParameters.UpdateK(this._kfParameters.P * this._kfParameters.C.Transpose() * (this._kfParameters.C * this._kfParameters.P * this._kfParameters.C.Transpose() + this._kfParameters.R).Inverse());
            this._kfParameters.UpdateX(this._kfParameters.Xt + this._kfParameters.K * (this._kfParameters.Yt - this._kfParameters.C * this._kfParameters.Xt));
            this._kfParameters.UpdateP((Matrix<double>.Build.DenseIdentity(this._kfParameters.P.RowCount) - this._kfParameters.K * this._kfParameters.C) * this._kfParameters.P);
            
        }
    }
}
