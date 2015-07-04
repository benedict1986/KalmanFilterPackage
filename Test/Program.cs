using KalmanFilterPackage;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.Data.Matlab;

namespace Test
{
    class Program
    {
        static void Main(string[] args)
        {
            int T = 1000;
            double dt = 1;
            Matrix<double> trajectory = Matrix<double>.Build.Dense(3, T, 0);
            for (int t = 0; t < T; t++)
            {
                trajectory[0, t] = Math.Cos(2*Math.PI*t/T);
                trajectory[1, t] = Math.Sin(2 * Math.PI * t / T);
                trajectory[2, t] = 0;
            }
            string name = DateTime.Now.ToString("dd_MM_yyyy_hh_mm_ss");
            MatlabWriter.Write<double>(name + "_clean" + ".mat", trajectory, "X1");
            trajectory += Matrix<double>.Build.Random(3, T)/10;
            MatlabWriter.Write<double>(name + "_raw" + ".mat", trajectory, "X2");
            Vector<double> x = Vector<double>.Build.Dense(9, 0);
            //x.SetSubVector(0, 3, trajectory.Column(0));
            double p = 0.00001;
            double q = 0.00000001;
            double r = 1;
            KfParameters parameters = new KfParameters(9, 3, x, dt, p, q, r);
            KalmanFilter kalmanfilter = new KalmanFilter();
            kalmanfilter.InitialiseKalmanFilter(parameters);
            for (int t = 0; t < T; t++)
            {
                kalmanfilter.Forward(trajectory.Column(t));
            }
            MatlabWriter.Write<double>(name + "_filtered" + ".mat", kalmanfilter._kfParameters.X, "X3");
        }
    }
}
