using Unity.Mathematics;

namespace Yohash.Propulsion.Entities.Tests.Physics2D
{
  /// <summary>
  /// Measures the frequency and damping of a recorded error trajectory. A 2nd-order linear
  /// system's error obeys e[k+2] = S e[k+1] - P e[k]; S and P are fitted by least squares, and
  /// the discrete poles they give are mapped back to a continuous frequency and damping ratio.
  /// </summary>
  internal struct MeasuredResponse
  {
    public double Frequency;
    public double Damping;

    public static MeasuredResponse Fit(float[] error, double dt)
    {
      // least squares for [S, P] in e[k+2] = S e[k+1] - P e[k]
      double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
      for (int k = 0; k + 2 < error.Length; k++) {
        double x1 = error[k + 1], x2 = -error[k], y = error[k + 2];
        a11 += x1 * x1; a12 += x1 * x2; a22 += x2 * x2;
        b1 += x1 * y; b2 += x2 * y;
      }
      double det = a11 * a22 - a12 * a12;
      double s = (b1 * a22 - b2 * a12) / det;
      double p = (a11 * b2 - a12 * b1) / det;

      // poles: |z|^2 = P, 2 Re(z) = S
      double decayRate = -math.log(p) / (2.0 * dt);                          // damping * omega
      double dampedOmega = math.acos(math.clamp(s / (2.0 * math.sqrt(p)), -1.0, 1.0)) / dt;
      double omega = math.sqrt(decayRate * decayRate + dampedOmega * dampedOmega);
      return new MeasuredResponse {
        Frequency = omega / (2.0 * math.PI_DBL),
        Damping = decayRate / omega,
      };
    }

    public override string ToString() => $"{Frequency:F4} Hz, damping {Damping:F4}";
  }
}
