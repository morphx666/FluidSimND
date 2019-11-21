using System;

namespace FluidSim2 {
    public class Fluid2D {
        public int size;
        public float dt;
        public float diff;
        public float visc;

        public float[] s;
        public float[] density;

        public float[] Vx;
        public float[] Vy;

        public float[] Vx0;
        public float[] Vy0;

        private readonly int N;
        private readonly int N2;
        private readonly int iter = 2;
        private readonly float dtd;

        public Fluid2D(int size, float diffusion, float viscosity, float dt) {
            N = size;
            N2 = N * N;

            dtd = dt * (N - 2);

            this.size = size;
            this.dt = dt;
            this.diff = diffusion;
            this.visc = viscosity;

            this.s = new float[N2];
            this.density = new float[N2];

            this.Vx = new float[N2];
            this.Vy = new float[N2];

            this.Vx0 = new float[N2];
            this.Vy0 = new float[N2];
        }

        public int Ix(int x, int y, int dummyZ) {
            if(x >= N) x = N - 1;
            if(y >= N) y = N - 1;
            return (x + y * N);
        }

        public int Ix(int x, int y) {
            return Ix(x, y, 0);
        }

        private void SetBounds(int b, float[] x) {
            for(int i = 1; i < N - 1; i++) {
                x[Ix(i, 0)] = b == 2 ? -x[Ix(i, 1)] : x[Ix(i, 1)];
                x[Ix(i, N - 1)] = b == 2 ? -x[Ix(i, N - 2)] : x[Ix(i, N - 2)];
            }
            for(int j = 1; j < N - 1; j++) {
                x[Ix(0, j)] = b == 1 ? -x[Ix(1, j)] : x[Ix(1, j)];
                x[Ix(N - 1, j)] = b == 1 ? -x[Ix(N - 2, j)] : x[Ix(N - 2, j)];
            }

            x[Ix(0, 0)] = 0.5f * (x[Ix(1, 0)] + x[Ix(0, 1)]);
            x[Ix(0, N - 1)] = 0.5f * (x[Ix(1, N - 1)] + x[Ix(0, N - 2)]);
            x[Ix(N - 1, 0)] = 0.5f * (x[Ix(N - 2, 0)] + x[Ix(N - 1, 1)]);
            x[Ix(N - 1, N - 1)] = 0.5f * (x[Ix(N - 2, N - 1)] + x[Ix(N - 1, N - 2)]);
        }

        private void LinearSolve(int b, float[] x, float[] x0, float a, float c) {
            float cRecip = 1.0f / c;
            for(int k = 0; k < iter; k++) {
                for(int j = 1; j < N - 1; j++) {
                    for(int i = 1; i < N - 1; i++) {
                        x[Ix(i, j)] =
                            (x0[Ix(i, j)]
                                   + a * (x[Ix(i + 1, j)]
                                        + x[Ix(i - 1, j)]
                                        + x[Ix(i, j + 1)]
                                        + x[Ix(i, j - 1)]
                               )) * cRecip;
                    }
                    SetBounds(b, x);
                }
            }
        }

        private void Diffuse(int b, ref float[] x, ref float[] x0, float diff) {
            float a = dt * diff * (N - 2) * (N - 2);
            LinearSolve(b, x, x0, a, 1 + 6 * a);
        }

        private void Advect(int b, ref float[] d, ref float[] d0, ref float[] velocX, ref float[] velocY) {
            float i0, i1, j0, j1;
            float s0, s1, t0, t1;
            float tmp1, tmp2, x, y;

            float Nfloat = N + 0.5f; // This is important, as the conversion from int to float requires many CPU cycles.
            float ifloat, jfloat;
            int i, j;

            for(j = 1, jfloat = 1; j < N - 1; j++, jfloat++) {
                for(i = 1, ifloat = 1; i < N - 1; i++, ifloat++) {
                    tmp1 = dtd * velocX[Ix(i, j)];
                    tmp2 = dtd * velocY[Ix(i, j)];
                    x = ifloat - tmp1;
                    y = jfloat - tmp2;

                    if(x < 0.5f) x = 0.5f;
                    if(x > Nfloat) x = Nfloat;
                    i0 = (float)Math.Floor(x);
                    i1 = i0 + 1.0f;
                    if(y < 0.5f) y = 0.5f;
                    if(y > Nfloat) y = Nfloat;
                    j0 = (float)Math.Floor(y);
                    j1 = j0 + 1.0f;

                    s1 = x - i0;
                    s0 = 1.0f - s1;
                    t1 = y - j0;
                    t0 = 1.0f - t1;

                    int i0i = (int)i0;
                    int i1i = (int)i1;
                    int j0i = (int)j0;
                    int j1i = (int)j1;

                    d[Ix(i, j)] =
                      s0 * (t0 * d0[Ix(i0i, j0i)] + t1 * d0[Ix(i0i, j1i)]) +
                      s1 * (t0 * d0[Ix(i1i, j0i)] + t1 * d0[Ix(i1i, j1i)]);
                }
            }

            SetBounds(b, d);
        }

        private void Project(ref float[] velocX, ref float[] velocY, ref float[] p, ref float[] div) {
            for(int j = 1; j < N - 1; j++) {
                for(int i = 1; i < N - 1; i++) {
                    div[Ix(i, j)] = -0.5f * (
                              velocX[Ix(i + 1, j)]
                            - velocX[Ix(i - 1, j)]
                            + velocY[Ix(i, j + 1)]
                            - velocY[Ix(i, j - 1)]
                        ) / N;
                    p[Ix(i, j)] = 0;
                }
            }

            SetBounds(0, div);
            SetBounds(0, p);
            LinearSolve(0, p, div, 1, 6);

            for(int j = 1; j < N - 1; j++) {
                for(int i = 1; i < N - 1; i++) {
                    velocX[Ix(i, j)] -= 0.5f * (p[Ix(i + 1, j)]
                                              - p[Ix(i - 1, j)]) * N;
                    velocY[Ix(i, j)] -= 0.5f * (p[Ix(i, j + 1)]
                                              - p[Ix(i, j - 1)]) * N;
                }
            }

            SetBounds(1, velocX);
            SetBounds(2, velocY);
        }

        public void Step() {
            Diffuse(1, ref Vx0, ref Vx, visc);
            Diffuse(2, ref Vy0, ref Vy, visc);

            Project(ref Vx0, ref Vy0, ref Vx, ref Vy);

            Advect(1, ref Vx, ref Vx0, ref Vx0, ref Vy0);
            Advect(2, ref Vy, ref Vy0, ref Vx0, ref Vy0);

            Project(ref Vx, ref Vy, ref Vx0, ref Vy0);

            Diffuse(0, ref s, ref density, diff);
            Advect(0, ref density, ref s, ref Vx, ref Vy);
        }

        public void AddDensity(int x, int y, float dummyZ, float amount) {
            density[Ix(x, y)] += amount;
        }

        public void AddVelocity(int x, int y, float dummyZ, float amountX, float amountY, float dummyAmountZ) {
            int index = Ix(x, y);

            Vx[index] += amountX;
            Vy[index] += amountY;
        }
    }
}