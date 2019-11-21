using System;

namespace FluidSim2 {
    public class Fluid3D {
        public int size;
        public float dt;
        public float diff;
        public float visc;

        public float[] s;
        public float[] density;

        public float[] Vx;
        public float[] Vy;
        public float[] Vz;

        public float[] Vx0;
        public float[] Vy0;
        public float[] Vz0;

        private readonly int N;
        private readonly int N2;
        private readonly int N3;
        private readonly int iter = 2;
        private readonly float dtd;

        public Fluid3D(int size, float diffusion, float viscosity, float dt) {
            N = size;
            N2 = N * N;
            N3 = N * N2;

            dtd = dt * (N - 2);

            this.size = size;
            this.dt = dt;
            this.diff = diffusion;
            this.visc = viscosity;

            this.s = new float[N3];
            this.density = new float[N3];

            this.Vx = new float[N3];
            this.Vy = new float[N3];
            this.Vz = new float[N3];

            this.Vx0 = new float[N3];
            this.Vy0 = new float[N3];
            this.Vz0 = new float[N3];
        }

        public int Ix(int x, int y, int z) {
            if(x >= N) x = N - 1;
            if(y >= N) y = N - 1;
            if(z >= N) z = N - 1;
            return (x + y * N + z * N2);
        }

        private void SetBounds(int b, float[] x) {
            for(int j = 1; j < N - 1; j++) {
                for(int i = 1; i < N - 1; i++) {
                    x[Ix(i, j, 0)] = b == 3 ? -x[Ix(i, j, 1)] : x[Ix(i, j, 1)];
                    x[Ix(i, j, N - 1)] = b == 3 ? -x[Ix(i, j, N - 2)] : x[Ix(i, j, N - 2)];
                }
            }
            for(int k = 1; k < N - 1; k++) {
                for(int i = 1; i < N - 1; i++) {
                    x[Ix(i, 0, k)] = b == 2 ? -x[Ix(i, 1, k)] : x[Ix(i, 1, k)];
                    x[Ix(i, N - 1, k)] = b == 2 ? -x[Ix(i, N - 2, k)] : x[Ix(i, N - 2, k)];
                }
            }
            for(int k = 1; k < N - 1; k++) {
                for(int j = 1; j < N - 1; j++) {
                    x[Ix(0, j, k)] = b == 1 ? -x[Ix(1, j, k)] : x[Ix(1, j, k)];
                    x[Ix(N - 1, j, k)] = b == 1 ? -x[Ix(N - 2, j, k)] : x[Ix(N - 2, j, k)];
                }
            }

            x[Ix(0, 0, 0)] = 0.33f * (x[Ix(1, 0, 0)]
                                          + x[Ix(0, 1, 0)]
                                          + x[Ix(0, 0, 1)]);
            x[Ix(0, N - 1, 0)] = 0.33f * (x[Ix(1, N - 1, 0)]
                                          + x[Ix(0, N - 2, 0)]
                                          + x[Ix(0, N - 1, 1)]);
            x[Ix(0, 0, N - 1)] = 0.33f * (x[Ix(1, 0, N - 1)]
                                          + x[Ix(0, 1, N - 1)]
                                          + x[Ix(0, 0, N)]);
            x[Ix(0, N - 1, N - 1)] = 0.33f * (x[Ix(1, N - 1, N - 1)]
                                          + x[Ix(0, N - 2, N - 1)]
                                          + x[Ix(0, N - 1, N - 2)]);
            x[Ix(N - 1, 0, 0)] = 0.33f * (x[Ix(N - 2, 0, 0)]
                                          + x[Ix(N - 1, 1, 0)]
                                          + x[Ix(N - 1, 0, 1)]);
            x[Ix(N - 1, N - 1, 0)] = 0.33f * (x[Ix(N - 2, N - 1, 0)]
                                          + x[Ix(N - 1, N - 2, 0)]
                                          + x[Ix(N - 1, N - 1, 1)]);
            x[Ix(N - 1, 0, N - 1)] = 0.33f * (x[Ix(N - 2, 0, N - 1)]
                                          + x[Ix(N - 1, 1, N - 1)]
                                          + x[Ix(N - 1, 0, N - 2)]);
            x[Ix(N - 1, N - 1, N - 1)] = 0.33f * (x[Ix(N - 2, N - 1, N - 1)]
                                          + x[Ix(N - 1, N - 2, N - 1)]
                                          + x[Ix(N - 1, N - 1, N - 2)]);
        }

        private void LinearSolve(int b, float[] x, float[] x0, float a, float c) {
            float cRecip = 1.0f / c;
            for(int k = 0; k < iter; k++) {
                for(int m = 1; m < N - 1; m++) {
                    for(int j = 1; j < N - 1; j++) {
                        for(int i = 1; i < N - 1; i++) {
                            x[Ix(i, j, m)] =
                                (x0[Ix(i, j, m)]
                                    + a * (x[Ix(i + 1, j, m)]
                                            + x[Ix(i - 1, j, m)]
                                            + x[Ix(i, j + 1, m)]
                                            + x[Ix(i, j - 1, m)]
                                            + x[Ix(i, j, m + 1)]
                                            + x[Ix(i, j, m - 1)]
                                   )) * cRecip;
                        }
                    }
                }
                SetBounds(b, x);
            }
        }

        private void Diffuse(int b, float[] x, float[] x0, float diff) {
            float a = dt * diff * (N - 2) * (N - 2);
            LinearSolve(b, x, x0, a, 1 + 6 * a);
        }

        private void Advect(int b, float[] d, float[] d0, float[] velocX, float[] velocY, float[] velocZ) {
            float i0, i1, j0, j1, k0, k1;

            float s0, s1, t0, t1, u0, u1;
            float tmp1, tmp2, tmp3, x, y, z;

            float Nfloat = N + 0.5f; // This is important, as the conversion from int to float requires many CPU cycles.
            float ifloat, jfloat, kfloat;
            int i, j, k;

            for(k = 1, kfloat = 1; k < N - 1; k++, kfloat++) {
                for(j = 1, jfloat = 1; j < N - 1; j++, jfloat++) {
                    for(i = 1, ifloat = 1; i < N - 1; i++, ifloat++) {
                        tmp1 = dtd * velocX[Ix(i, j, k)];
                        tmp2 = dtd * velocY[Ix(i, j, k)];
                        tmp3 = dtd * velocZ[Ix(i, j, k)];
                        x = ifloat - tmp1;
                        y = jfloat - tmp2;
                        z = kfloat - tmp3;

                        if(x < 0.5f) x = 0.5f;
                        if(x > Nfloat) x = Nfloat;
                        i0 = (float)Math.Floor(x);
                        i1 = i0 + 1.0f;
                        if(y < 0.5f) y = 0.5f;
                        if(y > Nfloat) y = Nfloat;
                        j0 = (float)Math.Floor(y);
                        j1 = j0 + 1.0f;
                        if(z < 0.5f) z = 0.5f;
                        if(z > Nfloat) z = Nfloat;
                        k0 = (float)Math.Floor(z);
                        k1 = k0 + 1.0f;

                        s1 = x - i0;
                        s0 = 1.0f - s1;
                        t1 = y - j0;
                        t0 = 1.0f - t1;
                        u1 = z - k0;
                        u0 = 1.0f - u1;

                        int i0i = (int)i0;
                        int i1i = (int)i1;
                        int j0i = (int)j0;
                        int j1i = (int)j1;
                        int k0i = (int)k0;
                        int k1i = (int)k1;

                        d[Ix(i, j, k)] =
                            s0 * (t0 * (u0 * d0[Ix(i0i, j0i, k0i)]
                                      + u1 * d0[Ix(i0i, j0i, k1i)])
                               + (t1 * (u0 * d0[Ix(i0i, j1i, k0i)]
                                      + u1 * d0[Ix(i0i, j1i, k1i)])))
                          + s1 * (t0 * (u0 * d0[Ix(i1i, j0i, k0i)]
                                      + u1 * d0[Ix(i1i, j0i, k1i)])
                               + (t1 * (u0 * d0[Ix(i1i, j1i, k0i)]
                                      + u1 * d0[Ix(i1i, j1i, k1i)])));
                    }
                }
            }
            SetBounds(b, d);
        }

        private void Project(float[] velocX, float[] velocY, float[] velocZ, float[] p, float[] div) {
            for(int k = 1; k < N - 1; k++) {
                for(int j = 1; j < N - 1; j++) {
                    for(int i = 1; i < N - 1; i++) {
                        div[Ix(i, j, k)] = -0.5f * (
                                 velocX[Ix(i + 1, j, k)]
                                - velocX[Ix(i - 1, j, k)]
                                + velocY[Ix(i, j + 1, k)]
                                - velocY[Ix(i, j - 1, k)]
                                + velocZ[Ix(i, j, k + 1)]
                                - velocZ[Ix(i, j, k - 1)]
                            ) / N;
                        p[Ix(i, j, k)] = 0;
                    }
                }
            }
            SetBounds(0, div);
            SetBounds(0, p);
            LinearSolve(0, p, div, 1, 6);

            for(int k = 1; k < N - 1; k++) {
                for(int j = 1; j < N - 1; j++) {
                    for(int i = 1; i < N - 1; i++) {
                        velocX[Ix(i, j, k)] -= 0.5f * (p[Ix(i + 1, j, k)]
                                                     - p[Ix(i - 1, j, k)]) * N;
                        velocY[Ix(i, j, k)] -= 0.5f * (p[Ix(i, j + 1, k)]
                                                     - p[Ix(i, j - 1, k)]) * N;
                        velocZ[Ix(i, j, k)] -= 0.5f * (p[Ix(i, j, k + 1)]
                                                     - p[Ix(i, j, k - 1)]) * N;
                    }
                }
            }
            SetBounds(1, velocX);
            SetBounds(2, velocY);
            SetBounds(3, velocZ);
        }

        public void Step() {
            Diffuse(1, Vx0, Vx, visc);
            Diffuse(2, Vy0, Vy, visc);
            Diffuse(3, Vz0, Vz, visc);

            Project(Vx0, Vy0, Vz0, Vx, Vy);

            Advect(1, Vx, Vx0, Vx0, Vy0, Vz0);
            Advect(2, Vy, Vy0, Vx0, Vy0, Vz0);
            Advect(3, Vz, Vz0, Vx0, Vy0, Vz0);

            Project(Vx, Vy, Vz, Vx0, Vy0);

            Diffuse(0, s, density, diff);
            Advect(0, density, s, Vx, Vy, Vz);
        }

        public void AddDensity(int x, int y, int z, float amount) {
            density[Ix(x, y, z)] += amount;
        }

        public void AddVelocity(int x, int y, int z, float amountX, float amountY, float amountZ) {
            int index = Ix(x, y, z);

            Vx[index] += amountX;
            Vy[index] += amountY;
            Vz[index] += amountZ;
        }
    }
}
