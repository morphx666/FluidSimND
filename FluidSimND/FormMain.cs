using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

// https://mikeash.com/pyblog/fluid-simulation-for-dummies.html
// https://www.mikeash.com/thesis/thesis-en.pdf

namespace FluidSim2 {
    public partial class FormMain : Form {
        private Fluid2D fluid;

        private int size = 96;
        private int zoom = 7;
        private int z;
        private bool isDrawing = false;
        private Point mousePosition;

        private readonly object syncObj = new object();

        public FormMain() {
            InitializeComponent();

            this.SetStyle(ControlStyles.AllPaintingInWmPaint, true);
            this.SetStyle(ControlStyles.UserPaint, true);
            this.SetStyle(ControlStyles.OptimizedDoubleBuffer, true);
        }

        private void FormMain_Load(object sender, EventArgs e) {
            fluid = new Fluid2D(size, 0.000005f, 0.0000001f, 0.02f);

            this.Text += " " + (fluid is Fluid2D ? "2D" : "3D");

            z = fluid.size / 2;
            this.Size = new Size(fluid.size * zoom, fluid.size * zoom);

            this.Paint += DrawFluidBW;
            //this.Paint += DrawFluidColor;
            this.MouseDown += (_, __) => { mousePosition = __.Location; isDrawing = true; };
            this.MouseMove += MouseDraw;
            this.MouseUp += (_, __) => { isDrawing = false; };

            Task.Run(() => {
                while(true) {
                    lock(syncObj) {
                        // Simulate floating (less dense than air) fluid
                        //for(int y = 0; y < fluid.size; y++) {
                        //    for(int x = 0; x < fluid.size; x++) {
                        //        fluid.AddVelocity(x, y, z, 0, -0.00005f, 0);

                        //    }
                        //}
                        fluid.Step();
                    }
                    Thread.Sleep(1);
                }
            });

            Task.Run(() => {
                while(true) {
                    this.Invalidate();
                    Thread.Sleep(30);
                }
            });
        }

        private void MouseDraw(object sender, MouseEventArgs e) {
            if(isDrawing && e.X >= 0 && e.Y >= 0) {
                lock(syncObj) {
                    fluid.AddDensity(e.X / zoom, e.Y / zoom, z, 3.0f);
                    fluid.AddVelocity(e.X / zoom, e.Y / zoom, z,
                            (e.X - mousePosition.X) * 0.5f,
                            (e.Y - mousePosition.Y) * 0.5f,
                            0.0f);
                }
                mousePosition = e.Location;
            }
        }

        private void DrawFluidColor(object sender, PaintEventArgs e) {
            Graphics g = e.Graphics;

            for(int y = 0; y < fluid.size; y++) {
                for(int x = 0; x < fluid.size; x++) {
                    double d = fluid.density[fluid.Ix(x, y, z)];
                    HLSRGB hls = new HLSRGB(360.0 * Math.Min(1.0, d),
                        Math.Min(1.0, d),
                        1.0) {
                        //Hue = 360.0 * Math.Sqrt(Math.Pow(fluid.Vx[ix], 2) + Math.Pow(fluid.Vy[ix], 2)),
                        Alpha = (int)(255 * d * 4)
                    };
                    using SolidBrush b = new SolidBrush(hls.Color);
                    g.FillRectangle(b, x * zoom, y * zoom, zoom, zoom);
                }
            }
        }

        private void DrawFluidBW(object sender, PaintEventArgs e) {
            Graphics g = e.Graphics;

            for(int y = 0; y < fluid.size; y++) {
                for(int x = 0; x < fluid.size; x++) {
                    int d = (int)Math.Min(255, 255 * fluid.density[fluid.Ix(x, y, z)]);
                    using SolidBrush b = new SolidBrush(Color.FromArgb(d, Color.White));
                    g.FillRectangle(b, x * zoom, y * zoom, zoom, zoom);
                }
            }
        }
    }
}