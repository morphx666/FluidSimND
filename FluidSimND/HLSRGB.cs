using System;
using System.Drawing;

[Serializable()]
public class HLSRGB {
    private byte mRed = 0;
    private byte mGreen = 0;
    private byte mBlue = 0;

    private double mHue = 0;
    private double mLuminance = 0;
    private double mSaturation = 0;
    private byte mAlpha;

    public struct HueLumSat {
        public double Hue { get; set; }
        public double Lum { get; set; }
        public double Sat { get; set; }

        public HueLumSat(double hue, double lum, double sat) {
            Hue = hue;
            Lum = lum;
            Sat = sat;
        }

        public static bool operator ==(HueLumSat v1, HueLumSat v2) {
            return (v1.Hue == v2.Hue) && (v1.Lum == v2.Lum) && (v1.Sat == v2.Sat);
        }

        public static bool operator !=(HueLumSat v1, HueLumSat v2) {
            return !(v1 == v2);
        }

        public override bool Equals(object obj) {
            return this == (HueLumSat)obj;
        }
    }

    public HLSRGB(Color color) {
        mAlpha = color.A;
        mRed = color.R;
        mGreen = color.G;
        mBlue = color.B;
        ToHLS();
    }

    public HLSRGB(double hue, double luminance, double saturation) {
        mAlpha = 255;
        mHue = ChkHue(hue);
        mLuminance = ChkLum(luminance);
        mSaturation = ChkSat(saturation);
        ToRGB();
    }

    public HLSRGB(byte alpha, byte red, byte green, byte blue) {
        mAlpha = alpha;
        mRed = red;
        mGreen = green;
        mBlue = blue;
    }

    public HLSRGB(HLSRGB hlsrgb) {
        mRed = hlsrgb.Red;
        mBlue = hlsrgb.Blue;
        mGreen = hlsrgb.Green;
        mLuminance = hlsrgb.Luminance;
        mHue = hlsrgb.Hue;
        mSaturation = hlsrgb.Saturation;
    }

    public HLSRGB() {
    }

    public byte Red {
        get {
            return mRed;
        }
        set {
            mRed = value;
            ToHLS();
        }
    }

    public byte Green {
        get {
            return mGreen;
        }
        set {
            mGreen = value;
            ToHLS();
        }
    }

    public byte Blue {
        get {
            return mBlue;
        }
        set {
            mBlue = value;
            ToHLS();
        }
    }

    public double Luminance {
        get {
            return mLuminance;
        }
        set {
            mLuminance = ChkLum(value);
            ToRGB();
        }
    }

    public double Hue {
        get {
            return mHue;
        }
        set {
            mHue = ChkHue(value);
            ToRGB();
        }
    }

    public double Saturation {
        get {
            return mSaturation;
        }
        set {
            mSaturation = ChkSat(value);
            ToRGB();
        }
    }

    public HueLumSat HLS {
        get {
            return new HueLumSat(mHue, mLuminance, mSaturation);
        }
        set {
            mHue = ChkHue(value.Hue);
            mLuminance = ChkLum(value.Lum);
            mSaturation = ChkSat(value.Sat);
            ToRGB();
        }
    }

    public Color Color {
        get {
            return Color.FromArgb(mAlpha, mRed, mGreen, mBlue);
        }
        set {
            mAlpha = Color.A;
            mRed = value.R;
            mGreen = value.G;
            mBlue = value.B;
            ToHLS();
        }
    }

    public void LightenColor(double lightenBy) {
        mLuminance *= (1.0 + lightenBy);
        if(mLuminance > 1.0)
            Luminance = 1.0;
        ToRGB();
    }

    public int Alpha {
        get {
            return mAlpha;
        }
        set {
            mAlpha = ChkAlpha(value);
        }
    }

    public void DarkenColor(double darkenBy) {
        Luminance *= darkenBy;
        ToRGB();
    }

    private void ToHLS() {
        byte minval = Math.Min(mRed, Math.Min(mGreen, mBlue));
        byte maxval = Math.Max(mRed, Math.Max(mGreen, mBlue));

        double mdiff = (double)maxval - (double)minval;
        double msum = (double)maxval + (double)minval;

        mLuminance = msum / 510.0;

        if(maxval == minval) {
            mSaturation = 0.0;
            mHue = 0.0;
        } else {
            double rnorm = (maxval - mRed) / mdiff;
            double gnorm = (maxval - mGreen) / mdiff;
            double bnorm = (maxval - mBlue) / mdiff;

            if(mLuminance <= 0.5F)
                mSaturation = mdiff / msum;
            else
                mSaturation = mdiff / (510.0 - msum);

            if(mRed == maxval)
                mHue = 60.0 * (6.0 + bnorm - gnorm);
            if(mGreen == maxval)
                mHue = 60.0 * (2.0 + rnorm - bnorm);
            if(mBlue == maxval)
                mHue = 60.0 * (4.0 + gnorm - rnorm);
            if(mHue > 360.0)
                mHue = Hue - 360.0;
        }
    }

    private void ToRGB() {
        if(mSaturation == 0.0) {
            mAlpha = 255;
            mRed = (byte)(mLuminance * 255.0);
            mGreen = mRed;
            mBlue = mRed;
        } else {
            double rm1;
            double rm2;

            if(mLuminance <= 0.5F)
                rm2 = mLuminance + mLuminance * mSaturation;
            else
                rm2 = mLuminance + mSaturation - mLuminance * mSaturation;
            rm1 = 2.0 * mLuminance - rm2;
            mRed = ToRGB1(rm1, rm2, mHue + 120.0);
            mGreen = ToRGB1(rm1, rm2, mHue);
            mBlue = ToRGB1(rm1, rm2, mHue - 120.0);
        }
    }

    private byte ToRGB1(double rm1, double rm2, double rh) {
        if(rh > 360.0)
            rh -= 360.0;
        else if(rh < 0.0)
            rh += 360.0;

        if((rh < 60.0))
            rm1 += (rm2 - rm1) * rh / 60.0;
        else if((rh < 180.0))
            rm1 = rm2;
        else if((rh < 240.0))
            rm1 += (rm2 - rm1) * (240.0 - rh) / 60.0;

        // TODO: Fix this... we shouldn't have to use a Try/Catch
        try {
            return Convert.ToByte(rm1 * 255);
        } catch {
            return Convert.ToByte(255);
        }
    }

    private double ChkHue(double value) {
        if(value < 0.0)
            value = Math.Abs((360.0 + value) % 360.0);
        else if(value > 360.0)
            value %= 360.0;

        return value;
    }

    private double ChkLum(double value) {
        if(value < 0.0)
            value = 0.0;
        else if(value > 1.0)
            value = 1.0;

        return value;
    }

    private double ChkSat(double value) {
        if(value < 0.0)
            value = 0;
        else if(value > 1.0)
            value = 1.0;

        return value;
    }

    private byte ChkAlpha(int value) {
        if(value < 0)
            value = 0;
        else if(value > 255)
            value = 255;

        return (byte)value;
    }
}