using System;
using System.Runtime.InteropServices;

namespace CompDriveBench;

/// <summary>
/// P/Invoke surface for libcdhost. The host owns the audio thread; nothing
/// here is called from it, so no managed code sits in the render path.
/// </summary>
internal static class Native
{
    private const string Lib = "cdhost";

    public const int TouchReleased = 0, TouchPressed = 1, TouchMoved = 2;

    [StructLayout(LayoutKind.Sequential)]
    public struct ParamInfo
    {
        public short Min, Max, Center, Init;
        public byte Type, Frac, FracMode;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
        public byte[] NameBytes;

        public string Name
        {
            get
            {
                if (NameBytes == null) return "";
                int n = Array.IndexOf(NameBytes, (byte)0);
                if (n < 0) n = NameBytes.Length;
                return System.Text.Encoding.ASCII.GetString(NameBytes, 0, n);
            }
        }
    }

    [DllImport(Lib)] public static extern int cdh_platform_count();
    [DllImport(Lib)] private static extern IntPtr cdh_platform_name(int i);
    /// <summary>Returns the slot the unit loaded into, or a negative error.</summary>
    [DllImport(Lib, CharSet = CharSet.Ansi)]
    public static extern int cdh_open(int platform, string dylibPath,
        [Out] byte[] err, int errlen);
    [DllImport(Lib)] public static extern void cdh_close_slot(int slot);
    [DllImport(Lib)] public static extern void cdh_close_all();
    [DllImport(Lib)] public static extern int cdh_max_units();
    [DllImport(Lib)] public static extern void cdh_set_active(int slot);
    [DllImport(Lib)] public static extern int cdh_get_active();
    [DllImport(Lib)] private static extern IntPtr cdh_unit_name(int slot);
    [DllImport(Lib)] private static extern IntPtr cdh_unit_platform(int slot);
    [DllImport(Lib)] public static extern int cdh_param_count(int slot);
    [DllImport(Lib)] public static extern int cdh_param_get(int slot, int idx, out ParamInfo info);
    [DllImport(Lib)] public static extern void cdh_set_param(int slot, int id, int value);
    [DllImport(Lib)] public static extern int cdh_get_param(int slot, int id);
    [DllImport(Lib)] public static extern int cdh_in_channels(int slot);
    [DllImport(Lib)] public static extern int cdh_get_bitmap(int slot, int id, [Out] byte[] out32);
    [DllImport(Lib)] public static extern int cdh_has_touch(int slot);
    [DllImport(Lib)] public static extern void cdh_touch(int slot, int id, float x, float y, int state);
    [DllImport(Lib)] public static extern void cdh_set_tempo(int slot, float bpm);
    [DllImport(Lib, CharSet = CharSet.Ansi)] public static extern int cdh_load_wav(string path);
    [DllImport(Lib, CharSet = CharSet.Ansi)] public static extern int cdh_load_sidechain_wav(string path);
    [DllImport(Lib)] public static extern void cdh_clear_sidechain();
    [DllImport(Lib)] public static extern int cdh_start_audio();
    [DllImport(Lib)] public static extern void cdh_stop_audio();
    [DllImport(Lib)] public static extern int cdh_is_running();
    [DllImport(Lib)] public static extern void cdh_set_looping(int on);
    [DllImport(Lib)] public static extern void cdh_set_bypass(int on);
    [DllImport(Lib)] public static extern double cdh_duration();
    [DllImport(Lib)] public static extern double cdh_position();
    [DllImport(Lib)] public static extern void cdh_seek(double seconds);
    [DllImport(Lib)] public static extern void cdh_get_meters(out float inL, out float inR, out float outL, out float outR);
    /// <summary>Fraction of output samples at or over 0 dBFS — a gain fault.</summary>
    [DllImport(Lib)] public static extern float cdh_get_output_over();
    [DllImport(Lib)] public static extern int cdh_diag_count(int slot);
    [DllImport(Lib)] private static extern IntPtr cdh_diag_get(int slot, int idx, out float value);

    /// <summary>Optional per-unit diagnostic; null if the unit exposes none.</summary>
    public static string? DiagGet(int slot, int idx, out float value)
    {
        IntPtr p = cdh_diag_get(slot, idx, out value);
        return p == IntPtr.Zero ? null : Marshal.PtrToStringAnsi(p);
    }
    [DllImport(Lib, CharSet = CharSet.Ansi)] public static extern int cdh_render_offline(string path);

    public static string PlatformName(int i) => Marshal.PtrToStringAnsi(cdh_platform_name(i)) ?? "?";
    public static string UnitName(int slot) => Marshal.PtrToStringAnsi(cdh_unit_name(slot)) ?? "";
    public static string UnitPlatform(int slot) => Marshal.PtrToStringAnsi(cdh_unit_platform(slot)) ?? "";
}

/// <summary>
/// Formats a raw parameter value the way the device's screen would, honouring
/// the descriptor's frac / frac_mode and appending the type's unit.
/// </summary>
internal static class ParamFormat
{
    public static double Scaled(in Native.ParamInfo p, int raw) =>
        p.FracMode == 0 ? raw / (double)(1 << p.Frac) : raw / Math.Pow(10, p.Frac);

    public static string Display(in Native.ParamInfo p, int raw)
    {
        double v = Scaled(p, raw);
        string n = p.Frac > 0 ? v.ToString("0.0") : ((int)v).ToString();
        switch (p.Type)
        {
            case 1:  return n + "%";
            case 2:  return n + " dB";
            case 3:  return (v > 0 ? "+" : "") + n + " C";
            case 4:
            case 5:  return (v > 0 ? "+" : "") + n;
            case 6:  return n + " Hz";
            case 7:  return n + " kHz";
            case 9:  return n + " ms";
            case 10: return n + " s";
            case 14: return v == 0 ? "BAL" : (v < 0 ? "D" + Math.Abs(v).ToString("0.#") : "W" + n);
            case 15: return v == 0 ? "C" : (v < 0 ? "L" + Math.Abs(v).ToString("0") + "%" : "R" + n + "%");
            case 17: return v != 0 ? "on" : "off";
            default: return n;
        }
    }
}
