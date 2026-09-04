using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using System;

namespace CompDriveBench;

/// <summary>Horizontal peak level bar, scaled -60..0 dBFS.</summary>
public class LevelBar : Control
{
    private double _db = -100;

    public void SetLinear(float v)
    {
        double d = 20.0 * Math.Log10(Math.Max(v, 1e-6));
        // fast attack, eased decay so the bar is readable
        _db = d > _db ? d : _db + (d - _db) * 0.25;
        InvalidateVisual();
    }

    public override void Render(DrawingContext ctx)
    {
        double w = Bounds.Width, h = Bounds.Height;
        if (w <= 0 || h <= 0) return;
        ctx.FillRectangle(new SolidColorBrush(Color.FromRgb(30, 32, 36)), new Rect(0, 0, w, h));

        double norm = Math.Clamp((_db + 60.0) / 60.0, 0, 1);
        if (norm <= 0) return;

        IBrush brush = _db > -1.0 ? Brushes.OrangeRed
                     : _db > -6.0 ? Brushes.Goldenrod
                                  : new SolidColorBrush(Color.FromRgb(80, 200, 120));
        ctx.FillRectangle(brush, new Rect(0, 0, w * norm, h));
    }
}
