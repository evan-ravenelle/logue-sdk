using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using System;

namespace CompDriveBench;

/// <summary>
/// Shows what fraction of the signal is being clipped, 0..1, with an eased
/// decay so brief events stay readable. Two of these run side by side:
/// intentional saturation from the drive stage, and output overs — which are
/// a gain-staging fault, so they get a different colour.
/// </summary>
public class ClipBar : Control
{
    private double _v;

    public static readonly StyledProperty<bool> IsFaultProperty =
        AvaloniaProperty.Register<ClipBar, bool>(nameof(IsFault));

    public bool IsFault
    {
        get => GetValue(IsFaultProperty);
        set => SetValue(IsFaultProperty, value);
    }

    public void SetFraction(float f)
    {
        double t = Math.Clamp(f, 0, 1);
        _v = t > _v ? t : _v + (t - _v) * 0.2;   // snap up, ease down
        InvalidateVisual();
    }

    public override void Render(DrawingContext ctx)
    {
        double w = Bounds.Width, h = Bounds.Height;
        if (w <= 0 || h <= 0) return;
        ctx.FillRectangle(new SolidColorBrush(Color.FromRgb(30, 32, 36)), new Rect(0, 0, w, h));
        if (_v <= 0.001) return;

        var brush = IsFault
            ? new SolidColorBrush(Color.FromRgb(230, 80, 70))     // overs: a problem
            : new SolidColorBrush(Color.FromRgb(240, 170, 60));   // drive: intended
        ctx.FillRectangle(brush, new Rect(0, 0, w * _v, h));
    }
}
