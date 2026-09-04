using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using System;

namespace CompDriveBench;

/// <summary>
/// Kaoss-style X/Y pad. Emits normalised 0..1 coordinates; the nts-3 shim
/// converts them to the unit's touch-area units.
/// </summary>
public class TouchPad : Control
{
    private Point? _pos;

    public event Action<float, float, int>? Touch;

    public TouchPad()
    {
        Focusable = true;
        PointerPressed += (_, e) => Emit(e.GetPosition(this), Native.TouchPressed);
        PointerMoved += (_, e) =>
        {
            if (_pos is not null) Emit(e.GetPosition(this), Native.TouchMoved);
        };
        PointerReleased += (_, e) =>
        {
            Emit(e.GetPosition(this), Native.TouchReleased);
            _pos = null;
            InvalidateVisual();
        };
    }

    private void Emit(Point p, int state)
    {
        if (Bounds.Width <= 0 || Bounds.Height <= 0) return;
        float x = (float)Math.Clamp(p.X / Bounds.Width, 0, 1);
        float y = (float)Math.Clamp(p.Y / Bounds.Height, 0, 1);
        if (state != Native.TouchReleased) _pos = p;
        Touch?.Invoke(x, y, state);
        InvalidateVisual();
    }

    public override void Render(DrawingContext ctx)
    {
        double w = Bounds.Width, h = Bounds.Height;
        if (w <= 0 || h <= 0) return;

        ctx.FillRectangle(new SolidColorBrush(Color.FromRgb(22, 26, 30)), new Rect(0, 0, w, h));
        var grid = new Pen(new SolidColorBrush(Color.FromRgb(44, 50, 56)), 1);
        for (int i = 1; i < 4; i++)
        {
            ctx.DrawLine(grid, new Point(w * i / 4, 0), new Point(w * i / 4, h));
            ctx.DrawLine(grid, new Point(0, h * i / 4), new Point(w, h * i / 4));
        }

        if (_pos is { } p)
        {
            var accent = new SolidColorBrush(Color.FromRgb(138, 180, 248));
            var pen = new Pen(accent, 1);
            ctx.DrawLine(pen, new Point(p.X, 0), new Point(p.X, h));
            ctx.DrawLine(pen, new Point(0, p.Y), new Point(w, p.Y));
            ctx.DrawEllipse(accent, null, p, 6, 6);
        }
    }
}
